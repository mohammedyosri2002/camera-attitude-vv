"""
analysis/common/out_of_plane.py
===============================
Reproduction engine shared by the Board-X and Board-Y out-of-plane runs.
CREATED POST-CAMPAIGN.

Every headline number is RAW. No filtering, no fitted scale, no drift removal.
The camera pose is RE-SOLVED for every frame from the stored raw AprilTag corner pixels
using the run's own configured board layout; stored pose columns are ignored.
"""
from __future__ import annotations
import math
import numpy as np

from . import io as cio
from . import motor_reference as mref
from . import camera_motor_sync as cms
from . import imu_motor_sync as ims
from . import geometry_utils as geo
from .attitude_metrics import (metrics, pooled, finite_difference_rate,
                               displacement_rate, HOLD_GUARD_S, LEG_TRIM_FRAC)
from . import rate_estimators as rest


def _euler_component(rvec, which):
    """Relative Euler component (intrinsic X-Y-Z) from the relative rotation vector."""
    import cv2
    out = np.full(len(rvec), np.nan)
    idx = {"roll": 0, "pitch": 1, "yaw": 2}[which]
    for k in range(len(rvec)):
        if not np.isfinite(rvec[k, 0]):
            continue
        R, _ = cv2.Rodrigues(rvec[k])
        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        e = (math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy),
             math.atan2(R[1, 0], R[0, 0]))
        out[k] = math.degrees(e[idx])
    return out


def process(key: str, verbose=True):
    say = (lambda *a: print(*a)) if verbose else (lambda *a: None)
    info = cio.run_info(key)
    say(f"\n=== {key}  [{info['canonical_axis']}]  original label: "
        f"{info['original_run_label']} ===")

    # ---- motor: the PULSE-DERIVED COMMANDED REFERENCE -----------------------
    ev, fit_logged = cio.load_motor(key)
    holds, legs, motion, baselines, T0, T1 = mref.segment_profile(ev)
    say(f"  motor: {len(ev)} events, {len(holds)} holds, {len(legs)} rate legs, "
        f"span {T1 - T0:.3f} s")

    # ---- camera <-> motor clock, refitted from serial events only -----------
    cam, cmeta, cam_name = cio.load_camera(key)
    fit = cms.fit_from_events(ev)
    tc = cms.camera_times_to_motor_clock(cam, fit)
    say(f"  camera<-motor: a={fit['a']:.9f} ({fit['ppm']:+.1f} ppm), "
        f"residual SD {fit['residual_sd_ms']:.3f} ms, {fit['n_events']} events")

    # ---- camera pose re-solved from RAW CORNER PIXELS -----------------------
    g, gname = geo.select_geometry(key)
    obj = geo.build_object_points(g)
    K, _, _ = cio.camera_intrinsics(key)
    say(f"  geometry: {gname}  [{g.get('geometry_source')}]  (camera-derived, "
        f"not dimensional ground truth)")
    # Conditioning diagnostics are ALWAYS recomputed from raw corner pixels.
    # The attitude itself comes from whichever pose source RUN_INFO.json declares.
    say(f"  re-solving {len(cam)} frames from raw corner pixels ...")
    sol = geo.solve_board_sequence(cam, obj, K)

    pose_source = info["pose_source"]
    if pose_source == "reprocessed_from_raw_corners":
        # Acquisition geometry was wrong; logged pose columns are ignored entirely.
        ok = sol["ok"]
        _, rvec, n_ref, scat = geo.relative_attitude(sol["R"], ok, tc,
                                                     baselines["start"])
    elif pose_source == "acquisition_logged":
        # Acquisition geometry was correct; the logged relative attitude is authoritative.
        # Re-solving here would change the zero reference (the acquisition established it
        # from the first frames of the CAPTURE, which precede the motor baseline) and
        # would silently shift every number.
        rvec = np.column_stack([cam[f"rel_rvec_{c}"].to_numpy(float) for c in "xyz"])
        ok = cam["pose_ok"].to_numpy(int).astype(bool) & np.isfinite(rvec[:, 0])
        n_ref = int(cmeta.get("ref_min_frames", geo.REF_MIN_FRAMES))
        scat = float("nan")
    else:
        raise ValueError(f"unknown pose_source {pose_source!r} in RUN_INFO.json")
    say(f"  pose source: {pose_source}   valid {100 * ok.mean():.4f} %  "
        f"flips {sol['n_flips']}  ambiguous {int(sol['ambiguous'].sum())}")

    n_hat, dominance, offaxis = geo.calibrated_axis(rvec, tc, ok, holds, HOLD_GUARD_S)

    # RAW scalar camera attitude.
    if info["scalar_attitude"] == "rotation_vector_axis_projection":
        theta = np.degrees(rvec @ n_hat)
        theta_alt = _euler_component(rvec, info["active_euler_component"])
    else:
        theta = _euler_component(rvec, info["active_euler_component"])
        theta_alt = np.degrees(rvec @ n_hat)
    say(f"  scalar: {info['scalar_attitude']} (active Euler component "
        f"{info['active_euler_component']})")
    rate_cam = finite_difference_rate(theta, tc)     # RAW, unsmoothed -- PRIMARY
    # SECONDARY DIAGNOSTIC: one frozen SG estimator, identical across all three axes.
    # Never replaces the RAW result and never labelled a RAW accuracy.
    _sg = np.full(len(theta), np.nan)
    _m = ok & np.isfinite(theta)
    _d, _n_sg, _fs_sg = rest.sg_rate(tc[_m], theta[_m])
    if _d is not None:
        _sg[_m] = _d
    say(f"  board-frame hinge axis ({n_hat[0]:+.5f}, {n_hat[1]:+.5f}, {n_hat[2]:+.5f})  "
        f"s1/s2={dominance:.1f}  off-axis {offaxis:.4f} deg")

    # ---- IMU <-> motor clock, rate domain, coverage-constrained -------------
    imu, imu_name = cio.load_imu(key)
    f_imu = ims.fit(imu, motion, T0, T1, dominant_axis=info["gyro_channel"])
    ti = ims.apply(imu, f_imu)
    say(f"  IMU<-motor: a={f_imu['a']:.9f} ({f_imu['ppm']:+.1f} ppm), "
        f"residual {f_imu['residual_dps_rms']:.4f} deg/s, "
        f"coverage {f_imu['coverage_pct']:.2f} % (>= {f_imu['min_coverage_pct']:.0f} %)")

    gyro, gyro_meta = geo.gyro_reference(imu, ti, legs, baselines, LEG_TRIM_FRAC)
    tilt, grav_meta = geo.gravity_tilt(imu, ti, holds, baselines, HOLD_GUARD_S)

    # ================= STATIC, RAW =========================================
    hold_rows, S = [], dict(cm=[], im=[], ci=[])
    for i, h in enumerate(holds):
        mc = (tc >= h["t0"] + HOLD_GUARD_S) & (tc <= h["t1"] - HOLD_GUARD_S) & ok
        mi = (ti >= h["t0"] + HOLD_GUARD_S) & (ti <= h["t1"] - HOLD_GUARD_S)
        c = theta[mc]; gt = tilt[mi]
        gi = np.interp(tc[mc], ti, tilt)     # raw tilt onto the camera grid
        e_cm, e_im, e_ci = c - h["target_deg"], gt - h["target_deg"], c - gi
        S["cm"].append(e_cm); S["im"].append(e_im); S["ci"].append(e_ci)
        # approach_direction is a derived LABEL only; it changes no residual.
        if i == 0:
            approach = "initial"
        elif h["target_deg"] > holds[i - 1]["target_deg"] + 1e-9:
            approach = "ascending"
        elif h["target_deg"] < holds[i - 1]["target_deg"] - 1e-9:
            approach = "descending"
        else:
            approach = "repeat"
        hold_rows.append(dict(
            hold=i, approach_direction=approach,
            commanded_reference_deg=h["target_deg"], pulses=h["pulses"],
            tags=int(np.median(sol["n_visible"][mc])),
            camera_mean_deg=float(c.mean()), gravity_tilt_mean_deg=float(gt.mean()),
            **{f"Camera_Motor_{k}": v for k, v in metrics(e_cm).items()},
            **{f"IMUgrav_Motor_{k}": v for k, v in metrics(e_im).items()},
            **{f"Camera_IMUgrav_{k}": v for k, v in metrics(e_ci).items()}))

    # sensitivity: the SAME data under the other scalar convention, reported so the
    # choice of scalar is visible rather than hidden.
    alt = []
    for h in holds:
        mc = (tc >= h["t0"] + HOLD_GUARD_S) & (tc <= h["t1"] - HOLD_GUARD_S) & ok
        alt.append(theta_alt[mc] - h["target_deg"])
    static_alt = pooled(alt)

    static = dict(camera_vs_commanded=pooled(S["cm"]),
                  gravity_vs_commanded=pooled(S["im"]),
                  camera_vs_gravity=pooled(S["ci"]))

    # zero-return departure -- reported, NEVER removed from headline numbers
    zero = []
    for h in holds:
        if abs(h["target_deg"]) > 0.5:
            continue
        mc = (tc >= h["t0"] + HOLD_GUARD_S) & (tc <= h["t1"] - HOLD_GUARD_S) & ok
        mi = (ti >= h["t0"] + HOLD_GUARD_S) & (ti <= h["t1"] - HOLD_GUARD_S)
        zero.append(dict(t_motor_s=float(h["t0"]), camera_deg=float(theta[mc].mean()),
                         gravity_tilt_deg=float(tilt[mi].mean()),
                         difference_deg=float(theta[mc].mean() - tilt[mi].mean())))

    # hysteresis: same commanded target from opposite approach directions
    hyst = []
    by = {}
    for r in hold_rows:
        by.setdefault(round(r["commanded_reference_deg"], 5), []).append(r)
    for q, rows in sorted(by.items()):
        if len(rows) < 2 or abs(q) < 0.5:
            continue
        hyst.append(dict(commanded_reference_deg=q,
                         camera_first_deg=rows[0]["camera_mean_deg"],
                         camera_second_deg=rows[1]["camera_mean_deg"],
                         camera_difference_deg=rows[1]["camera_mean_deg"] - rows[0]["camera_mean_deg"],
                         gravity_difference_deg=rows[1]["gravity_tilt_mean_deg"] - rows[0]["gravity_tilt_mean_deg"]))

    # ================= DYNAMIC, RAW ========================================
    leg_rows, D = [], dict(cm=[], im=[], ci=[])
    scale_rows = []
    for j, L in enumerate(legs):
        tr = LEG_TRIM_FRAC * (L["t1"] - L["t0"])
        a0, a1 = L["t0"] + tr, L["t1"] - tr
        mc = (tc >= a0) & (tc <= a1) & ok
        mi = (ti >= a0) & (ti <= a1)
        r = L["rate_dps"]
        c = rate_cam[mc]; gy = gyro[mi]
        c_on_i = np.interp(ti[mi], tc[ok], rate_cam[ok])
        e_cm, e_im, e_ci = c - r, gy - r, c_on_i - gy
        D["cm"].append(e_cm); D["im"].append(e_im); D["ci"].append(e_ci)
        e_sg = _sg[mc] - r
        D.setdefault("sg", []).append(e_sg)
        leg_rows.append(dict(leg=j, commanded_rate_dps=r,
                             **{f"SG_Camera_Motor_{k}": v for k, v in metrics(e_sg).items()},
                             direction="positive" if r > 0 else "negative",
                             leg_duration_s=float(L["t1"] - L["t0"]),
                             camera_mean_rate_dps=float(np.mean(c)),
                             gyro_mean_rate_dps=float(np.mean(gy)),
                             **{f"Camera_Motor_{k}": v for k, v in metrics(e_cm).items()},
                             **{f"IMU_Motor_{k}": v for k, v in metrics(e_im).items()},
                             **{f"Camera_IMU_{k}": v for k, v in metrics(e_ci).items()}))
        # rate-SCALE metric: displacement over the leg. Not an instantaneous RMSE.
        dr = displacement_rate(theta[mc], tc[mc])
        scale_rows.append(dict(leg=j, commanded_rate_dps=r,
                               direction="positive" if r > 0 else "negative",
                               camera_displacement_rate_dps=dr,
                               error_dps=dr - r, error_pct=100.0 * (dr - r) / r))

    dynamic = dict(camera_raw_rate_vs_commanded=pooled(D["cm"]),
                   gyro_vs_commanded=pooled(D["im"]),
                   camera_raw_rate_vs_gyro=pooled(D["ci"]))
    # SECONDARY, clearly separated from every RAW key above
    sg_by_rate = {}
    for j, L in enumerate(legs):
        sg_by_rate.setdefault(abs(L["rate_dps"]), []).append(D["sg"][j])
    dynamic_sg_SECONDARY = dict(
        estimator=rest.describe(),
        sg_window_samples=int(_n_sg), sample_rate_hz=float(_fs_sg),
        camera_sg_rate_vs_commanded=pooled(D["sg"]),
        per_commanded_rate={str(k): pooled(v) for k, v in sorted(sg_by_rate.items())})

    # ================= CONDITIONING ========================================
    # Conditioning is a property of the POSE SOLVE, so it always uses the re-solved
    # result, independent of which pose source supplies the attitude.
    inrun = sol["ok"] & (tc >= T0) & (tc <= T1)
    tl = geo.board_tilt_from_fronto_parallel(sol["R"], inrun, 100)
    nv = sol["n_visible"]
    cond = dict(
        n_frames=int(len(cam)),
        valid_pose_pct=float(100.0 * sol["ok"].mean()),
        relative_attitude_available_pct=float(100.0 * ok.mean()),
        relative_attitude_note=("frames before the zero reference is established carry a "
                                "valid absolute pose but no relative attitude"),
        branch_flips=int(sol["n_flips"]),
        ambiguous_frames=int(sol["ambiguous"].sum()),
        four_tag_visibility_pct=float(100.0 * (nv == 4).mean()),
        visible_count_histogram={int(k): int(v) for k, v in
                                 zip(*np.unique(nv, return_counts=True))},
        per_tag_visibility_pct={str(t): float(100.0 * cam[f"tag_{t}_visible"].mean())
                                for t in geo.EXPECTED_TAG_IDS},
        board_reprojection_px=dict(
            mean=float(np.nanmean(sol["reproj"][inrun])),
            median=float(np.nanmedian(sol["reproj"][inrun])),
            p95=float(np.nanpercentile(sol["reproj"][inrun], 95)),
            max=float(np.nanmax(sol["reproj"][inrun]))),
        candidate_separation_deg=dict(
            min=float(np.nanmin(sol["sep"][inrun])),
            median=float(np.nanmedian(sol["sep"][inrun])),
            max=float(np.nanmax(sol["sep"][inrun]))),
        reprojection_ratio=dict(
            min=float(np.nanmin(sol["ratio"][inrun])),
            median=float(np.nanmedian(sol["ratio"][inrun])),
            max=float(np.nanmax(sol["ratio"][inrun]))),
        board_tilt_from_fronto_parallel_deg=dict(min=float(tl.min()), max=float(tl.max())),
        camera_fps_median=float(1.0 / np.median(np.diff(tc))),
        reference_frames=int(n_ref), reference_scatter_deg=float(scat))

    return dict(
        run_key=key, canonical_axis=info["canonical_axis"],
        original_run_label=info["original_run_label"],
        camera_file=cam_name, imu_file=imu_name,
        geometry_file=gname, geometry_source=g.get("geometry_source"),
        sync=dict(camera_motor=fit, imu_motor=f_imu),
        hinge_axis_board_frame=n_hat.tolist(), axis_dominance=dominance,
        axis_off_axis_rms_deg=offaxis,
        angle_to_board_plus_y_deg=float(math.degrees(math.acos(min(abs(n_hat[1]), 1.0)))),
        gyro_reference=gyro_meta, gravity_reference=grav_meta,
        pose_source=pose_source, scalar_attitude=info["scalar_attitude"],
        static=static, static_alternative_scalar=static_alt, dynamic=dynamic,
        dynamic_sg_SECONDARY=dynamic_sg_SECONDARY,
        conditioning=cond,
        rate_scale_pct=dict(min=float(min(r["error_pct"] for r in scale_rows)),
                            max=float(max(r["error_pct"] for r in scale_rows))),
        zero_return=zero, hysteresis=hyst,
        tables=dict(holds=hold_rows, legs=leg_rows, rate_scale=scale_rows))
