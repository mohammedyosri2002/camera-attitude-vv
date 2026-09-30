#!/usr/bin/env python3
"""
analysis/board_x/audit_board_x_reference.py
===========================================
BOARD-X EXACT-REFERENCE REPRODUCIBILITY AUDIT. CREATED POST-CAMPAIGN.

    python analysis/board_x/audit_board_x_reference.py

Question under audit
--------------------
Offline re-solution of the Board-X headline run from stored raw AprilTag corner pixels
returned a different static RMSE from the submitted 0.2678 deg. The working hypothesis was
that the difference came from the ZERO-REFERENCE DEFINITION. This script tested that
hypothesis instead of assuming it.

RESULT: THE HYPOTHESIS WAS WRONG. The reference definition accounts for only about
0.004 deg of a 0.093 deg difference; the two candidate references differ by 0.006 deg.
The real cause was the INTRINSIC MATRIX. The acquisition undistorts each frame and solves
with `newK` from getOptimalNewCameraMatrix, so the logged corners are in undistorted image
coordinates and newK is the matrix that belongs with them. The first offline pipeline used
the raw `camera_matrix` from the metadata - a 7.54 % focal-length error (fx 1443.25 vs
1334.37). With newK AND the acquisition reference construction, the re-solved attitude
matches the logged acquisition attitude to 0.000000 deg and the submitted 0.2678 deg is
reproduced exactly. Both fixes are now in the pipeline
(analysis/common/io.py:camera_intrinsics, analysis/common/geometry_utils.py:relative_attitude).

This script is retained so the diagnosis can be re-run and checked.

What the acquisition code actually did
--------------------------------------
From src/original/out_of_plane/basler_apriltag_pitch_roll_FINAL.py:

    REF_MIN_FRAMES = 200
    ...
    if R_ref is None:
        ref_pool.append(R)
        if len(ref_pool) >= REF_MIN_FRAMES:
            R_ref = chordal_mean_rotation(ref_pool)

The pool is filled with the FIRST 200 VALID-POSE FRAMES OF THE CAPTURE, in capture order,
with no reference to motor time. The camera starts before the motor, so those frames
precede the motor baseline window that offline reprocessing would naturally use.

Procedure
---------
1. Identify the exact first 200 valid-pose frames the acquisition used (pose_ok == 1,
   capture order).
2. Re-solve those same raw corner observations with the same Board-X configured layout,
   the same per-run intrinsics, the same IPPE candidate-selection logic, the same corner
   ordering, and the same relative-rotation definition R_rel = R_ref^T R_board.
3. Build the zero reference from those exact frames by chordal mean.
4. Recompute the Board-X scalar under both conventions:
     (a) acquisition convention  - active relative Euler component
     (b) board-axis projection   - relative rotation vector onto the calibrated hinge axis
5. Compare against the logged acquisition attitude, sample by sample.

Expected values are never modified to force agreement.
"""
from __future__ import annotations

import json
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from common import io as cio                                      # noqa: E402
from common import motor_reference as mref                        # noqa: E402
from common import camera_motor_sync as cms                       # noqa: E402
from common import geometry_utils as geo                          # noqa: E402
from common.attitude_metrics import metrics, pooled, HOLD_GUARD_S  # noqa: E402

KEY = "board_x/axis_b_run01"
SUBMITTED_RMSE = 0.2678   # Board-X headline, as submitted


def euler_component(rvec, which):
    idx = {"roll": 0, "pitch": 1, "yaw": 2}[which]
    out = np.full(len(rvec), np.nan)
    for k in range(len(rvec)):
        if not np.isfinite(rvec[k, 0]):
            continue
        R, _ = cv2.Rodrigues(rvec[k])
        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        e = (math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy),
             math.atan2(R[1, 0], R[0, 0]))
        out[k] = math.degrees(e[idx])
    return out


def static_metrics(theta, ok, tc, holds):
    E = []
    for h in holds:
        m = (tc >= h["t0"] + HOLD_GUARD_S) & (tc <= h["t1"] - HOLD_GUARD_S) & ok
        E.append(theta[m] - h["target_deg"])
    return pooled(E)


def main():
    info = cio.run_info(KEY)
    print("=" * 78)
    print("BOARD-X EXACT-REFERENCE REPRODUCIBILITY AUDIT")
    print(f"run: {KEY}   (original label {info['original_run_label']})")
    print("=" * 78)

    cam, cmeta, cam_name = cio.load_camera(KEY)
    ev, _ = cio.load_motor(KEY)
    holds, _, _, baselines, T0, T1 = mref.segment_profile(ev)
    fit = cms.fit_from_events(ev)
    tc = cms.camera_times_to_motor_clock(cam, fit)
    g, gname = geo.select_geometry(KEY)
    obj = geo.build_object_points(g)
    K, _, _ = cio.camera_intrinsics(KEY)
    ref_n = int(cmeta.get("ref_min_frames", geo.REF_MIN_FRAMES))

    print(f"\n  configured board layout : {gname}  [{g['geometry_source']}]")
    print(f"  intrinsics              : from {cam_name.replace('.csv', '_meta.json')}")
    print(f"  REF_MIN_FRAMES          : {ref_n} (from the acquisition metadata)")
    print(f"  ambiguity ratio         : {geo.AMBIGUITY_RATIO}")

    # ---- STEP 1: the exact frames the acquisition used ----------------------
    pose_ok_logged = cam["pose_ok"].to_numpy(int).astype(bool)
    acq_ref_idx = np.where(pose_ok_logged)[0][:ref_n]
    rvec_logged = np.column_stack([cam[f"rel_rvec_{c}"].to_numpy(float) for c in "xyz"])
    first_rel = int(np.where(np.isfinite(rvec_logged[:, 0]))[0][0])
    print(f"\n[1] acquisition reference pool: frames "
          f"{acq_ref_idx[0]}..{acq_ref_idx[-1]} (capture order, {len(acq_ref_idx)} frames)")
    print(f"    first frame with a logged relative attitude: {first_rel}")
    print(f"    -> consistent with a pool of the first {ref_n} valid frames: "
          f"{first_rel == acq_ref_idx[-1] + 1 or first_rel == acq_ref_idx[-1]}")
    print(f"    motor-clock time of the pool: {tc[acq_ref_idx[0]]:.3f} .. "
          f"{tc[acq_ref_idx[-1]]:.3f} s")
    print(f"    motor baseline window      : {baselines['start'][0]:.3f} .. "
          f"{baselines['start'][1]:.3f} s")
    print(f"    the pool PRECEDES the motor baseline by "
          f"{baselines['start'][0] - tc[acq_ref_idx[-1]]:.3f} s")

    # ---- STEP 2: re-solve from raw corner pixels ---------------------------
    print(f"\n[2] re-solving all {len(cam)} frames from raw corner pixels ...")
    sol = geo.solve_board_sequence(cam, obj, K)
    ok_re = sol["ok"]
    print(f"    valid pose {100 * ok_re.mean():.4f} %   branch flips {sol['n_flips']}   "
          f"ambiguous {int(sol['ambiguous'].sum())}")

    # ---- STEP 3: acquisition-equivalent reference --------------------------
    usable = [int(i) for i in acq_ref_idx if ok_re[i]]
    R_ref_acq = geo.chordal_mean_rotation([sol["R"][i] for i in usable])
    scat = float(np.mean([geo.geodesic_deg(R_ref_acq, sol["R"][i]) for i in usable]))
    print(f"\n[3] acquisition-equivalent reference from the SAME {len(usable)} frames")
    print(f"    scatter about the reference: {scat:.4f} deg")

    # offline-natural reference, for contrast
    _, rvec_off, n_off, scat_off = geo.relative_attitude(
        sol["R"], ok_re, tc, baselines["start"])
    print(f"    (contrast) offline-natural reference from {n_off} motor-baseline frames, "
          f"scatter {scat_off:.4f} deg")
    print(f"    angle between the two references: "
          f"{geo.geodesic_deg(R_ref_acq, geo.chordal_mean_rotation([sol['R'][i] for i in np.where(ok_re & (tc >= baselines['start'][0]) & (tc <= baselines['start'][1]))[0][:n_off]])):.4f} deg")

    rvec_acqref = np.full((len(cam), 3), np.nan)
    for k in np.where(ok_re)[0]:
        rvec_acqref[k] = cv2.Rodrigues(R_ref_acq.T @ sol["R"][k])[0].ravel()

    # ---- STEP 4: both scalar conventions -----------------------------------
    active = info["active_euler_component"]
    n_hat, dom, off = geo.calibrated_axis(rvec_acqref, tc, ok_re, holds, HOLD_GUARD_S)
    theta_logged = euler_component(rvec_logged, active)
    ok_logged = pose_ok_logged & np.isfinite(rvec_logged[:, 0])
    theta_re_euler = euler_component(rvec_acqref, active)
    theta_re_proj = np.degrees(rvec_acqref @ n_hat)
    theta_off_euler = euler_component(rvec_off, active)

    variants = [
        ("A  logged acquisition pose, Euler component", theta_logged, ok_logged),
        ("B  re-solved, ACQUISITION-EQUIVALENT ref, Euler", theta_re_euler, ok_re),
        ("C  re-solved, ACQUISITION-EQUIVALENT ref, axis projection", theta_re_proj, ok_re),
        ("D  re-solved, offline-natural (motor-baseline) ref, Euler", theta_off_euler, ok_re),
    ]
    print(f"\n[4] Board-X static metrics, RAW, all against the pulse-derived commanded "
          f"reference")
    print(f"    {'variant':<58} {'RMSE':>9} {'Bias':>9} {'MAE':>9} {'MaxAbs':>9} {'N':>7}")
    out = {}
    for lbl, th, okk in variants:
        s = static_metrics(th, okk, tc, holds)
        out[lbl[0]] = s
        print(f"    {lbl:<58} {s['RMSE']:9.4f} {s['Bias']:9.4f} {s['MAE']:9.4f} "
              f"{s['MaxAbs']:9.4f} {s['N']:7d}")

    # ---- STEP 5: sample-wise comparison ------------------------------------
    both = ok_logged & ok_re
    d_euler = np.abs(theta_re_euler[both] - theta_logged[both])
    d_proj = np.abs(theta_re_proj[both] - theta_logged[both])
    d_off = np.abs(theta_off_euler[both] - theta_logged[both])
    # full pose difference, independent of any scalar choice
    idx = np.where(both)[0][::37]
    d_pose = np.array([geo.geodesic_deg(
        cv2.Rodrigues(rvec_logged[k])[0], cv2.Rodrigues(rvec_acqref[k])[0]) for k in idx])

    print(f"\n[5] sample-wise difference, logged vs re-solved ({int(both.sum())} frames)")
    print(f"    acquisition-equivalent ref, Euler   : max {d_euler.max():.6f} deg, "
          f"median {np.median(d_euler):.6f}, RMS {np.sqrt((d_euler**2).mean()):.6f}")
    print(f"    acquisition-equivalent ref, axis proj: max {d_proj.max():.6f} deg, "
          f"median {np.median(d_proj):.6f}")
    print(f"    offline-natural ref, Euler          : max {d_off.max():.6f} deg, "
          f"median {np.median(d_off):.6f}")
    print(f"    full relative-pose geodesic (n={len(idx)}) : max {d_pose.max():.6f} deg, "
          f"median {np.median(d_pose):.6f}")

    # ---- verdict ------------------------------------------------------------
    dB = abs(out["B"]["RMSE"] - SUBMITTED_RMSE)
    dD = abs(out["D"]["RMSE"] - SUBMITTED_RMSE)
    reproduced = dB < 5e-4
    print("\n" + "=" * 78)
    print("VERDICT")
    print("=" * 78)
    print(f"  submitted Board-X headline                       : {SUBMITTED_RMSE:.4f} deg")
    print(f"  A logged acquisition pose                        : {out['A']['RMSE']:.4f} deg "
          f"(delta {out['A']['RMSE'] - SUBMITTED_RMSE:+.6f})")
    print(f"  B re-solved with acquisition-equivalent reference : {out['B']['RMSE']:.4f} deg "
          f"(delta {out['B']['RMSE'] - SUBMITTED_RMSE:+.6f})")
    print(f"  D re-solved with offline-natural reference        : {out['D']['RMSE']:.4f} deg "
          f"(delta {out['D']['RMSE'] - SUBMITTED_RMSE:+.6f})")
    print(f"  reference choice accounts for                     : "
          f"{abs(out['B']['RMSE'] - out['D']['RMSE']):.4f} deg of the difference")
    print(f"  residual, not explained by the reference          : {dB:.4f} deg")
    print()
    if reproduced:
        print("  RESULT: the submitted 0.2678 deg IS reproduced from raw corner pixels")
        print("          once the exact acquisition reference construction is replicated.")
    else:
        print("  RESULT: the submitted 0.2678 deg is NOT fully reproduced from raw corner")
        print("          pixels even with the acquisition reference construction replicated.")
        print("          The reference definition explains part of the difference; a")
        print("          residual remains and is documented, not hidden.")
    print("=" * 78)

    rep = dict(
        run=KEY, original_label=info["original_run_label"],
        submitted_rmse_deg=SUBMITTED_RMSE,
        acquisition_reference_frames=[int(acq_ref_idx[0]), int(acq_ref_idx[-1])],
        acquisition_reference_frames_usable=len(usable),
        acquisition_pool_motor_clock_s=[float(tc[acq_ref_idx[0]]), float(tc[acq_ref_idx[-1]])],
        motor_baseline_window_s=[float(baselines["start"][0]), float(baselines["start"][1])],
        pool_precedes_baseline_by_s=float(baselines["start"][0] - tc[acq_ref_idx[-1]]),
        variants={k: v for k, v in out.items()},
        samplewise_difference_deg=dict(
            acq_ref_euler_max=float(d_euler.max()),
            acq_ref_euler_median=float(np.median(d_euler)),
            acq_ref_euler_rms=float(np.sqrt((d_euler ** 2).mean())),
            acq_ref_projection_max=float(d_proj.max()),
            offline_ref_euler_max=float(d_off.max()),
            full_relative_pose_geodesic_max=float(d_pose.max()),
            full_relative_pose_geodesic_median=float(np.median(d_pose))),
        reference_choice_explains_deg=float(abs(out["B"]["RMSE"] - out["D"]["RMSE"])),
        residual_unexplained_deg=float(dB),
        submitted_value_reproduced_from_raw_corners=bool(reproduced),
        calibrated_axis=n_hat.tolist(), axis_dominance=float(dom),
        conclusion=("Reproduced from raw corners with the acquisition reference "
                    "construction." if reproduced else
                    "Not fully reproduced from raw corners; residual documented."))
    p = os.path.join(cio.RESULTS, "board_x_reference_audit.json")
    os.makedirs(cio.RESULTS, exist_ok=True)
    json.dump(rep, open(p, "w"), indent=2, default=float)
    print(f"\n-> {os.path.relpath(p, cio.REPO_ROOT)}")
    return rep


if __name__ == "__main__":
    main()
