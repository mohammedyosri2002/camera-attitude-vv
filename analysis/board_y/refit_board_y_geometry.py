#!/usr/bin/env python3
"""
analysis/board_y/refit_board_y_geometry.py
========================================
DETERMINISTIC REFIT OF THE RUN05 BOARD LAYOUT WITH THE CORRECT INTRINSIC MATRIX.
CREATED POST-CAMPAIGN.

    python analysis/board_y/refit_board_y_geometry.py

Why this exists
---------------
`config/board_geometry_boardY_bundle_adjusted.json` was produced BEFORE the
raw-`camera_matrix` versus undistorted-`newK` issue was found. This script determines
conclusively which matrix that original fit used, and reproduces the layout with the
correct one.

The acquisition undistorts every frame before detection and solves with

    newK, _ = cv2.getOptimalNewCameraMatrix(CAMERA_MATRIX, DIST_COEFFS, (w,h), 1, (w,h))

so the stored corner pixels are in UNDISTORTED image coordinates and `newK` is the matrix
that belongs with them. On this rig fx is 1443.2497 (raw) versus 1334.3666 (newK), a
7.54 % difference.

METHOD (identical to the original export except for the intrinsic matrix)
------------------------------------------------------------------------
FRAME SELECTION      deterministic, no randomness:
                       * 3 frames per static hold, evenly spaced by np.linspace over the
                         window trimmed 1.5 s at each end;
                       * 2 frames per constant-rate leg, evenly spaced by np.linspace over
                         the central 70 % of the leg;
                       * a frame is kept only if at least 3 expected tags are visible and
                         pose_ok == 1.
OPTIMISED PARAMETERS 9 geometry (tags 1, 2, 3: centre x, centre y, in-plane rotation)
                     + 6 per selected frame (Rodrigues rotation vector + translation).
GAUGE CONSTRAINTS    tag 0 held fixed in position and at rotation_deg = 0, which fixes the
                     in-plane rotation and the origin of the frame during the fit. After
                     convergence all four centres are translated so their centroid is the
                     origin; a pure translation of the object points is absorbed by the
                     pose translation and changes no residual and no attitude.
FIXED TAG SIZE       tag_black_square_size_m = 0.070 m, HELD FIXED. It sets the absolute
                     scale and a monocular camera has no independent length reference;
                     fitting it would trade scale against range.
INITIAL GEOMETRY     the Board-X layout (config/board_geometry_boardX.json), i.e. the same
                     starting point the original fit used.
OPTIMISER            scipy.optimize.least_squares, method 'trf', x_scale='jac'.
TERMINATION          xtol = ftol = gtol = 1e-14, max_nfev = 600.

Outputs `results/board_y_geometry_refit.json` and, only if the refit differs materially,
`config/board_geometry_boardY_bundle_adjusted_newK.json`.
"""
from __future__ import annotations

import json
import math
import os
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from common import io as cio                                   # noqa: E402
from common import motor_reference as mref                     # noqa: E402
from common import camera_motor_sync as cms                    # noqa: E402
from common import geometry_utils as geo                       # noqa: E402
from common.attitude_metrics import HOLD_GUARD_S, LEG_TRIM_FRAC  # noqa: E402

KEY = "board_y/run05"
SIZE_M = 0.070
IDS = geo.EXPECTED_TAG_IDS
FRAMES_PER_HOLD = 3
FRAMES_PER_LEG = 2
MAX_NFEV = 600

# A refit counts as MATERIALLY DIFFERENT if any tag centre moves by more than this, or any
# rotation by more than ROT_TOL_DEG. 0.05 mm is well inside the parameter uncertainty of
# the original fit (0.08-0.15 mm per coordinate), so anything below it cannot change a
# reported result.
CENTRE_TOL_MM = 0.05
ROT_TOL_DEG = 0.01


def select_frames(cam, tc, holds, legs):
    """Deterministic frame selection. Returns sorted row indices."""
    vis = np.column_stack([cam[f"tag_{t}_visible"].to_numpy(int) for t in IDS]).sum(1)
    ok = cam["pose_ok"].to_numpy(int).astype(bool) & (vis >= 3)
    sel = []
    for h in holds:
        idx = np.where((tc >= h["t0"] + HOLD_GUARD_S) &
                       (tc <= h["t1"] - HOLD_GUARD_S) & ok)[0]
        if len(idx):
            sel += list(idx[np.linspace(0, len(idx) - 1, FRAMES_PER_HOLD).astype(int)])
    for L in legs:
        tr = LEG_TRIM_FRAC * (L["t1"] - L["t0"])
        idx = np.where((tc >= L["t0"] + tr) & (tc <= L["t1"] - tr) & ok)[0]
        if len(idx):
            sel += list(idx[np.linspace(0, len(idx) - 1, FRAMES_PER_LEG).astype(int)])
    return sorted(set(int(i) for i in sel))


def layout_from_params(p, tag0):
    """p = [x1,y1,r1, x2,y2,r2, x3,y3,r3] in metres / radians. Tag 0 is the gauge."""
    h = SIZE_M / 2.0
    local = np.array([[-h, -h], [h, -h], [h, h], [-h, h]], float)
    P = {0: tag0}
    for j, tid in enumerate((1, 2, 3)):
        P[tid] = (p[3 * j], p[3 * j + 1], p[3 * j + 2])
    out = {}
    for tid in IDS:
        cx, cy, a = P[tid]
        c, s = math.cos(a), math.sin(a)
        q = (np.array([[c, -s], [s, c]]) @ local.T).T + np.array([cx, cy])
        out[tid] = np.column_stack([q, np.zeros(4)]).astype(float)
    return out


def bundle_adjust(cam, sel, K, init, tag0, label):
    """Run the bundle adjustment with intrinsic matrix K. Returns (params, report)."""
    FR = []
    for k in sel:
        v = [t for t in IDS if cam.at[k, f"tag_{t}_visible"] == 1]
        img = np.array([[cam.at[k, f"tag_{t}_c{c}_x_px"], cam.at[k, f"tag_{t}_c{c}_y_px"]]
                        for t in v for c in range(4)], float)
        FR.append((v, img))

    L0 = layout_from_params(init, tag0)
    poses = []
    for v, img in FR:
        O = np.vstack([L0[t] for t in v])
        ok, rv, tv = cv2.solvePnP(O, img, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
        poses += list(np.asarray(rv).ravel()) + list(np.asarray(tv).ravel())

    def residual(z):
        L = layout_from_params(z[:9], tag0)
        out = []
        for i, (v, img) in enumerate(FR):
            O = np.vstack([L[t] for t in v])
            pr, _ = cv2.projectPoints(O, z[9 + 6 * i:9 + 6 * i + 3],
                                      z[9 + 6 * i + 3:9 + 6 * i + 6], K, None)
            out.append((pr.reshape(-1, 2) - img).ravel())
        return np.concatenate(out)

    z0 = np.array(list(init) + poses, float)
    r0 = residual(z0)
    sol = least_squares(residual, z0, method="trf", x_scale="jac",
                        xtol=1e-14, ftol=1e-14, gtol=1e-14, max_nfev=MAX_NFEV)
    rms = float(np.sqrt(np.mean(sol.fun ** 2)))

    J = sol.jac
    dof = max(len(sol.fun) - len(sol.x), 1)
    s2 = float(sol.fun @ sol.fun) / dof
    try:
        JTJ = J.T @ J
        cov = np.linalg.pinv(JTJ.toarray() if hasattr(JTJ, "toarray") else JTJ) * s2
        sig = np.sqrt(np.clip(np.diag(cov)[:9], 0, None))
    except Exception:
        sig = np.full(9, np.nan)

    n4 = sum(1 for v, _ in FR if len(v) == 4)
    rep = dict(label=label, n_frames=len(FR), n_frames_4tag=n4,
               n_frames_3tag=len(FR) - n4,
               n_residuals=int(len(sol.fun)), n_parameters=int(len(sol.x)),
               rms_before_px=float(np.sqrt(np.mean(r0 ** 2))), rms_after_px=rms,
               optimizer="scipy.optimize.least_squares, trf, x_scale=jac",
               termination=dict(xtol=1e-14, ftol=1e-14, gtol=1e-14, max_nfev=MAX_NFEV,
                                status=int(sol.status), nfev=int(sol.nfev),
                                message=str(sol.message)),
               sigma_mm_deg={f"tag{t}": [float(sig[3 * j] * 1000),
                                         float(sig[3 * j + 1] * 1000),
                                         float(math.degrees(sig[3 * j + 2]))]
                             for j, t in enumerate((1, 2, 3))})
    return sol.x[:9], rep


def to_geometry(p, tag0, source, note, refit_report=None):
    cent = {0: np.array([tag0[0] * 1000.0, tag0[1] * 1000.0]),
            1: np.array([p[0], p[1]]) * 1000.0,
            2: np.array([p[3], p[4]]) * 1000.0,
            3: np.array([p[6], p[7]]) * 1000.0}
    rot = {0: math.degrees(tag0[2]) % 360.0,
           1: math.degrees(p[2]) % 360.0,
           2: math.degrees(p[5]) % 360.0,
           3: math.degrees(p[8]) % 360.0}
    origin = np.mean([cent[t] for t in IDS], axis=0)
    for t in IDS:
        cent[t] = cent[t] - origin
    g = {
        "geometry_source": source,
        "_comment": note,
        "_warning": ("Camera-derived board geometry for the RUN05 mechanical "
                     "configuration. NOT an independent dimensional reference and NOT "
                     "mechanically measured. Call it the configured board layout."),
        "_applies_to": ("RUN05 / Board-Y configuration ONLY. Never apply to Board-X or "
                        "yaw runs."),
        "board_origin": "geometric centre of the four tag centres",
        "tag_black_square_size_m": SIZE_M,
        "tags": {str(t): {"center_x_mm": float(cent[t][0]),
                          "center_y_mm": float(cent[t][1]),
                          "rotation_deg": float(rot[t])} for t in IDS},
    }
    if refit_report:
        g["refit_report"] = refit_report
    return g


def main():
    print("=" * 78)
    print("RUN05 BOARD-LAYOUT REFIT WITH THE CORRECT UNDISTORTED INTRINSIC MATRIX")
    print("=" * 78)

    cam, meta, cam_name = cio.load_camera(KEY)
    ev, _ = cio.load_motor(KEY)
    holds, legs, _, _, _, _ = mref.segment_profile(ev)
    fit = cms.fit_from_events(ev)
    tc = cms.camera_times_to_motor_clock(cam, fit)

    K_raw = np.array(meta["camera_matrix"], float)
    D = np.array(meta["dist_coeffs"], float)
    w, h = int(meta["width"]), int(meta["height"])
    K_new, _ = cv2.getOptimalNewCameraMatrix(K_raw, D, (w, h), 1, (w, h))
    print(f"\n  raw camera_matrix fx = {K_raw[0,0]:.4f}   "
          f"undistorted newK fx = {K_new[0,0]:.4f}   "
          f"({100*(K_raw[0,0]/K_new[0,0]-1):+.2f} %)")

    # initial geometry: the Board-X layout, as the original fit used
    gx = cio.load_json(cio.config_path("board_geometry_boardX.json"))
    tag0 = (gx["tags"]["0"]["center_x_mm"] / 1000.0,
            gx["tags"]["0"]["center_y_mm"] / 1000.0,
            math.radians(gx["tags"]["0"]["rotation_deg"]))
    init = []
    for t in (1, 2, 3):
        q = gx["tags"][str(t)]
        init += [q["center_x_mm"] / 1000.0, q["center_y_mm"] / 1000.0,
                 math.radians(q["rotation_deg"])]
    print(f"  initial geometry        : board_geometry_boardX.json "
          f"[{gx['geometry_source']}]")
    print(f"  tag size (held fixed)   : {SIZE_M:.3f} m")
    print(f"  gauge                   : tag 0 fixed, rotation_deg = 0; centroid "
          f"re-centred after convergence")

    sel = select_frames(cam, tc, holds, legs)
    print(f"\n  frame selection: {FRAMES_PER_HOLD}/hold x {len(holds)} holds + "
          f"{FRAMES_PER_LEG}/leg x {len(legs)} legs -> {len(sel)} frames")

    stored = cio.load_json(cio.config_path("board_geometry_boardY_bundle_adjusted.json"))

    results = {}
    for label, K in (("raw camera_matrix", K_raw), ("undistorted newK", K_new)):
        print(f"\n--- bundle adjustment with the {label} ---")
        p, rep = bundle_adjust(cam, sel, K, init, tag0, label)
        g = to_geometry(p, tag0, "tmp", "tmp")
        rep["rms_px"] = rep["rms_after_px"]
        print(f"    frames {rep['n_frames']} ({rep['n_frames_4tag']} four-tag, "
              f"{rep['n_frames_3tag']} three-tag), residuals {rep['n_residuals']}, "
              f"parameters {rep['n_parameters']}")
        print(f"    RMS reprojection {rep['rms_before_px']:.4f} -> "
              f"{rep['rms_after_px']:.4f} px   ({rep['termination']['nfev']} evaluations, "
              f"status {rep['termination']['status']})")
        d = {t: math.hypot(g["tags"][str(t)]["center_x_mm"]
                           - stored["tags"][str(t)]["center_x_mm"],
                           g["tags"][str(t)]["center_y_mm"]
                           - stored["tags"][str(t)]["center_y_mm"]) for t in IDS}
        dr = {t: abs(g["tags"][str(t)]["rotation_deg"]
                     - stored["tags"][str(t)]["rotation_deg"]) for t in IDS}
        print(f"    vs STORED geometry: max centre shift {max(d.values()):.4f} mm, "
              f"max rotation shift {max(dr.values()):.4f} deg")
        results[label] = dict(params=p.tolist(), report=rep, geometry=g,
                              centre_shift_mm=d, rotation_shift_deg=dr,
                              max_centre_shift_mm=float(max(d.values())),
                              max_rotation_shift_deg=float(max(dr.values())))

    # ---- which matrix produced the stored file? ----------------------------
    raw_d = results["raw camera_matrix"]["max_centre_shift_mm"]
    new_d = results["undistorted newK"]["max_centre_shift_mm"]
    produced_with = ("raw camera_matrix" if raw_d < new_d else "undistorted newK")
    print("\n" + "=" * 78)
    print("WHICH INTRINSIC MATRIX PRODUCED THE STORED GEOMETRY?")
    print("=" * 78)
    print(f"  refit with raw camera_matrix reproduces it to {raw_d:.4f} mm")
    print(f"  refit with undistorted newK  reproduces it to {new_d:.4f} mm")
    print(f"  -> the stored file was produced with the {produced_with.upper()}")
    print(f"  (documentary confirmation: the original exporter read "
          f"meta['camera_matrix'] and never called getOptimalNewCameraMatrix)")

    # ---- Case A or Case B? --------------------------------------------------
    corr = results["undistorted newK"]
    material = (corr["max_centre_shift_mm"] > CENTRE_TOL_MM or
                corr["max_rotation_shift_deg"] > ROT_TOL_DEG)
    print("\n" + "=" * 78)
    print("DOES THE CORRECTED REFIT CHANGE THE LAYOUT MATERIALLY?")
    print("=" * 78)
    print(f"  {'tag':>4} {'stored x_mm':>13} {'refit x_mm':>13} "
          f"{'stored y_mm':>13} {'refit y_mm':>13} {'shift_mm':>10} {'d_rot_deg':>10}")
    for t in IDS:
        s = stored["tags"][str(t)]
        r = corr["geometry"]["tags"][str(t)]
        print(f"  {t:>4} {s['center_x_mm']:13.6f} {r['center_x_mm']:13.6f} "
              f"{s['center_y_mm']:13.6f} {r['center_y_mm']:13.6f} "
              f"{corr['centre_shift_mm'][t]:10.5f} {corr['rotation_shift_deg'][t]:10.5f}")
    print(f"\n  materiality thresholds: centre {CENTRE_TOL_MM} mm, "
          f"rotation {ROT_TOL_DEG} deg")
    print(f"  max centre shift {corr['max_centre_shift_mm']:.5f} mm, "
          f"max rotation shift {corr['max_rotation_shift_deg']:.5f} deg")
    print(f"\n  -> CASE {'B: MATERIAL CHANGE' if material else 'A: NO MATERIAL CHANGE'}")

    out = dict(
        run=KEY, camera_file=cam_name,
        raw_camera_matrix_fx=float(K_raw[0, 0]),
        undistorted_newK_fx=float(K_new[0, 0]),
        focal_length_difference_pct=float(100 * (K_raw[0, 0] / K_new[0, 0] - 1)),
        stored_geometry_produced_with=produced_with,
        documentary_evidence=("The original exporter (export_run05_geometry.py) read "
                              "meta['camera_matrix'] and never called "
                              "cv2.getOptimalNewCameraMatrix."),
        frame_selection_rule=(f"{FRAMES_PER_HOLD} frames per static hold, evenly spaced by "
                              f"np.linspace over the window trimmed {HOLD_GUARD_S} s at "
                              f"each end; {FRAMES_PER_LEG} frames per rate leg, evenly "
                              f"spaced over the central "
                              f"{100*(1-2*LEG_TRIM_FRAC):.0f} % of the leg; a frame is "
                              f"kept only if >= 3 expected tags are visible and "
                              f"pose_ok == 1"),
        n_frames_selected=len(sel),
        gauge_constraints=("tag 0 fixed in position and at rotation_deg = 0 during the "
                           "fit; centroid of the four centres re-centred to the origin "
                           "afterwards (pure translation, absorbed by the pose)"),
        fixed_tag_size_m=SIZE_M,
        initial_geometry="config/board_geometry_boardX.json",
        materiality_thresholds=dict(centre_mm=CENTRE_TOL_MM, rotation_deg=ROT_TOL_DEG),
        case=("B" if material else "A"),
        conclusion=("Corrected refit changes the layout materially; the new geometry is "
                    "saved and Board-Y processing must be rerun."
                    if material else
                    "Corrected refit agrees with the stored geometry within the "
                    "materiality thresholds. The stored geometry is KEPT unchanged; "
                    "no Board-Y number is affected."),
        fits={k: dict(report=v["report"], geometry=v["geometry"],
                      centre_shift_mm={str(a): b for a, b in v["centre_shift_mm"].items()},
                      rotation_shift_deg={str(a): b for a, b in
                                          v["rotation_shift_deg"].items()},
                      max_centre_shift_mm=v["max_centre_shift_mm"],
                      max_rotation_shift_deg=v["max_rotation_shift_deg"])
              for k, v in results.items()})

    if material:
        g = to_geometry(np.array(corr["params"]), tag0,
                        "camera_derived_bundle_adjustment_RUN05_newK",
                        "Refitted with the undistorted intrinsic matrix newK.",
                        refit_report=corr["report"])
        p = cio.config_path("board_geometry_boardY_bundle_adjusted_newK.json")
        json.dump(g, open(p, "w"), indent=2)
        print(f"\n  wrote {os.path.relpath(p, cio.REPO_ROOT)}")
        out["corrected_geometry_file"] = os.path.basename(p)

    p = os.path.join(cio.RESULTS, "board_y_geometry_refit.json")
    os.makedirs(cio.RESULTS, exist_ok=True)
    json.dump(out, open(p, "w"), indent=2, default=float)
    print(f"  -> {os.path.relpath(p, cio.REPO_ROOT)}")
    return out


if __name__ == "__main__":
    main()
