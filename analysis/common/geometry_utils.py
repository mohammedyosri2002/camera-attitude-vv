"""
analysis/common/geometry_utils.py
=================================
Configured board layout, rigid multi-tag IPPE pose, and the IMU references.
CREATED POST-CAMPAIGN.

GEOMETRY PROVENANCE -- read before using any number from this module
--------------------------------------------------------------------
Both board layout files in config/ are CAMERA-DERIVED:

  board_geometry_boardX.json                  camera_derived_planar_homography
  board_geometry_boardY_bundle_adjusted.json   camera_derived_bundle_adjustment_RUN05

They are ACQUISITION / REPROCESSING GEOMETRY MODELS. They are NOT independent dimensional
ground truth and were NOT mechanically measured. Call them the "configured board layout"
or "camera-derived board geometry" -- never a "measured board layout".

The board was physically re-mounted relative to the hinge between the Board-X and Board-Y
configurations, so the two layouts describe different physical boards. Applying the RUN05
layout to a Board-X run, or vice versa, produces a plausible-looking wrong answer:
with the stale layout RUN05 reprojects at 1.21 px instead of 0.37 px and under-reads a
20 deg command by 3.5 deg. select_geometry() enforces the correct pairing.
"""
from __future__ import annotations
import json
import math
import os

import cv2
import numpy as np

from . import io as cio

EXPECTED_TAG_IDS = [0, 1, 2, 3]
AMBIGUITY_RATIO = 1.5          # matches the production acquisition code
REF_MIN_FRAMES = 200           # frames used to establish the relative zero
BRANCH_FLIP_DEG = 20.0


def select_geometry(key: str):
    """Return (geometry dict, filename). The pairing comes from RUN_INFO.json."""
    info = cio.run_info(key)
    name = info["geometry_file"]
    g = cio.load_json(cio.config_path(name))
    if info["canonical_axis"] == "board_y" and "boardY" not in name:
        raise RuntimeError(f"{key} is board_y but the geometry file is {name}")
    if info["canonical_axis"] in ("board_x", "yaw") and "boardY" in name:
        raise RuntimeError(f"{key} is {info['canonical_axis']}; the Board-Y geometry must "
                           f"never be applied to it")
    if "SUPERSEDED" in name or "SUPERSEDED" in str(g.get("geometry_source", "")):
        raise RuntimeError(f"{key} points at a SUPERSEDED geometry file: {name}")
    return g, name


def build_object_points(geom):
    """Board object points, one 4x3 block per tag, in the detector's corner order.

    local corners (un-rotated): (-h,-h), (+h,-h), (+h,+h), (-h,+h), h = size/2
    p = Rz(rotation_deg) @ local + (center_x_mm, center_y_mm)/1000 ; z = 0
    This matches build_object_points() in the production acquisition code exactly.
    """
    h = float(geom["tag_black_square_size_m"]) / 2.0
    local = np.array([[-h, -h], [h, -h], [h, h], [-h, h]], float)
    out = {}
    for tid in EXPECTED_TAG_IDS:
        t = geom["tags"][str(tid)]
        a = math.radians(float(t["rotation_deg"]))
        c, s = math.cos(a), math.sin(a)
        p = (np.array([[c, -s], [s, c]]) @ local.T).T + \
            np.array([float(t["center_x_mm"]) / 1000.0, float(t["center_y_mm"]) / 1000.0])
        out[tid] = np.column_stack([p, np.zeros(4)]).astype(float)
    return out


def chordal_mean_rotation(Rs):
    M = sum(Rs) / len(Rs)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1.0
        R = U @ Vt
    return R


def geodesic_deg(Ra, Rb):
    return float(np.degrees(np.linalg.norm(cv2.Rodrigues(Ra.T @ Rb)[0])))


def solve_board_sequence(cam, obj, K, progress=None):
    """Re-solve the rigid multi-tag board pose for every frame from STORED RAW CORNER
    PIXELS, using the production candidate-selection logic:

      * solvePnPGeneric with SOLVEPNP_IPPE returns both planar solutions
      * candidates with non-positive depth are rejected as physically invalid
      * both reprojection errors are retained
      * the lower-error candidate is taken UNLESS the second is within AMBIGUITY_RATIO,
        in which case temporal continuity selects the candidate closer to the previous pose
      * jumps larger than BRANCH_FLIP_DEG are counted as branch flips

    Stored pose columns in the camera CSV are ignored: they were produced at acquisition
    time, and for RUN05 they were produced with the wrong board layout.
    """
    n = len(cam)
    vis = np.column_stack([cam[f"tag_{t}_visible"].to_numpy(int) for t in EXPECTED_TAG_IDS])
    CX = {t: np.column_stack([cam[f"tag_{t}_c{c}_x_px"].to_numpy(float) for c in range(4)])
          for t in EXPECTED_TAG_IDS}
    CY = {t: np.column_stack([cam[f"tag_{t}_c{c}_y_px"].to_numpy(float) for c in range(4)])
          for t in EXPECTED_TAG_IDS}

    R_all = np.zeros((n, 3, 3))
    ok = np.zeros(n, bool)
    reproj = np.full(n, np.nan)
    ratio = np.full(n, np.nan)
    sep = np.full(n, np.nan)
    amb = np.zeros(n, bool)
    R_prev = None
    flips = 0

    for k in range(n):
        v = [t for t in EXPECTED_TAG_IDS if vis[k, EXPECTED_TAG_IDS.index(t)] == 1]
        if len(v) < 3:
            continue
        img = np.vstack([np.column_stack([CX[t][k], CY[t][k]]) for t in v])
        O = np.vstack([obj[t] for t in v])
        good, rvs, tvs, err = cv2.solvePnPGeneric(O, img, K, None,
                                                  flags=cv2.SOLVEPNP_IPPE)
        if not good:
            continue
        ee = np.asarray(err).ravel()
        cands = []
        for j in range(len(rvs)):
            Rm, _ = cv2.Rodrigues(rvs[j])
            if float(np.asarray(tvs[j]).ravel()[2]) > 0:     # positive-depth check
                cands.append((float(ee[j]), Rm))
        if not cands:
            continue
        cands.sort(key=lambda z: z[0])
        best = cands[0]
        if len(cands) > 1:
            ratio[k] = cands[1][0] / max(cands[0][0], 1e-12)
            sep[k] = geodesic_deg(cands[0][1], cands[1][1])
            if cands[1][0] < AMBIGUITY_RATIO * cands[0][0]:
                amb[k] = True
                if R_prev is not None:
                    best = min(cands[:2],
                               key=lambda z: geodesic_deg(R_prev, z[1]))
        if R_prev is not None and geodesic_deg(R_prev, best[1]) > BRANCH_FLIP_DEG:
            flips += 1
        R_prev = best[1]
        R_all[k] = best[1]
        reproj[k] = best[0]
        ok[k] = True
        if progress and k % 10000 == 0:
            progress(k, n)

    return dict(R=R_all, ok=ok, reproj=reproj, ratio=ratio, sep=sep, ambiguous=amb,
                n_flips=int(flips), n_visible=vis.sum(1))


def relative_attitude(R_all, ok, t, baseline_window):
    """R_rel = R_ref^T R_board, replicating the ACQUISITION reference construction.

    From the acquisition code:

        REF_MIN_FRAMES = 200
        if R_ref is None:
            ref_pool.append(R)
            if len(ref_pool) >= REF_MIN_FRAMES:
                R_ref = chordal_mean_rotation(ref_pool)

    The pool is the FIRST REF_MIN_FRAMES VALID-POSE FRAMES OF THE CAPTURE, in capture
    order, with no reference to motor time. The camera starts before the motor, so these
    frames precede the motor baseline window. Reproducing that construction exactly is what
    makes offline re-solution match the instrument: on Board X, newK plus this reference
    reproduces the logged acquisition attitude to 0.000000 deg.

    Using the motor baseline window instead shifts the Board-X static RMSE by about
    0.004 deg. The `baseline_window` argument is retained for the diagnostic returned
    alongside, not used to select the pool.

    Never a frame-by-frame subtraction of absolute Euler angles. A constant camera mounting
    orientation cancels exactly.
    """
    idx = np.where(ok)[0][:REF_MIN_FRAMES]
    if len(idx) < 10:
        raise RuntimeError("too few valid frames to establish the relative zero")
    R_ref = chordal_mean_rotation([R_all[i] for i in idx])
    scatter = float(np.mean([geodesic_deg(R_ref, R_all[i]) for i in idx]))
    rvec = np.full((len(R_all), 3), np.nan)
    for k in np.where(ok)[0]:
        rvec[k] = cv2.Rodrigues(R_ref.T @ R_all[k])[0].ravel()
    return R_ref, rvec, len(idx), scatter


def calibrated_axis(rvec, t, ok, holds, guard_s):
    """Hinge axis in the BOARD frame: principal direction of the per-hold mean rotation
    vectors. Estimated from camera data alone -- the commanded reference supplies only the
    sign, never a scale."""
    M = np.array([np.nanmean(rvec[(t >= h["t0"] + guard_s) &
                                  (t <= h["t1"] - guard_s) & ok], 0) for h in holds])
    U, S, Vt = np.linalg.svd(M, full_matrices=False)
    n = Vt[0] / np.linalg.norm(Vt[0])
    mid = len(holds) // 2
    if M[min(4, mid)] @ n < 0:
        n = -n
    off = float(np.degrees(np.sqrt(np.mean(np.sum((M - np.outer(M @ n, n)) ** 2, 1)))))
    return n, float(S[0] / S[1]), off


def board_tilt_from_fronto_parallel(R_all, mask, step=100):
    """Angle between the board normal and the camera optical axis."""
    out = []
    for k in np.where(mask)[0][::step]:
        out.append(math.degrees(math.acos(min(abs(R_all[k][2, 2]), 1.0))))
    return np.array(out)


# ---------------------------------------------------------------------------------
# IMU references
# ---------------------------------------------------------------------------------
def gyro_reference(imu, ti, legs, baselines, trim=0.15):
    """RAW bias-corrected gyro projected onto its own rotation axis.

    The axis is the principal direction of the gyro samples during the commanded legs --
    camera-free and motor-free. Only a stationary bias from THIS run's own baselines is
    subtracted. NO SCALE FACTOR IS FITTED, to the motor or to the camera; the datasheet
    sensitivity tolerance therefore remains in the error budget.
    """
    G = imu[["gyro_x_dps", "gyro_y_dps", "gyro_z_dps"]].to_numpy(float)
    bm = np.zeros(len(ti), bool)
    for w in baselines.values():
        if w:
            bm |= (ti >= w[0] + 5) & (ti <= w[1] - 5)
    bias = G[bm].mean(0)
    noise = G[bm].std(0, ddof=1)
    Gc = G - bias

    lm = np.zeros(len(ti), bool)
    pl = np.zeros(len(ti), bool)
    for L in legs:
        tr = trim * (L["t1"] - L["t0"])
        m = (ti >= L["t0"] + tr) & (ti <= L["t1"] - tr)
        lm |= m
        if L["rate_dps"] > 0:
            pl |= m
    U, S, Vt = np.linalg.svd(Gc[lm], full_matrices=False)
    mhat = Vt[0] / np.linalg.norm(Vt[0])
    if np.mean(Gc[pl] @ mhat) < 0:
        mhat = -mhat
    return Gc @ mhat, dict(bias_dps=bias.tolist(), noise_dps=noise.tolist(),
                           axis=mhat.tolist(), dominance=float(S[0] / S[1]),
                           n_baseline_samples=int(bm.sum()))


def gravity_tilt(imu, ti, holds, baselines, guard_s):
    """Signed gravity-derived tilt from RAW accelerometer samples during stationary holds.

    The tilt is the angle between the measured gravity DIRECTION and a reference gravity
    direction from this run's own stationary baseline, signed by projection onto an axis
    derived from the gravity motion itself. Because both vectors are normalised before the
    angle is taken, a common gain error largely cancels; the residual sensitivity is to
    per-axis bias and cross-axis terms. The unit is uncalibrated, so this reference carries
    an uncorrected error of that kind -- see docs/LIMITATIONS.md. Do not infer a tilt-angle
    error directly from the accelerometer magnitude at rest.
    """
    A = imu[["accel_x_g", "accel_y_g", "accel_z_g"]].to_numpy(float)
    nrm = np.linalg.norm(A, axis=1)
    Ua = A / np.maximum(nrm, 1e-12)[:, None]

    w = baselines["start"]
    bm = (ti >= w[0] + 5) & (ti <= w[1] - 5)
    gref = A[bm].mean(0)
    gref /= np.linalg.norm(gref)

    hm = np.zeros(len(ti), bool)
    for h in holds:
        hm |= (ti >= h["t0"] + guard_s) & (ti <= h["t1"] - guard_s)
    cr = np.cross(np.broadcast_to(gref, Ua.shape), Ua)
    _, _, V2 = np.linalg.svd(cr[hm], full_matrices=False)
    k = V2[0] / np.linalg.norm(V2[0])

    sg = np.sign(cr @ k)
    tilt = np.degrees(np.arccos(np.clip(Ua @ gref, -1, 1))) * np.where(sg == 0, 1.0, sg)

    pos = [h for h in holds if h["target_deg"] > 5]
    if pos and np.nanmean([np.nanmean(tilt[(ti >= h["t0"] + guard_s) &
                                           (ti <= h["t1"] - guard_s)])
                           for h in pos]) < 0:
        tilt = -tilt
        k = -k
    return tilt, dict(gravity_reference=gref.tolist(), rotation_axis=k.tolist(),
                      accel_magnitude_at_rest_g=float(nrm[bm].mean()),
                      axis_to_gravity_deg=float(
                          math.degrees(math.acos(min(abs(float(k @ gref)), 1.0)))))
