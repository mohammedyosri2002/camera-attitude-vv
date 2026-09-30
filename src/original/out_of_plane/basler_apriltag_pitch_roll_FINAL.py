#!/usr/bin/env python3
# =============================================================================
# basler_apriltag_pitch_roll_FINAL.py
#
# Rigid multi-tag planar-board attitude acquisition for the pitch/roll
# campaign. Basler monocular camera + four tag36h11 fiducials treated as ONE
# rigid board, solved with a generic planar PnP (IPPE) over all simultaneously
# visible corners.
#
#   # 1. one-off, after you measure the board (or to check your measurements)
#   python3 basler_apriltag_pitch_roll_FINAL.py --mode calibrate-board \
#       --duration 60 --geometry board_geometry.json
#
#   # 2. mandatory stationary validation before every campaign
#   python3 basler_apriltag_pitch_roll_FINAL.py --mode validate --duration 60
#
#   # 3. production
#   python3 basler_apriltag_pitch_roll_FINAL.py --mode acquire \
#       --run 1 --axis AXIS_A --duration 560 --out runs_pr
#
# -----------------------------------------------------------------------------
# WHY THIS IS NOT THE YAW CAMERA SCRIPT
# -----------------------------------------------------------------------------
# The yaw campaign solved each 70 mm tag independently and reported per-tag
# Euler angles. In-plane yaw was excellent (0.02-0.03 deg 1-sigma) but the
# out-of-plane angles from the same frames scattered by 1-5 deg with 10-25 deg
# excursions even while the platform was provably stationary. That is the
# classic planar-tag pose ambiguity: a single 70 mm square viewed near
# fronto-parallel at 1.35 m has two nearly equally good pose solutions.
#
# Three changes fix it, and all three are implemented here:
#   (a) RIGID BOARD. All visible corners are solved as one object with a known
#       fixed geometry. A synthetic test at 1.35 m with 0.15 px corner noise
#       gives 0.135 deg per-frame out-of-plane SD for the 4-tag board versus
#       0.677 deg for a single tag - a 5x improvement from geometry alone.
#   (b) EXPLICIT CANDIDATE HANDLING. solvePnPGeneric with SOLVEPNP_IPPE returns
#       both planar solutions. Negative-depth solutions are rejected, both
#       reprojection errors are logged, and the lower-error branch is taken
#       UNLESS the two are within AMBIGUITY_RATIO of each other, in which case
#       temporal continuity decides. Continuity alone is not enough: in the
#       synthetic test, continuity-first tracking followed the mirror branch
#       through a sign reversal and reported +21.6 deg for a true -20 deg.
#       Reprojection-gated continuity gave <= 0.11 deg over the whole +/-20 deg
#       sweep.
#   (c) RELATIVE ROTATION MATRICES. Attitude is reported as
#       R_rel = R_ref^T * R_board, never as a frame-by-frame subtraction of
#       absolute Euler angles.
#
# SOLVEPNP_IPPE_SQUARE is used ONLY for the optional per-tag diagnostic and for
# the board-geometry bootstrap, never for the multi-tag board, where it would
# be a misuse.
#
# -----------------------------------------------------------------------------
# CONDITIONING WARNING THAT CAME OUT OF THE SYNTHETIC TESTS
# -----------------------------------------------------------------------------
# With the board fronto-parallel at the zero reference, the two IPPE branches
# are nearly degenerate: baseline-orientation error 0.66 deg, per-frame SD at
# zero 1.21 deg, and 154/1020 frames ambiguous. With a deliberate ~15 deg
# standoff tilt about the NON-active axis, the same test gave baseline error
# 0.01 deg, SD at zero 0.13 deg and 2/1020 ambiguous frames.
# See README_PITCH_ROLL.md. The validate mode below will tell you which
# regime you are actually in.
# =============================================================================

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import pandas as pd

# pypylon and pupil_apriltags are imported lazily inside open_camera() and
# make_detector(). Everything above the hardware layer -- geometry loading,
# object-point construction, the board solver, the corner-order self-check and
# the rotation utilities -- is therefore importable on a machine with no
# camera and no AprilTag library, which is what lets
# test_pitch_roll_pose_solver.py exercise the REAL production functions
# instead of a copy of them.
pylon = None
Detector = None


def _require_hardware():
    """Import the camera and detector libraries on first use."""
    global pylon, Detector
    if pylon is None:
        from pypylon import pylon as _pylon
        pylon = _pylon
    if Detector is None:
        from pupil_apriltags import Detector as _Detector

        # WORKAROUND:
        # pupil_apriltags Detector.__del__ crashes inside
        # apriltag_detector_destroy / quick_decode_uninit on this
        # Python 3.14 environment when the Detector goes out of scope.
        #
        # One Detector is created per process.  We intentionally skip its
        # native destructor; the operating system reclaims the process memory
        # on exit.  This does NOT change detection, pose estimation, timing,
        # or any recorded data.
        if hasattr(_Detector, "__del__"):
            _Detector.__del__ = lambda self: None

        Detector = _Detector


def make_detector():
    _require_hardware()
    return Detector(families=TAG_FAMILY, nthreads=4, quad_decimate=1.0,
                    quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25,
                    debug=0)


# =============================================================================
# FIXED ACQUISITION CONFIGURATION
# Carried over unchanged from the validated yaw acquisition.
# =============================================================================
TAG_FAMILY = "tag36h11"
DEFAULT_TAG_SIZE_M = 0.070   # yaw-campaign value; override with --tag-size
EXPOSURE_US = 5000.0
GAIN_DB = 0.0
EXPECTED_TAG_IDS = [0, 1, 2, 3]

SAVE_VIDEO = False          # production: never
DRAW_OVERLAYS = False       # production: never
USE_ONLINE_FILTER = False   # permanently off; all filtering is offline

CAMERA_MATRIX = np.array([
    [1443.2496917062335, 0.0, 963.2151010468531],
    [0.0, 1443.737705496836, 580.3332937596235],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

DIST_COEFFS = np.array([
    -0.1575447, 0.16190211, -0.00044674, -0.00022088, -0.11840788
], dtype=np.float64)

# ---- board solver ----------------------------------------------------------
# Two IPPE branches are treated as ambiguous when the worse reprojection error
# is within this factor of the better one. Only then does temporal continuity
# decide. 1.5 was chosen from the synthetic sweep, where the true branch beat
# the mirror by >= 6x at |tilt| >= 5 deg and by 2.2x at 0 deg.
AMBIGUITY_RATIO = 1.5
MAX_BOARD_REPROJ_PX = 3.0       # frames above this are flagged, never silently dropped
STATUS_PERIOD_S = 1.0
REF_MIN_FRAMES = 200            # frames required to establish the zero reference

# ---- validate-mode GO criteria --------------------------------------------
GO_SD_DEG = 0.20
GO_PTP_DEG = 1.00
GO_MAX_FLIPS = 0

# [FIX] CONDITIONING CRITERIA -- added after the first AXIS_A run.
# That configuration passed the old validator (stationary SD 0.040 deg, zero
# branch flips) and then produced camera errors up to 3.5 deg, because the
# board passed within 2.5 deg of fronto-parallel where the two planar IPPE
# branches collapse to 7.4 deg apart. Stationary scatter is blind to this:
# a biased pose is perfectly steady. These four checks test CONDITIONING
# rather than repeatability, using columns already logged every frame.
#
# Thresholds from the first AXIS_A dataset:
#   separation 33.6 deg -> ~0.05 deg residual        (good)
#   separation 25.3 deg -> ~0.27 deg residual        (marginal)
#   separation 19.5 deg -> ~1.0  deg reference bias  (bad, and it PASSED before)
#   separation  7.4 deg -> 3.5   deg error           (catastrophic)
# The reference pose sets the zero for every later measurement, so it is held
# to a stricter standard than the sweep.
GO_MIN_CAND_SEP_DEG = 50.0      # median candidate_separation_deg
GO_MIN_REPROJ_RATIO = 5.0       # median candidate_other_reproj_px / board_reproj_rms_px
GO_MAX_AMBIGUOUS = 0            # pose_ambiguous frames
GO_MIN_TAG_VIS_PCT = 100.0      # per-tag tag_<id>_visible
GO_MAX_REPROJ_PX = 1.0          # median board_reproj_rms_px


# =============================================================================
# BOARD GEOMETRY
# =============================================================================
GEOMETRY_TEMPLATE = {
    "_comment": "Rigid four-tag board geometry. MEASURE these; do not guess.",
    "board_origin": "geometric centre of the four tag centres",
    "tag_black_square_size_m": 0.07,
    "tags": {
        "0": {"center_x_mm": None, "center_y_mm": None, "rotation_deg": None},
        "1": {"center_x_mm": None, "center_y_mm": None, "rotation_deg": None},
        "2": {"center_x_mm": None, "center_y_mm": None, "rotation_deg": None},
        "3": {"center_x_mm": None, "center_y_mm": None, "rotation_deg": None}
    }
}


def geometry_help():
    return (
        "\nBOARD GEOMETRY IS MISSING OR INCOMPLETE.\n"
        "This script will not run without it, and it must not be guessed.\n\n"
        "Fill in board_geometry.json:\n\n"
        "  Tag ID | Center X mm | Center Y mm | Rotation deg\n"
        "  -------+-------------+-------------+-------------\n"
        "     0   |      ?      |      ?      |      ?\n"
        "     1   |      ?      |      ?      |      ?\n"
        "     2   |      ?      |      ?      |      ?\n"
        "     3   |      ?      |      ?      |      ?\n\n"
        "  plus tag_black_square_size_m (the yaw campaign used 0.070 m;\n"
        "  measure the printed black square edge and confirm).\n\n"
        "HOW TO MEASURE (see README_PITCH_ROLL.md for the full procedure):\n"
        "  X = 0, Y = 0 is the geometric centre of the four tag centres.\n"
        "  +X points to the RIGHT and +Y points UP as the OVERHEAD CAMERA\n"
        "  sees the board with the platform at its established zero.\n"
        "  Rotation is measured counter-clockwise about +Z (out of the board,\n"
        "  toward the camera); 0 deg means the tag is printed in the same\n"
        "  orientation as tag 0.\n\n"
        "Or run the bootstrap and then CHECK it against your ruler:\n"
        "  python3 basler_apriltag_pitch_roll_FINAL.py --mode calibrate-board \\\n"
        "      --duration 60 --geometry board_geometry.json\n")


def load_geometry(path):
    if not os.path.exists(path):
        with open(path, "w") as f:
            json.dump(GEOMETRY_TEMPLATE, f, indent=2)
        print(f"Wrote a blank template to {path}")
        sys.exit(geometry_help())

    g = json.load(open(path))
    size = g.get("tag_black_square_size_m")
    tags = g.get("tags", {})
    missing = []
    if not isinstance(size, (int, float)) or size <= 0:
        missing.append("tag_black_square_size_m")
    for tid in EXPECTED_TAG_IDS:
        t = tags.get(str(tid), {})
        for k in ("center_x_mm", "center_y_mm", "rotation_deg"):
            if not isinstance(t.get(k), (int, float)):
                missing.append(f"tag {tid} {k}")
    if missing:
        print("Missing geometry fields: " + ", ".join(missing))
        sys.exit(geometry_help())
    return g


def build_object_points(geom):
    """Board object points in metres, one 4x3 block per tag, in ascending tag
    id and in the pupil_apriltags corner order.

    pupil_apriltags returns corners as
        [bottom-left, bottom-right, top-right, top-left]
    in the tag's own frame, so the un-rotated local corners are
        (-h,-h), (+h,-h), (+h,+h), (-h,+h)
    with h = half the black-square edge. Each tag is then rotated in the board
    plane by its printed rotation and translated to its measured centre.
    Board Z is 0 everywhere: the four tags are coplanar by construction.
    """
    h = float(geom["tag_black_square_size_m"]) / 2.0
    # pupil_apriltags / AprilTag tag coordinates:
    # p0 = bottom-left, p1 = bottom-right,
    # p2 = top-right,    p3 = top-left.
    # Tag frame uses +X right and +Y down.
    local = np.array([
        [-h, -h],
        [ h, -h],
        [ h,  h],
        [-h,  h]
    ], dtype=np.float64)
    obj = {}
    for tid in EXPECTED_TAG_IDS:
        t = geom["tags"][str(tid)]
        a = math.radians(float(t["rotation_deg"]))
        c, s = math.cos(a), math.sin(a)
        Rz = np.array([[c, -s], [s, c]], dtype=np.float64)
        p = (Rz @ local.T).T + np.array([float(t["center_x_mm"]) / 1000.0,
                                         float(t["center_y_mm"]) / 1000.0])
        obj[tid] = np.column_stack([p, np.zeros(4)]).astype(np.float64)
    return obj


# =============================================================================
# ROTATION UTILITIES
# =============================================================================
def euler_xyz_deg(R):
    """Intrinsic X-Y-Z (roll about board X, pitch about board Y, yaw about
    board Z), degrees. Same convention as the yaw campaign so the two are
    directly comparable. Near |pitch| = 90 deg this parameterisation is
    singular; the exported rotation vector and quaternion are not, which is
    why both are logged."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def rot_to_quat(R):
    """w, x, y, z with w >= 0."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    return w, x, y, z


def chordal_mean_rotation(Rs):
    """Mean of a set of rotations by SVD projection of the arithmetic mean.
    Used only to establish the stationary zero reference."""
    M = np.zeros((3, 3))
    for R in Rs:
        M += R
    M /= len(Rs)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1.0
        R = U @ Vt
    return R


def geodesic_deg(Ra, Rb):
    return float(np.degrees(np.linalg.norm(cv2.Rodrigues(Ra.T @ Rb)[0])))


# =============================================================================
# BOARD POSE SOLVER
# =============================================================================
def solve_board(obj_pts, img_pts, K, R_prev):
    """Rigid planar board pose from all simultaneously visible corners.

    Returns a dict with the selected pose, BOTH candidate reprojection errors,
    the ambiguity decision and the rejection diagnostics. Nothing is hidden:
    when the branch decision is close, the frame says so.
    """
    out = dict(n_points=len(img_pts), ok=False, R=None, t=None,
               reproj_px=np.nan, reproj_other_px=np.nan,
               n_candidates=0, n_rejected_depth=0,
               ambiguous=False, selected_by="none",
               candidate_geodesic_deg=np.nan)
    if len(img_pts) < 4:
        return out

    ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(
        obj_pts, img_pts, K, None, flags=cv2.SOLVEPNP_IPPE)
    if not ok or len(rvecs) == 0:
        return out

    cands = []
    for j in range(len(rvecs)):
        R, _ = cv2.Rodrigues(rvecs[j])
        t = np.asarray(tvecs[j], dtype=np.float64).ravel()
        e = float(np.asarray(errs).ravel()[j])
        if t[2] <= 0.0:                      # physically invalid: behind camera
            out["n_rejected_depth"] += 1
            continue
        cands.append([e, R, t])
    out["n_candidates"] = len(cands)
    if not cands:
        return out

    cands.sort(key=lambda c: c[0])
    best, other = cands[0], (cands[1] if len(cands) > 1 else None)
    out["reproj_px"] = best[0]
    if other is not None:
        out["reproj_other_px"] = other[0]
        out["candidate_geodesic_deg"] = geodesic_deg(best[1], other[1])
        if other[0] < AMBIGUITY_RATIO * best[0]:
            out["ambiguous"] = True
            if R_prev is not None:
                # reprojection cannot separate them: let temporal continuity decide
                pick = min(cands[:2], key=lambda c: geodesic_deg(R_prev, c[1]))
                best = pick
                out["selected_by"] = "continuity"
                out["reproj_px"] = best[0]
                out["reproj_other_px"] = (other[0] if best is cands[0]
                                          else cands[0][0])
            else:
                out["selected_by"] = "min_reproj_no_prior"
        else:
            out["selected_by"] = "min_reproj"
    else:
        out["selected_by"] = "single_candidate"

    out["ok"] = True
    out["R"] = best[1]
    out["t"] = best[2]
    return out


def solve_tag_square(obj4, img4, K, size):
    """Per-tag diagnostic only. IPPE_SQUARE is legitimate here because this IS
    a single square, and it is never used for the multi-tag board."""
    h = size / 2.0
    sq = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
                  dtype=np.float64)
    try:
        ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            sq, img4, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return np.nan, np.nan
        e = np.asarray(errs).ravel()
        return float(np.min(e)), float(np.max(e))
    except cv2.error:
        return np.nan, np.nan


# =============================================================================
# CORNER-ORDER SELF-CHECK
# =============================================================================
# pupil_apriltags documents tag.corners as the four corners in the order
#     (-1,-1), (+1,-1), (+1,+1), (-1,+1)
# in the TAG's own coordinate frame, i.e. bottom-left, bottom-right, top-right,
# top-left, and the ordering is orientation-aware: it follows the decoded tag,
# so a tag printed rotated by 90 deg still reports its own bottom-left first.
# build_object_points() is written to that convention.
#
# That is documentation, not evidence. This self-check measures it on the real
# board instead of assuming it. Every candidate below is a consistent
# re-indexing of the four detected corners applied identically to all tags; a
# cyclic shift is equivalent to adding 90 deg to every tag's printed rotation,
# and the reversed set covers a mirrored winding. If the documented order is
# not the best one, acquisition stops.
#
# It cannot, and does not claim to, detect a wrong rotation on ONE tag. That
# shows up as a large board reprojection error and in the per-tag column, both
# of which are printed here too (entering tag 3 as 0 deg instead of 90 gave
# 27 px in the synthetic test).
CORNER_ORDERS = {
    "documented BL,BR,TR,TL": (0, 1, 2, 3),
    "cyclic +90":             (1, 2, 3, 0),
    "cyclic +180":            (2, 3, 0, 1),
    "cyclic +270":            (3, 0, 1, 2),
    "mirrored":               (3, 2, 1, 0),
    "mirrored +90":           (0, 3, 2, 1),
    "mirrored +180":          (1, 0, 3, 2),
    "mirrored +270":          (2, 1, 0, 3),
}
SELFCHECK_FRAMES = 60
SELFCHECK_MAX_REPROJ_PX = 2.0
SELFCHECK_MARGIN = 1.25     # the documented order must beat the runner-up by this


def corner_order_self_check(frames, obj_by_tag, K, verbose=True):
    """Check detected-corner correspondence using one planar homography.

    This guard does NOT estimate the production 3-D pose.  Its only purpose
    is to verify the detector corner ordering against the rigid planar board
    geometry before acquisition.

    Using a planar homography here avoids the numerical singularities that
    SOLVEPNP_IPPE can occasionally produce for a nearly fronto-parallel
    stationary board.

    The production pose solver remains unchanged and still uses IPPE.
    """

    if not frames:
        return False, {"error": "no frames with the full tag set"}

    scores = {}

    for name, perm in CORNER_ORDERS.items():
        errs = []

        for fr in frames:
            O = []
            I = []

            for tid in sorted(fr):
                if tid not in obj_by_tag:
                    continue

                O.append(
                    np.asarray(obj_by_tag[tid], dtype=np.float64)[:, :2]
                )

                I.append(
                    np.asarray(fr[tid], dtype=np.float64)[list(perm)]
                )

            if not O:
                continue

            O = np.vstack(O).astype(np.float64)
            I = np.vstack(I).astype(np.float64)

            if len(O) < 4:
                continue

            try:
                H, _ = cv2.findHomography(O, I, 0)

                if H is None or not np.all(np.isfinite(H)):
                    continue

                P = cv2.perspectiveTransform(
                    O.reshape(-1, 1, 2),
                    H
                ).reshape(-1, 2)

                e = float(
                    np.sqrt(
                        np.mean(
                            np.sum((P - I) ** 2, axis=1)
                        )
                    )
                )

                if np.isfinite(e):
                    errs.append(e)

            except cv2.error:
                continue

        scores[name] = (
            float(np.median(errs))
            if errs
            else float("inf")
        )

    ranked = sorted(
        scores.items(),
        key=lambda kv: kv[1]
    )

    best_name, best_err = ranked[0]

    doc_name = "documented BL,BR,TR,TL"
    doc_err = scores[doc_name]

    others = [
        v for k, v in scores.items()
        if k != doc_name and np.isfinite(v)
    ]

    runner_up = (
        min(others)
        if others
        else float("inf")
    )

    # ---------------------------------------------------------
    # Per-tag contribution under the documented corner order.
    # Fit ONE homography from all 16 points, then inspect each tag.
    # ---------------------------------------------------------
    per_tag_samples = {
        tid: []
        for tid in EXPECTED_TAG_IDS
    }

    for fr in frames:
        O = []
        I = []

        for tid in sorted(fr):
            if tid not in obj_by_tag:
                continue

            O.append(
                np.asarray(obj_by_tag[tid], dtype=np.float64)[:, :2]
            )

            I.append(
                np.asarray(fr[tid], dtype=np.float64)
            )

        if not O:
            continue

        O = np.vstack(O).astype(np.float64)
        I = np.vstack(I).astype(np.float64)

        try:
            H, _ = cv2.findHomography(O, I, 0)

            if H is None or not np.all(np.isfinite(H)):
                continue

            for tid in EXPECTED_TAG_IDS:
                if tid not in fr or tid not in obj_by_tag:
                    continue

                Ot = np.asarray(
                    obj_by_tag[tid],
                    dtype=np.float64
                )[:, :2]

                Pt = cv2.perspectiveTransform(
                    Ot.reshape(-1, 1, 2),
                    H
                ).reshape(-1, 2)

                It = np.asarray(
                    fr[tid],
                    dtype=np.float64
                )

                e = float(
                    np.sqrt(
                        np.mean(
                            np.sum((Pt - It) ** 2, axis=1)
                        )
                    )
                )

                if np.isfinite(e):
                    per_tag_samples[tid].append(e)

        except cv2.error:
            continue

    per_tag = {
        tid: (
            float(np.median(vals))
            if vals
            else float("nan")
        )
        for tid, vals in per_tag_samples.items()
    }

    ok = (
        best_name == doc_name
        and np.isfinite(doc_err)
        and doc_err <= SELFCHECK_MAX_REPROJ_PX
        and runner_up >= SELFCHECK_MARGIN * doc_err
    )

    if verbose:
        print(
            "\n  CORNER-ORDER SELF-CHECK "
            f"({len(frames)} frames, planar homography RMS)"
        )

        for name, v in ranked:
            mark = (
                "  <-- documented"
                if name == doc_name
                else ""
            )

            print(
                f"    {name:26s} "
                f"{v:9.4f} px{mark}"
            )

        print(
            "    per-tag reprojection "
            "under the documented order:"
        )

        for tid, v in per_tag.items():
            print(
                f"      tag {tid}: "
                f"{v:8.4f} px"
            )

    return ok, dict(
        scores=scores,
        best=best_name,
        documented_px=doc_err,
        runner_up_px=runner_up,
        per_tag_px=per_tag,
        passed=bool(ok),
        method="planar_homography"
    )



# =============================================================================
# AUTOMATIC RECTANGULAR-BOARD INFERENCE
# =============================================================================
# The four tags sit on one rigid flat board in a regular rectangle. Everything
# below is pure geometry on already-collected observations, with no camera and
# no detector, so it is unit-testable: test_pitch_roll_pose_solver.py calls
# infer_rectangular_board() directly on synthetic observations.
#
# FRAME. The derived board frame is the REFERENCE TAG's own frame (lowest
# visible tag id by default). That is the only choice that makes
# "rotation_deg = 0 / 90 / 180 / 270" literally true, because
# build_object_points() lays a rotation_deg = 0 tag out exactly as
# pupil_apriltags defines the tag frame. Expressing the centres in the same
# frame keeps the object points internally consistent.
#
# A constant in-plane offset between this frame and anything else is harmless:
# it is a rotation of the whole board about its normal, and R_rel = R_ref^T
# R_board cancels it exactly. The console therefore also prints each tag's
# mean IMAGE pixel position, so the board-frame corner labels can be matched
# to what the operator actually sees without relying on a sign convention.
CAL_MIN_FRAMES = 100            # frames with the full tag set
CAL_MIN_FULL_FRAME_PCT = 50.0   # of all grabbed frames
CAL_MIN_TAG_DETECT_PCT = 80.0   # per tag
CAL_OPPOSITE_SIDE_TOL = 0.01    # 1 % of the side, or CAL_OPPOSITE_SIDE_MM
CAL_OPPOSITE_SIDE_MM = 1.0
CAL_COPLANAR_MAX_MM = 2.0
CAL_ROT_SNAP_MAX_DEG = 10.0
CAL_AXIS_ALIGN_MAX_DEG = 5.0
CAL_MAX_REPROJ_PX = 1.5
CAL_OUTLIER_MAD = 4.0


def _circmean_deg(a):
    a = np.asarray(a, float)
    return float(np.degrees(np.arctan2(np.mean(np.sin(np.radians(a))),
                                       np.mean(np.cos(np.radians(a))))))


def _mad(v):
    v = np.asarray(v, float)
    m = np.median(v)
    return float(np.median(np.abs(v - m))) * 1.4826


def infer_rectangular_board(frames, tag_size_m, expected_ids=None,
                            ref_tag=None, camera_matrix=None):
    """Infer the rectangular layout from many stationary observations.

    frames : list of {tag_id: {"R": 3x3 camera<-tag rotation,
                               "t": (3,) tag centre in camera coords [m],
                               "corners": (4,2) image pixels}}
    Returns (ok, geometry_dict_or_None, report).
    """
    ids = sorted(expected_ids or EXPECTED_TAG_IDS)
    rep = {"checks": [], "n_frames_total": len(frames)}

    def fail(msg):
        rep["checks"].append(("FAIL", msg))
        return False, None, rep

    def ok_(msg):
        rep["checks"].append(("OK", msg))

    # ---- detection statistics -------------------------------------------
    seen = {t: sum(1 for f in frames if t in f) for t in ids}
    rep["detect_pct"] = {t: 100.0 * seen[t] / max(len(frames), 1) for t in ids}
    full = [f for f in frames if all(t in f for t in ids)]
    rep["n_frames_full"] = len(full)
    rep["full_frame_pct"] = 100.0 * len(full) / max(len(frames), 1)

    for t in ids:
        if rep["detect_pct"][t] < CAL_MIN_TAG_DETECT_PCT:
            return fail(f"tag {t} was detected in only "
                        f"{rep['detect_pct'][t]:.1f} % of frames "
                        f"(need {CAL_MIN_TAG_DETECT_PCT:.0f} %)")
    if len(full) < CAL_MIN_FRAMES:
        return fail(f"only {len(full)} frames had all {len(ids)} tags "
                    f"(need {CAL_MIN_FRAMES})")
    if rep["full_frame_pct"] < CAL_MIN_FULL_FRAME_PCT:
        return fail(f"only {rep['full_frame_pct']:.1f} % of frames had the "
                    f"full tag set (need {CAL_MIN_FULL_FRAME_PCT:.0f} %)")
    ok_(f"{len(full)} frames with all {len(ids)} tags "
        f"({rep['full_frame_pct']:.1f} % of {len(frames)})")

    ref = ref_tag if ref_tag is not None else ids[0]
    if ref not in ids:
        return fail(f"reference tag {ref} is not in the expected set {ids}")
    rep["reference_tag"] = ref

    # ---- per-frame layout in the reference tag's frame -------------------
    P = {t: [] for t in ids}        # in-plane centre, mm
    A = {t: [] for t in ids}        # in-plane rotation vs the reference, deg
    Z = {t: [] for t in ids}        # out-of-plane residual, mm
    for f in full:
        Rr = np.asarray(f[ref]["R"], float)
        c = np.mean([np.asarray(f[t]["t"], float) for t in ids], axis=0)
        for t in ids:
            p = Rr.T @ (np.asarray(f[t]["t"], float) - c)
            P[t].append(p[:2] * 1000.0)
            Z[t].append(p[2] * 1000.0)
            M = Rr.T @ np.asarray(f[t]["R"], float)
            A[t].append(math.degrees(math.atan2(M[1, 0], M[0, 0])))

    # ---- robust outlier rejection on the frame-level layout --------------
    span = np.array([np.linalg.norm(np.asarray(P[ids[0]][k])
                                    - np.asarray(P[ids[-1]][k]))
                     for k in range(len(full))])
    s_med, s_mad = float(np.median(span)), _mad(span)
    keep = (np.abs(span - s_med) <= CAL_OUTLIER_MAD * max(s_mad, 1e-6)) \
        if s_mad > 0 else np.ones(len(span), bool)
    rep["n_frames_used"] = int(keep.sum())
    rep["frames_rejected"] = int((~keep).sum())
    rep["used_pct"] = 100.0 * rep["n_frames_used"] / max(len(frames), 1)
    if rep["n_frames_used"] < CAL_MIN_FRAMES:
        return fail(f"after outlier rejection only {rep['n_frames_used']} "
                    f"frames remain (need {CAL_MIN_FRAMES})")
    ok_(f"{rep['n_frames_used']} frames used after robust rejection "
        f"({rep['frames_rejected']} rejected, {rep['used_pct']:.1f} % of all)")

    ctr = {t: np.median(np.asarray(P[t])[keep], axis=0) for t in ids}
    ctr_sd = {t: np.asarray(P[t])[keep].std(axis=0, ddof=1) for t in ids}
    rot_raw = {t: _circmean_deg(np.asarray(A[t])[keep]) for t in ids}
    zmax = {t: float(np.max(np.abs(np.asarray(Z[t])[keep]))) for t in ids}

    # Build ONE common board plane from all four robust median tag centres.
    # Do not use the reference tag IPPE normal: near fronto-parallel viewing
    # that single-tag normal is ambiguous and can rotate by several degrees.
    kept_idx = np.where(keep)[0]

    c_cam = {}
    for t in ids:
        arr = np.asarray(
            [full[k][t]["t"] for k in kept_idx],
            dtype=float
        )
        c_cam[t] = np.median(arr, axis=0)

    board_c = np.mean(
        np.vstack([c_cam[t] for t in ids]),
        axis=0
    )

    centered = np.vstack(
        [c_cam[t] - board_c for t in ids]
    )

    _, _, Vt_plane = np.linalg.svd(
        centered,
        full_matrices=False
    )

    n_board = Vt_plane[-1]
    n_board = n_board / np.linalg.norm(n_board)

    # Reference-tag printed X/Y are used only as IN-PLANE direction hints.
    ref_x = np.median(
        np.asarray(
            [full[k][ref]["R"][:, 0] for k in kept_idx],
            dtype=float
        ),
        axis=0
    )

    ref_y = np.median(
        np.asarray(
            [full[k][ref]["R"][:, 1] for k in kept_idx],
            dtype=float
        ),
        axis=0
    )

    normal_hint = np.cross(ref_x, ref_y)

    if np.dot(n_board, normal_hint) < 0:
        n_board = -n_board

    x_board = ref_x - np.dot(ref_x, n_board) * n_board

    x_norm = np.linalg.norm(x_board)
    if x_norm < 1e-9:
        return fail("cannot establish board X axis from reference tag")

    x_board = x_board / x_norm

    y_board = np.cross(n_board, x_board)
    y_board = y_board / np.linalg.norm(y_board)

    if np.dot(y_board, ref_y) < 0:
        n_board = -n_board
        y_board = np.cross(n_board, x_board)
        y_board = y_board / np.linalg.norm(y_board)

    # Re-project all four tag centres into this common fitted board plane.
    ctr_global = {}
    plane_residual = {}

    for t in ids:
        d = c_cam[t] - board_c

        ctr_global[t] = np.array([
            np.dot(d, x_board),
            np.dot(d, y_board)
        ]) * 1000.0

        plane_residual[t] = float(
            np.dot(d, n_board) * 1000.0
        )

    ctr = ctr_global

    rep["global_plane_residual_mm"] = plane_residual
    rep["global_plane_worst_residual_mm"] = max(
        abs(v) for v in plane_residual.values()
    )

    ok_(
        f"global four-tag plane established; "
        f"worst median-centre residual "
        f"{rep['global_plane_worst_residual_mm']:.2f} mm"
    )

    # ---- coplanarity -----------------------------------------------------
    rep["out_of_plane_mm"] = {t: zmax[t] for t in ids}
    worst_z = max(zmax.values())
    if worst_z > CAL_COPLANAR_MAX_MM:
        rep["checks"].append((
            "WARN",
            f"single-tag IPPE plane disagreement {worst_z:.2f} mm "
            f"(old diagnostic limit {CAL_COPLANAR_MAX_MM:.1f} mm); "
            f"NOT used as a GO/NO-GO check because planar single-tag pose "
            f"is ambiguous near fronto-parallel viewing"
        ))
    else:
        ok_(f"single-tag plane disagreement {worst_z:.2f} mm")

    # ---- rotation snapping ----------------------------------------------
    rot_snap, rot_res = {}, {}
    for t in ids:
        k = int(round(rot_raw[t] / 90.0)) % 4
        rot_snap[t] = float(90 * k)
        r = rot_raw[t] - 90.0 * k
        rot_res[t] = float((r + 180.0) % 360.0 - 180.0)
    rep["rotation_raw_deg"] = rot_raw
    rep["rotation_snapped_deg"] = rot_snap
    rep["rotation_residual_deg"] = rot_res
    worst_r = max(abs(v) for v in rot_res.values())
    if worst_r > CAL_ROT_SNAP_MAX_DEG:
        return fail(f"a printed tag rotation is not close to a multiple of "
                    f"90 deg: worst residual {worst_r:.2f} deg "
                    f"(limit {CAL_ROT_SNAP_MAX_DEG:.0f} deg)")
    ok_(f"all printed rotations within {worst_r:.2f} deg of 0/90/180/270")

    # ---- automatically align the in-plane frame with the rectangle -------
    # The rigid board may be rotated by an arbitrary in-plane angle relative
    # to the printed frame of reference tag 0. Search that angle rather than
    # requiring tag-0 X/Y to coincide with the rectangle edges.

    mean_ctr = np.mean(
        np.vstack([np.asarray(ctr[t], float) for t in ids]),
        axis=0
    )

    ctr0 = {
        t: np.asarray(ctr[t], float) - mean_ctr
        for t in ids
    }

    best = None

    # 0.1-degree search is far finer than required for board geometry.
    for alpha in np.linspace(-90.0, 90.0, 1801):
        a = math.radians(alpha)

        # coordinates in a frame rotated by -alpha
        Q = np.array([
            [ math.cos(a),  math.sin(a)],
            [-math.sin(a),  math.cos(a)]
        ])

        q = {t: Q @ ctr0[t] for t in ids}

        corner_try = {}
        for t in ids:
            right = q[t][0] > 0.0
            top   = q[t][1] > 0.0
            corner_try[t] = (
                ("top" if top else "bottom")
                + "-"
                + ("right" if right else "left")
            )

        if sorted(corner_try.values()) != [
            "bottom-left", "bottom-right",
            "top-left", "top-right"
        ]:
            continue

        by_try = {v: k for k, v in corner_try.items()}

        TL0 = by_try["top-left"]
        TR0 = by_try["top-right"]
        BL0 = by_try["bottom-left"]
        BR0 = by_try["bottom-right"]

        w_top0 = np.linalg.norm(q[TR0] - q[TL0])
        w_bot0 = np.linalg.norm(q[BR0] - q[BL0])
        h_left0 = np.linalg.norm(q[TL0] - q[BL0])
        h_right0 = np.linalg.norm(q[TR0] - q[BR0])

        W0 = float(np.median([w_top0, w_bot0]))
        H0 = float(np.median([h_left0, h_right0]))

        ideal0 = {
            TL0: np.array([-W0/2.0,  H0/2.0]),
            TR0: np.array([ W0/2.0,  H0/2.0]),
            BL0: np.array([-W0/2.0, -H0/2.0]),
            BR0: np.array([ W0/2.0, -H0/2.0]),
        }

        rms = float(np.sqrt(np.mean([
            np.sum((q[t] - ideal0[t])**2)
            for t in ids
        ])))

        if best is None or rms < best[0]:
            best = (rms, alpha, q, corner_try, by_try)

    if best is None:
        return fail(
            "could not find any in-plane rotation that produces "
            "one tag at each rectangle corner"
        )

    rectangle_fit_rms, board_alpha_deg, ctr, corner, by = best

    rep["board_frame_rotation_from_reference_deg"] = float(board_alpha_deg)
    rep["rectangle_alignment_fit_rms_mm"] = float(rectangle_fit_rms)
    rep["corner_assignment"] = corner

    # Tag rotations must be expressed in the NEW rectangle-aligned board
    # frame, not in the old reference-tag frame.
    rot_geom = {
        t: float(
            ((rot_snap[t] - board_alpha_deg + 180.0) % 360.0) - 180.0
        )
        for t in ids
    }

    rep["rotation_geometry_deg"] = rot_geom

    ok_(
        f"rectangle frame automatically aligned by "
        f"{board_alpha_deg:+.2f} deg "
        f"(fit RMS {rectangle_fit_rms:.3f} mm)"
    )

    ok_(f"one tag per corner: TL={by['top-left']} TR={by['top-right']} "
        f"BL={by['bottom-left']} BR={by['bottom-right']}")

    TL, TR = by["top-left"], by["top-right"]
    BL, BR = by["bottom-left"], by["bottom-right"]

    # ---- side lengths, opposite-side agreement ---------------------------
    w_top = float(np.linalg.norm(ctr[TR] - ctr[TL]))
    w_bot = float(np.linalg.norm(ctr[BR] - ctr[BL]))
    h_left = float(np.linalg.norm(ctr[TL] - ctr[BL]))
    h_right = float(np.linalg.norm(ctr[TR] - ctr[BR]))
    W = float(np.median([w_top, w_bot]))
    H = float(np.median([h_left, h_right]))
    rep.update(W_mm=W, H_mm=H, w_top_mm=w_top, w_bottom_mm=w_bot,
               h_left_mm=h_left, h_right_mm=h_right,
               w_opposite_diff_mm=abs(w_top - w_bot),
               h_opposite_diff_mm=abs(h_left - h_right))

    tol_w = max(CAL_OPPOSITE_SIDE_TOL * W, CAL_OPPOSITE_SIDE_MM)
    tol_h = max(CAL_OPPOSITE_SIDE_TOL * H, CAL_OPPOSITE_SIDE_MM)

    dW = abs(w_top - w_bot)
    dH = abs(h_left - h_right)

    side_warning = False

    if dW > tol_w:
        side_warning = True
        rep["checks"].append((
            "WARN",
            f"opposite horizontal sides differ by {dW:.3f} mm "
            f"({w_top:.3f} vs {w_bot:.3f} mm; old limit {tol_w:.3f} mm). "
            f"Because the physical board is known to be a regular rectangle, "
            f"the geometry will be regularized using W={W:.3f} mm. "
            f"Joint-board reprojection remains mandatory."
        ))

    if dH > tol_h:
        side_warning = True
        rep["checks"].append((
            "WARN",
            f"opposite vertical sides differ by {dH:.3f} mm "
            f"({h_left:.3f} vs {h_right:.3f} mm; old limit {tol_h:.3f} mm). "
            f"Because the physical board is known to be a regular rectangle, "
            f"the geometry will be regularized using H={H:.3f} mm. "
            f"Joint-board reprojection remains mandatory."
        ))

    if not side_warning:
        ok_(f"opposite sides agree: dW {dW:.3f} mm, dH {dH:.3f} mm")

    # ---- per-frame spread of W and H ------------------------------------
    Wf, Hf = [], []
    idx = np.where(keep)[0]
    for k in idx:
        p = {t: np.asarray(P[t][k]) for t in ids}
        Wf.append(0.5 * (np.linalg.norm(p[TR] - p[TL])
                         + np.linalg.norm(p[BR] - p[BL])))
        Hf.append(0.5 * (np.linalg.norm(p[TL] - p[BL])
                         + np.linalg.norm(p[TR] - p[BR])))
    rep["W_sd_mm"] = float(np.std(Wf, ddof=1))
    rep["H_sd_mm"] = float(np.std(Hf, ddof=1))
    rep["centre_sd_mm"] = {t: [float(q) for q in ctr_sd[t]] for t in ids}

    # ---- rectangle is axis-aligned in the reference frame ----------------
    def ang(v):
        return math.degrees(math.atan2(v[1], v[0]))
    a_top = ang(ctr[TR] - ctr[TL])
    a_left = ang(ctr[TL] - ctr[BL])
    skew_x = abs((a_top + 180.0) % 360.0 - 180.0)
    skew_y = abs(((a_left - 90.0) + 180.0) % 360.0 - 180.0)
    rep["axis_skew_deg"] = dict(horizontal=skew_x, vertical=skew_y)
    if max(skew_x, skew_y) > CAL_AXIS_ALIGN_MAX_DEG:
        return fail(f"the rectangle is not aligned with the printed tag axes "
                    f"(skew {skew_x:.2f} / {skew_y:.2f} deg, limit "
                    f"{CAL_AXIS_ALIGN_MAX_DEG:.0f} deg). The tags are rotated "
                    f"relative to the board edges; measure the board by hand.")
    ok_(f"rectangle aligned with the tag axes (skew {skew_x:.2f} / "
        f"{skew_y:.2f} deg)")

    # ---- winding: reject a mirrored layout -------------------------------
    quad = np.array([ctr[TL], ctr[TR], ctr[BR], ctr[BL]])
    area2 = 0.0
    for k in range(4):
        x1, y1 = quad[k]
        x2, y2 = quad[(k + 1) % 4]
        area2 += x1 * y2 - x2 * y1
    rep["signed_area_mm2"] = float(area2 / 2.0)
    rep["handedness"] = "right-handed" if area2 < 0 else "LEFT-HANDED"
    # This is a defensive consistency report, not a check that can fire on
    # valid pose data: the corner labels are assigned from the sign of each
    # centre in the board frame, so TL -> TR -> BR -> BL always winds
    # clockwise. It is printed because a left-handed result would mean the
    # frame construction itself had been broken by a future edit.
    if area2 >= 0:
        return fail("internal inconsistency: the corner labelling produced a "
                    "left-handed layout. The board-frame construction is "
                    "broken; do not use this geometry.")
    ok_(f"layout handedness consistent ({rep['handedness']}, area "
        f"{abs(area2)/2.0/100.0:.1f} cm^2)")

    # ---- ideal rectangle, origin at the geometric centre -----------------
    ideal = {TL: (-W / 2.0, H / 2.0), TR: (W / 2.0, H / 2.0),
             BL: (-W / 2.0, -H / 2.0), BR: (W / 2.0, -H / 2.0)}
    rep["fit_residual_mm"] = {
        t: float(np.linalg.norm(ctr[t] - np.array(ideal[t]))) for t in ids}

    # ---- resolve tag-corner rotation convention from REAL reprojection ----
    #
    # The rectangle frame may be rotated relative to the decoded tag frame,
    # and camera/tag coordinate conventions can differ in sign.  Do not guess.
    # Search the finite physically-equivalent convention set and select it
    # ONLY by joint 16-corner board reprojection on the real observations.

    if camera_matrix is not None:

        def _wrap_deg(x):
            return float((x + 180.0) % 360.0 - 180.0)

        def _score_geometry(rots, sign_x=1.0):
            g = {
                "tag_black_square_size_m": float(tag_size_m),
                "tags": {
                    str(t): {
                        "center_x_mm": float(sign_x * ideal[t][0]),
                        "center_y_mm": float(ideal[t][1]),
                        "rotation_deg": float(rots[t]),
                    }
                    for t in ids
                },
            }

            ob = build_object_points(g)
            e = []
            Rp = None

            test_frames = [
                x for x in full if all(t in x for t in ids)
            ][:160]

            for f in test_frames:
                O = np.vstack([ob[t] for t in ids])
                I = np.vstack([
                    np.asarray(f[t]["corners"], float)
                    for t in ids
                ])

                sol = solve_board(
                    O, I, camera_matrix, Rp
                )

                if sol["ok"]:
                    Rp = sol["R"]
                    e.append(sol["reproj_px"])

            return float(np.median(e)) if e else float("inf")

        candidates = []

        for rot_sign in (+1.0, -1.0):
            for alpha_sign in (+1.0, -1.0):
                for offset in (0.0, 90.0, 180.0, 270.0):

                    rots = {
                        t: _wrap_deg(
                            rot_sign * rot_snap[t]
                            + alpha_sign * board_alpha_deg
                            + offset
                        )
                        for t in ids
                    }

                    score = _score_geometry(rots)

                    candidates.append({
                        "score": score,
                        "rot_sign": rot_sign,
                        "alpha_sign": alpha_sign,
                        "offset": offset,
                        "rots": rots,
                    })

        candidates.sort(key=lambda x: x["score"])
        best_rot = candidates[0]

        rot_geom = best_rot["rots"]

        rep["rotation_geometry_deg"] = {
            int(t): float(rot_geom[t])
            for t in ids
        }

        rep["rotation_convention_search"] = [
            {
                "reproj_px": float(c["score"]),
                "rot_sign": float(c["rot_sign"]),
                "alpha_sign": float(c["alpha_sign"]),
                "offset_deg": float(c["offset"]),
            }
            for c in candidates[:5]
        ]

        best_r = float(best_rot["score"])

        if not np.isfinite(best_r) or best_r > CAL_MAX_REPROJ_PX:
            return fail(
                f"no tag-corner rotation convention gives an acceptable "
                f"joint-board reprojection: best {best_r:.3f} px "
                f"(limit {CAL_MAX_REPROJ_PX:.1f} px)"
            )

        ok_(
            f"tag-corner rotation convention resolved from real "
            f"joint-board reprojection: {best_r:.4f} px "
            f"(rot_sign={best_rot['rot_sign']:+.0f}, "
            f"alpha_sign={best_rot['alpha_sign']:+.0f}, "
            f"offset={best_rot['offset']:.0f} deg)"
        )

        # Mirror remains a diagnostic.  The mandatory protection after this
        # is the absolute reprojection limit plus the corner-order self-check.
        mirror_r = _score_geometry(rot_geom, sign_x=-1.0)

        rep["reproj_derived_px"] = best_r
        rep["reproj_mirrored_px"] = float(mirror_r)

        if mirror_r > 1.25 * best_r:
            ok_(
                f"derived handedness preferred: "
                f"{best_r:.4f} px vs mirror {mirror_r:.4f} px"
            )
        else:
            rep["checks"].append((
                "WARN",
                f"mirror is not strongly separated "
                f"({best_r:.4f} vs {mirror_r:.4f} px); "
                f"continuing only because absolute joint-board "
                f"reprojection passed. Corner-order self-check remains "
                f"mandatory."
            ))

    geom = {
        "_comment": "AUTOMATICALLY DERIVED from camera observations by "
                    "--mode calibrate-board. Not a mechanical measurement.",
        "geometry_source": "camera_derived_calibration",
        "_warning": "Camera-derived board geometry. For final "
                    "publication-quality independent validation, mechanically "
                    "measure W and H with a ruler or caliper and replace or "
                    "verify these values.",
        "board_origin": "geometric centre of the rectangle",
        "board_frame": "rectangle-aligned frame automatically derived "
                       "from the four rigid tag centres",
        "tag_black_square_size_m": float(tag_size_m),
        "horizontal_center_spacing_mm": W,
        "vertical_center_spacing_mm": H,
        "corner_assignment": {corner[t]: t for t in ids},
        "tags": {str(t): {"center_x_mm": float(ideal[t][0]),
                          "center_y_mm": float(ideal[t][1]),
                          "rotation_deg": float(rot_geom[t])} for t in ids},
    }
    return True, geom, rep


# =============================================================================
# CAMERA
# =============================================================================
def open_camera():
    _require_hardware()
    cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    cam.Open()
    try:
        cam.UserSetSelector.SetValue("Default")
        cam.UserSetLoad.Execute()
    except Exception:
        pass
    for setter in (lambda: cam.PixelFormat.SetValue("Mono8"),
                   lambda: cam.TriggerMode.SetValue("Off"),
                   lambda: cam.AcquisitionMode.SetValue("Continuous"),
                   lambda: cam.ExposureAuto.SetValue("Off"),
                   lambda: cam.GainAuto.SetValue("Off")):
        try:
            setter()
        except Exception:
            pass
    try:
        cam.ExposureTime.SetValue(EXPOSURE_US)
    except Exception:
        pass
    try:
        cam.Gain.SetValue(GAIN_DB)
    except Exception:
        pass
    conv = pylon.ImageFormatConverter()
    conv.OutputPixelFormat = pylon.PixelType_Mono8
    conv.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
    cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    return cam, conv


def build_undistort(cam):
    w, h = cam.Width.Value, cam.Height.Value
    newK, _ = cv2.getOptimalNewCameraMatrix(CAMERA_MATRIX, DIST_COEFFS,
                                            (w, h), 1, (w, h))
    m1, m2 = cv2.initUndistortRectifyMap(CAMERA_MATRIX, DIST_COEFFS, None,
                                         newK, (w, h), cv2.CV_16SC2)
    return w, h, newK, m1, m2


# =============================================================================
# PRE-FLIGHT PREVIEW
# =============================================================================
def preflight(grab_detect):
    print("\nPRE-FLIGHT PREVIEW")
    print(f"  check the image and that tags {EXPECTED_TAG_IDS} are all visible")
    print("  press  g  to accept,  q / ESC to abort\n")
    win = "PRE-FLIGHT  (g = go, q/ESC = abort)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)
    accepted = False
    while True:
        und, tags, _ = grab_detect()
        if und is None:
            continue
        disp = cv2.cvtColor(und, cv2.COLOR_GRAY2BGR)
        seen = []
        for tg in tags:
            seen.append(int(tg.tag_id))
            c = tg.corners.astype(np.int32)
            cv2.polylines(disp, [c.reshape(-1, 1, 2)], True, (0, 255, 0), 2,
                          cv2.LINE_AA)
            ti = int(np.argmin(c[:, 1]))
            cv2.putText(disp, f"ID {int(tg.tag_id)} dm={tg.decision_margin:.0f}",
                        (int(c[ti][0]) - 10, int(c[ti][1]) - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        miss = [t for t in EXPECTED_TAG_IDS if t not in seen]
        lines = [f"visible: {sorted(seen)}",
                 ("ALL EXPECTED TAGS VISIBLE" if not miss else f"MISSING: {miss}"),
                 "g = go    q/ESC = abort"]
        for i, txt in enumerate(lines):
            y = 40 + i * 36
            cv2.putText(disp, txt, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(disp, txt, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 0, 255) if miss else (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow(win, cv2.resize(disp, (1280, 720)))
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("g"), ord("G")):
            accepted = True
            break
        if k in (ord("q"), ord("Q"), 27):
            break
    cv2.destroyWindow(win)
    cv2.destroyAllWindows()
    for _ in range(5):
        cv2.waitKey(1)
    return accepted


# =============================================================================
# CORE CAPTURE LOOP
# =============================================================================
def run_capture(args, geom, obj_by_tag, mode):
    cam, conv = open_camera()
    w, h, newK, m1, m2 = build_undistort(cam)
    fx, fy, cx, cy = newK[0, 0], newK[1, 1], newK[0, 2], newK[1, 2]
    det = make_detector()
    size = float(geom["tag_black_square_size_m"])

    def grab_detect():
        """Grab one frame and detect tags.

        HOST TIMESTAMP POSITION: perf_counter_ns and time_ns are latched
        IMMEDIATELY after RetrieveResult reports success, BEFORE
        converter.Convert(), GetArray(), cv2.remap() and AprilTag detection.
        Conversion plus undistortion plus detection is several milliseconds
        with its own jitter, and putting the stamp after them would fold that
        jitter into the camera<->motor time mapping. This matches the
        validated yaw acquisition, where the stamp was taken at frame grab.

        The Basler device timestamp (camera-side clock, GevTimestamp /
        ChunkTimestamp / TimestampLatch depending on the model) is logged as an
        ADDITIONAL diagnostic column when available. It is on the camera's own
        clock with no established mapping to the host, so it is never used in
        place of pc_perf_counter_ns.
        """
        gr = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if not gr.GrabSucceeded():
            gr.Release()
            return None, None, None

        # ---- latch the host clocks first, before ANY image processing ----
        t_wall = time.time()
        perf_ns = time.perf_counter_ns()
        wall_ns = time.time_ns()

        dev_ts = np.nan
        try:
            v = gr.TimeStamp
            if v:
                dev_ts = float(v)
        except Exception:
            pass

        frame = conv.Convert(gr).GetArray()
        gr.Release()
        stamps = (t_wall, perf_ns, wall_ns, dev_ts)

        und = cv2.remap(frame, m1, m2, cv2.INTER_LINEAR)
        tags = det.detect(und, estimate_tag_pose=False)
        return und, tags, stamps

    if not args.skip_preflight and mode != "calibrate-board":
        if not preflight(grab_detect):
            cam.StopGrabbing()
            cam.Close()
            print("Aborted at pre-flight. Nothing was recorded.")
            return None, None
    elif mode == "calibrate-board" and not args.skip_preflight:
        if not preflight(grab_detect):
            cam.StopGrabbing()
            cam.Close()
            print("Aborted at pre-flight.")
            return None, None

    # ---- corner-order self-check on real frames, before anything is logged --
    sc_frames = []
    t_sc = time.time()
    while len(sc_frames) < SELFCHECK_FRAMES and (time.time() - t_sc) < 20.0:
        und, tags, _ = grab_detect()
        if und is None:
            continue
        fr = {int(t.tag_id): np.asarray(t.corners, float) for t in tags
              if int(t.tag_id) in obj_by_tag}
        if len(fr) == len(EXPECTED_TAG_IDS):
            sc_frames.append(fr)
    sc_ok, sc_report = corner_order_self_check(sc_frames, obj_by_tag, newK)
    if not sc_ok:
        print("\n  CORNER-ORDER SELF-CHECK FAILED.")
        print(f"    best ordering was '{sc_report.get('best')}' at "
              f"{min(sc_report.get('scores', {'x': float('nan')}).values()):.4f} px; "
              f"the documented order gave "
              f"{sc_report.get('documented_px', float('nan')):.4f} px.")
        print("    Either the detector's corner convention differs from the one "
              "build_object_points() assumes, or the board geometry is wrong.")
        print("    Check rotation_deg for each tag and the tag size, then "
              "re-run. Nothing was recorded.")
        cam.StopGrabbing()
        cam.Close()
        return None, None

    rows = []
    R_prev = None
    R_ref = None
    ref_pool = []
    n_flips = 0
    last_status = 0.0
    last_n = 0
    last_t = 0.0
    t_start = time.time()
    start_perf = time.perf_counter_ns()
    start_wall = time.time_ns()

    print(f"\n{'CALIBRATING BOARD' if mode=='calibrate-board' else 'RECORDING'} "
          f"for {args.duration:.1f} s ...\n")

    try:
        while cam.IsGrabbing():
            if (time.time() - t_start) >= args.duration:
                break
            und, tags, stamps = grab_detect()
            if und is None:
                continue
            t_now, perf_ns, wall_ns, dev_ts = stamps
            elapsed = t_now - t_start

            vis, img_pts, obj_pts, per_tag = [], [], [], {}
            for tg in tags:
                tid = int(tg.tag_id)
                if tid not in obj_by_tag:
                    continue
                vis.append(tid)
                c = np.asarray(tg.corners, dtype=np.float64)
                img_pts.append(c)
                obj_pts.append(obj_by_tag[tid])
                per_tag[tid] = dict(
                    dm=float(tg.decision_margin),
                    hamming=int(getattr(tg, "hamming", -1)),
                    corners=c,
                    cx=float(np.mean(c[:, 0])), cy=float(np.mean(c[:, 1])))

            n_vis = len(vis)
            row = {
                "pc_wall_time_ns": wall_ns,
                "pc_perf_counter_ns": perf_ns,
                "basler_device_timestamp": dev_ts,   # diagnostic only
                "elapsed_time_s": elapsed,
                "visible_count": n_vis,
                "visible_ids": ",".join(map(str, sorted(vis))) if vis else "N/A",
                "n_corners_used": 4 * n_vis,
                "frame_brightness": float(np.mean(und)),
                "filter_enabled": int(USE_ONLINE_FILTER),
            }

            sol = dict(ok=False)
            if n_vis >= 1:
                O = np.vstack(obj_pts)
                I = np.vstack(img_pts)
                sol = solve_board(O, I, newK, R_prev)

            if sol["ok"]:
                R = sol["R"]
                if R_prev is not None and geodesic_deg(R_prev, R) > 20.0:
                    n_flips += 1
                    row["branch_flip_flag"] = 1
                else:
                    row["branch_flip_flag"] = 0
                R_prev = R

                if R_ref is None:
                    ref_pool.append(R)
                    if len(ref_pool) >= REF_MIN_FRAMES:
                        R_ref = chordal_mean_rotation(ref_pool)
                        spread = [geodesic_deg(R_ref, X) for X in ref_pool]
                        print(f"  zero reference established from "
                              f"{len(ref_pool)} frames; scatter about the "
                              f"reference: mean {np.mean(spread):.4f} deg, "
                              f"max {np.max(spread):.4f} deg")

                if R_ref is not None:
                    Rrel = R_ref.T @ R
                    rr, pp, yy = euler_xyz_deg(Rrel)
                    rvec = cv2.Rodrigues(Rrel)[0].ravel()
                    qw, qx, qy, qz = rot_to_quat(Rrel)
                    row.update(camera_roll_rel_deg=rr,
                               camera_pitch_rel_deg=pp,
                               camera_yaw_rel_deg=yy,
                               rel_rvec_x=float(rvec[0]),
                               rel_rvec_y=float(rvec[1]),
                               rel_rvec_z=float(rvec[2]),
                               rel_quat_w=qw, rel_quat_x=qx,
                               rel_quat_y=qy, rel_quat_z=qz,
                               rel_total_angle_deg=float(
                                   np.degrees(np.linalg.norm(rvec))))
                ar, ap, ay = euler_xyz_deg(R)
                row.update(board_roll_abs_deg=ar, board_pitch_abs_deg=ap,
                           board_yaw_abs_deg=ay,
                           board_tx_m=float(sol["t"][0]),
                           board_ty_m=float(sol["t"][1]),
                           board_tz_m=float(sol["t"][2]))

            row.update(board_reproj_rms_px=sol.get("reproj_px", np.nan),
                       candidate_other_reproj_px=sol.get("reproj_other_px", np.nan),
                       n_pose_candidates=sol.get("n_candidates", 0),
                       n_rejected_negative_depth=sol.get("n_rejected_depth", 0),
                       candidate_separation_deg=sol.get("candidate_geodesic_deg",
                                                        np.nan),
                       pose_ambiguous=int(bool(sol.get("ambiguous", False))),
                       selected_by=sol.get("selected_by", "none"),
                       pose_ok=int(bool(sol.get("ok", False))),
                       reproj_over_limit=int(
                           sol.get("reproj_px", 0) > MAX_BOARD_REPROJ_PX
                           if sol.get("ok") else 0))

            for tid in EXPECTED_TAG_IDS:
                if tid in per_tag:
                    d = per_tag[tid]
                    row[f"tag_{tid}_visible"] = 1
                    row[f"tag_{tid}_decision_margin"] = d["dm"]
                    row[f"tag_{tid}_hamming"] = d["hamming"]
                    row[f"tag_{tid}_cx_px"] = d["cx"]
                    row[f"tag_{tid}_cy_px"] = d["cy"]
                    for ci in range(4):
                        row[f"tag_{tid}_c{ci}_x_px"] = float(d["corners"][ci, 0])
                        row[f"tag_{tid}_c{ci}_y_px"] = float(d["corners"][ci, 1])
                    if args.per_tag_diag:
                        e1, e2 = solve_tag_square(None, d["corners"], newK, size)
                        row[f"tag_{tid}_square_reproj_best_px"] = e1
                        row[f"tag_{tid}_square_reproj_worst_px"] = e2
                else:
                    row[f"tag_{tid}_visible"] = 0

            rows.append(row)

            if elapsed - last_status >= STATUS_PERIOD_S:
                n = len(rows)
                fps = ((n - last_n) / (elapsed - last_t)) if last_t > 0 else n / max(elapsed, 1e-6)
                rr = row.get("camera_roll_rel_deg", float("nan"))
                pp = row.get("camera_pitch_rel_deg", float("nan"))
                print(f"elapsed={elapsed:7.1f} s | visible={sorted(vis)} | "
                      f"FPS={fps:5.2f} | reproj={row['board_reproj_rms_px']:.3f} px | "
                      f"roll={rr:+7.3f} pitch={pp:+7.3f} | flips={n_flips}")
                last_status, last_n, last_t = elapsed, n, elapsed
    except KeyboardInterrupt:
        print("\nInterrupted by operator.")
    finally:
        if cam.IsGrabbing():
            cam.StopGrabbing()
        cam.Close()

    df = pd.DataFrame(rows)
    meta = dict(script="basler_apriltag_pitch_roll_FINAL.py", mode=mode,
                run=args.run, axis=args.axis,
                requested_duration_s=args.duration,
                n_frames=int(len(df)),
                width=int(w), height=int(h),
                exposure_us=EXPOSURE_US, gain_db=GAIN_DB,
                tag_family=TAG_FAMILY,
                tag_black_square_size_m=size,
                expected_tag_ids=EXPECTED_TAG_IDS,
                filter_enabled=int(USE_ONLINE_FILTER),
                save_video=SAVE_VIDEO, draw_overlays=DRAW_OVERLAYS,
                solver="solvePnPGeneric SOLVEPNP_IPPE on the rigid multi-tag board",
                ambiguity_ratio=AMBIGUITY_RATIO,
                ref_min_frames=REF_MIN_FRAMES,
                n_branch_flips=int(n_flips),
                camera_matrix=CAMERA_MATRIX.tolist(),
                dist_coeffs=DIST_COEFFS.tolist(),
                board_geometry=geom,
                start_wall_time_ns=int(start_wall),
                start_perf_counter_ns=int(start_perf),
                reference_established=bool(R_ref is not None),
                corner_order_self_check=sc_report,
                euler_convention="intrinsic X-Y-Z, relative rotation "
                                 "R_rel = R_ref^T * R_board")
    if len(df) > 1:
        dur = df["elapsed_time_s"].iloc[-1] - df["elapsed_time_s"].iloc[0]
        meta["measured_fps"] = float((len(df) - 1) / dur) if dur > 0 else float("nan")
    return df, meta


# =============================================================================
# MODES
# =============================================================================
def stat(v):
    v = np.asarray(v, float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return dict(mean=np.nan, sd=np.nan, ptp=np.nan, n=0)
    return dict(mean=float(np.mean(v)),
                sd=float(np.std(v, ddof=1)) if v.size > 1 else np.nan,
                ptp=float(np.ptp(v)), n=int(v.size))


def mode_validate(args, geom, obj_by_tag):
    df, meta = run_capture(args, geom, obj_by_tag, "validate")
    if df is None:
        return 1
    os.makedirs(args.out, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    p = os.path.join(args.out, f"validate_{stamp}.csv")
    df.to_csv(p, index=False)
    with open(os.path.join(args.out, f"validate_{stamp}_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    print("\n" + "=" * 72)
    print("STATIONARY VALIDATION RESULT")
    print("=" * 72)
    if not meta["reference_established"]:
        print(f"  Zero reference was never established "
              f"({len(df)} frames, need {REF_MIN_FRAMES} with a valid pose).")
        print("\nNO-GO FOR PITCH/ROLL ACQUISITION")
        print("  The board pose could not be solved on enough frames. Check tag "
              "visibility, focus, exposure and the board geometry file.")
        return 2

    d = df[df["camera_roll_rel_deg"].notna()]
    res = {}
    for ax in ("roll", "pitch", "yaw"):
        s = stat(d[f"camera_{ax}_rel_deg"])
        res[ax] = s
        print(f"  {ax.capitalize():6s}  mean {s['mean']:+9.4f} deg   "
              f"SD {s['sd']:8.4f} deg   peak-to-peak {s['ptp']:8.4f} deg")
    rp = stat(d["board_reproj_rms_px"])
    print(f"\n  Board reprojection error: mean {rp['mean']:.4f} px, "
          f"SD {rp['sd']:.4f} px, max {np.nanmax(d['board_reproj_rms_px']):.4f} px")
    print(f"  Pose-branch flips (>20 deg jump): {meta['n_branch_flips']}")
    amb = int(d["pose_ambiguous"].sum())
    print(f"  Frames where the two IPPE branches were within "
          f"{AMBIGUITY_RATIO}x: {amb} / {len(d)} ({100*amb/max(len(d),1):.2f} %)")
    sep = stat(d["candidate_separation_deg"])

    # ---- [FIX] conditioning block -----------------------------------------
    med_sep = float(np.nanmedian(d["candidate_separation_deg"]))
    med_rp = float(np.nanmedian(d["board_reproj_rms_px"]))
    ratio = (d["candidate_other_reproj_px"].to_numpy(float)
             / np.maximum(d["board_reproj_rms_px"].to_numpy(float), 1e-9))
    ratio = ratio[np.isfinite(ratio)]
    med_ratio = float(np.nanmedian(ratio)) if ratio.size else float("nan")
    vis = {}
    for tid in EXPECTED_TAG_IDS:
        c = f"tag_{tid}_visible"
        if c in df:
            vis[tid] = 100.0 * float(df[c].mean())
    worst_vis = min(vis.values()) if vis else float("nan")

    print("\n  --- POSE CONDITIONING (how far this pose is from the planar "
          "ambiguity) ---")
    print(f"  Median IPPE candidate separation : {med_sep:8.3f} deg "
          f"(need >= {GO_MIN_CAND_SEP_DEG:.0f})")
    print(f"  Median second/selected reproj    : {med_ratio:8.3f} x   "
          f"(need >= {GO_MIN_REPROJ_RATIO:.1f})")
    print(f"  Ambiguous frames                 : {amb:8d}     "
          f"(need <= {GO_MAX_AMBIGUOUS})")
    print(f"  Median board reprojection        : {med_rp:8.4f} px  "
          f"(need <= {GO_MAX_REPROJ_PX:.1f})")
    print(f"  Worst per-tag visibility         : {worst_vis:8.2f} %   "
          f"(need >= {GO_MIN_TAG_VIS_PCT:.0f})")
    print(f"  Mean candidate separation        : {sep['mean']:8.3f} deg")
    # -----------------------------------------------------------------------

    for tid in EXPECTED_TAG_IDS:
        c = f"tag_{tid}_visible"
        if c in df:
            print(f"  Tag {tid} visibility: {vis[tid]:.2f} %")
    print(f"  Frames with a valid board pose: "
          f"{100*df['pose_ok'].mean():.2f} %   ({len(df)} frames, "
          f"{meta.get('measured_fps', float('nan')):.2f} fps)")

    # the two out-of-plane axes are the ones that matter here
    worst_sd = max(res["roll"]["sd"], res["pitch"]["sd"])
    worst_ptp = max(res["roll"]["ptp"], res["pitch"]["ptp"])
    fails = []
    if not np.isfinite(worst_sd) or worst_sd > GO_SD_DEG:
        fails.append(f"out-of-plane SD {worst_sd:.4f} deg exceeds "
                     f"{GO_SD_DEG:.2f} deg")
    if not np.isfinite(worst_ptp) or worst_ptp > GO_PTP_DEG:
        fails.append(f"out-of-plane peak-to-peak {worst_ptp:.4f} deg exceeds "
                     f"{GO_PTP_DEG:.2f} deg")
    if meta["n_branch_flips"] > GO_MAX_FLIPS:
        fails.append(f"{meta['n_branch_flips']} unresolved pose-branch flips")
    # ---- [FIX] conditioning criteria are now GO/NO-GO, not commentary -----
    if not np.isfinite(med_sep) or med_sep < GO_MIN_CAND_SEP_DEG:
        fails.append(f"median IPPE candidate separation {med_sep:.3f} deg is "
                     f"below {GO_MIN_CAND_SEP_DEG:.0f} deg -- the pose is too "
                     f"close to the planar ambiguity")
    if not np.isfinite(med_ratio) or med_ratio < GO_MIN_REPROJ_RATIO:
        fails.append(f"median second/selected reprojection ratio "
                     f"{med_ratio:.3f} is below {GO_MIN_REPROJ_RATIO:.1f} -- "
                     f"the wrong branch is nearly as good a fit")
    if amb > GO_MAX_AMBIGUOUS:
        fails.append(f"{amb} ambiguous frames (limit {GO_MAX_AMBIGUOUS})")
    if not np.isfinite(med_rp) or med_rp > GO_MAX_REPROJ_PX:
        fails.append(f"median board reprojection {med_rp:.4f} px exceeds "
                     f"{GO_MAX_REPROJ_PX:.1f} px")
    if not np.isfinite(worst_vis) or worst_vis < GO_MIN_TAG_VIS_PCT:
        fails.append(f"worst per-tag visibility {worst_vis:.2f} % is below "
                     f"{GO_MIN_TAG_VIS_PCT:.0f} %")

    print("\n" + "-" * 72)
    if not fails:
        print("GO. Stationary behaviour AND pose conditioning meet the "
              "criteria:")
        print(f"     SD <= {GO_SD_DEG} deg, peak-to-peak <= {GO_PTP_DEG} deg, "
              f"no branch flips,")
        print(f"     candidate separation >= {GO_MIN_CAND_SEP_DEG:.0f} deg, "
              f"reproj ratio >= {GO_MIN_REPROJ_RATIO:.1f}, "
              f"0 ambiguous frames,")
        print(f"     board reprojection <= {GO_MAX_REPROJ_PX:.1f} px, "
              f"tag visibility {GO_MIN_TAG_VIS_PCT:.0f} %.")
        print(f"Saved {p}")
        return 0

    print("NO-GO FOR PITCH/ROLL ACQUISITION")
    for f_ in fails:
        print("  - " + f_)
    print("\nMost likely causes, in the order worth checking:")
    if med_sep < GO_MIN_CAND_SEP_DEG or med_ratio < GO_MIN_REPROJ_RATIO:
        print("  * The board is too close to fronto-parallel, so the two "
              "planar pose branches are nearly degenerate. This is the "
              "failure mode that invalidated the first AXIS_A run: it passed "
              "the old stationary checks and still produced 3.5 deg errors. "
              "If the camera cannot be moved, command a larger PREBIAS so the "
              "whole sweep stays away from fronto-parallel. This is the "
              "first thing to fix.")
    if rp["mean"] > 1.0:
        print(f"  * Board reprojection RMS is {rp['mean']:.3f} px. Above about "
              "1 px the entered board geometry is probably wrong. Re-measure, "
              "or run --mode calibrate-board and compare.")
    print("  * Check focus, exposure, and that all four tags are fully in view "
          "and not motion-blurred.")
    print(f"\nSaved {p}")
    return 2


def mode_calibrate_board(args, tag_size_m):
    """Automatic rectangular-board calibration.

    Collects many stationary frames, solves each tag individually with
    IPPE_SQUARE (legitimate: it IS a single square), infers the rectangle,
    verifies it, then writes board_geometry.json. Nothing is saved unless
    every check passes.
    """
    _require_hardware()
    cam, conv = open_camera()
    w, h, newK, m1, m2 = build_undistort(cam)
    fx, fy, cx, cy = newK[0, 0], newK[1, 1], newK[0, 2], newK[1, 2]
    det = make_detector()

    print("=" * 72)
    print("BOARD CALIBRATION  (automatic rectangular-layout inference)")
    print("=" * 72)
    print(f"  tag black-square size in use : {tag_size_m*1000:.2f} mm "
          f"({tag_size_m:.5f} m)")
    print(f"  duration                     : {args.duration:.0f} s")
    print(f"  expected tag ids             : {EXPECTED_TAG_IDS}")
    print(f"  reference tag                : "
          f"{args.ref_tag if args.ref_tag is not None else min(EXPECTED_TAG_IDS)}"
          f"  (its printed frame becomes the board frame, rotation_deg = 0)")
    print("\n  KEEP THE PLATFORM COMPLETELY STATIONARY for the whole capture.\n")

    if not args.skip_preflight:
        def gd():
            gr = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if not gr.GrabSucceeded():
                gr.Release()
                return None, None, None
            frame = conv.Convert(gr).GetArray()
            gr.Release()
            und = cv2.remap(frame, m1, m2, cv2.INTER_LINEAR)
            return und, det.detect(und, estimate_tag_pose=False), None
        if not preflight(gd):
            cam.StopGrabbing()
            cam.Close()
            print("Aborted at pre-flight. Nothing was written.")
            return 1

    frames, pix = [], {t: [] for t in EXPECTED_TAG_IDS}
    t0 = time.time()
    last = 0.0
    try:
        while cam.IsGrabbing() and (time.time() - t0) < args.duration:
            gr = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if not gr.GrabSucceeded():
                gr.Release()
                continue
            frame = conv.Convert(gr).GetArray()
            gr.Release()
            und = cv2.remap(frame, m1, m2, cv2.INTER_LINEAR)
            tags = det.detect(und, estimate_tag_pose=True,
                              camera_params=[fx, fy, cx, cy],
                              tag_size=tag_size_m)
            fr = {}
            for tg in tags:
                tid = int(tg.tag_id)
                if tid not in EXPECTED_TAG_IDS or not hasattr(tg, "pose_R"):
                    continue
                c = np.asarray(tg.corners, float)
                fr[tid] = {"R": np.asarray(tg.pose_R, float),
                           "t": np.asarray(tg.pose_t, float).ravel(),
                           "corners": c}
                pix[tid].append(c.mean(axis=0))
            frames.append(fr)
            el = time.time() - t0
            if el - last >= 1.0:
                print(f"  elapsed={el:5.1f} s | frames={len(frames)} | "
                      f"this frame saw {sorted(fr)}")
                last = el
    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        if cam.IsGrabbing():
            cam.StopGrabbing()
        cam.Close()

    ok, geom, rep = infer_rectangular_board(
        frames, tag_size_m, EXPECTED_TAG_IDS, args.ref_tag,
        camera_matrix=newK)

    print("\n  Frames grabbed: " + str(rep.get("n_frames_total", 0)))
    for t in EXPECTED_TAG_IDS:
        if "detect_pct" in rep:
            n = len(pix[t])
            print(f"    tag {t}: detected in {rep['detect_pct'][t]:6.2f} % "
                  f"of frames" + (f", mean image position "
                                  f"({np.mean(pix[t], axis=0)[0]:7.1f}, "
                                  f"{np.mean(pix[t], axis=0)[1]:7.1f}) px"
                                  if n else ""))

    print("\n  CHECKS")
    for status, msg in rep.get("checks", []):
        print(f"    [{status}] {msg}")

    if not ok:
        print("\n" + "=" * 72)
        print("BOARD CALIBRATION NO-GO")
        print("=" * 72)
        for status, msg in rep.get("checks", []):
            if status == "FAIL":
                print(f"  reason: {msg}")
        print("  Nothing was written. Fix the cause and re-run, or measure "
              "the board by hand and fill in board_geometry.json.")
        return 2

    by = {v: k for k, v in rep["corner_assignment"].items()}
    print("\n  Detected rectangular board")
    print("  --------------------------")
    print(f"  Top-left     : Tag {by['top-left']}")
    print(f"  Top-right    : Tag {by['top-right']}")
    print(f"  Bottom-left  : Tag {by['bottom-left']}")
    print(f"  Bottom-right : Tag {by['bottom-right']}")
    print()
    print(f"  Horizontal center spacing = {rep['W_mm']:.3f} mm")
    print(f"  Vertical center spacing   = {rep['H_mm']:.3f} mm")
    print()
    for t in EXPECTED_TAG_IDS:
        print(f"  Tag {t} rotation = {rep['rotation_snapped_deg'][t]:.0f} deg   "
              f"(measured {rep['rotation_raw_deg'][t]:+8.3f} deg, residual "
              f"{rep['rotation_residual_deg'][t]:+6.3f} deg)")
    print()
    print(f"  W SD = {rep['W_sd_mm']:.4f} mm   "
          f"(opposite-side difference {rep['w_opposite_diff_mm']:.4f} mm)")
    print(f"  H SD = {rep['H_sd_mm']:.4f} mm   "
          f"(opposite-side difference {rep['h_opposite_diff_mm']:.4f} mm)")
    print(f"  frames used = {rep['n_frames_used']} / {rep['n_frames_total']} "
          f"({rep['used_pct']:.1f} %), {rep['frames_rejected']} rejected as "
          f"outliers")
    print(f"  worst out-of-plane residual = "
          f"{max(rep['out_of_plane_mm'].values()):.3f} mm")
    print(f"  worst corner fit residual   = "
          f"{max(rep['fit_residual_mm'].values()):.3f} mm")

    print("\n  Board-frame layout (the frame of reference tag "
          f"{rep['reference_tag']}; +X right, +Y up IN THAT FRAME):")
    print(f"      (-W/2,+H/2) Tag {by['top-left']}  ---------  "
          f"Tag {by['top-right']} (+W/2,+H/2)")
    print(f"                    |                      |")
    print(f"      (-W/2,-H/2) Tag {by['bottom-left']}  ---------  "
          f"Tag {by['bottom-right']} (-)(+W/2,-H/2)")
    print("    Match these to the image positions printed above; the board "
          "frame need not")
    print("    look 'upright' on screen, and it does not have to: "
          "R_rel = R_ref^T R_board")
    print("    cancels any constant board or camera orientation.")

    # ---- end-to-end verification with the derived geometry ---------------
    obj = build_object_points(geom)
    full = [f for f in frames if all(t in f for t in EXPECTED_TAG_IDS)]
    errs = []
    Rp = None
    for f in full:
        O = np.vstack([obj[t] for t in sorted(f)])
        I = np.vstack([f[t]["corners"] for t in sorted(f)])
        sol = solve_board(O, I, newK, Rp)
        if sol["ok"]:
            Rp = sol["R"]
            errs.append(sol["reproj_px"])
    med = float(np.median(errs)) if errs else float("inf")
    print(f"\n  End-to-end check: board reprojection with the DERIVED "
          f"geometry = {med:.4f} px (median of {len(errs)} frames)")
    if med > CAL_MAX_REPROJ_PX:
        print("\n" + "=" * 72)
        print("BOARD CALIBRATION NO-GO")
        print("=" * 72)
        print(f"  reason: the derived geometry reprojects at {med:.3f} px, "
              f"above the {CAL_MAX_REPROJ_PX:.1f} px limit. The rectangle "
              f"model does not describe this board well enough.")
        print("  Nothing was written.")
        return 2

    sc_frames = [{t: f[t]["corners"] for t in f} for f in full[:60]]
    sc_ok, sc_rep = corner_order_self_check(sc_frames, obj, newK, verbose=True)
    if not sc_ok:
        print("\n" + "=" * 72)
        print("BOARD CALIBRATION NO-GO")
        print("=" * 72)
        print(f"  reason: the corner-order self-check preferred "
              f"'{sc_rep.get('best')}' over the documented order with this "
              f"geometry.")
        print("  Nothing was written.")
        return 2

    geom["calibration_report"] = {
        k: v for k, v in rep.items() if k != "checks"}
    geom["calibration_report"]["end_to_end_reproj_px"] = med
    geom["calibration_report"]["corner_order_self_check"] = sc_rep
    geom["calibration_report"]["timestamp_utc"] = time.strftime(
        "%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    geom["calibration_report"]["mean_image_position_px"] = {
        str(t): [float(q) for q in np.mean(pix[t], axis=0)]
        for t in EXPECTED_TAG_IDS if pix[t]}

    if os.path.exists(args.geometry):
        bak = args.geometry + time.strftime(".bak_%Y%m%d_%H%M%S")
        os.replace(args.geometry, bak)
        print(f"\n  existing {args.geometry} moved to {bak}")
    with open(args.geometry, "w") as f:
        json.dump(geom, f, indent=2)

    print("\n" + "=" * 72)
    print("BOARD CALIBRATION GO")
    print("=" * 72)
    print(f"  wrote {args.geometry}")
    print(f'  geometry_source = "camera_derived_calibration"')
    print()
    print("  " + "!" * 68)
    print("  WARNING:")
    print("  Camera-derived board geometry is being used for "
          "acquisition/development.")
    print("  For final publication-quality independent validation, "
          "mechanically measure")
    print("  W and H with a ruler/caliper and replace or verify these values.")
    print("  " + "!" * 68)
    print()
    print("  NEXT STEP:")
    print()
    print("  python3 basler_apriltag_pitch_roll_FINAL.py \\")
    print("      --mode validate \\")
    print("      --duration 60 \\")
    print(f"      --geometry {args.geometry}")
    print()
    return 0


def mode_acquire(args, geom, obj_by_tag):
    if args.run is None or args.axis is None:
        sys.exit("--mode acquire needs --run and --axis")
    run_dir = os.path.join(args.out, f"run{args.run:02d}_{args.axis}")
    os.makedirs(run_dir, exist_ok=True)
    stamp = f"run{args.run:02d}_{args.axis}_" + time.strftime("%Y%m%d_%H%M%S")
    csv_p = os.path.join(run_dir, f"camera_{stamp}.csv")
    meta_p = os.path.join(run_dir, f"camera_{stamp}_meta.json")

    print("=" * 72)
    print(f"PITCH/ROLL ACQUISITION   run {args.run:02d}  {args.axis}")
    print(f"  duration : {args.duration:.1f} s")
    print(f"  folder   : {run_dir}")
    print(f"  solver   : rigid {len(EXPECTED_TAG_IDS)}-tag board, "
          f"SOLVEPNP_IPPE, ambiguity ratio {AMBIGUITY_RATIO}")
    print(f"  filter   : {'ON' if USE_ONLINE_FILTER else 'OFF (raw)'}   "
          f"video {SAVE_VIDEO}   overlays {DRAW_OVERLAYS}")
    print("=" * 72)

    df, meta = run_capture(args, geom, obj_by_tag, "acquire")
    if df is None:
        return 1
    df.to_csv(csv_p, index=False)
    with open(meta_p, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"\nSaved {csv_p}  ({len(df)} rows, {len(df.columns)} columns)")
    print(f"Saved {meta_p}")
    if not meta["reference_established"]:
        print("\n!! WARNING: the zero reference was never established. "
              "Relative attitude columns are empty. Do not use this run.")
        return 2
    print(f"Branch flips: {meta['n_branch_flips']}   "
          f"valid board pose on {100*df['pose_ok'].mean():.2f} % of frames")
    return 0


# =============================================================================
def main():
    ap = argparse.ArgumentParser(
        description="Rigid multi-tag board attitude acquisition (pitch/roll).")
    ap.add_argument("--mode", required=True,
                    choices=["validate", "acquire", "calibrate-board"])
    ap.add_argument("--run", type=int, default=None, choices=range(1, 21),
                    metavar="1-20")
    ap.add_argument("--axis", default=None, choices=["AXIS_A", "AXIS_B"])
    ap.add_argument("--duration", type=float, default=None,
                    help="seconds (default 20 for calibrate-board, "
                         "60 for validate, required for acquire)")
    ap.add_argument("--out", default="runs_pr")
    ap.add_argument("--geometry", default="board_geometry.json")
    ap.add_argument("--tag-size", type=float, default=None,
                    help="tag black-square edge in metres "
                         f"(default {DEFAULT_TAG_SIZE_M})")
    ap.add_argument("--ref-tag", type=int, default=None,
                    help="calibrate-board: tag whose printed frame becomes "
                         "the board frame (default: lowest expected id)")
    ap.add_argument("--per-tag-diag", action="store_true",
                    help="also log per-tag IPPE_SQUARE reprojection errors")
    ap.add_argument("--skip-preflight", action="store_true")
    args = ap.parse_args()

    if args.duration is None:
        args.duration = 20.0 if args.mode == "calibrate-board" else 60.0

    # ---- calibrate-board does NOT need a filled geometry file ------------
    if args.mode == "calibrate-board":
        size = args.tag_size
        if size is None and os.path.exists(args.geometry):
            try:
                v = json.load(open(args.geometry)).get(
                    "tag_black_square_size_m")
                if isinstance(v, (int, float)) and v > 0:
                    size = float(v)
                    print(f"tag size taken from {args.geometry}")
            except Exception:
                pass
        if size is None:
            size = DEFAULT_TAG_SIZE_M
            print(f"tag size: built-in default (override with --tag-size)")
        sys.exit(mode_calibrate_board(args, float(size)))

    geom = load_geometry(args.geometry)
    if args.tag_size is not None:
        geom["tag_black_square_size_m"] = float(args.tag_size)
    obj_by_tag = build_object_points(geom)

    src = geom.get("geometry_source", "manually_measured")
    print(f"Board geometry from {args.geometry}  [geometry_source = {src}]")
    print(f"  tag size {geom['tag_black_square_size_m']*1000:.2f} mm, centres "
          + ", ".join(f"{t}=({geom['tags'][str(t)]['center_x_mm']:+.2f},"
                      f"{geom['tags'][str(t)]['center_y_mm']:+.2f})mm@"
                      f"{geom['tags'][str(t)]['rotation_deg']:.0f}deg"
                      for t in EXPECTED_TAG_IDS))
    if src == "camera_derived_calibration":
        print("  WARNING: camera-derived board geometry is being used for "
              "acquisition/development.")
        print("           For final publication-quality independent "
              "validation, mechanically measure")
        print("           W and H with a ruler/caliper and replace or verify "
              "these values.")

    if args.mode == "validate":
        rc = mode_validate(args, geom, obj_by_tag)
        if rc == 0:
            print("\n  NEXT STEP — AXIS_A acquisition:")
            print()
            print("  python3 basler_apriltag_pitch_roll_FINAL.py \\")
            print("      --mode acquire \\")
            print("      --run 1 --axis AXIS_A \\")
            print("      --duration 560 \\")
            print(f"      --geometry {args.geometry} \\")
            print(f"      --out {args.out}")
            print()
            print("  then, in a second terminal:")
            print()
            print("  python3 motor_serial_logger_pitch_roll_FINAL.py \\")
            print("      --port /dev/ttyACM0 --run 1 --axis AXIS_A "
                  f"--out {args.out}")
            print()
        sys.exit(rc)
    sys.exit(mode_acquire(args, geom, obj_by_tag))


if __name__ == "__main__":
    main()
