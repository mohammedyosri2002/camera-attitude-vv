"""
analysis/common/io.py
=====================
Repository-relative file discovery and raw-log loading.

CREATED POST-CAMPAIGN. This module was written after the experimental campaign to
reproduce the submitted results from the archived raw data. It was NOT used during
acquisition. The code that ran on the testbed is preserved unmodified in src/original/.

All paths are resolved relative to the repository root, which is located by walking up
from this file. No absolute or user-specific path appears anywhere in this repository.
"""
from __future__ import annotations

import glob
import json
import os

import cv2
import numpy as np
import pandas as pd

# --------------------------------------------------------------------------------------
# Repository-relative paths
# --------------------------------------------------------------------------------------
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DATA_RAW = os.path.join(REPO_ROOT, "data", "raw")
CONFIG = os.path.join(REPO_ROOT, "config")
RESULTS = os.path.join(REPO_ROOT, "results")

# Canonical campaign map. Keys are PHYSICAL board-frame axes, not the original folder
# labels. The original labels are preserved in each run's RUN_INFO.json for provenance.
RUNS = {
    "yaw/run01_CW":            dict(axis="yaw",     accepted=True,  headline=False),
    "yaw/run02_CCW":           dict(axis="yaw",     accepted=True,  headline=False),
    "board_x/axis_a":          dict(axis="board_x", accepted=True,  headline=False),
    "board_x/axis_b_run01":    dict(axis="board_x", accepted=True,  headline=True),
    "board_x/axis_b_run04":    dict(axis="board_x", accepted=True,  headline=False),
    "board_y/run05":           dict(axis="board_y", accepted=True,  headline=True),
}


def run_dir(key: str) -> str:
    p = os.path.join(DATA_RAW, key)
    if not os.path.isdir(p):
        raise FileNotFoundError(f"run directory missing: {p}")
    return p


def run_info(key: str) -> dict:
    p = os.path.join(run_dir(key), "RUN_INFO.json")
    if not os.path.exists(p):
        raise FileNotFoundError(f"RUN_INFO.json missing for {key}")
    return json.load(open(p))


def config_path(name: str) -> str:
    return os.path.join(CONFIG, name)


def load_json(p: str) -> dict:
    with open(p) as f:
        return json.load(f)


# --------------------------------------------------------------------------------------
# Raw-log loaders
# --------------------------------------------------------------------------------------
def load_camera(key: str):
    """Camera CSV + its acquisition metadata JSON."""
    d = run_dir(key)
    csvs = [p for p in sorted(glob.glob(os.path.join(d, "camera_*.csv")))
            if not p.endswith("_meta.json")]
    if not csvs:
        raise FileNotFoundError(f"no camera_*.csv in {d}")
    metas = sorted(glob.glob(os.path.join(d, "camera_*_meta.json")))
    meta = load_json(metas[0]) if metas else {}
    return pd.read_csv(csvs[0]), meta, os.path.basename(csvs[0])


def load_motor(key: str):
    """Motor event log + the clock-fit JSON produced by the serial logger at run time."""
    d = run_dir(key)
    ev = pd.read_csv(os.path.join(d, "motor_events.csv"))
    fit = load_json(os.path.join(d, "motor_clockfit.json"))
    return ev, fit


def load_imu(key: str):
    """Load the IMU record named in RUN_INFO.json.

    The IMU file is NOT chosen by filename convention. Several run folders contain more
    than one Teensy record (stationary recordings, aborted recordings, manual-handling
    recordings). The correct file was identified from the DATA -- gyro motion content
    matching the commanded profile -- and that decision is recorded in RUN_INFO.json so
    that reproduction is deterministic. identify_imu_file() below re-derives the same
    decision from the data for anyone who wants to check it.
    """
    info = run_info(key)
    p = os.path.join(run_dir(key), info["imu_file"])
    if not os.path.exists(p):
        raise FileNotFoundError(f"IMU file named in RUN_INFO.json not found: {p}")
    return pd.read_csv(p, comment="#"), info["imu_file"]


def identify_imu_file(key: str, motor_span_s: float) -> list:
    """Re-derive the IMU-file choice from the data alone.

    Returns a ranked list of (filename, moving_fraction, peak_deviation_dps, n, span_s).
    The correct record is the one whose gyro shows commanded-profile motion over a span
    at least as long as the motor window. Stationary or aborted records show essentially
    no motion; manual-handling records show very large, short-lived excursions.
    """
    d = run_dir(key)
    cands = []
    for p in sorted(glob.glob(os.path.join(d, "*.CSV")) + glob.glob(os.path.join(d, "*.csv"))):
        b = os.path.basename(p)
        if b.startswith("camera_") or b.startswith("motor_"):
            continue
        try:
            df = pd.read_csv(p, comment="#")
            if "gyro_x_dps" not in df.columns or len(df) < 10:
                cands.append((b, 0.0, 0.0, len(df), 0.0))
                continue
            G = df[["gyro_x_dps", "gyro_y_dps", "gyro_z_dps"]].to_numpy(float)
            t = df["teensy_time_us"].to_numpy(float) * 1e-6
            mag = np.linalg.norm(G - np.median(G, axis=0), axis=1)
            cands.append((b, float((mag > 1.0).mean()), float(mag.max()),
                          int(len(df)), float(t[-1] - t[0])))
        except Exception:
            continue
    # a valid record must span the motor window and contain sustained commanded motion
    ok = [c for c in cands if c[4] >= motor_span_s and 0.02 < c[1] < 0.60]
    ok.sort(key=lambda c: -c[1])
    return ok + [c for c in cands if c not in ok]


def camera_intrinsics(key: str):
    """Intrinsics for the LOGGED CORNER COORDINATES of this run.

    CRITICAL. The acquisition program undistorts each frame before detection:

        newK, _ = cv2.getOptimalNewCameraMatrix(CAMERA_MATRIX, DIST_COEFFS, (w,h), 1, (w,h))
        m1, m2  = cv2.initUndistortRectifyMap(CAMERA_MATRIX, DIST_COEFFS, None, newK, ...)
        und     = cv2.remap(frame, m1, m2, cv2.INTER_LINEAR)
        tags    = detector.detect(und, ...)
        sol     = solve_board(O, I, newK, R_prev)

    The corner pixels in the camera CSV are therefore in UNDISTORTED image coordinates and
    the matrix that belongs with them is newK, NOT the raw camera_matrix stored in the
    metadata. Distortion is already removed, so zero coefficients are used with newK.

    Using the raw camera_matrix on undistorted corners is a 7.54 % focal-length error on
    this rig (fx 1443.25 vs 1334.37). It does not fail loudly: the pose still solves, board
    reprojection rises only from about 0.28 px to 0.37 px, and the recovered angles shift by
    a few tenths of a degree. analysis/board_x/audit_board_x_reference.py isolates it; with
    newK the re-solved attitude matches the logged acquisition attitude to 0.000000 deg.

    Returns (K_for_logged_corners, zero_distortion, metadata).
    """
    _, meta, _ = load_camera(key)
    K = np.array(meta["camera_matrix"], float)
    D = np.array(meta.get("dist_coeffs", [0, 0, 0, 0, 0]), float)
    w = int(meta["width"]); h = int(meta["height"])
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    return newK, np.zeros(5), meta
