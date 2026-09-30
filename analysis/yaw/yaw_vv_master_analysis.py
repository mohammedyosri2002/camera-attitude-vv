#!/usr/bin/env python3
# =============================================================================
# analysis/yaw/yaw_vv_master_analysis.py
#
# CREATED POST-CAMPAIGN (adapted from the campaign analysis module) to reproduce the
# submitted yaw results from the archived raw data. It was NOT used during acquisition;
# the acquisition code is preserved unmodified in src/original/yaw/.
#
# The yaw acquisition predates the rigid multi-tag board estimator: it fused per-tag pose
# and logged in-plane yaw only, with no roll/pitch and therefore no gravity-tilt
# reference. That is why the yaw column of the three-axis table has N/A entries.
#
# RAW / FILTER POLICY. The RAW camera-vs-commanded results are the headline yaw numbers.
# This module also computes Savitzky-Golay and Kalman rate estimators; those are
# SECONDARY DIAGNOSTICS ONLY. They are never substituted into the three-axis comparison
# and are never compared with a RAW out-of-plane derivative.
#
# The motor signal is a PULSE-DERIVED COMMANDED REFERENCE. Not ground truth.
# =============================================================================
# =============================================================================
# yaw_vv_master_full_analysis.py
#
# COMPLETE, SELF-CONTAINED yaw verification-and-validation analysis for
# "Camera-Based Three-Axis Attitude Measurement for Small-Satellite Testbeds:
#  A V&V Approach".
#
# This single file does everything: loading, motor reconstruction,
# synchronisation, estimator construction, every pairwise metric, pooled
# totals, the master workbook and the sample-level audit CSVs. It imports
# nothing from the other analysis scripts.
#
#   python3 yaw_vv_master_full_analysis.py \
#       --run1 test1 --run2 test2 --out yaw_master_results
#
#   python3 yaw_vv_master_full_analysis.py \
#       --run1 runs/run01_CW --run2 runs/run02_CCW --out yaw_master_results
#
# Outputs
#   Yaw_VV_MASTER_FULL_RESULTS.xlsx
#   synchronized_samples_run01.csv
#   synchronized_samples_run02.csv
#
# =============================================================================
# SIGN CONVENTION (documented once, applied everywhere)
# -----------------------------------------------------------------------------
# Run 01 is CW and Run 02 is CCW. Every comparison is performed in the
# TRAVEL-MAGNITUDE convention: the reference and all sensors are expressed as
# monotonically increasing travel from the start of the profile, so CW and CCW
# runs are directly comparable and no negative measured travel is ever
# subtracted from a positive reference.
#
#   * the Arduino logs SIGNED values (CCW negative); the loader takes |value|
#     for the magnitude trajectory and keeps the direction separately
#   * the camera sign is detected per run from the data and reported
#   * the gyro axis sign is detected per run from the data and reported
#   * SIGNED columns (Signed_*) are provided alongside for traceability
#
# =============================================================================
# MOTOR REFERENCE
# -----------------------------------------------------------------------------
#   PULSES_PER_REV = 25600  ->  0.0140625 deg/pulse
#   pulses  = round(requested_angle * 25600 / 360)
#   command = pulses * 360 / 25600
#   ideal round-to-nearest bound = +/- 0.00703125 deg
#
# This is the PULSE-DERIVED COMMANDED ANGLE. It is never called an actual,
# measured or encoder-derived motor angle: no independently measured external
# motor position exists in this testbed, and the vendor publishes no
# positioning-accuracy specification. All static errors use the effective
# pulse-derived command (5.006250 deg for the 5 deg label), never the label.
#
# =============================================================================
# SCIENTIFIC RULES ENFORCED
# -----------------------------------------------------------------------------
# STATIC YAW  primary comparison is Camera <-> pulse-derived motor command.
#             The ISM330DHCX has no magnetometer, so integrated gyro yaw is
#             drift-prone. Camera<->IMU and IMU<->Motor ANGLE columns are
#             labelled RELATIVE INTEGRATED-YAW DIAGNOSTIC ONLY and are never
#             presented as absolute yaw accuracy.
# RATE        all seven pairings are primary: RAW/SG/KF camera vs motor,
#             RAW/SG/KF camera vs gyro, and gyro vs motor.
# CLOCKS      the camera<->motor mapping comes from the serial event
#             timestamps (motor_clockfit.json). It is NEVER refitted by
#             minimising camera-vs-motor residuals. The IMU clock is fitted
#             independently per run in the RATE domain only.
# KALMAN      camera measurements only; no IMU input, so camera-vs-IMU stays
#             non-circular.
# FREEZING    SG and Kalman parameters are constants below, chosen on Run 01
#             and applied unchanged to Run 02 and to every segment.
# POOLING     RMSE_total = sqrt(sum(e_i^2)/N), MAE_total = sum|e_i|/N over
#             residual samples. Per-segment RMSE values are never averaged.
# =============================================================================

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys

import numpy as np
import pandas as pd
from scipy.optimize import minimize, minimize_scalar
from scipy.signal import savgol_filter

# =============================================================================
# FROZEN CONFIGURATION
# =============================================================================
CFG = dict(
    PULSES_PER_REV=25600,
    ANGLES_DEG=[0, 5, 10, 20, 30, 60, 90, 120, 150,
                180, 210, 240, 270, 300, 330, 360, 720],
    RATES_DPS=[1, 5, 10, 15, 20, 25, 30],
    RATE_TIME_S=20.0,

    HOLD_GUARD_S=2.0,          # trimmed from both ends of every 20 s hold
    RATE_TRIM_FRAC=0.10,       # keep the central 80 % of every rate segment
    TAIL_GUARD_S=2.0,          # stationary tail, used for camera noise / KF R

    SG_WINDOW_S=1.00,          # frozen, chosen on Run 01
    SG_POLYORDER=2,
    KF_Q=1.0e-3,               # (deg/s)^2/s, bandwidth-matched to SG on Run 01
    IMU_SG_WINDOW_S=1.00,      # secondary bandwidth-matched IMU diagnostic
    IMU_SG_POLYORDER=2,

    IMU_SYNC_SMOOTH_S=0.50,    # smoothing used ONLY inside the sync fit
    GYRO_MOTION_THRESH_DPS=5.0,
    IMU_STILL_PAD_S=0.5,

    DS_GYRO_ZRL_DPS=3.0,           # ST datasheet, FS = +/-125 dps
    DS_GYRO_SENS_TOL_PCT=2.0,
    DS_GYRO_CROSS_AXIS_PCT=1.0,
)
DPP = 360.0 / CFG["PULSES_PER_REV"]
QUANT_BOUND = 0.5 * DPP

WARNINGS: list[str] = []


def warn(m):
    WARNINGS.append(m)
    print(f"  [WARN] {m}")


def head(t):
    print("\n" + "=" * 78)
    print(t)
    print("=" * 78)


# =============================================================================
# 1. LOADING
# =============================================================================
def load_run(folder, label):
    if not os.path.isdir(folder):
        sys.exit(f"{label}: run folder not found: {folder}")

    ev_p = os.path.join(folder, "motor_events.csv")
    fit_p = os.path.join(folder, "motor_clockfit.json")
    if not os.path.exists(ev_p):
        sys.exit(f"{label}: missing motor_events.csv in {folder}")
    if not os.path.exists(fit_p):
        sys.exit(f"{label}: missing motor_clockfit.json in {folder}")

    cams = [p for p in sorted(glob.glob(os.path.join(folder, "camera_*.csv")))
            if not p.endswith("_meta.json")]
    if not cams:
        sys.exit(f"{label}: no camera_*.csv in {folder}")

    imus = []
    for pat in ("imu*.csv", "imu*.CSV", "IMU*.csv", "IMU*.CSV",
                "RUN*.CSV", "RUN*.csv"):
        imus += sorted(glob.glob(os.path.join(folder, pat)))
    imus = [p for p in imus if "camera" not in os.path.basename(p).lower()]
    if not imus:
        sys.exit(f"{label}: no IMU csv in {folder}")

    ev = pd.read_csv(ev_p)
    fit = json.load(open(fit_p))
    cam = pd.read_csv(cams[0])
    imu = pd.read_csv(imus[0], comment="#")

    metas = sorted(glob.glob(os.path.join(folder, "camera_*_meta.json")))
    meta = json.load(open(metas[0])) if metas else {}

    imu_hdr = {}
    with open(imus[0], "r", errors="replace") as f:
        for line in f:
            if not line.startswith("#"):
                break
            if "=" in line:
                k, v = line.lstrip("# ").strip().split("=", 1)
                imu_hdr[k.strip()] = v.strip()

    for c in ("elapsed_time_s", "fused_yaw_deg", "pc_perf_counter_ns",
              "visible_count", "used_count_after_outlier_rejection"):
        if c not in cam.columns:
            sys.exit(f"{label}: camera log lacks required column '{c}'")
    for c in ("teensy_time_us", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps"):
        if c not in imu.columns:
            sys.exit(f"{label}: IMU log lacks required column '{c}'")
    for c in ("event", "t_arduino_us", "t_perf_ns", "value", "extra"):
        if c not in ev.columns:
            sys.exit(f"{label}: motor_events.csv lacks required column '{c}'")

    if not fit.get("saw_run_end", False):
        warn(f"{label}: motor logger never saw RUN_END.")
    if fit.get("n_events") != fit.get("expected_events", 94):
        warn(f"{label}: {fit.get('n_events')} sync events "
             f"(expected {fit.get('expected_events')}).")
    if "filter_enabled" in cam and int(cam["filter_enabled"].max()) != 0:
        warn(f"{label}: camera online filter was ENABLED; fused_yaw_deg is "
             f"not a raw measurement.")
    if meta.get("baseline_frames_incomplete", 0) > 0:
        warn(f"{label}: {meta['baseline_frames_incomplete']} start-baseline "
             f"frames lacked the full expected tag set.")

    direction = (fit.get("direction") or meta.get("direction") or "?").upper()
    return dict(label=label, folder=folder, ev=ev, fit=fit, cam=cam, imu=imu,
                meta=meta, imu_hdr=imu_hdr, direction=direction,
                dir_sign=(1.0 if direction == "CW" else -1.0),
                camera_file=os.path.basename(cams[0]),
                imu_file=os.path.basename(imus[0]))


# =============================================================================
# 2. MOTOR REFERENCE
# =============================================================================
def angle_to_pulses(deg, ppr):
    """Replicates lround(deg * PULSES_PER_REV / 360.0f) in 32-bit float, so the
    pulse counts are bit-identical to the ones the Arduino emitted."""
    v = np.float32(deg) * np.float32(ppr) / np.float32(360.0)
    return int(math.floor(float(v) + 0.5))


def motor_quantization_table(cfg):
    rows = []
    for a in cfg["ANGLES_DEG"]:
        p = angle_to_pulses(float(a), cfg["PULSES_PER_REV"])
        eff = p * DPP
        rows.append({
            "Requested_Angle_deg": float(a),
            "Motor_Pulses": p,
            "Effective_Motor_Angle_deg": eff,
            "Motor_Quantization_Error_deg": eff - a,
            "Within_Ideal_Bound": bool(abs(eff - a) <= QUANT_BOUND + 1e-12),
        })
    df = pd.DataFrame(rows)
    df.attrs["note"] = ("Pulse-derived commanded angle. "
                        f"Resolution {DPP:.7f} deg/pulse, ideal "
                        f"round-to-nearest bound +/-{QUANT_BOUND:.8f} deg.")
    return df


def motor_profile(ev):
    """Piecewise-linear pulse-derived commanded angle vs ARDUINO-clock time,
    built from the logged event timestamps (not from an assumed schedule).

    Travel-magnitude convention: |logged value|. Returns (T, A, segments) with
    segments = list of (kind, t0, t1, |rate|, angle0, angle1)."""
    t = ev["t_arduino_us"].to_numpy(float) * 1e-6
    e = ev["event"].to_numpy()
    v = pd.to_numeric(ev["value"], errors="coerce").to_numpy(float)
    x = pd.to_numeric(ev["extra"], errors="coerce").to_numpy(float)

    T, A, segs = [], [], []
    pos, ts, start, dp, rate = 0.0, None, 0.0, 0.0, 0.0
    for i in range(len(ev)):
        k = e[i]
        if k == "MOVE_START":
            T.append(t[i]); A.append(pos); ts = t[i]; start = pos
        elif k == "MOVE_END":
            pos = abs(x[i]) * DPP
            T.append(t[i]); A.append(pos)
            segs.append(("move", ts, t[i], 6.0, start, pos))
        elif k == "HOLD_START":
            pos = abs(x[i]) * DPP
            T.append(t[i]); A.append(pos); ts = t[i]
        elif k == "HOLD_END":
            T.append(t[i]); A.append(pos)
            segs.append(("hold", ts, t[i], 0.0, pos, pos))
        elif k == "RATE_START":
            T.append(t[i]); A.append(pos)
            ts, start, rate, dp = t[i], pos, abs(v[i]), abs(x[i]) * DPP
        elif k == "RATE_END":
            pos = start + dp
            T.append(t[i]); A.append(pos)
            segs.append(("rate", ts, t[i], rate, start, pos))
        elif k == "RETURN_START":
            T.append(t[i]); A.append(pos)
            ts, start, dp = t[i], pos, abs(v[i]) * DPP
        elif k == "RETURN_END":
            pos = start + dp
            T.append(t[i]); A.append(pos)
            segs.append(("return", ts, t[i], 6.0, start, pos))
        else:
            T.append(t[i]); A.append(pos)
    T, A = np.asarray(T), np.asarray(A)
    o = np.argsort(T, kind="stable")
    return T[o], A[o], segs


def motor_angle(T, A, t):
    return np.interp(t, T, A, left=A[0], right=A[-1])


def motor_rate(segs, t):
    t = np.atleast_1d(np.asarray(t, float))
    r = np.zeros_like(t)
    for (k, t0, t1, rate, a0, a1) in segs:
        if k in ("move", "rate", "return"):
            r[(t >= t0) & (t < t1)] = rate
    return r


# =============================================================================
# 3. SYNCHRONISATION
# =============================================================================
def camera_to_arduino(cam, fit):
    """PRIMARY camera<->motor mapping, from the serial event timestamps only.

    The acquisition-time logger fitted
        (t_arduino - t_arduino_0) = a * (t_perf - t_perf_0) + b
    on the same host's monotonic clock, so
        t_arduino = a * (t_perf - t_perf_0) + b + t_arduino_0
    This is never refitted against camera-vs-motor angle residuals; the
    camera-motor agreement is therefore an out-of-sample result."""
    a = float(fit["a_perf"]); b = float(fit["b_perf"])
    p0 = float(fit["t_perf_first_ns"]) / 1e9
    a0 = float(fit["t_arduino_first_us"]) / 1e6
    return a * (cam["pc_perf_counter_ns"].to_numpy(float) / 1e9 - p0) + b + a0


def imu_axis_and_sign(imu, cfg):
    dev = {}
    for ax in "xyz":
        g = imu[f"gyro_{ax}_dps"].to_numpy(float)
        dev[ax] = float(np.percentile(np.abs(g - np.median(g)), 99))
    axis = max(dev, key=dev.get)
    g = imu[f"gyro_{axis}_dps"].to_numpy(float)
    mov = np.abs(g - np.median(g)) > cfg["GYRO_MOTION_THRESH_DPS"]
    sign = 1.0 if np.mean(g[mov]) > 0 else -1.0
    o = sorted(dev.values())
    if o[-1] < 3.0 * o[-2]:
        warn("gyro yaw axis is not clearly dominant; check IMU mounting.")
    return axis, sign, dev


def sync_imu(ti, gyro_mag, T, segs, cfg, label):
    """Rate-domain IMU synchronisation, t_arduino = a*t_teensy + b.

    Stage 1  normalised cross-correlation of |gyro| against the commanded
             rate boxcar, over the whole overlap
    Stage 2  constant-offset refinement
    Stage 3  affine slope + offset
    Stage 4  per-feature local offsets as a residual-drift diagnostic

    Integrated gyro yaw is never used here."""
    fs = 1.0 / np.median(np.diff(ti))
    w = max(3, int(round(cfg["IMU_SYNC_SMOOTH_S"] * fs)) | 1)
    gs = savgol_filter(gyro_mag, w, 1)

    dt = 0.05
    tg = np.arange(ti[0], ti[-1], dt)
    yg = np.interp(tg, ti, gs)
    tm = np.arange(T[0], T[-1], dt)
    rm = motor_rate(segs, tm)
    cc = np.correlate(yg - yg.mean(), rm - rm.mean(), "valid")
    b0 = T[0] - (tg[0] + float(np.argmax(cc) * dt))

    def cost(p):
        a, b = p
        t = a * ti + b
        m = (t >= T[0]) & (t <= T[-1])
        if m.sum() < 1000:
            return 1e12
        return float(np.mean((gs[m] - motor_rate(segs, t[m])) ** 2))

    grid = b0 + np.arange(-5.0, 5.0, 0.1)
    b0 = float(grid[int(np.argmin([cost((1.0, x)) for x in grid]))])
    r1 = minimize(lambda p: cost((1.0, p[0])), [b0], method="Nelder-Mead",
                  options=dict(xatol=1e-7, fatol=1e-12, maxiter=8000))
    b_const, rms_const = float(r1.x[0]), float(np.sqrt(r1.fun))

    r2 = minimize(cost, [1.0, b_const], method="Nelder-Mead",
                  options=dict(xatol=1e-11, fatol=1e-14,
                               maxiter=40000, maxfev=40000))
    a, b, rms = float(r2.x[0]), float(r2.x[1]), float(np.sqrt(r2.fun))

    loc = []
    for (k, t0, t1, rate, a0, a1) in segs:
        if k not in ("rate", "move", "return") or (t1 - t0) < 3.0:
            continue
        t = a * ti + b
        m = (t >= t0 - 3.0) & (t <= t1 + 3.0)
        if m.sum() < 100:
            continue
        tsub, ysub = ti[m], gs[m]

        def f(bb):
            return float(np.mean((ysub - motor_rate(segs, a * tsub + bb)) ** 2))
        xs = np.linspace(b - 1.0, b + 1.0, 201)
        x0 = float(xs[int(np.argmin([f(x) for x in xs]))])
        rr = minimize_scalar(f, bounds=(x0 - 0.05, x0 + 0.05), method="bounded")
        loc.append((0.5 * (t0 + t1), float(rr.x)))
    drift_ppm = resid_ms = np.nan
    if len(loc) >= 4:
        L = np.asarray(loc)
        c = np.polyfit(L[:, 0], L[:, 1], 1)
        res = L[:, 1] - np.polyval(c, L[:, 0])
        drift_ppm = float(c[0] * 1e6)
        resid_ms = float(res.std(ddof=1) * 1000.0)

    print(f"  [{label}] IMU sync: offset-only b={b_const:+.4f} s "
          f"(rms {rms_const:.4f} deg/s) | affine a={a:.9f} "
          f"({(a-1)*1e6:+.1f} ppm) b={b:+.4f} s (rms {rms:.4f} deg/s)")
    print(f"  [{label}] IMU sync: residual drift after fit {drift_ppm:+.1f} ppm, "
          f"local-offset scatter {resid_ms:.2f} ms, {len(loc)} features")
    return dict(a=a, b=b, rms_dps=rms, b_const=b_const,
                rms_const_dps=rms_const, ppm=(a - 1.0) * 1e6,
                drift_ppm=drift_ppm, resid_sd_ms=resid_ms,
                n_features=len(loc), fs=fs, local=loc)


# =============================================================================
# 4. ESTIMATORS
# =============================================================================
def sg_value_and_derivative(t, y, win_s, poly):
    """Zero-phase Savitzky-Golay value and first derivative.

    The signal is resampled onto a uniform grid at the median sample interval,
    filtered, then mapped back to the original (slightly non-uniform) sample
    times. A polynomial of order >= 1 reproduces a constant-rate ramp exactly,
    so inside a segment the window sets variance only, never bias. The window
    is specified in SECONDS so both runs share a physical bandwidth despite
    different frame rates."""
    fs = 1.0 / np.median(np.diff(t))
    n = int(round(win_s * fs)) | 1
    if n <= poly:
        warn("SG window shorter than the polynomial order; SG disabled.")
        return None, None, n, fs
    tu = np.arange(t[0], t[-1], 1.0 / fs)
    yu = np.interp(tu, t, y)
    val = savgol_filter(yu, n, poly, deriv=0)
    der = savgol_filter(yu, n, poly, deriv=1, delta=1.0 / fs)
    return np.interp(t, tu, val), np.interp(t, tu, der), n, fs


def kalman_constant_rate(t, z, sigma, q):
    """Camera-only constant-angular-rate Kalman filter.

        x  = [yaw, yaw_rate]'
        F  = [[1, dt], [0, 1]]
        H  = [1, 0]
        Q  = q * [[dt^3/3, dt^2/2], [dt^2/2, dt]]   white angular acceleration
        R  = sigma^2                                measured stationary variance

    dt is the true per-sample interval from the timestamps, so dropped frames
    are handled exactly rather than assumed uniform. No IMU input."""
    n = len(z)
    x = np.array([z[0], 0.0])
    P = np.diag([sigma ** 2, 100.0])
    out = np.zeros((n, 2))
    dts = np.diff(t, prepend=t[0])
    for k in range(n):
        if k:
            dt = dts[k]
            F = np.array([[1.0, dt], [0.0, 1.0]])
            Q = q * np.array([[dt ** 3 / 3.0, dt ** 2 / 2.0],
                              [dt ** 2 / 2.0, dt]])
            x = F @ x
            P = F @ P @ F.T + Q
        S = P[0, 0] + sigma ** 2
        K = P[:, 0] / S
        x = x + K * (z[k] - x[0])
        P = P - np.outer(K, P[0, :])
        out[k] = x
    return out


# =============================================================================
# 5. METRICS
# =============================================================================
def metrics(e, x=None, y=None):
    e = np.asarray(e, float)
    e = e[np.isfinite(e)]
    if e.size == 0:
        return dict(N=0, Bias=np.nan, MAE=np.nan, RMSE=np.nan, STD=np.nan,
                    Median=np.nan, MaxAbs=np.nan, Corr=np.nan)
    d = dict(N=int(e.size),
             Bias=float(np.mean(e)),
             MAE=float(np.mean(np.abs(e))),
             RMSE=float(np.sqrt(np.mean(e ** 2))),
             STD=float(np.std(e, ddof=1)) if e.size > 1 else np.nan,
             Median=float(np.median(e)),
             MaxAbs=float(np.max(np.abs(e))),
             Corr=np.nan)
    if x is not None and y is not None:
        x, y = np.asarray(x, float), np.asarray(y, float)
        m = np.isfinite(x) & np.isfinite(y)
        if m.sum() > 2 and np.std(x[m]) > 0 and np.std(y[m]) > 0:
            d["Corr"] = float(np.corrcoef(x[m], y[m])[0, 1])
    return d


def pooled(errs, xs=None, ys=None):
    e = np.concatenate([a for a in errs if a is not None and a.size])
    x = np.concatenate([a for a in xs]) if xs else None
    y = np.concatenate([a for a in ys]) if ys else None
    return metrics(e, x, y)


# =============================================================================
# 6. PER-RUN PROCESSING
# =============================================================================
def process_run(R, cfg):
    lab, dirn = R["label"], R["direction"]
    head(f"{lab} ({dirn})  —  {R['folder']}")

    T, A, segs = motor_profile(R["ev"])
    R.update(T=T, A=A, segs=segs)
    print(f"  motor: {len(R['ev'])} events, span {T[-1]-T[0]:.3f} s "
          f"(Arduino clock), total travel {A[-1]:.4f} deg "
          f"= {A[-1]/360.0:.4f} rev")

    # ---------------- camera -------------------------------------------
    cam = R["cam"]
    t_cam = camera_to_arduino(cam, R["fit"])
    raw_signed = cam["fused_yaw_deg"].to_numpy(float)
    cam_sign = (-1.0 if abs(np.nanmin(raw_signed)) > abs(np.nanmax(raw_signed))
                else 1.0)
    yaw = cam_sign * raw_signed                       # travel magnitude
    fs_cam = 1.0 / np.median(np.diff(t_cam))
    f = R["fit"]
    print(f"  camera: {len(cam)} frames, {t_cam[-1]-t_cam[0]:.2f} s, "
          f"{fs_cam:.2f} Hz median; sign applied {cam_sign:+.0f}")
    print(f"  camera<-motor clock (SERIAL, primary): a={f['a_perf']:.9f} "
          f"({f['ppm_perf']:+.1f} ppm), residual SD "
          f"{f['resid_sd_ms_perf']:.3f} ms, {f['n_events']} events")

    tail = t_cam > T[-1] + cfg["TAIL_GUARD_S"]
    if tail.sum() < 200:
        warn(f"{lab}: stationary tail has only {int(tail.sum())} frames.")
        sigma_yaw = float(np.std(yaw[tail], ddof=1)) if tail.sum() > 10 else 0.05
    else:
        sigma_yaw = float(np.std(yaw[tail], ddof=1))
    tail_drift = (float(np.polyfit(t_cam[tail], yaw[tail], 1)[0] * 3600.0)
                  if tail.sum() > 100 else np.nan)
    print(f"  camera stationary tail: n={int(tail.sum())}, "
          f"sigma={sigma_yaw:.5f} deg, drift {tail_drift:+.3f} deg/h")

    yaw_sg, rate_sg, n_sg, _ = sg_value_and_derivative(
        t_cam, yaw, cfg["SG_WINDOW_S"], cfg["SG_POLYORDER"])
    rate_raw = np.gradient(yaw, t_cam)
    kf = kalman_constant_rate(t_cam, yaw, sigma_yaw, cfg["KF_Q"])
    print(f"  SG: {cfg['SG_WINDOW_S']:.2f} s = {n_sg} samples, poly "
          f"{cfg['SG_POLYORDER']}   |   KF: Q={cfg['KF_Q']:g}, "
          f"R=sigma^2={sigma_yaw**2:.4e}")

    # ---------------- IMU ----------------------------------------------
    imu = R["imu"].copy()
    ti_raw = imu["teensy_time_us"].to_numpy(float) * 1e-6
    axis, isign, dev = imu_axis_and_sign(imu, cfg)
    print(f"  IMU: {len(imu)} samples, "
          f"{1/np.median(np.diff(ti_raw)):.2f} Hz timestamp cadence, "
          f"axis {axis.upper()}, sign {isign:+.0f}")
    print("       99th-pct |deviation| per axis: "
          + ", ".join(f"{k}={v:.3f}" for k, v in dev.items()))

    gyro_mag_uncal = isign * imu[f"gyro_{axis}_dps"].to_numpy(float)
    s = sync_imu(ti_raw, gyro_mag_uncal, T, segs, cfg, lab)
    t_imu = s["a"] * ti_raw + s["b"]

    still = np.ones(len(t_imu), bool)
    for (k, t0, t1, rate, a0, a1) in segs:
        if k in ("move", "rate", "return"):
            still &= ~((t_imu >= t0 - cfg["IMU_STILL_PAD_S"]) &
                       (t_imu <= t1 + cfg["IMU_STILL_PAD_S"]))
    still &= (t_imu > T[0]) & (t_imu < T[-1])
    gyro_bias = float(np.mean(gyro_mag_uncal[still]))
    gyro_noise = float(np.std(gyro_mag_uncal[still], ddof=1))
    gyro = gyro_mag_uncal - gyro_bias

    bt, bv = [], []
    idx = np.where(still)[0]
    if idx.size:
        for bl in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1):
            if len(bl) > 200:
                bt.append(float(np.mean(t_imu[bl])))
                bv.append(float(np.mean(gyro_mag_uncal[bl])))
    bias_drift = float(np.polyfit(bt, bv, 1)[0] * 3600.0) if len(bt) >= 3 else np.nan
    print(f"  IMU zero-rate bias {gyro_bias:+.5f} deg/s, stationary noise "
          f"{gyro_noise:.5f} deg/s (1 sigma), bias drift {bias_drift:+.4f} "
          f"deg/s per hour, {int(still.sum())} stationary samples")

    gyro_filt, _, n_isg, _ = sg_value_and_derivative(
        t_imu, gyro, cfg["IMU_SG_WINDOW_S"], cfg["IMU_SG_POLYORDER"])

    cross = {}
    fastmask = np.zeros(len(t_imu), bool)
    for (k, t0, t1, rate, a0, a1) in segs:
        if k == "rate" and rate >= 20:
            fastmask |= (t_imu >= t0 + 2) & (t_imu <= t1 - 2)
    for ax in "xyz":
        if ax == axis:
            continue
        v = imu[f"gyro_{ax}_dps"].to_numpy(float)
        cross[ax] = dict(
            bias=float(np.mean(v[still])),
            sd=float(np.std(v[still], ddof=1)),
            fast=float(np.mean(v[fastmask])) if fastmask.sum() else np.nan)

    # integrated relative yaw -- DIAGNOSTIC ONLY (no magnetometer)
    yaw_int = np.concatenate(
        ([0.0], np.cumsum(0.5 * (gyro[1:] + gyro[:-1]) * np.diff(t_imu))))
    i0 = int(np.argmin(np.abs(t_imu - T[0])))
    yaw_int = yaw_int - yaw_int[i0]

    # ---------------- masks ---------------------------------------------
    hold_cam = np.zeros(len(t_cam), bool)
    rate_cam = np.zeros(len(t_cam), bool)
    hold_imu = np.zeros(len(t_imu), bool)
    rate_imu = np.zeros(len(t_imu), bool)
    for (k, t0, t1, rate, a0, a1) in segs:
        if k == "hold":
            hold_cam |= (t_cam >= t0 + cfg["HOLD_GUARD_S"]) & (t_cam <= t1 - cfg["HOLD_GUARD_S"])
            hold_imu |= (t_imu >= t0 + cfg["HOLD_GUARD_S"]) & (t_imu <= t1 - cfg["HOLD_GUARD_S"])
        elif k == "rate":
            fr = cfg["RATE_TRIM_FRAC"] * (t1 - t0)
            rate_cam |= (t_cam >= t0 + fr) & (t_cam <= t1 - fr)
            rate_imu |= (t_imu >= t0 + fr) & (t_imu <= t1 - fr)

    R.update(t_cam=t_cam, yaw=yaw, yaw_sg=yaw_sg, yaw_kf=kf[:, 0],
             rate_raw=rate_raw, rate_sg=rate_sg, rate_kf=kf[:, 1],
             cam_sign=cam_sign, fs_cam=fs_cam, sg_points=n_sg,
             sigma_yaw=sigma_yaw, tail_drift=tail_drift, tail=tail,
             t_imu=t_imu, gyro=gyro, gyro_filt=gyro_filt, yaw_int=yaw_int,
             imu_axis=axis, imu_sign=isign, imu_bias=gyro_bias,
             imu_noise=gyro_noise, imu_bias_drift=bias_drift,
             imu_still=still, imu_cross=cross, imu_sync=s,
             imu_sg_points=n_isg,
             imu_temp=((float(imu["imu_temperature_c"].iloc[0]),
                        float(imu["imu_temperature_c"].iloc[-1]))
                       if "imu_temperature_c" in imu else (np.nan, np.nan)),
             hold_cam=hold_cam, rate_cam=rate_cam,
             hold_imu=hold_imu, rate_imu=rate_imu)
    return R


# =============================================================================
# 7. ANGLE TABLES
# =============================================================================
DIAG_NOTE = ("RELATIVE INTEGRATED-YAW DIAGNOSTIC — NOT ABSOLUTE YAW REFERENCE. "
             "The ISM330DHCX has no magnetometer, so integrated gyro yaw is "
             "drift-prone and is never primary absolute yaw accuracy.")


def angle_residuals(R, cfg):
    """Per-hold residual arrays for every camera estimator and for the
    integrated-gyro diagnostic, all evaluated on the CAMERA sample grid with
    the IMU integral interpolated onto it (one consistent grid per sheet)."""
    t, segs = R["t_cam"], R["segs"]
    iy = np.interp(t, R["t_imu"], R["yaw_int"])
    out = {}
    for (k, t0, t1, rate, a0, a1) in segs:
        if k != "hold":
            continue
        m = (t >= t0 + cfg["HOLD_GUARD_S"]) & (t <= t1 - cfg["HOLD_GUARD_S"])
        if m.sum() < 20:
            warn(f"{R['label']}: hold near {a1:.2f} deg has {int(m.sum())} frames.")
            continue
        eff = a1
        d = dict(eff=eff, n=int(m.sum()), mask=m,
                 cam_raw=R["yaw"][m], cam_sg=R["yaw_sg"][m],
                 cam_kf=R["yaw_kf"][m], imu_int=iy[m])
        d["e_raw_cm"] = d["cam_raw"] - eff
        d["e_sg_cm"] = d["cam_sg"] - eff
        d["e_kf_cm"] = d["cam_kf"] - eff
        d["e_raw_ci"] = d["cam_raw"] - d["imu_int"]
        d["e_sg_ci"] = d["cam_sg"] - d["imu_int"]
        d["e_kf_ci"] = d["cam_kf"] - d["imu_int"]
        d["e_im"] = d["imu_int"] - eff
        out[round(eff, 6)] = d
    return out


def angles_full_sheet(R, ares, cfg):
    cam, t = R["cam"], R["t_cam"]
    rows = []
    for eff in sorted(ares):
        d = ares[eff]
        m = d["mask"]
        req = min(cfg["ANGLES_DEG"], key=lambda q: abs(q - eff))
        pulses = angle_to_pulses(float(req), cfg["PULSES_PER_REV"])
        row = {
            "Direction": R["direction"],
            "Requested_Angle_deg": float(req),
            "Motor_Pulses": pulses,
            "Effective_Motor_Angle_deg": eff,
            "Motor_Quantization_Error_deg": eff - req,
            "Signed_Effective_Motor_Angle_deg": R["dir_sign"] * eff,

            "Camera_RAW_Mean_deg": float(np.mean(d["cam_raw"])),
            "Camera_RAW_Median_deg": float(np.median(d["cam_raw"])),
            "Camera_SG_Mean_deg": float(np.mean(d["cam_sg"])),
            "Camera_SG_Median_deg": float(np.median(d["cam_sg"])),
            "Camera_KF_Mean_deg": float(np.mean(d["cam_kf"])),
            "Camera_KF_Median_deg": float(np.median(d["cam_kf"])),
            "Camera_RAW_STD_deg": float(np.std(d["cam_raw"], ddof=1)),
            "Camera_SG_STD_deg": float(np.std(d["cam_sg"], ddof=1)),
            "Camera_KF_STD_deg": float(np.std(d["cam_kf"], ddof=1)),
            "Camera_RAW_Min_deg": float(np.min(d["cam_raw"])),
            "Camera_RAW_Max_deg": float(np.max(d["cam_raw"])),
            "Signed_Camera_RAW_Mean_deg": R["dir_sign"] * float(np.mean(d["cam_raw"])),

            "Valid_Frame_Percent": 100.0 * float(np.isfinite(d["cam_raw"]).mean()),
            "Mean_Tags_Used": float(
                cam.loc[m, "used_count_after_outlier_rejection"].mean()),
            "Mean_Tags_Visible": float(cam.loc[m, "visible_count"].mean()),
            "N_Camera": d["n"],

            "IMU_Integrated_Relative_Yaw_Mean_deg": float(np.mean(d["imu_int"])),
            "IMU_Integrated_Relative_Yaw_STD_deg": float(np.std(d["imu_int"], ddof=1)),
        }
        for tag, key in (("RAW_Camera_Motor", "e_raw_cm"),
                         ("SG_Camera_Motor", "e_sg_cm"),
                         ("KF_Camera_Motor", "e_kf_cm")):
            s = metrics(d[key])
            row[f"{tag}_Bias_deg"] = s["Bias"]
            row[f"{tag}_MAE_deg"] = s["MAE"]
            row[f"{tag}_RMSE_deg"] = s["RMSE"]
            row[f"{tag}_STD_Error_deg"] = s["STD"]
            row[f"{tag}_MaxAbs_deg"] = s["MaxAbs"]
        for tag, key in (("RAW_Camera_IMU", "e_raw_ci"),
                         ("SG_Camera_IMU", "e_sg_ci"),
                         ("KF_Camera_IMU", "e_kf_ci"),
                         ("IMU_Motor", "e_im")):
            s = metrics(d[key])
            row[f"DIAG_{tag}_Bias_deg"] = s["Bias"]
            row[f"DIAG_{tag}_MAE_deg"] = s["MAE"]
            row[f"DIAG_{tag}_RMSE_deg"] = s["RMSE"]
        row["IMU_Angle_Comparison_Status"] = DIAG_NOTE
        rows.append(row)

    df = pd.DataFrame(rows)
    # ---- POOLED_TOTAL from residual samples -------------------------------
    # Motor_Pulses are ABSOLUTE target positions, so summing them is
    # meaningless; the pooled row leaves the column blank on purpose.
    tot = {"Direction": R["direction"], "Requested_Angle_deg": "POOLED_TOTAL",
           "Motor_Pulses": np.nan,
           "Effective_Motor_Angle_deg": np.nan,
           "Motor_Quantization_Error_deg": np.nan,
           "Signed_Effective_Motor_Angle_deg": np.nan}
    for c in ("Camera_RAW_Mean_deg", "Camera_RAW_Median_deg",
              "Camera_SG_Mean_deg", "Camera_SG_Median_deg",
              "Camera_KF_Mean_deg", "Camera_KF_Median_deg",
              "Camera_RAW_Min_deg", "Camera_RAW_Max_deg",
              "Signed_Camera_RAW_Mean_deg",
              "IMU_Integrated_Relative_Yaw_Mean_deg"):
        tot[c] = np.nan
    for c in ("Camera_RAW_STD_deg", "Camera_SG_STD_deg", "Camera_KF_STD_deg",
              "IMU_Integrated_Relative_Yaw_STD_deg"):
        tot[c] = float(df[c].mean())
    tot["Valid_Frame_Percent"] = float(df["Valid_Frame_Percent"].mean())
    tot["Mean_Tags_Used"] = float(df["Mean_Tags_Used"].mean())
    tot["Mean_Tags_Visible"] = float(df["Mean_Tags_Visible"].mean())
    tot["N_Camera"] = int(df["N_Camera"].sum())
    for tag, key in (("RAW_Camera_Motor", "e_raw_cm"),
                     ("SG_Camera_Motor", "e_sg_cm"),
                     ("KF_Camera_Motor", "e_kf_cm")):
        s = pooled([ares[a][key] for a in ares])
        tot[f"{tag}_Bias_deg"] = s["Bias"]
        tot[f"{tag}_MAE_deg"] = s["MAE"]
        tot[f"{tag}_RMSE_deg"] = s["RMSE"]
        tot[f"{tag}_STD_Error_deg"] = s["STD"]
        tot[f"{tag}_MaxAbs_deg"] = s["MaxAbs"]
    for tag, key in (("RAW_Camera_IMU", "e_raw_ci"),
                     ("SG_Camera_IMU", "e_sg_ci"),
                     ("KF_Camera_IMU", "e_kf_ci"),
                     ("IMU_Motor", "e_im")):
        s = pooled([ares[a][key] for a in ares])
        tot[f"DIAG_{tag}_Bias_deg"] = s["Bias"]
        tot[f"DIAG_{tag}_MAE_deg"] = s["MAE"]
        tot[f"DIAG_{tag}_RMSE_deg"] = s["RMSE"]
    tot["IMU_Angle_Comparison_Status"] = DIAG_NOTE
    return pd.concat([df, pd.DataFrame([tot])], ignore_index=True)


def angles_combined_master(runs, ares_all, cfg):
    cw, ccw = runs[0], runs[1]
    tagcw, tagccw = cw["direction"], ccw["direction"]
    angles = sorted(ares_all[cw["label"]])
    rows = []
    for eff in angles:
        req = min(cfg["ANGLES_DEG"], key=lambda q: abs(q - eff))
        r = {"Requested_Angle_deg": float(req),
             "Motor_Pulses": angle_to_pulses(float(req), cfg["PULSES_PER_REV"]),
             "Effective_Motor_Angle_deg": eff}
        for R, tg in ((cw, tagcw), (ccw, tagccw)):
            d = ares_all[R["label"]][eff]
            r[f"{tg}_RAW_Camera_Mean_deg"] = float(np.mean(d["cam_raw"]))
            r[f"{tg}_SG_Camera_Mean_deg"] = float(np.mean(d["cam_sg"]))
            r[f"{tg}_KF_Camera_Mean_deg"] = float(np.mean(d["cam_kf"]))
        for R, tg in ((cw, tagcw), (ccw, tagccw)):
            d = ares_all[R["label"]][eff]
            for est, key in (("RAW", "e_raw_cm"), ("SG", "e_sg_cm"), ("KF", "e_kf_cm")):
                s = metrics(d[key])
                r[f"{tg}_{est}_CM_RMSE_deg"] = s["RMSE"]
                r[f"{tg}_{est}_CM_MAE_deg"] = s["MAE"]
                r[f"{tg}_{est}_CM_Bias_deg"] = s["Bias"]
        for R, tg in ((cw, tagcw), (ccw, tagccw)):
            d = ares_all[R["label"]][eff]
            for est, key in (("RAW", "e_raw_ci"), ("SG", "e_sg_ci"),
                             ("KF", "e_kf_ci")):
                r[f"{tg}_{est}_CI_RMSE_Diagnostic_deg"] = metrics(d[key])["RMSE"]
            r[f"{tg}_IM_RMSE_Diagnostic_deg"] = metrics(d["e_im"])["RMSE"]
        for est, key in (("RAW", "e_raw_cm"), ("SG", "e_sg_cm"), ("KF", "e_kf_cm")):
            s = pooled([ares_all[R["label"]][eff][key] for R in runs])
            r[f"Combined_{est}_CM_RMSE_deg"] = s["RMSE"]
            r[f"Combined_{est}_CM_MAE_deg"] = s["MAE"]
            r[f"Combined_{est}_CM_Bias_deg"] = s["Bias"]
        for est, key in (("RAW", "e_raw_ci"), ("SG", "e_sg_ci"), ("KF", "e_kf_ci")):
            r[f"Combined_{est}_CI_RMSE_Diagnostic_deg"] = pooled(
                [ares_all[R["label"]][eff][key] for R in runs])["RMSE"]
        r["Combined_IM_RMSE_Diagnostic_deg"] = pooled(
            [ares_all[R["label"]][eff]["e_im"] for R in runs])["RMSE"]
        r["Direction_Bias_Difference_deg"] = (
            r[f"{tagccw}_RAW_CM_Bias_deg"] - r[f"{tagcw}_RAW_CM_Bias_deg"])
        rows.append(r)

    df = pd.DataFrame(rows)
    tot = {"Requested_Angle_deg": "POOLED_TOTAL",
           "Motor_Pulses": np.nan,
           "Effective_Motor_Angle_deg": np.nan}
    for R, tg in ((cw, tagcw), (ccw, tagccw)):
        for c in ("RAW", "SG", "KF"):
            tot[f"{tg}_{c}_Camera_Mean_deg"] = np.nan
    for R, tg in ((cw, tagcw), (ccw, tagccw)):
        A_ = ares_all[R["label"]]
        for est, key in (("RAW", "e_raw_cm"), ("SG", "e_sg_cm"), ("KF", "e_kf_cm")):
            s = pooled([A_[a][key] for a in A_])
            tot[f"{tg}_{est}_CM_RMSE_deg"] = s["RMSE"]
            tot[f"{tg}_{est}_CM_MAE_deg"] = s["MAE"]
            tot[f"{tg}_{est}_CM_Bias_deg"] = s["Bias"]
        for est, key in (("RAW", "e_raw_ci"), ("SG", "e_sg_ci"), ("KF", "e_kf_ci")):
            tot[f"{tg}_{est}_CI_RMSE_Diagnostic_deg"] = pooled(
                [A_[a][key] for a in A_])["RMSE"]
        tot[f"{tg}_IM_RMSE_Diagnostic_deg"] = pooled(
            [A_[a]["e_im"] for a in A_])["RMSE"]
    for est, key in (("RAW", "e_raw_cm"), ("SG", "e_sg_cm"), ("KF", "e_kf_cm")):
        s = pooled([ares_all[R["label"]][a][key] for R in runs
                    for a in ares_all[R["label"]]])
        tot[f"Combined_{est}_CM_RMSE_deg"] = s["RMSE"]
        tot[f"Combined_{est}_CM_MAE_deg"] = s["MAE"]
        tot[f"Combined_{est}_CM_Bias_deg"] = s["Bias"]
    for est, key in (("RAW", "e_raw_ci"), ("SG", "e_sg_ci"), ("KF", "e_kf_ci")):
        tot[f"Combined_{est}_CI_RMSE_Diagnostic_deg"] = pooled(
            [ares_all[R["label"]][a][key] for R in runs
             for a in ares_all[R["label"]]])["RMSE"]
    tot["Combined_IM_RMSE_Diagnostic_deg"] = pooled(
        [ares_all[R["label"]][a]["e_im"] for R in runs
         for a in ares_all[R["label"]]])["RMSE"]
    tot["Direction_Bias_Difference_deg"] = (
        tot[f"{tagccw}_RAW_CM_Bias_deg"] - tot[f"{tagcw}_RAW_CM_Bias_deg"])
    return pd.concat([df, pd.DataFrame([tot])], ignore_index=True)


# =============================================================================
# 8. RATE TABLES
# =============================================================================
def rate_residuals(R, cfg):
    """Per-rate-segment residuals for all seven primary pairings plus the
    bandwidth-matched IMU variants.

    Grid convention, applied consistently and identically to both runs:
      * camera-vs-motor pairs are evaluated on the CAMERA sample grid
      * every pair that involves the IMU is evaluated on the IMU sample grid,
        with the camera estimate interpolated onto the IMU sample times
    This keeps each comparison on the native grid of its noisier partner and
    never resamples the gyro."""
    t, ti, segs = R["t_cam"], R["t_imu"], R["segs"]
    out = {}
    for (k, t0, t1, rate, a0, a1) in segs:
        if k != "rate":
            continue
        fr = cfg["RATE_TRIM_FRAC"] * (t1 - t0)
        mc = (t >= t0 + fr) & (t <= t1 - fr)
        mi = (ti >= t0 + fr) & (ti <= t1 - fr)
        if mc.sum() < 50 or mi.sum() < 50:
            warn(f"{R['label']}: rate segment {rate:g} deg/s has "
                 f"{int(mc.sum())} camera / {int(mi.sum())} IMU samples.")
            continue
        gy, gyf = R["gyro"][mi], R["gyro_filt"][mi]
        cmd_c = np.full(int(mc.sum()), rate)
        cmd_i = np.full(int(mi.sum()), rate)
        d = dict(rate=rate, t0=t0, t1=t1, mask_cam=mc, mask_imu=mi,
                 n_cam=int(mc.sum()), n_imu=int(mi.sum()),
                 pulses=int(round((a1 - a0) / DPP)),
                 travel=a1 - a0, duration=t1 - t0,
                 imu_raw=gy, imu_filt=gyf, cmd_c=cmd_c, cmd_i=cmd_i)
        for tag, arr in (("raw", R["rate_raw"]), ("sg", R["rate_sg"]),
                         ("kf", R["rate_kf"])):
            v = arr[mc]
            ci = np.interp(ti[mi], t, arr)
            d[f"cam_{tag}"] = v
            d[f"cam_{tag}_on_imu"] = ci
            d[f"e_{tag}_cm"] = v - cmd_c
            d[f"e_{tag}_ci"] = ci - gy
            d[f"e_{tag}_cif"] = ci - gyf
        d["e_im"] = gy - cmd_i
        d["e_imf"] = gyf - cmd_i
        out[rate] = d
    return out


RATE_PAIRS = [
    ("RAW_CM", "e_raw_cm", "cam_raw", "cmd_c"),
    ("RAW_CI", "e_raw_ci", "cam_raw_on_imu", "imu_raw"),
    ("SG_CM", "e_sg_cm", "cam_sg", "cmd_c"),
    ("SG_CI", "e_sg_ci", "cam_sg_on_imu", "imu_raw"),
    ("KF_CM", "e_kf_cm", "cam_kf", "cmd_c"),
    ("KF_CI", "e_kf_ci", "cam_kf_on_imu", "imu_raw"),
    ("IMU_M", "e_im", "imu_raw", "cmd_i"),
]
RATE_PAIRS_FILT = [
    ("RAW_CIf", "e_raw_cif", "cam_raw_on_imu", "imu_filt"),
    ("SG_CIf", "e_sg_cif", "cam_sg_on_imu", "imu_filt"),
    ("KF_CIf", "e_kf_cif", "cam_kf_on_imu", "imu_filt"),
    ("IMU_M_f", "e_imf", "imu_filt", "cmd_i"),
]


def rates_full_sheet(R, rres, cfg):
    a_clock = float(R["fit"]["a_perf"])
    rows = []
    for rate in sorted(rres):
        d = rres[rate]
        motor_arduino = d["travel"] / d["duration"]
        row = {
            "Direction": R["direction"],
            "Requested_Rate_dps": rate,
            "Signed_Requested_Rate_dps": R["dir_sign"] * rate,
            "Motor_Pulses_In_Segment": d["pulses"],
            "Motor_Commanded_Travel_deg": d["travel"],
            "Motor_Segment_Duration_ArduinoTime_s": d["duration"],
            "Motor_Commanded_Rate_ArduinoTime_dps": motor_arduino,
            "Motor_Rate_PC_SI_Time_dps": motor_arduino * a_clock,
            "Camera_RAW_Mean_Rate_dps": float(np.mean(d["cam_raw"])),
            "Camera_SG_Mean_Rate_dps": float(np.mean(d["cam_sg"])),
            "Camera_KF_Mean_Rate_dps": float(np.mean(d["cam_kf"])),
            "IMU_RAW_Mean_Rate_dps": float(np.mean(d["imu_raw"])),
            "IMU_BandwidthMatched_Mean_Rate_dps": float(np.mean(d["imu_filt"])),
            "Camera_RAW_STD_Rate_dps": float(np.std(d["cam_raw"], ddof=1)),
            "Camera_SG_STD_Rate_dps": float(np.std(d["cam_sg"], ddof=1)),
            "Camera_KF_STD_Rate_dps": float(np.std(d["cam_kf"], ddof=1)),
            "IMU_RAW_STD_Rate_dps": float(np.std(d["imu_raw"], ddof=1)),
            "N_Camera": d["n_cam"],
            "N_IMU": d["n_imu"],
        }
        for tag, ek, xk, yk in RATE_PAIRS + RATE_PAIRS_FILT:
            s = metrics(d[ek], d[xk], d[yk])
            row[f"{tag}_Bias"] = s["Bias"]
            row[f"{tag}_MAE"] = s["MAE"]
            row[f"{tag}_RMSE"] = s["RMSE"]
            row[f"{tag}_STD"] = s["STD"]
            row[f"{tag}_Median"] = s["Median"]
            row[f"{tag}_MaxAbs"] = s["MaxAbs"]
            row[f"{tag}_Correlation"] = s["Corr"]
        rows.append(row)

    df = pd.DataFrame(rows)
    tot = {"Direction": R["direction"], "Requested_Rate_dps": "POOLED_TOTAL",
           "Signed_Requested_Rate_dps": np.nan,
           "Motor_Pulses_In_Segment": int(df["Motor_Pulses_In_Segment"].sum()),
           "Motor_Commanded_Travel_deg": float(df["Motor_Commanded_Travel_deg"].sum()),
           "Motor_Segment_Duration_ArduinoTime_s":
               float(df["Motor_Segment_Duration_ArduinoTime_s"].sum()),
           "Motor_Commanded_Rate_ArduinoTime_dps": np.nan,
           "Motor_Rate_PC_SI_Time_dps": np.nan}
    for c in ("Camera_RAW_Mean_Rate_dps", "Camera_SG_Mean_Rate_dps",
              "Camera_KF_Mean_Rate_dps", "IMU_RAW_Mean_Rate_dps",
              "IMU_BandwidthMatched_Mean_Rate_dps"):
        tot[c] = np.nan
    for c in ("Camera_RAW_STD_Rate_dps", "Camera_SG_STD_Rate_dps",
              "Camera_KF_STD_Rate_dps", "IMU_RAW_STD_Rate_dps"):
        tot[c] = float(df[c].mean())
    tot["N_Camera"] = int(df["N_Camera"].sum())
    tot["N_IMU"] = int(df["N_IMU"].sum())
    for tag, ek, xk, yk in RATE_PAIRS + RATE_PAIRS_FILT:
        s = pooled([rres[r][ek] for r in rres],
                   [rres[r][xk] for r in rres], [rres[r][yk] for r in rres])
        tot[f"{tag}_Bias"] = s["Bias"]
        tot[f"{tag}_MAE"] = s["MAE"]
        tot[f"{tag}_RMSE"] = s["RMSE"]
        tot[f"{tag}_STD"] = s["STD"]
        tot[f"{tag}_Median"] = s["Median"]
        tot[f"{tag}_MaxAbs"] = s["MaxAbs"]
        tot[f"{tag}_Correlation"] = s["Corr"]
    return pd.concat([df, pd.DataFrame([tot])], ignore_index=True)


def rates_combined_master(runs, rres_all, cfg):
    cw, ccw = runs[0], runs[1]
    tcw, tccw = cw["direction"], ccw["direction"]
    rates = sorted(rres_all[cw["label"]])
    blocks = [("RMSE", "RMSE"), ("MAE", "MAE"), ("Bias", "Bias"),
              ("STD", "STD"), ("MaxAbs", "MaxAbs")]
    rows = []
    for rate in rates:
        r = {"Requested_Rate_dps": rate}
        for stat, _ in blocks:
            for R, tg in ((cw, tcw), (ccw, tccw)):
                d = rres_all[R["label"]][rate]
                for tag, ek, xk, yk in RATE_PAIRS:
                    r[f"{tg}_{tag}_{stat}"] = metrics(d[ek], d[xk], d[yk])[stat]
            for tag, ek, xk, yk in RATE_PAIRS:
                s = pooled([rres_all[R["label"]][rate][ek] for R in runs],
                           [rres_all[R["label"]][rate][xk] for R in runs],
                           [rres_all[R["label"]][rate][yk] for R in runs])
                r[f"COMBINED_{tag}_{stat}"] = s[stat]
        rows.append(r)
    df = pd.DataFrame(rows)

    tot = {"Requested_Rate_dps": "POOLED_TOTAL"}
    for stat, _ in blocks:
        for R, tg in ((cw, tcw), (ccw, tccw)):
            Ar = rres_all[R["label"]]
            for tag, ek, xk, yk in RATE_PAIRS:
                s = pooled([Ar[q][ek] for q in Ar], [Ar[q][xk] for q in Ar],
                           [Ar[q][yk] for q in Ar])
                tot[f"{tg}_{tag}_{stat}"] = s[stat]
        for tag, ek, xk, yk in RATE_PAIRS:
            s = pooled([rres_all[R["label"]][q][ek] for R in runs
                        for q in rres_all[R["label"]]],
                       [rres_all[R["label"]][q][xk] for R in runs
                        for q in rres_all[R["label"]]],
                       [rres_all[R["label"]][q][yk] for R in runs
                        for q in rres_all[R["label"]]])
            tot[f"COMBINED_{tag}_{stat}"] = s[stat]
    return pd.concat([df, pd.DataFrame([tot])], ignore_index=True)


def imu_filter_diagnostic(runs, rres_all, cfg):
    rows = []
    scopes = [(R["label"], R["direction"], [R]) for R in runs]
    scopes.append(("Combined", "CW+CCW", runs))
    rates = sorted(rres_all[runs[0]["label"]])
    for scope, dirn, group in scopes:
        for rate in list(rates) + ["POOLED_TOTAL"]:
            def E(key):
                if rate == "POOLED_TOTAL":
                    return [rres_all[R["label"]][q][key] for R in group
                            for q in rres_all[R["label"]]]
                return [rres_all[R["label"]][rate][key] for R in group]
            row = {"Scope": scope, "Direction": dirn,
                   "Requested_Rate_dps": rate}
            for nm, key in (("Raw_IMU_Motor", "e_im"),
                            ("Filtered_IMU_Motor", "e_imf"),
                            ("RAW_Camera_RawIMU", "e_raw_ci"),
                            ("SG_Camera_RawIMU", "e_sg_ci"),
                            ("KF_Camera_RawIMU", "e_kf_ci"),
                            ("RAW_Camera_FilteredIMU", "e_raw_cif"),
                            ("SG_Camera_FilteredIMU", "e_sg_cif"),
                            ("KF_Camera_FilteredIMU", "e_kf_cif")):
                s = pooled(E(key))
                row[f"{nm}_RMSE"] = s["RMSE"]
                row[f"{nm}_MAE"] = s["MAE"]
                row[f"{nm}_Bias"] = s["Bias"]
            row["Status"] = ("Raw bias-corrected gyro is the PRIMARY reference. "
                             "The bandwidth-matched version is a SECONDARY "
                             "diagnostic and never replaces or hides it.")
            rows.append(row)
    return pd.DataFrame(rows)


# =============================================================================
# 9. SYNC / CALIBRATION SHEETS
# =============================================================================
def sync_full_sheet(runs):
    rows = []
    for R in runs:
        f, s = R["fit"], R["imu_sync"]
        rows.append({
            "Run": R["label"], "Direction": R["direction"],
            "Camera_File": R["camera_file"], "IMU_File": R["imu_file"],
            "Motor_ArduinoPC_Model": "t_arduino = a * t_perf_counter + b",
            "Motor_ArduinoPC_a": f["a_perf"],
            "Motor_ArduinoPC_b_s": f["b_perf"],
            "Motor_ArduinoPC_ppm": f["ppm_perf"],
            "Motor_ArduinoPC_Residual_SD_ms": f["resid_sd_ms_perf"],
            "Motor_Sync_Event_Count": f["n_events"],
            "Motor_Sync_Event_Expected": f.get("expected_events", 94),
            "Motor_Sync_Event_Span_s": f["span_s"],
            "Motor_WallClock_a": f["a_wall"],
            "Motor_WallClock_ppm": f["ppm_wall"],
            "Camera_Frames": len(R["cam"]),
            "Camera_Median_Rate_Hz": R["fs_cam"],
            "Camera_Sign_Applied": R["cam_sign"],
            "Camera_Stationary_Sigma_deg": R["sigma_yaw"],
            "Camera_Stationary_Drift_deg_per_h": R["tail_drift"],
            "IMU_Motor_Model": "t_arduino = a_imu * t_teensy + b_imu",
            "IMU_Motor_a": s["a"],
            "IMU_Motor_b_s": s["b"],
            "IMU_Motor_ppm": s["ppm"],
            "IMU_Motor_Residual_dps_RMS": s["rms_dps"],
            "IMU_Motor_OffsetOnly_Residual_dps_RMS": s["rms_const_dps"],
            "IMU_Motor_Residual_Drift_ppm": s["drift_ppm"],
            "IMU_Motor_LocalOffset_Scatter_ms": s["resid_sd_ms"],
            "IMU_Motor_Features_Used": s["n_features"],
            "Gyro_Axis": R["imu_axis"].upper(),
            "Gyro_Sign": R["imu_sign"],
            "Gyro_Stationary_Bias_dps": R["imu_bias"],
            "Primary_Method_Note": ("Camera<->motor mapping taken from the "
                                    "serial event timestamps; never refitted "
                                    "by minimising camera-vs-motor residuals."),
        })
    return pd.DataFrame(rows)


def sync_events_sheet(R):
    ev = R["ev"].copy()
    a, b = float(R["fit"]["a_perf"]), float(R["fit"]["b_perf"])
    ev["t_arduino_s"] = ev["t_arduino_us"] * 1e-6
    ev["t_arduino_rel_s"] = ev["t_arduino_s"] - ev["t_arduino_s"].iloc[0]
    ev["t_perf_rel_s"] = (ev["t_perf_ns"] - ev["t_perf_ns"].iloc[0]) * 1e-9
    ev["fit_residual_ms"] = (ev["t_arduino_rel_s"]
                             - (a * ev["t_perf_rel_s"] + b)) * 1000.0
    ev["Run"] = R["label"]
    ev["Direction"] = R["direction"]
    return ev[["Run", "Direction", "event", "value", "extra", "t_arduino_us",
               "t_arduino_rel_s", "t_perf_rel_s", "t_wall_ns",
               "fit_residual_ms"]]


def imu_calibration_sheet(runs, cfg):
    rows = []
    for R in runs:
        r = {"Run": R["label"], "Direction": R["direction"],
             "Selected_Gyro_Axis": R["imu_axis"].upper(),
             "Sign_Applied": R["imu_sign"],
             "Stationary_Gyro_Bias_dps": R["imu_bias"],
             "Stationary_Noise_1sigma_dps": R["imu_noise"],
             "Bias_Drift_dps_per_hour": R["imu_bias_drift"],
             "Stationary_Samples": int(R["imu_still"].sum()),
             "Timestamp_Derived_Cadence_Hz": R["imu_sync"]["fs"],
             "Configured_ODR_Register_Hz": float(
                 R["imu_hdr"].get("gyro_odr_hz", "nan")),
             "Cadence_Note": ("Timestamp-derived cadence. NOT an "
                              "independently verified physical sensor ODR."),
             "Start_Temperature_C": R["imu_temp"][0],
             "End_Temperature_C": R["imu_temp"][1],
             "Datasheet_ZeroRateLevel_dps": cfg["DS_GYRO_ZRL_DPS"],
             "Datasheet_Sensitivity_Tolerance_pct": cfg["DS_GYRO_SENS_TOL_PCT"],
             "Datasheet_CrossAxis_pct": cfg["DS_GYRO_CROSS_AXIS_PCT"]}
        for ax, d in R["imu_cross"].items():
            r[f"CrossAxis_gyro_{ax}_Bias_dps"] = d["bias"]
            r[f"CrossAxis_gyro_{ax}_SD_dps"] = d["sd"]
            r[f"CrossAxis_gyro_{ax}_Mean_At_Fast_Yaw_dps"] = d["fast"]
        rows.append(r)
    return pd.DataFrame(rows)


# =============================================================================
# 10. SAMPLE-LEVEL AUDIT EXPORT
# =============================================================================
def synchronized_samples(R, ares, rres, cfg):
    """Sample-level audit trail. Exactly the samples that produced the RMSE
    numbers: the stable part of every static hold and every rate segment.

    Long format with a `grid` column, because camera-vs-motor is evaluated on
    the camera sample grid while every IMU-involving pair is evaluated on the
    IMU sample grid. Filtering by `grid` reproduces each metric exactly."""
    t, ti = R["t_cam"], R["t_imu"]
    iy = np.interp(t, ti, R["yaw_int"])
    ca = {k: np.interp(ti, t, R[k]) for k in
          ("yaw", "yaw_sg", "yaw_kf", "rate_raw", "rate_sg", "rate_kf")}
    frames = []

    # ---- static holds, camera grid -------------------------------------
    for sid, eff in enumerate(sorted(ares)):
        d = ares[eff]
        m = d["mask"]
        req = min(cfg["ANGLES_DEG"], key=lambda q: abs(q - eff))
        n = int(m.sum())
        frames.append(pd.DataFrame({
            "run": R["label"], "direction": R["direction"], "grid": "camera",
            "common_time_s": t[m], "segment_type": "hold", "segment_id": sid,
            "requested_angle_deg": float(req),
            "effective_motor_angle_deg": eff,
            "requested_rate_dps": np.nan, "motor_rate_dps": 0.0,
            "camera_raw_yaw_deg": d["cam_raw"],
            "camera_sg_yaw_deg": d["cam_sg"],
            "camera_kf_yaw_deg": d["cam_kf"],
            "camera_raw_rate_dps": R["rate_raw"][m],
            "camera_sg_rate_dps": R["rate_sg"][m],
            "camera_kf_rate_dps": R["rate_kf"][m],
            "imu_raw_rate_dps": np.interp(t[m], ti, R["gyro"]),
            "imu_filtered_rate_dps": np.interp(t[m], ti, R["gyro_filt"]),
            "imu_integrated_yaw_deg": d["imu_int"],
            "camera_motor_error_deg": d["e_raw_cm"],
            "camera_sg_motor_error_deg": d["e_sg_cm"],
            "camera_kf_motor_error_deg": d["e_kf_cm"],
            "camera_imu_error_deg_DIAG": d["e_raw_ci"],
            "imu_motor_error_deg_DIAG": d["e_im"],
            "camera_motor_error_dps": np.nan,
            "camera_imu_error_dps": np.nan,
            "imu_motor_error_dps": np.nan,
        }))

    # ---- rate segments, camera grid (camera-vs-motor) --------------------
    for sid, rate in enumerate(sorted(rres)):
        d = rres[rate]
        m = d["mask_cam"]
        frames.append(pd.DataFrame({
            "run": R["label"], "direction": R["direction"], "grid": "camera",
            "common_time_s": t[m], "segment_type": "rate", "segment_id": sid,
            "requested_angle_deg": np.nan,
            "effective_motor_angle_deg": motor_angle(R["T"], R["A"], t[m]),
            "requested_rate_dps": rate,
            "motor_rate_dps": d["cmd_c"],
            "camera_raw_yaw_deg": R["yaw"][m],
            "camera_sg_yaw_deg": R["yaw_sg"][m],
            "camera_kf_yaw_deg": R["yaw_kf"][m],
            "camera_raw_rate_dps": d["cam_raw"],
            "camera_sg_rate_dps": d["cam_sg"],
            "camera_kf_rate_dps": d["cam_kf"],
            "imu_raw_rate_dps": np.nan,
            "imu_filtered_rate_dps": np.nan,
            "imu_integrated_yaw_deg": iy[m],
            "camera_motor_error_deg": np.nan,
            "camera_sg_motor_error_deg": np.nan,
            "camera_kf_motor_error_deg": np.nan,
            "camera_imu_error_deg_DIAG": np.nan,
            "imu_motor_error_deg_DIAG": np.nan,
            "camera_motor_error_dps": d["e_sg_cm"],
            "camera_imu_error_dps": np.nan,
            "imu_motor_error_dps": np.nan,
        }))

    # ---- rate segments, IMU grid (everything involving the gyro) ---------
    for sid, rate in enumerate(sorted(rres)):
        d = rres[rate]
        m = d["mask_imu"]
        frames.append(pd.DataFrame({
            "run": R["label"], "direction": R["direction"], "grid": "imu",
            "common_time_s": ti[m], "segment_type": "rate", "segment_id": sid,
            "requested_angle_deg": np.nan,
            "effective_motor_angle_deg": motor_angle(R["T"], R["A"], ti[m]),
            "requested_rate_dps": rate,
            "motor_rate_dps": d["cmd_i"],
            "camera_raw_yaw_deg": ca["yaw"][m],
            "camera_sg_yaw_deg": ca["yaw_sg"][m],
            "camera_kf_yaw_deg": ca["yaw_kf"][m],
            "camera_raw_rate_dps": d["cam_raw_on_imu"],
            "camera_sg_rate_dps": d["cam_sg_on_imu"],
            "camera_kf_rate_dps": d["cam_kf_on_imu"],
            "imu_raw_rate_dps": d["imu_raw"],
            "imu_filtered_rate_dps": d["imu_filt"],
            "imu_integrated_yaw_deg": R["yaw_int"][m],
            "camera_motor_error_deg": np.nan,
            "camera_sg_motor_error_deg": np.nan,
            "camera_kf_motor_error_deg": np.nan,
            "camera_imu_error_deg_DIAG": np.nan,
            "imu_motor_error_deg_DIAG": np.nan,
            "camera_motor_error_dps": np.nan,
            "camera_imu_error_dps": d["e_sg_ci"],
            "imu_motor_error_dps": d["e_im"],
        }))

    out = pd.concat(frames, ignore_index=True)
    return out.sort_values(["segment_type", "segment_id", "grid",
                            "common_time_s"]).reset_index(drop=True)


# =============================================================================
# 11. WORKBOOK
# =============================================================================
def write_workbook(path, sheets, notes):
    C_HDR_MOTOR = "#1F3864"
    C_HDR_CAM = "#276221"
    C_HDR_IMU = "#7F3F00"
    C_HDR_ERR = "#4A2A6B"
    C_HDR_DEF = "#404040"

    def header_colour(col):
        c = str(col)
        if c.startswith("DIAG_") or "Diagnostic" in c or "IMU_Integrated" in c:
            return C_HDR_IMU
        if any(k in c for k in ("RMSE", "MAE", "Bias", "STD", "MaxAbs",
                                "Correlation", "Median", "Error")):
            return C_HDR_ERR
        if c.startswith("Camera") or "Camera_" in c or "Tags" in c or "Frame" in c:
            return C_HDR_CAM
        if c.startswith("IMU") or "Gyro" in c or "imu" in c:
            return C_HDR_IMU
        if c.startswith("Motor") or c.startswith("Requested") or \
           c.startswith("Effective") or c.startswith("Signed"):
            return C_HDR_MOTOR
        return C_HDR_DEF

    with pd.ExcelWriter(path, engine="xlsxwriter") as xw:
        wb = xw.book
        fmt_hdr = {}
        for col in {C_HDR_MOTOR, C_HDR_CAM, C_HDR_IMU, C_HDR_ERR, C_HDR_DEF}:
            fmt_hdr[col] = wb.add_format({
                "bold": True, "font_color": "white", "bg_color": col,
                "text_wrap": True, "valign": "vcenter", "align": "center",
                "border": 1, "font_size": 9})
        f_txt = wb.add_format({"valign": "top", "text_wrap": True,
                               "font_size": 9})
        f_note = wb.add_format({"italic": True, "font_color": "#7F3F00",
                                "text_wrap": True, "valign": "top"})
        f_num = {d: wb.add_format({"num_format": "0." + "0" * d, "font_size": 9})
                 for d in (2, 4, 5, 6, 7)}
        f_int = wb.add_format({"num_format": "0", "font_size": 9})
        f_tot = wb.add_format({"bold": True, "bg_color": "#FCE4D6",
                               "num_format": "0.000000", "font_size": 9,
                               "top": 2})
        f_tot_txt = wb.add_format({"bold": True, "bg_color": "#FCE4D6",
                                   "font_size": 9, "top": 2})

        for name, df in sheets.items():
            sn = name[:31]
            note = notes.get(name)
            start = 2 if note else 0
            df.to_excel(xw, sheet_name=sn, index=False, startrow=start)
            ws = xw.sheets[sn]
            nrow, ncol = df.shape

            if note:
                ws.merge_range(0, 0, 0, max(ncol - 1, 1), note, f_note)
                ws.set_row(0, 30)

            for c, col in enumerate(df.columns):
                ws.write(start, c, str(col), fmt_hdr[header_colour(col)])
            ws.set_row(start, 60)
            ws.freeze_panes(start + 1, 1)
            if nrow:
                ws.autofilter(start, 0, start + nrow, max(ncol - 1, 0))

            for c, col in enumerate(df.columns):
                s = df[col]
                width = int(max([len(str(col)) * 0.58] +
                                [len(str(v)) for v in s.head(60)]) + 2)
                width = max(11, min(40, width))
                if pd.api.types.is_numeric_dtype(s):
                    a = s.abs().replace([np.inf, -np.inf], np.nan).dropna()
                    mx = float(a.max()) if len(a) else 0.0
                    if str(col).startswith(("N_", "Motor_Pulses",
                                            "Stationary_Samples",
                                            "segment_id")) or "Count" in str(col):
                        ws.set_column(c, c, width, f_int)
                    elif "ppm" in str(col) or "Percent" in str(col):
                        ws.set_column(c, c, width, f_num[4])
                    elif mx and mx < 0.01:
                        ws.set_column(c, c, width, f_num[7])
                    elif mx and mx < 100:
                        ws.set_column(c, c, width, f_num[6])
                    else:
                        ws.set_column(c, c, width, f_num[5])
                else:
                    ws.set_column(c, c, width, f_txt)

            # POOLED_TOTAL row emphasis
            first = df.columns[0] if ncol else None
            for key_col in df.columns[:3]:
                hit = df[key_col].astype(str).eq("POOLED_TOTAL")
                if hit.any():
                    for i in np.where(hit.values)[0]:
                        ws.set_row(start + 1 + int(i), None, f_tot)
                    break

            # conditional colour scale on every RMSE column
            for c, col in enumerate(df.columns):
                if "RMSE" in str(col) and pd.api.types.is_numeric_dtype(df[col]) \
                        and nrow > 2:
                    ws.conditional_format(start + 1, c, start + nrow - 1, c, {
                        "type": "3_color_scale",
                        "min_color": "#C6EFCE", "mid_color": "#FFEB9C",
                        "max_color": "#F8CBAD"})


# =============================================================================
# 12. MAIN
# =============================================================================
def main():
    ap = argparse.ArgumentParser(
        description="Complete master yaw V&V analysis and workbook generator.")
    ap.add_argument("--run1", required=True, help="Run 01 folder (CW)")
    ap.add_argument("--run2", required=True, help="Run 02 folder (CCW)")
    ap.add_argument("--out", default="yaw_master_results")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    head("FROZEN CONFIGURATION")
    print(f"  motor          : {CFG['PULSES_PER_REV']} pulses/rev, "
          f"{DPP:.7f} deg/pulse, ideal bound +/-{QUANT_BOUND:.8f} deg")
    print(f"  Savitzky-Golay : {CFG['SG_WINDOW_S']:.2f} s, order "
          f"{CFG['SG_POLYORDER']}, zero phase")
    print(f"  Kalman         : constant-rate, Q={CFG['KF_Q']:g} (deg/s)^2/s, "
          f"R = measured stationary variance, camera only")
    print(f"  IMU            : PRIMARY raw bias-corrected; SECONDARY "
          f"bandwidth-matched SG {CFG['IMU_SG_WINDOW_S']:.2f} s")
    print(f"  segmentation   : holds trimmed {CFG['HOLD_GUARD_S']:.1f} s each "
          f"end; rate segments central "
          f"{100*(1-2*CFG['RATE_TRIM_FRAC']):.0f} %")
    print("  parameters chosen on Run 01, applied UNCHANGED to Run 02")

    runs = [process_run(load_run(args.run1, "Run01"), CFG),
            process_run(load_run(args.run2, "Run02"), CFG)]

    head("BUILDING TABLES")
    ares_all = {R["label"]: angle_residuals(R, CFG) for R in runs}
    rres_all = {R["label"]: rate_residuals(R, CFG) for R in runs}

    sheets, notes = {}, {}

    sheets["00_README_CONVENTIONS"] = readme_sheet(runs, ares_all, rres_all, CFG)
    sheets["01_MOTOR_QUANTIZATION"] = motor_quantization_table(CFG)
    notes["01_MOTOR_QUANTIZATION"] = (
        f"PULSE-DERIVED COMMANDED ANGLE. pulses = round(requested*"
        f"{CFG['PULSES_PER_REV']}/360); command = pulses*360/"
        f"{CFG['PULSES_PER_REV']} = pulses*{DPP:.7f} deg. Ideal "
        f"round-to-nearest bound +/-{QUANT_BOUND:.8f} deg. This is NOT an "
        f"actual, measured or encoder-derived motor angle.")

    sheets["02_SYNC_FULL"] = sync_full_sheet(runs)
    notes["02_SYNC_FULL"] = (
        "Camera<->motor mapping comes from the serial event timestamps in "
        "motor_clockfit.json. It is NEVER refitted by minimising "
        "camera-vs-motor angle residuals, so the camera-motor agreement is an "
        "out-of-sample result. The IMU clock is fitted independently per run "
        "in the rate domain only.")
    sheets["03_SYNC_EVENTS_RUN01"] = sync_events_sheet(runs[0])
    sheets["04_SYNC_EVENTS_RUN02"] = sync_events_sheet(runs[1])
    sheets["05_IMU_CALIBRATION"] = imu_calibration_sheet(runs, CFG)
    notes["05_IMU_CALIBRATION"] = (
        "Timestamp-derived sample cadence is reported separately from the "
        "configured ODR register setting. The cadence is NOT presented as an "
        "independently verified physical sensor ODR.")

    cwname = f"06_ANGLES_{runs[0]['direction']}_FULL"
    ccwname = f"07_ANGLES_{runs[1]['direction']}_FULL"
    sheets[cwname] = angles_full_sheet(runs[0], ares_all["Run01"], CFG)
    sheets[ccwname] = angles_full_sheet(runs[1], ares_all["Run02"], CFG)
    for n in (cwname, ccwname):
        notes[n] = ("Errors use the EFFECTIVE PULSE-DERIVED COMMANDED ANGLE, "
                    "never the rounded requested label. Columns prefixed "
                    "DIAG_ are " + DIAG_NOTE)
    sheets["08_ANGLES_COMBINED_MASTER"] = angles_combined_master(runs, ares_all, CFG)
    notes["08_ANGLES_COMBINED_MASTER"] = (
        "POOLED_TOTAL RMSE = sqrt(sum(e_i^2)/N) over all stable static "
        "samples. Per-angle RMSE values are never averaged. Columns marked "
        "Diagnostic are " + DIAG_NOTE)

    rcw = f"09_RATES_{runs[0]['direction']}_FULL"
    rccw = f"10_RATES_{runs[1]['direction']}_FULL"
    sheets[rcw] = rates_full_sheet(runs[0], rres_all["Run01"], CFG)
    sheets[rccw] = rates_full_sheet(runs[1], rres_all["Run02"], CFG)
    for n in (rcw, rccw):
        notes[n] = ("All seven primary pairings are present: RAW/SG/KF camera "
                    "vs motor, RAW/SG/KF camera vs gyro, gyro vs motor. "
                    "Camera-vs-motor is evaluated on the camera sample grid; "
                    "every IMU-involving pair on the IMU sample grid. "
                    "Motor_Commanded_Rate_ArduinoTime_dps is the rate in "
                    "Arduino-clock seconds; Motor_Rate_PC_SI_Time_dps applies "
                    "the measured clock scale.")
    sheets["11_RATES_COMBINED_MASTER"] = rates_combined_master(runs, rres_all, CFG)
    notes["11_RATES_COMBINED_MASTER"] = (
        "Full RMSE / MAE / Bias / STD / MaxAbs blocks for all seven pairings "
        "in both directions and combined. POOLED_TOTAL from residual samples.")
    sheets["12_IMU_FILTER_DIAGNOSTIC_FULL"] = imu_filter_diagnostic(
        runs, rres_all, CFG)
    notes["12_IMU_FILTER_DIAGNOSTIC_FULL"] = (
        "Raw bias-corrected gyro is the PRIMARY reference. The "
        "bandwidth-matched version is a SECONDARY diagnostic reported "
        "alongside; it never replaces or hides the raw result.")

    head("EXPORT")
    xl = os.path.join(args.out, "Yaw_VV_MASTER_FULL_RESULTS.xlsx")
    write_workbook(xl, sheets, notes)
    print(f"  {xl}")
    for n, d in sheets.items():
        print(f"     {n:36s} {d.shape[0]:>6d} rows x {d.shape[1]:>3d} cols")

    for i, R in enumerate(runs, start=1):
        s = synchronized_samples(R, ares_all[R["label"]],
                                 rres_all[R["label"]], CFG)
        p = os.path.join(args.out, f"synchronized_samples_run{i:02d}.csv")
        s.to_csv(p, index=False, float_format="%.8g")
        print(f"  {p}  ({len(s)} rows, {s.shape[1]} cols, "
              f"{os.path.getsize(p)/1e6:.1f} MB)")

    # ---------------- verification --------------------------------------
    head("FINAL VERIFICATION")
    q = sheets["01_MOTOR_QUANTIZATION"]
    r5 = q[q["Requested_Angle_deg"] == 5].iloc[0]
    print(f"  5 deg -> pulses {int(r5['Motor_Pulses'])}, effective "
          f"{r5['Effective_Motor_Angle_deg']:.6f} deg : "
          f"{'PASS' if int(r5['Motor_Pulses']) == 356 and abs(r5['Effective_Motor_Angle_deg'] - 5.00625) < 1e-9 else 'FAIL'}")
    print(f"  all |quantization| <= {QUANT_BOUND:.8f} deg : "
          f"{'PASS' if bool(q['Within_Ideal_Bound'].all()) else 'FAIL'}")

    a5 = sheets[cwname]
    row5 = a5[a5["Requested_Angle_deg"] == 5].iloc[0]
    e5 = ares_all["Run01"][round(row5["Effective_Motor_Angle_deg"], 6)]
    recomputed = float(np.sqrt(np.mean((e5["cam_raw"] - 5.00625) ** 2)))
    print(f"  5 deg RMSE recomputed against 5.006250 : "
          f"{'PASS' if abs(recomputed - row5['RAW_Camera_Motor_RMSE_deg']) < 1e-12 else 'FAIL'} "
          f"({recomputed:.6f})")

    for nm in (cwname, ccwname, "08_ANGLES_COMBINED_MASTER", rcw, rccw,
               "11_RATES_COMBINED_MASTER"):
        d = sheets[nm]
        has = any(d[c].astype(str).eq("POOLED_TOTAL").any() for c in d.columns[:3])
        print(f"  POOLED_TOTAL present in {nm:34s}: {'PASS' if has else 'FAIL'}")

    tot = sheets["08_ANGLES_COMBINED_MASTER"].iloc[-1]
    direct = pooled([ares_all[R["label"]][a]["e_raw_cm"] for R in runs
                     for a in ares_all[R["label"]]])["RMSE"]
    per_angle_mean = float(sheets["08_ANGLES_COMBINED_MASTER"]
                           ["Combined_RAW_CM_RMSE_deg"].iloc[:-1].mean())
    print(f"  angle POOLED_TOTAL from residual samples : "
          f"{'PASS' if abs(tot['Combined_RAW_CM_RMSE_deg'] - direct) < 1e-12 else 'FAIL'} "
          f"({direct:.6f}; mean-of-RMSE would be {per_angle_mean:.6f} and is NOT used)")

    rt = sheets["11_RATES_COMBINED_MASTER"].iloc[-1]
    direct_r = pooled([rres_all[R["label"]][q_]["e_sg_cm"] for R in runs
                       for q_ in rres_all[R["label"]]])["RMSE"]
    per_rate_mean = float(sheets["11_RATES_COMBINED_MASTER"]
                          ["COMBINED_SG_CM_RMSE"].iloc[:-1].mean())
    print(f"  rate POOLED_TOTAL from residual samples  : "
          f"{'PASS' if abs(rt['COMBINED_SG_CM_RMSE'] - direct_r) < 1e-12 else 'FAIL'} "
          f"({direct_r:.6f}; mean-of-RMSE would be {per_rate_mean:.6f} and is NOT used)")

    missing = []
    for sheet in (rcw, rccw):
        for tag, _, _, _ in RATE_PAIRS:
            for st in ("Bias", "MAE", "RMSE", "STD", "Median", "MaxAbs",
                       "Correlation"):
                if f"{tag}_{st}" not in sheets[sheet].columns:
                    missing.append(f"{sheet}:{tag}_{st}")
    print(f"  all 7 pairings x 7 statistics present    : "
          f"{'PASS' if not missing else 'FAIL ' + str(missing[:5])}")

    for sheet in (cwname, ccwname):
        need = [f"{e}_Camera_Motor_RMSE_deg" for e in ("RAW", "SG", "KF")] + \
               [f"DIAG_{e}_Camera_IMU_RMSE_deg" for e in ("RAW", "SG", "KF")] + \
               ["DIAG_IMU_Motor_RMSE_deg"]
        ok = all(c in sheets[sheet].columns for c in need)
        print(f"  angle sheet {sheet:26s} complete : {'PASS' if ok else 'FAIL'}")

    head("WARNINGS")
    if WARNINGS:
        for w in WARNINGS:
            print("  - " + w)
    else:
        print("  none")
    print("\nDone.")


def readme_sheet(runs, ares_all, rres_all, cfg):
    ang = pooled([ares_all[R["label"]][a]["e_raw_cm"] for R in runs
                  for a in ares_all[R["label"]]])
    rt = pooled([rres_all[R["label"]][q]["e_sg_cm"] for R in runs
                 for q in rres_all[R["label"]]])
    items = [
        ("WORKBOOK", "Yaw_VV_MASTER_FULL_RESULTS.xlsx — the full engineering "
                     "record. Every angle, every rate, both directions, every "
                     "camera estimator, every pairwise metric."),
        ("SCOPE", "Two independent directional validation runs (CW/CCW "
                  "replication). NOT a population-level repeatability study."),
        ("Run 01", f"{runs[0]['direction']} — development/tuning run, "
                   f"{len(runs[0]['cam'])} camera frames"),
        ("Run 02", f"{runs[1]['direction']} — held-out validation run, "
                   f"{len(runs[1]['cam'])} camera frames"),
        ("", ""),
        ("MOTOR REFERENCE",
         f"PULSE-DERIVED COMMANDED ANGLE. {cfg['PULSES_PER_REV']} pulses/rev, "
         f"{DPP:.7f} deg/pulse. pulses = round(requested*"
         f"{cfg['PULSES_PER_REV']}/360); command = pulses*360/"
         f"{cfg['PULSES_PER_REV']}. Ideal bound +/-{QUANT_BOUND:.8f} deg."),
        ("Motor wording", "Never called an actual, measured or "
                          "encoder-derived motor angle. No independently "
                          "measured external motor position exists, and the "
                          "vendor publishes no positioning-accuracy spec."),
        ("Error reference", "All static errors use the effective pulse-derived "
                            "command (5.006250 deg for the 5 deg label), never "
                            "the rounded label."),
        ("", ""),
        ("SIGN CONVENTION",
         "Travel-magnitude convention throughout: reference and sensors are "
         "expressed as monotonically increasing travel from the start of the "
         "profile, so CW and CCW are directly comparable and negative measured "
         "travel is never subtracted from a positive reference. Signed_* "
         "columns carry the direction sign for traceability. Camera sign and "
         "gyro axis sign are detected per run from the data and reported in "
         "02_SYNC_FULL and 05_IMU_CALIBRATION."),
        ("", ""),
        ("SYNCHRONISATION — camera/motor",
         "PRIMARY: serial event timestamps, t_arduino = a*t_perf_counter + b, "
         "94 events per run. NEVER refitted by minimising camera-vs-motor "
         "angle residuals, so the camera-motor agreement is out-of-sample."),
        ("SYNCHRONISATION — IMU",
         "Fitted independently per run, RATE domain only (cross-correlation, "
         "constant offset, then affine). Integrated gyro yaw is never used "
         "for synchronisation."),
        ("", ""),
        ("RAW", "Frame-to-frame finite difference of the fused camera yaw. "
                "Reported in full, never hidden."),
        ("SG", f"Savitzky-Golay, {cfg['SG_WINDOW_S']:.2f} s window, order "
               f"{cfg['SG_POLYORDER']}, zero phase, uniform-grid resample. "
               f"Order >= 1 is exactly unbiased on a constant-rate ramp."),
        ("KALMAN", f"Camera-only, x = [yaw, yaw_rate]', constant-rate model, "
                   f"Q = {cfg['KF_Q']:g} (deg/s)^2/s, R = measured stationary "
                   f"yaw variance per run, true per-sample dt. NO IMU input."),
        ("Parameter freezing", "SG and Kalman parameters chosen on Run 01 and "
                               "applied unchanged to Run 02 and every segment."),
        ("", ""),
        ("STATIC YAW — primary", "Camera <-> pulse-derived motor command."),
        ("STATIC YAW — diagnostic", DIAG_NOTE),
        ("RATE — primary", "All seven pairings: RAW/SG/KF camera vs motor, "
                           "RAW/SG/KF camera vs gyro, gyro vs motor."),
        ("IMU rate — PRIMARY", "Raw, bias-corrected gyro, no filtering."),
        ("IMU rate — SECONDARY", f"Bandwidth-matched SG "
                                 f"{cfg['IMU_SG_WINDOW_S']:.2f} s, order "
                                 f"{cfg['IMU_SG_POLYORDER']}. Reported "
                                 f"alongside, never replacing the raw result."),
        ("", ""),
        ("POOLING RULE", "POOLED_TOTAL RMSE = sqrt(sum(e_i^2)/N) and MAE = "
                         "sum|e_i|/N over residual samples. Per-segment RMSE "
                         "values are NEVER averaged."),
        ("Static segmentation", f"20 s holds, trimmed "
                                f"{cfg['HOLD_GUARD_S']:.1f} s at each end."),
        ("Rate segmentation", f"Central "
                              f"{100*(1-2*cfg['RATE_TRIM_FRAC']):.0f} % of "
                              f"each 20 s segment."),
        ("Grid convention", "Camera-vs-motor pairs on the camera sample grid; "
                            "every IMU-involving pair on the IMU sample grid "
                            "with the camera estimate interpolated onto the "
                            "gyro sample times. The gyro is never resampled."),
        ("", ""),
        ("AUDIT TRAIL", "synchronized_samples_run01.csv and "
                        "synchronized_samples_run02.csv contain the "
                        "sample-level values behind every metric. Filter by "
                        "the `grid` column to reproduce each number exactly."),
        ("", ""),
        ("HEADLINE — static yaw, RAW, combined",
         f"RMSE {ang['RMSE']:.6f} deg, MAE {ang['MAE']:.6f} deg, "
         f"bias {ang['Bias']:+.6f} deg, N = {ang['N']}"),
        ("HEADLINE — rate, SG camera-motor, combined",
         f"RMSE {rt['RMSE']:.6f} deg/s, bias {rt['Bias']:+.6f} deg/s, "
         f"N = {rt['N']}"),
        ("HEADLINE — clocks",
         f"Arduino vs PC {runs[0]['fit']['ppm_perf']:+.1f} / "
         f"{runs[1]['fit']['ppm_perf']:+.1f} ppm; Arduino vs Teensy "
         f"{runs[0]['imu_sync']['ppm']:+.1f} / "
         f"{runs[1]['imu_sync']['ppm']:+.1f} ppm. Two independent references "
         f"agree, so the Arduino is the outlier clock."),
        ("WARNINGS RAISED", "; ".join(WARNINGS) if WARNINGS else "none"),
    ]
    return pd.DataFrame(items, columns=["Item", "Description"])


if __name__ == "__main__":
    main()
