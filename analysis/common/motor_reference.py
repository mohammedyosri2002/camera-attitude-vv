"""
analysis/common/motor_reference.py
==================================
The PULSE-DERIVED COMMANDED REFERENCE and the commanded profile segmentation.

CREATED POST-CAMPAIGN for reproduction from archived raw data.

TERMINOLOGY -- this is not negotiable anywhere in this repository:
    The motor signal is a PULSE-DERIVED COMMANDED REFERENCE (or COMMANDED REFERENCE).
    It is NOT ground truth, NOT an actual angle, NOT a measured motor angle.
    No independent encoder-position telemetry was logged for this campaign.
"""
from __future__ import annotations
import math
import numpy as np
import pandas as pd

PULSES_PER_REV = 25600
DEG_PER_PULSE = 360.0 / PULSES_PER_REV            # 0.0140625 deg
QUANTIZATION_BOUND_DEG = DEG_PER_PULSE / 2.0      # +/- 0.00703125 deg


def angle_to_pulses(deg: float) -> int:
    """Round-to-nearest in float32, matching the firmware arithmetic exactly."""
    v = np.float32(abs(deg)) * np.float32(PULSES_PER_REV) / np.float32(360.0)
    p = int(math.floor(float(v) + 0.5))
    return -p if deg < 0 else p


def effective_commanded_deg(deg_requested: float) -> float:
    """The commanded reference actually realisable by the pulse generator.

        5 deg  ->  356 pulses ->  5.0062500 deg
       10 deg  ->  711 pulses ->  9.9984375 deg
       15 deg  -> 1067 pulses -> 15.0046875 deg
       20 deg  -> 1422 pulses -> 19.9968750 deg

    All errors in this repository are computed against these values, never against the
    nominal request.
    """
    return angle_to_pulses(deg_requested) * DEG_PER_PULSE


def segment_profile(ev: pd.DataFrame):
    """Segment the commanded profile from the logged serial events.

    Returns (holds, legs, motion_segments, baselines, T0, T1) in ARDUINO-clock seconds.
      holds  : [{t0, t1, target_deg, pulses}]        static dwells
      legs   : [{t0, t1, rate_dps}]                  constant-rate legs, signed
      motion : [(t0, t1, |rate|)]                    every commanded motion incl. transits
    """
    te = ev["t_arduino_us"].to_numpy(float) * 1e-6
    e = ev["event"].to_numpy()
    val = pd.to_numeric(ev["value"], errors="coerce").to_numpy(float)
    ext = pd.to_numeric(ev["extra"], errors="coerce").to_numpy(float)

    holds, legs, motion = [], [], []
    ts = None
    for i in range(len(ev)):
        k = e[i]
        if k == "HOLD_START":
            ts = te[i]; tgt = ext[i] * DEG_PER_PULSE; pul = int(round(ext[i]))
        elif k == "HOLD_END":
            holds.append(dict(t0=ts, t1=te[i], target_deg=tgt, pulses=pul))
        elif k == "RATE_LEG_START":
            ts = te[i]; rr = val[i]; motion.append([ts, None, abs(rr)])
        elif k == "RATE_LEG_END":
            legs.append(dict(t0=ts, t1=te[i], rate_dps=rr)); motion[-1][1] = te[i]
        elif k in ("MOVE_START", "RATE_POSITION_START", "RETURN_ZERO_START"):
            ts = te[i]; motion.append([ts, None, 5.0])
        elif k in ("MOVE_END", "RATE_POSITION_END", "RETURN_ZERO_END"):
            motion[-1][1] = te[i]
    motion = [tuple(m) for m in motion if m[1] is not None]

    def _win(a, b):
        ia = np.where(e == a)[0]; ib = np.where(e == b)[0]
        return (te[ia[0]], te[ib[0]]) if len(ia) and len(ib) else None

    baselines = dict(start=_win("BASELINE_START", "BASELINE_START_END"),
                     end=_win("BASELINE_END", "BASELINE_END_END"))
    return holds, legs, motion, baselines, te[0], te[-1]


def commanded_rate_profile(motion, t):
    """|commanded rate| at Arduino-clock times t. Used only for IMU clock alignment."""
    t = np.atleast_1d(np.asarray(t, float))
    out = np.zeros_like(t)
    for t0, t1, r in motion:
        out[(t >= t0) & (t < t1)] = r
    return out
