"""
analysis/common/rate_estimators.py
==================================
ONE FROZEN Savitzky-Golay rate estimator, applied identically to all three headline axes.
CREATED POST-CAMPAIGN.

STATUS: SECONDARY DIAGNOSTIC ONLY.
The primary reported camera rate in this work is the RAW finite difference in
analysis/common/attitude_metrics.py. Nothing here replaces it. An SG result is never
labelled a RAW accuracy, never substituted into the RAW comparison, and never quoted as
the camera's rate accuracy without naming the estimator and its bandwidth.

THE ESTIMATOR IS FROZEN
-----------------------
Parameters are taken unchanged from the established yaw method
(analysis/yaw/yaw_vv_master_analysis.py: sg_value_and_derivative, SG_WINDOW_S = 1.00,
SG_POLYORDER = 2) and are applied to Yaw, Board X and Board Y without modification:

    window          1.00 s          specified in SECONDS, so all axes share a physical
                                    bandwidth despite different frame rates
    polynomial      order 2
    derivative      first
    grid            uniform resample at the median sample interval, filtered, mapped back
    phase           zero phase, offline (scipy.signal.savgol_filter is symmetric)

The window is NOT tuned per axis and NOT optimised to minimise any error. A polynomial of
order >= 1 reproduces a constant-rate ramp exactly, so inside a constant-rate segment the
window sets variance only, never bias -- which is why one window can be shared honestly
across axes with different frame rates.

WHAT AN SG NUMBER MEANS
-----------------------
It is a bandwidth-limited estimate. The RAW derivative and the SG estimate answer different
questions: the RAW derivative reports the unprocessed per-sample disagreement, dominated by
differentiation noise; the SG estimate reports the disagreement after the noise above
~1 Hz has been removed. Neither is "the" camera rate accuracy on its own.
"""
from __future__ import annotations

import numpy as np
from scipy.signal import savgol_filter

# ---- FROZEN. Do not tune per axis. -------------------------------------------------
SG_WINDOW_S = 1.00
SG_POLYORDER = 2


def sg_value_and_derivative(t, y, win_s=SG_WINDOW_S, poly=SG_POLYORDER):
    """Zero-phase Savitzky-Golay value and first derivative.

    Identical in form to the established yaw implementation: resample onto a uniform grid
    at the median sample interval, filter, then map back to the original (slightly
    non-uniform) sample times.

    Returns (value, derivative, window_samples, sample_rate_hz) or (None, None, n, fs) if
    the window is shorter than the polynomial order.
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    fs = 1.0 / np.median(np.diff(t))
    n = int(round(win_s * fs)) | 1
    if n <= poly:
        return None, None, n, fs
    tu = np.arange(t[0], t[-1], 1.0 / fs)
    yu = np.interp(tu, t, y)
    val = savgol_filter(yu, n, poly, deriv=0)
    der = savgol_filter(yu, n, poly, deriv=1, delta=1.0 / fs)
    return np.interp(t, tu, val), np.interp(t, tu, der), n, fs


def sg_rate(t, theta, win_s=SG_WINDOW_S, poly=SG_POLYORDER):
    """SG angular rate only. SECONDARY DIAGNOSTIC -- never a RAW result."""
    _, der, n, fs = sg_value_and_derivative(t, theta, win_s, poly)
    return der, n, fs


def describe():
    return dict(estimator="Savitzky-Golay, zero phase, offline",
                window_s=SG_WINDOW_S, polyorder=SG_POLYORDER, derivative=1,
                grid="uniform resample at the median sample interval, mapped back",
                frozen=True,
                tuned_per_axis=False,
                optimised_to_minimise_error=False,
                provenance=("parameters unchanged from the established yaw method in "
                            "analysis/yaw/yaw_vv_master_analysis.py"),
                status=("SECONDARY DIAGNOSTIC ONLY - never replaces the RAW finite "
                        "difference and is never labelled a RAW accuracy"))
