"""
analysis/common/attitude_metrics.py
===================================
RAW error metrics. CREATED POST-CAMPAIGN.

POLICY -- applies to every headline number in this repository:
  * No low-pass, no moving average, no Savitzky-Golay, no Kalman.
  * No fitted camera scale factor. No fitted IMU scale factor.
  * No drift subtraction. No zero-return interpolation correction.
  * Angular rate is the PLAIN finite difference of the raw attitude, unsmoothed.
  * Pooling is over residual SAMPLES: RMSE = sqrt(sum(e^2)/N).
    Per-segment RMSE values are never averaged.
  * Hold and leg trimming is SEGMENTATION (excluding transients at segment edges),
    not sample rejection inside the retained window.
"""
from __future__ import annotations
import numpy as np

HOLD_GUARD_S = 1.5      # trimmed from each end of every static dwell
LEG_TRIM_FRAC = 0.15    # central 70 % of every constant-rate leg retained


def metrics(e):
    e = np.asarray(e, float)
    e = e[np.isfinite(e)]
    if e.size == 0:
        return dict(Bias=np.nan, MAE=np.nan, RMSE=np.nan, STD=np.nan,
                    MaxAbs=np.nan, N=0)
    return dict(Bias=float(e.mean()),
                MAE=float(np.abs(e).mean()),
                RMSE=float(np.sqrt((e ** 2).mean())),
                STD=float(e.std(ddof=1)) if e.size > 1 else float("nan"),
                MaxAbs=float(np.abs(e).max()),
                N=int(e.size))


def pooled(parts):
    parts = [np.asarray(p, float) for p in parts if p is not None and len(p)]
    return metrics(np.concatenate(parts)) if parts else metrics([])


def finite_difference_rate(theta, t):
    """RAW angular rate. np.gradient on the true, non-uniform sample times.

    The residual of this estimator against a commanded rate is dominated by
    differentiation noise, not by attitude error: the signed bias is orders of magnitude
    below the RMSE. That is a property of differentiating a noisy signal at 80-120 Hz, and
    it is reported honestly rather than filtered away. The rate-SCALE metric in
    displacement_rate() is the tracking-fidelity quantity.
    """
    return np.gradient(np.asarray(theta, float), np.asarray(t, float))


def displacement_rate(theta, t):
    """Rate-SCALE metric: total attitude displacement over elapsed time for one leg.

    NOT an instantaneous rate RMSE and must never be quoted as one. It carries no
    differentiation noise, so it measures how faithfully the camera tracks the commanded
    angular travel.
    """
    theta = np.asarray(theta, float); t = np.asarray(t, float)
    if theta.size < 2:
        return float("nan")
    return float((theta[-1] - theta[0]) / (t[-1] - t[0]))
