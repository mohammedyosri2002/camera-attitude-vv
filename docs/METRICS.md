# Metrics

## Raw / filter policy

Every headline number in this repository and in the paper is **RAW**.

**Not used anywhere in a headline result:**

- low-pass filtering
- moving average
- Savitzky-Golay smoothing
- Kalman filtering
- fitted camera scale factor
- fitted IMU scale factor
- drift subtraction
- zero-return interpolation correction

Angular rate is the **plain finite difference** of the raw attitude on the true,
non-uniform sample times. Nothing is smoothed before differentiation.

The yaw dataset supports offline Savitzky-Golay and Kalman rate estimators. They are
retained in `results/yaw.json` under the key
`secondary_filtered_diagnostics_DO_NOT_COMPARE_WITH_RAW`, are never substituted into the
three-axis RAW comparison, and are **never** compared with a RAW out-of-plane derivative.
`tests/test_expected_results.py` asserts that no filtered result appears in the RAW
dynamic block.

## Segmentation, not sample rejection

- Static dwells are trimmed **1.5 s at each end** to exclude settling transients.
- Constant-rate legs retain their **central 70 %** to exclude acceleration and deceleration.

This is segmentation of *which interval* is a valid steady-state observation. Inside the
retained window, no sample is discarded.

## Pooling

    RMSE = sqrt( sum(e_i^2) / N )    over residual SAMPLES

Per-segment RMSE values are **never averaged**. Bias, MAE, STD, MaxAbs and N are reported
alongside every RMSE.

## Static comparisons

Three RAW pairings per out-of-plane run:

1. **camera − commanded reference**
2. **gravity tilt − commanded reference**
3. **camera − gravity tilt**

The gravity tilt is evaluated on the IMU sample grid and interpolated onto the camera grid
for pairing (3). The yaw runs have no gravity-tilt reference — the yaw acquisition logged no
roll/pitch — so pairings (2) and (3) are **N/A**, not estimated.

## Dynamic comparisons

Three RAW pairings:

1. **camera RAW finite-difference rate − commanded rate**
2. **raw bias-corrected gyro − commanded rate**
3. **camera RAW rate − raw gyro**

Positive and negative legs are kept separate before pooling so that a signed bias cannot
cancel.

### Interpreting the raw derivative

The raw derivative RMSE is dominated by **numerical differentiation noise**, not attitude
error: differentiating a signal with a few hundredths of a degree of per-frame scatter at
80–120 Hz produces a large, near-zero-mean residual. The signed bias is far smaller than
the RMSE in every run. Do **not** state that the signed bias is below a single threshold
across all axes — the yaw value is about 0.0067 deg/s while the out-of-plane values are
around 0.003–0.004 deg/s. Quote each axis's own value.

The raw result is reported honestly. The tracking-fidelity quantity is the rate scale below.

### Filtered rate: SECONDARY DIAGNOSTIC ONLY

`analysis/common/rate_estimators.py` holds ONE frozen Savitzky-Golay estimator
(1.00 s window, polynomial order 2, first derivative, zero
phase, uniform-grid resample), taken unchanged from the established yaw method and applied
identically to Yaw, Board X and Board Y over the same constant-rate leg windows as the RAW
result. It is **not tuned per axis** and **not optimised to minimise any error**; the window
is set in seconds so all axes share a physical bandwidth despite different frame rates.

Reported SG RMSE: yaw 0.0421, Board X 0.1196,
Board Y 0.1241 deg/s.

An SG figure is **never** labelled a RAW accuracy, never substituted into the RAW
comparison, and never quoted without naming the estimator and its bandwidth. A polynomial of
order >= 1 reproduces a constant-rate ramp exactly, so inside a segment the window sets
variance only, never bias -- which is what makes one shared window honest across axes.
Output: `results/tables/three_axis_rate_comparison.csv` with `RAW_`, `SG_SECONDARY_` and
`DISPSCALE_` column prefixes.

## Rate-scale metric

For each constant-rate leg:

    camera displacement rate = (total camera attitude displacement) / (elapsed time)

compared with the commanded rate, positive and negative legs separate.

**This is a rate-SCALE metric. It is not an instantaneous rate RMSE and must never be
quoted as one.** It carries no differentiation noise, so it measures how faithfully the
camera tracks commanded angular travel.

Output: `results/tables/board_y_rate_scale.csv`.

## Conditioning metrics

Recomputed from raw corner pixels for **every** run, independently of which pose source
supplies the attitude:

- valid board pose fraction
- pose-branch flips (frame-to-frame jumps beyond 20 deg)
- ambiguous frames (the two IPPE candidates within a factor of 1.5)
- board reprojection error: mean, median, p95, max
- IPPE candidate separation: min, median, max
- second-to-selected reprojection ratio: min, median, max
- board tilt from fronto-parallel: min, max
- per-tag and four-tag visibility

## Zero-return departure and hysteresis

Both are **reported and never removed** from headline values.

At each visit to the commanded zero, the camera and the gravity tilt are recorded
independently (`results/tables/zero_return_drift.csv`). Where both sensors observe the same
departure from the commanded position, that departure is **measured platform motion
relative to the commanded reference**, not camera error.

No mechanical cause is assigned to it. The data establish that the motion is real; they do
not identify what produced it.
