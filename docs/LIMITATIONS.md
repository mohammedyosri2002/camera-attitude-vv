# Limitations

Stated plainly. A reviewer should not have to find these.

## 1. The motor command is not encoder-verified ground truth

The motor signal is a **pulse-derived commanded reference**. Quantization is bounded at
±0.00703125 deg and does not accumulate, but **physical fidelity is uncharacterised**: no
independent encoder-position telemetry was logged. Any statement that the camera "agrees to
X deg" is a statement about agreement with a *commanded position* whose own error is
unknown — and, per item 5 below, demonstrably nonzero.

## 2. Board geometry is camera-derived

Both layout files are camera-derived (`camera_derived_planar_homography` and
`camera_derived_bundle_adjustment_RUN05`). **Neither is an independent dimensional
reference and neither was mechanically measured.** The RUN05 layout's absolute scale comes
from an entered tag size held fixed at 0.070 m. Mechanical measurement of the tag
centre-to-centre distances and the printed black-square edge is outstanding.
See [`GEOMETRY.md`](GEOMETRY.md).

## 2b. Board-Y values were revised twice by intrinsics audits

The Board-Y figures originally submitted used the raw `camera_matrix` on undistorted corner
coordinates, both for the pose and for the bundle-adjusted board layout. The final values
use `newK` for both: static camera − commanded RMSE **0.2918 deg** rather than
0.2987 deg, camera − gravity RMSE **0.2474 deg** rather than
0.2641 deg, and a rate-scale range of **-0.70 % to +0.52 %** rather than
-0.47 % to +0.75 %. Board-X and yaw are unaffected. See
[`GEOMETRY.md`](GEOMETRY.md). **Any paper text quoting superseded Board-Y figures must be
updated.**

## 3. No motion-capture or external optical reference was used

The accepted abstract mentioned such a system only conditionally ("where available"). None
was available and none was used. No claim in this work rests on one.

## 4. The IMU was not independently precision-calibrated

Neither the accelerometer nor the gyroscope was calibrated against an external standard.

- **Gyro**: only a stationary bias from each run's own baseline is subtracted. **No scale
  factor is fitted**, to the motor or the camera, so the datasheet sensitivity tolerance
  remains in the error budget and is coarser than the camera-versus-motor rate
  disagreement. That makes the camera-versus-gyro rate comparison reference-limited rather
  than camera-limited.
- **Accelerometer**: the gravity tilt is the angle between two **normalised** direction
  vectors, so a common gain error largely cancels. The residual sensitivity is to per-axis
  bias and cross-axis terms, not to the magnitude at rest. **Do not infer a tilt-angle
  error directly from the accelerometer magnitude reading at rest** — that inference does
  not follow. The honest statement is that the unit is uncalibrated and therefore carries
  an uncorrected per-axis bias and cross-axis error of unquantified size.

## 5. Platform departure from the commanded reference is real, and its cause is unknown

At repeated visits to the commanded zero, camera and gravity tilt independently report the
same departure (camera/gravity, deg):

- Board X (`axis_b_run01`): -0.0070/+0.0005, +0.2338/+0.2621, +0.2957/+0.3378
- Board Y (`run05`): -0.0096/-0.0009, +0.2340/+0.2504, +0.3471/+0.3673

Two sensors on independent clocks using independent physics agree closely while both report
a departure from the commanded position. This is **measured platform motion relative to the
commanded reference, not camera error**.

**No mechanical cause is assigned.** The data establish that the motion is real; they do
not identify what produced it. It is **not removed** from any headline number.

## 6. Headline results are directional tests, not population statistics

Board X and Board Y each have **one headline run**. Two further Board-X runs exist as
replication under different camera-view and IMU-mounting configurations. This is
directional replication, not a population-level repeatability study, and no run-to-run
standard deviation should be quoted from it.

## 7. Raw camera rate derivatives contain numerical differentiation noise

The RAW finite-difference rate RMSE is dominated by differentiation noise rather than
attitude error; the signed bias is far smaller than the RMSE. The rate-SCALE metric
(displacement per leg) is the tracking-fidelity quantity and must not be relabelled as an
instantaneous rate RMSE. Each axis's signed bias differs — quote each axis's own value
rather than a single threshold across all of them.

## 7b. Filtered rate results are bandwidth-limited, not accuracy

The Savitzky-Golay figures in `results/tables/three_axis_rate_comparison.csv` are a
SECONDARY diagnostic from one frozen estimator (1.00 s, order 2,
zero phase) applied identically to all three axes. They remove noise above roughly 1 Hz and
therefore answer a different question from the RAW derivative. They must never be presented
as the camera's rate accuracy, never substituted into a RAW comparison, and never compared
with a RAW derivative from another axis. SG RMSE is below 0.1 deg/s for yaw
(0.0421) but not for Board X (0.1196) or
Board Y (0.1241).

## 8. Acquisition frame rates differ between runs

Median camera rate: `axis_b_run01` 80.33 Hz, `axis_a` 117.10 Hz, `axis_b_run04` 113.30 Hz, `run05` 120.77 Hz.
Because the raw derivative's noise depends on sample rate, **raw dynamic results are not
directly comparable between runs** without accounting for this.

## 9. Tag visibility changes at high tilt

One tag drops out beyond about 40 deg of board tilt in the out-of-plane runs, reducing the
solve from 16 corners to 12. Four-tag visibility: `axis_b_run01` 87.4727 %, `axis_a` 84.1011 %, `axis_b_run04` 87.0517 %, `run05` 89.3959 %.

Conditioning remains good on three tags. Critically, the three-tag subset is **fully
confounded with commanded angle and sign** — it occurs only at the negative extremes — so
any difference between four-tag and three-tag statistics **cannot be attributed to tag
count** from this data.

## 10. Rate ranges differ between experiments

Yaw ran to 30 deg/s over a large travel; the out-of-plane runs to 10 deg/s over ±10 deg.
The yaw and out-of-plane dynamic columns are **not** a like-for-like comparison and must
not be presented as ranking the axes.

## 11. Conditioning results are configuration-specific

Every reported figure is conditioned on this viewing geometry, this board, this working
point and this commanded pre-position. Nothing here establishes an accuracy figure for
camera-based attitude measurement in general.

## 12. What this campaign does not establish

- an accuracy figure for the method in general;
- absolute board dimensions;
- a camera scale factor (none was fitted);
- the cause of the platform's departure from the commanded reference;
- population-level repeatability on either out-of-plane axis;
- any comparison between a filtered yaw rate result and a RAW out-of-plane derivative.
