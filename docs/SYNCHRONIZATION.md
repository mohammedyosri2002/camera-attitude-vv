# Synchronization

Three clocks free-run independently: the camera host PC, the motor controller (Arduino) and
the IMU host (Teensy). Every comparison in this work is made **after** alignment, so that a
timing offset is never read as an attitude error.

## Camera to motor

Affine map, fitted by ordinary least squares:

    t_motor = a * t_camera_host + b

fitted to the **serial motor-event timestamps only**. Each profile transition is stamped
with the controller's `micros()` and with the host's `perf_counter_ns()` on receipt; 114
such events exist per out-of-plane run and 94 per yaw run.

**The map is never optimised against camera-versus-motor attitude error.** The
camera-motor agreement reported in the paper is therefore out of sample with respect to the
synchronisation. `analysis/common/camera_motor_sync.py` records this in its `fitted_to`
field and `tests/test_synchronization.py` asserts it.

A wall-clock fit is computed alongside the monotonic-clock fit as a cross-check; the two
agree to better than 1 ppm on every run.

### On reporting the ppm figure

The ppm value is `(a - 1) * 1e6` **for the map above** — the signed fractional rate of the
controller clock expressed in the host time base. Report the coefficient and the signed ppm
together. Do **not** paraphrase it as "the Arduino is N ppm fast" without restating the map
and its direction: that phrasing is only meaningful once the sign convention and which
clock is the independent variable are both fixed.

## IMU to motor

Fitted **independently, in the gyro rate domain**, by matching the smoothed magnitude of the
dominant gyro channel against the |commanded rate| profile. **The camera is never used.**
The smoothing exists only inside the fit objective and touches no reported measurement.

### The >= 95 % coverage constraint

The commanded pre-position workflow introduces a long and variable delay between controller
reset and the start of the profile, which moves the IMU-to-motor offset far from any naive
search window. Without a constraint the optimiser can select a **partial-overlap minimum**
in which only a stationary tail of the IMU record overlaps a stationary part of the motor
profile: the residual is tiny and the solution is nonsense.

This was observed in practice as a physically implausible

    a = 1.033994589   (+33 994.6 ppm)

The fit is therefore restricted to solutions whose mapped IMU record covers at least
**95 %** of the motor experiment window. With the constraint the same data give
`a = 0.998395237` (−1604.8 ppm) at 100 % coverage for RUN05.

`MIN_COVERAGE` is defined in `analysis/common/imu_motor_sync.py`;
`tests/test_synchronization.py` asserts both that the constraint is active and that every
reproduced slope lies in a physically plausible band, which rejects the artefact.

## Reproduced coefficients

| Run | camera↔motor a | ppm | residual SD [ms] | events | IMU↔motor a | ppm | residual [deg/s] | coverage [%] |
|---|---|---|---|---|---|---|---|---|
| `board_x/axis_b_run01` | 0.998483425 | -1516.6 | 1.213 | 114 | 0.998433646 | -1566.4 | 0.3242 | 100.00 |
| `board_x/axis_a` | 0.998503709 | -1496.3 | 1.222 | 114 | 0.998424304 | -1575.7 | 0.3481 | 100.00 |
| `board_x/axis_b_run04` | 0.998487897 | -1512.1 | 1.181 | 114 | 0.998627271 | -1372.7 | 0.3478 | 100.00 |
| `board_y/run05` | 0.998498247 | -1501.8 | 1.247 | 114 | 0.998395237 | -1604.8 | 0.3493 | 100.00 |
| `yaw/Run01` (CW) | — | -1516.3 | 1.227 | 94 | — | -1516.4 | 0.4084 | — |
| `yaw/Run02` (CCW) | — | -1513.5 | 1.131 | 94 | — | -1499.0 | 0.4068 | — |

Machine-readable: `results/tables/synchronization_summary.csv` and the `sync` block of each
run in `results/reproduced_numbers.json`.

## Independent cross-check

Cross-correlating the raw camera rate against the gyro over the rate phase bounds the
relative camera-IMU alignment without using either to set the motor map. This is a
diagnostic, not an input to any fit.
