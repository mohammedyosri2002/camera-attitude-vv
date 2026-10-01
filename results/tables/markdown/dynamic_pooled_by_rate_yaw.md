# Dynamic RAW pooled by rate magnitude - Yaw

RAW MAE / RMSE / MaxAbs pooled by rate magnitude; signed bias kept direction-separated.

Machine-readable source: [`../dynamic_pooled_by_rate_yaw.csv`](../dynamic_pooled_by_rate_yaw.csv)

**Notes**

- Motor quantity = PULSE-DERIVED COMMANDED REFERENCE. Not ground truth, not an actual angle, not a measured motor angle. No encoder-position telemetry was logged.
- N is PER COMPARISON. Evaluation grids differ by comparison; see the evaluation_grid columns and README 'Detailed Result Tables'.
- RAW-rate MaxAbs is a sample extreme and is not interpreted as a deterministic error bound; it scales with sample count.
- Signed bias is reported direction-separated only. No pooled signed-bias column exists, because pooling opposite directions cancels it.
- Yaw direction is by RUN: CW = CW, CCW = CCW. Signed bias is reported per run and never pooled across directions.

**Column abbreviations.** `Cam` = camera, `Cmd` = pulse-derived commanded reference, `Grav` = gravity-derived tilt, `Dir` = direction, `SG` = SECONDARY Savitzky-Golay processed rate, `|w|` = magnitude.

Evaluation-grid columns are omitted here because each is constant for its column; see [`../README.md`](../README.md) for the grid of every comparison.

| Cmd rate |w| [deg/s] | Legs CW | Legs CCW | Cam-Cmd Bias CW | Cam-Cmd Bias CCW | Cam-Cmd MAE | Cam-Cmd RMSE | Cam-Cmd MaxAbs | Cam-Cmd N | Gyro-Cmd Bias CW | Gyro-Cmd Bias CCW | Gyro-Cmd MAE | Gyro-Cmd RMSE | Gyro-Cmd MaxAbs | Gyro-Cmd N | Cam-Gyro Bias CW | Cam-Gyro Bias CCW | Cam-Gyro MAE | Cam-Gyro RMSE | Cam-Gyro MaxAbs | Cam-Gyro N |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.000000 | 1 | 1 | -0.000692 | 0.002345 | 1.518278 | 2.147882 | 13.284358 | 2840 | 0.002740 | 0.000934 | 0.250082 | 0.318208 | 1.060899 | 7052 | -0.003331 | -0.001691 | 1.240435 | 1.750510 | 12.356534 | 7052 |
| 5.000000 | 1 | 1 | -0.000148 | 0.002444 | 2.708655 | 3.630378 | 18.787158 | 2753 | 0.004350 | 0.002414 | 1.660403 | 1.973356 | 4.457200 | 7054 | -0.004290 | -0.000140 | 1.990039 | 2.796590 | 19.007987 | 7054 |
| 10.000000 | 1 | 1 | 0.007808 | -0.003643 | 5.994778 | 7.052519 | 27.040658 | 2902 | 0.045445 | 0.007776 | 6.073036 | 6.754373 | 10.902800 | 6892 | -0.001013 | -0.009296 | 3.241734 | 4.225330 | 19.243185 | 6892 |
| 15.000000 | 1 | 1 | 0.014878 | 0.030474 | 3.120054 | 4.192872 | 21.297653 | 2935 | 0.015359 | 0.014299 | 1.413931 | 1.691698 | 3.974721 | 7050 | -0.004241 | 0.013442 | 2.444927 | 3.290916 | 19.219185 | 7050 |
| 20.000000 | 1 | 1 | 0.008772 | 0.016407 | 5.875738 | 7.087785 | 22.894149 | 2837 | 0.015095 | 0.013781 | 5.650437 | 6.350657 | 11.520899 | 7052 | -0.005723 | 0.004492 | 3.724143 | 4.681822 | 19.735352 | 7052 |
| 25.000000 | 1 | 1 | 0.001176 | 0.003492 | 3.734791 | 4.752808 | 20.161418 | 2805 | 0.013126 | 0.014117 | 1.564729 | 1.888721 | 5.253471 | 7052 | -0.016344 | -0.013400 | 2.902988 | 3.753529 | 19.920540 | 7052 |
| 30.000000 | 1 | 1 | -0.002654 | 0.010414 | 4.737625 | 5.849067 | 23.854623 | 2816 | 0.021563 | 0.011948 | 2.333660 | 2.682135 | 8.238399 | 7053 | -0.037698 | -0.007195 | 3.637540 | 4.478012 | 22.395084 | 7053 |
| POOLED_TOTAL | 7 | 7 | 0.004223 | 0.008924 | 3.962679 | 5.247050 | 27.040658 | 19888 | 0.016624 | 0.009323 | 2.695667 | 3.833468 | 11.520899 | 49205 | -0.010440 | -0.001970 | 2.738627 | 3.694289 | 22.395084 | 49205 |
