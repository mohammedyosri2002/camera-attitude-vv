# Dynamic RAW pooled by rate magnitude - Board X

RAW MAE / RMSE / MaxAbs pooled by rate magnitude; signed bias kept direction-separated.

Machine-readable source: [`../dynamic_pooled_by_rate_board_x.csv`](../dynamic_pooled_by_rate_board_x.csv)

**Notes**

- Motor quantity = PULSE-DERIVED COMMANDED REFERENCE. Not ground truth, not an actual angle, not a measured motor angle. No encoder-position telemetry was logged.
- N is PER COMPARISON. Evaluation grids differ by comparison; see the evaluation_grid columns and README 'Detailed Result Tables'.
- RAW-rate MaxAbs is a sample extreme and is not interpreted as a deterministic error bound; it scales with sample count.
- Signed bias is reported direction-separated only. No pooled signed-bias column exists, because pooling opposite directions cancels it.
- MAE and RMSE pooled at sample level by rate MAGNITUDE; signed bias stays direction-separated in *_Bias_Pos / *_Bias_Neg.
- POOLED_TOTAL is sample-level over all legs of this axis.

**Column abbreviations.** `Cam` = camera, `Cmd` = pulse-derived commanded reference, `Grav` = gravity-derived tilt, `Dir` = direction, `SG` = SECONDARY Savitzky-Golay processed rate, `|w|` = magnitude.

Evaluation-grid columns are omitted here because each is constant for its column; see [`../README.md`](../README.md) for the grid of every comparison.

| Cmd rate |w| [deg/s] | Legs + | Legs - | Cam-Cmd Bias+ | Cam-Cmd Bias- | Cam-Cmd MAE | Cam-Cmd RMSE | Cam-Cmd MaxAbs | Cam-Cmd N | Gyro-Cmd Bias+ | Gyro-Cmd Bias- | Gyro-Cmd MAE | Gyro-Cmd RMSE | Gyro-Cmd MaxAbs | Gyro-Cmd N | Cam-Gyro Bias+ | Cam-Gyro Bias- | Cam-Gyro MAE | Cam-Gyro RMSE | Cam-Gyro MaxAbs | Cam-Gyro N |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.000000 | 2 | 2 | 0.007620 | -0.003968 | 0.695863 | 0.911630 | 3.785325 | 4571 | -0.005905 | 0.004808 | 0.393110 | 0.489801 | 1.469492 | 12340 | 0.011969 | -0.011189 | 0.769048 | 0.971965 | 4.315552 | 12340 |
| 2.000000 | 2 | 2 | 0.021735 | -0.013356 | 0.718744 | 0.941032 | 3.894278 | 2223 | -0.011403 | 0.011323 | 0.340724 | 0.442949 | 1.503801 | 6171 | 0.031619 | -0.023433 | 0.712381 | 0.929560 | 4.553630 | 6171 |
| 5.000000 | 2 | 2 | 0.041784 | -0.044740 | 1.487039 | 1.840117 | 5.627732 | 909 | -0.027677 | 0.024672 | 1.204619 | 1.475498 | 4.255914 | 2467 | 0.065723 | -0.064977 | 1.974359 | 2.469565 | 6.935291 | 2467 |
| 10.000000 | 2 | 2 | 0.114607 | -0.142930 | 2.385695 | 2.932411 | 9.517473 | 457 | -0.112348 | 0.132551 | 1.987154 | 2.319293 | 5.413297 | 1234 | 0.208621 | -0.263720 | 3.493478 | 4.064288 | 12.356420 | 1234 |
| POOLED_TOTAL | 8 | 8 | 0.021228 | -0.018877 | 0.884870 | 1.251227 | 9.517473 | 8160 | -0.015752 | 0.015934 | 0.557245 | 0.853482 | 5.413297 | 22212 | 0.034302 | -0.034623 | 1.038531 | 1.536216 | 12.356420 | 22212 |
