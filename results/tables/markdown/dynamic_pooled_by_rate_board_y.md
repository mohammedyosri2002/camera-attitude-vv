# Dynamic RAW pooled by rate magnitude - Board Y

RAW MAE / RMSE / MaxAbs pooled by rate magnitude; signed bias kept direction-separated.

Machine-readable source: [`../dynamic_pooled_by_rate_board_y.csv`](../dynamic_pooled_by_rate_board_y.csv)

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
| 1.000000 | 2 | 2 | -0.000862 | 0.001742 | 0.758573 | 0.956400 | 4.121696 | 6765 | 0.004775 | 0.004742 | 0.474171 | 0.592259 | 2.171772 | 12338 | -0.005061 | -0.005196 | 0.731913 | 0.921698 | 4.126422 | 12338 |
| 2.000000 | 2 | 2 | -0.002287 | 0.008648 | 0.797496 | 0.994140 | 4.787324 | 3368 | 0.001740 | 0.003979 | 0.404916 | 0.531450 | 2.247023 | 6170 | -0.005861 | 0.002939 | 0.645765 | 0.817356 | 4.340562 | 6170 |
| 5.000000 | 2 | 2 | -0.016336 | 0.017938 | 1.349104 | 1.673796 | 6.123249 | 1355 | -0.006117 | 0.005058 | 0.844046 | 1.115715 | 4.555969 | 2468 | -0.008871 | 0.008198 | 1.316474 | 1.710187 | 5.839840 | 2468 |
| 10.000000 | 2 | 2 | 0.054970 | 0.010817 | 2.643785 | 3.318546 | 11.656237 | 687 | 0.062125 | -0.026607 | 2.150964 | 2.531389 | 6.529458 | 1232 | -0.030078 | 0.041097 | 2.698884 | 3.394763 | 9.924572 | 1232 |
| POOLED_TOTAL | 8 | 8 | 0.000182 | 0.005977 | 0.941440 | 1.309529 | 11.656237 | 12175 | 0.005903 | 0.002826 | 0.589056 | 0.875881 | 6.529458 | 22208 | -0.007094 | 0.001121 | 0.882061 | 1.273550 | 9.924572 | 22208 |
