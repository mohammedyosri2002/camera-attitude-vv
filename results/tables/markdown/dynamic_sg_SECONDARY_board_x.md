# SECONDARY Savitzky-Golay processed rate - Board X

SECONDARY bandwidth-limited diagnostic. NOT RAW accuracy.

Machine-readable source: [`../dynamic_sg_SECONDARY_board_x.csv`](../dynamic_sg_SECONDARY_board_x.csv)

**Notes**

- SECONDARY bandwidth-limited processed-rate diagnostic. Savitzky-Golay; uniform-time resampling; 1.00 s window; polynomial order 2; first derivative; zero-phase offline. Same frozen estimator for all axes. Not tuned per axis. Not optimized against an error metric. NOT RAW camera-rate accuracy and never a substitute for the RAW tables.
- Motor quantity = PULSE-DERIVED COMMANDED REFERENCE. Not ground truth, not an actual angle, not a measured motor angle. No encoder-position telemetry was logged.
- N is PER COMPARISON. Evaluation grids differ by comparison; see the evaluation_grid columns and README 'Detailed Result Tables'.

**Column abbreviations.** `Cam` = camera, `Cmd` = pulse-derived commanded reference, `Grav` = gravity-derived tilt, `Dir` = direction, `SG` = SECONDARY Savitzky-Golay processed rate, `|w|` = magnitude.

Evaluation-grid columns are omitted here because each is constant for its column; see [`../README.md`](../README.md) for the grid of every comparison.

| Cmd rate |w| [deg/s] | SG Cam-Cmd Bias+ | SG Cam-Cmd Bias- | SG Cam-Cmd MAE | SG Cam-Cmd RMSE | SG Cam-Cmd MaxAbs | SG Cam-Cmd N | SG win [s] | SG order | SG win [smp] | fps [Hz] |
|---|---|---|---|---|---|---|---|---|---|---|
| 1.000000 | 0.007871 | -0.007861 | 0.035151 | 0.046167 | 0.157578 | 4571 | 1.0 | 2 | 81 | 80.33 |
| 2.000000 | 0.015424 | -0.016497 | 0.044350 | 0.053326 | 0.127253 | 2223 | 1.0 | 2 | 81 | 80.33 |
| 5.000000 | 0.041141 | -0.038112 | 0.040308 | 0.046353 | 0.096381 | 909 | 1.0 | 2 | 81 | 80.33 |
| 10.000000 | -0.099633 | 0.096560 | 0.240738 | 0.464771 | 2.267419 | 457 | 1.0 | 2 | 81 | 80.33 |
| POOLED_TOTAL | 0.007614 | -0.007739 | 0.049745 | 0.119606 | 2.267419 | 8160 | 1.0 | 2 | 81 | 80.33 |
