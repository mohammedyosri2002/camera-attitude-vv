# SECONDARY Savitzky-Golay processed rate - Board Y

SECONDARY bandwidth-limited diagnostic. NOT RAW accuracy.

Machine-readable source: [`../dynamic_sg_SECONDARY_board_y.csv`](../dynamic_sg_SECONDARY_board_y.csv)

**Notes**

- SECONDARY bandwidth-limited processed-rate diagnostic. Savitzky-Golay; uniform-time resampling; 1.00 s window; polynomial order 2; first derivative; zero-phase offline. Same frozen estimator for all axes. Not tuned per axis. Not optimized against an error metric. NOT RAW camera-rate accuracy and never a substitute for the RAW tables.
- Motor quantity = PULSE-DERIVED COMMANDED REFERENCE. Not ground truth, not an actual angle, not a measured motor angle. No encoder-position telemetry was logged.
- N is PER COMPARISON. Evaluation grids differ by comparison; see the evaluation_grid columns and README 'Detailed Result Tables'.

**Column abbreviations.** `Cam` = camera, `Cmd` = pulse-derived commanded reference, `Grav` = gravity-derived tilt, `Dir` = direction, `SG` = SECONDARY Savitzky-Golay processed rate, `|w|` = magnitude.

Evaluation-grid columns are omitted here because each is constant for its column; see [`../README.md`](../README.md) for the grid of every comparison.

| Cmd rate |w| [deg/s] | SG Cam-Cmd Bias+ | SG Cam-Cmd Bias- | SG Cam-Cmd MAE | SG Cam-Cmd RMSE | SG Cam-Cmd MaxAbs | SG Cam-Cmd N | SG win [s] | SG order | SG win [smp] | fps [Hz] |
|---|---|---|---|---|---|---|---|---|---|---|
| 1.000000 | -0.000240 | 0.000382 | 0.031662 | 0.038778 | 0.118656 | 6765 | 1.0 | 2 | 121 | 120.77 |
| 2.000000 | 0.001747 | -0.002353 | 0.034345 | 0.042589 | 0.098666 | 3368 | 1.0 | 2 | 121 | 120.77 |
| 5.000000 | 0.005128 | -0.006559 | 0.033829 | 0.039179 | 0.091235 | 1355 | 1.0 | 2 | 121 | 120.77 |
| 10.000000 | -0.183624 | 0.188507 | 0.220043 | 0.496291 | 2.333938 | 687 | 1.0 | 2 | 121 | 120.77 |
| POOLED_TOTAL | -0.009438 | 0.009470 | 0.043275 | 0.124123 | 2.333938 | 12175 | 1.0 | 2 | 121 | 120.77 |
