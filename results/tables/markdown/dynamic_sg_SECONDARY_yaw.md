# SECONDARY Savitzky-Golay processed rate - Yaw

SECONDARY bandwidth-limited diagnostic. NOT RAW accuracy.

Machine-readable source: [`../dynamic_sg_SECONDARY_yaw.csv`](../dynamic_sg_SECONDARY_yaw.csv)

**Notes**

- SECONDARY bandwidth-limited processed-rate diagnostic. Savitzky-Golay; uniform-time resampling; 1.00 s window; polynomial order 2; first derivative; zero-phase offline. Same frozen estimator for all axes. Not tuned per axis. Not optimized against an error metric. NOT RAW camera-rate accuracy and never a substitute for the RAW tables.
- Motor quantity = PULSE-DERIVED COMMANDED REFERENCE. Not ground truth, not an actual angle, not a measured motor angle. No encoder-position telemetry was logged.
- N is PER COMPARISON. Evaluation grids differ by comparison; see the evaluation_grid columns and README 'Detailed Result Tables'.

**Column abbreviations.** `Cam` = camera, `Cmd` = pulse-derived commanded reference, `Grav` = gravity-derived tilt, `Dir` = direction, `SG` = SECONDARY Savitzky-Golay processed rate, `|w|` = magnitude.

Evaluation-grid columns are omitted here because each is constant for its column; see [`../README.md`](../README.md) for the grid of every comparison.

| Cmd rate |w| [deg/s] | SG Cam-Cmd Bias CW | SG Cam-Cmd Bias CCW | SG Cam-Cmd MAE | SG Cam-Cmd RMSE | SG Cam-Cmd MaxAbs | SG Cam-Cmd N | SG win [s] | SG order |
|---|---|---|---|---|---|---|---|---|
| 1.000000 | -0.004450 | -0.003095 | 0.035513 | 0.053807 | 0.414242 | 2840 | 1.0 | 2 |
| 5.000000 | -0.002422 | -0.005091 | 0.036643 | 0.048673 | 0.262021 | 2753 | 1.0 | 2 |
| 10.000000 | -0.001971 | 0.001337 | 0.027245 | 0.041519 | 0.270948 | 2902 | 1.0 | 2 |
| 15.000000 | -0.002266 | 0.001492 | 0.024300 | 0.032444 | 0.147282 | 2935 | 1.0 | 2 |
| 20.000000 | 0.002386 | 0.001074 | 0.028984 | 0.038530 | 0.146195 | 2837 | 1.0 | 2 |
| 25.000000 | -0.003779 | 0.005645 | 0.024964 | 0.035991 | 0.173623 | 2805 | 1.0 | 2 |
| 30.000000 | -0.006623 | 0.003030 | 0.025505 | 0.040626 | 0.355144 | 2816 | 1.0 | 2 |
| POOLED_TOTAL | -0.002725 | 0.000636 | 0.028972 | 0.042147 | 0.414242 | 19888 | 1.0 | 2 |
