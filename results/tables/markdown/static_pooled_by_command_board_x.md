# Static pooled by commanded angle - Board X

Residual samples pooled at each effective commanded angle, plus POOLED_TOTAL.

Machine-readable source: [`../static_pooled_by_command_board_x.csv`](../static_pooled_by_command_board_x.csv)

**Notes**

- Motor quantity = PULSE-DERIVED COMMANDED REFERENCE. Not ground truth, not an actual angle, not a measured motor angle. No encoder-position telemetry was logged.
- N is PER COMPARISON. Evaluation grids differ by comparison; see the evaluation_grid columns and README 'Detailed Result Tables'.
- Samples pooled at sample level via exact N-weighted identities; RMSE is never averaged across commands.
- Repeated visits to one command are pooled here; the per-hold table preserves approach direction and hysteresis.

**Column abbreviations.** `Cam` = camera, `Cmd` = pulse-derived commanded reference, `Grav` = gravity-derived tilt, `Dir` = direction, `SG` = SECONDARY Savitzky-Golay processed rate, `|w|` = magnitude.

Evaluation-grid columns are omitted here because each is constant for its column; see [`../README.md`](../README.md) for the grid of every comparison.

| Cmd ref [deg] | Visits | Cam mean [deg] | Grav mean [deg] | Cam-Cmd Bias | Cam-Cmd MAE | Cam-Cmd RMSE | Cam-Cmd MaxAbs | Cam-Cmd N | Grav-Cmd Bias | Grav-Cmd MAE | Grav-Cmd RMSE | Grav-Cmd MaxAbs | Grav-Cmd N | Cam-Grav Bias | Cam-Grav MAE | Cam-Grav RMSE | Cam-Grav MaxAbs | Cam-Grav N |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.000000 | 3 | 0.173723 | 0.200173 | 0.173723 | 0.180133 | 0.220996 | 0.358276 | 1683 | 0.200173 | 0.310135 | 0.365880 | 1.323211 | 4629 | -0.016869 | 0.161854 | 0.218845 | 0.960071 | 1683 |
| 5.006250 | 2 | 5.289005 | 5.237968 | 0.282755 | 0.282755 | 0.287267 | 0.403349 | 1085 | 0.231718 | 0.279256 | 0.346427 | 1.657392 | 3065 | 0.052766 | 0.167090 | 0.214459 | 0.900452 | 1085 |
| 9.998438 | 2 | 10.375555 | 10.207084 | 0.377118 | 0.377118 | 0.377659 | 0.440450 | 1119 | 0.208646 | 0.244073 | 0.294289 | 1.238913 | 3086 | 0.167384 | 0.194200 | 0.237455 | 0.895143 | 1119 |
| 15.004688 | 2 | 15.406435 | 15.224425 | 0.401748 | 0.401748 | 0.402745 | 0.469095 | 1102 | 0.219738 | 0.251622 | 0.303423 | 0.907532 | 3087 | 0.181439 | 0.207179 | 0.251189 | 1.043341 | 1102 |
| 19.996875 | 1 | 20.184429 | 20.138068 | 0.187554 | 0.187554 | 0.192818 | 0.269958 | 554 | 0.141193 | 0.174237 | 0.214451 | 0.682739 | 1542 | 0.048059 | 0.114977 | 0.143693 | 0.535955 | 554 |
| -5.006250 | 2 | -4.713958 | -4.628238 | 0.292292 | 0.292292 | 0.293939 | 0.358298 | 1126 | 0.378012 | 0.395328 | 0.455911 | 1.415431 | 3086 | -0.084547 | 0.156721 | 0.203210 | 0.926056 | 1126 |
| -9.998438 | 2 | -9.802918 | -9.533547 | 0.195520 | 0.195520 | 0.197058 | 0.267760 | 1128 | 0.464890 | 0.471838 | 0.529498 | 1.470269 | 3087 | -0.279378 | 0.292679 | 0.338290 | 1.062689 | 1128 |
| -15.004688 | 2 | -14.994498 | -14.631855 | 0.010190 | 0.047684 | 0.061866 | 0.166167 | 1138 | 0.372832 | 0.412782 | 0.490209 | 1.712170 | 3087 | -0.357835 | 0.379819 | 0.435187 | 1.318035 | 1138 |
| -19.996875 | 1 | -20.071281 | -19.663871 | -0.074406 | 0.079818 | 0.096883 | 0.167509 | 564 | 0.333004 | 0.389345 | 0.472679 | 1.372244 | 1543 | -0.410965 | 0.426760 | 0.479591 | 1.279395 | 564 |
| POOLED_TOTAL | 17 |  |  | 0.219717 | 0.234501 | 0.267835 | 0.469095 | 9499 | 0.283960 | 0.329686 | 0.400452 | 1.712170 | 26212 | -0.063860 | 0.225555 | 0.289371 | 1.318035 | 9499 |
