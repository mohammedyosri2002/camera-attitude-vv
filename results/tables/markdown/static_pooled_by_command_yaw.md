# Static pooled by commanded angle - Yaw

Residual samples pooled at each effective commanded angle, plus POOLED_TOTAL.

Machine-readable source: [`../static_pooled_by_command_yaw.csv`](../static_pooled_by_command_yaw.csv)

**Notes**

- Motor quantity = PULSE-DERIVED COMMANDED REFERENCE. Not ground truth, not an actual angle, not a measured motor angle. No encoder-position telemetry was logged.
- N is PER COMPARISON. Evaluation grids differ by comparison; see the evaluation_grid columns and README 'Detailed Result Tables'.
- Yaw static: camera minus commanded reference ONLY. No gravity-tilt comparison exists for yaw.
- POOLED_TOTAL pools both directional runs at sample level.

**Column abbreviations.** `Cam` = camera, `Cmd` = pulse-derived commanded reference, `Grav` = gravity-derived tilt, `Dir` = direction, `SG` = SECONDARY Savitzky-Golay processed rate, `|w|` = magnitude.

Evaluation-grid columns are omitted here because each is constant for its column; see [`../README.md`](../README.md) for the grid of every comparison.

| Cmd ref [deg] | Visits | Cam mean [deg] | Cam-Cmd Bias | Cam-Cmd MAE | Cam-Cmd RMSE | Cam-Cmd MaxAbs | Cam-Cmd N |
|---|---|---|---|---|---|---|---|
| 0.000000 | 2 | 0.001989 | 0.001989 | 0.025903 | 0.032140 | 0.165108 | 2851 |
| 5.006250 | 2 | 5.193672 | 0.187422 | 0.187441 | 0.205714 | 0.383814 | 2850 |
| 9.998438 | 2 | 10.228997 | 0.230559 | 0.230568 | 0.245131 | 0.462248 | 2846 |
| 19.996875 | 2 | 20.033554 | 0.036679 | 0.097611 | 0.107410 | 0.327826 | 2831 |
| 29.995312 | 2 | 30.201359 | 0.206047 | 0.206173 | 0.228181 | 0.422593 | 2796 |
| 60.004688 | 2 | 60.198307 | 0.193619 | 0.197718 | 0.224420 | 0.466709 | 2841 |
| 90.000000 | 2 | 90.106407 | 0.106407 | 0.108200 | 0.127434 | 0.271232 | 2866 |
| 119.995310 | 2 | 120.178979 | 0.183669 | 0.183676 | 0.196285 | 0.314117 | 2786 |
| 150.004690 | 2 | 150.084230 | 0.079540 | 0.121122 | 0.141909 | 0.278943 | 2926 |
| 180.000000 | 2 | 180.128701 | 0.128701 | 0.128714 | 0.147395 | 0.299608 | 2796 |
| 209.995310 | 2 | 210.244108 | 0.248798 | 0.248798 | 0.257866 | 0.377008 | 2811 |
| 240.004690 | 2 | 240.131006 | 0.126316 | 0.128011 | 0.147281 | 0.328791 | 2825 |
| 270.000000 | 2 | 270.200358 | 0.200358 | 0.200359 | 0.233912 | 0.448167 | 2821 |
| 299.995310 | 2 | 300.305115 | 0.309805 | 0.309805 | 0.334829 | 0.502888 | 2907 |
| 330.004690 | 2 | 330.240026 | 0.235336 | 0.259547 | 0.337682 | 0.677032 | 2833 |
| 360.000000 | 2 | 360.213207 | 0.213207 | 0.214116 | 0.268377 | 0.443008 | 2921 |
| 720.000000 | 2 | 720.212084 | 0.212084 | 0.212808 | 0.265779 | 0.489062 | 2862 |
| POOLED_TOTAL | 34 |  | 0.170641 | 0.180105 | 0.220583 | 0.677032 | 48369 |
