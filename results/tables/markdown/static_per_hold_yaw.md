# Static per hold - Yaw

One row per static dwell in acquisition order, with approach direction. Bias / SD / MaxAbs / N per comparison; MAE and RMSE omitted as redundant within a dwell.

Machine-readable source: [`../static_per_hold_yaw.csv`](../static_per_hold_yaw.csv)

**Notes**

- Motor quantity = PULSE-DERIVED COMMANDED REFERENCE. Not ground truth, not an actual angle, not a measured motor angle. No encoder-position telemetry was logged.
- N is PER COMPARISON. Evaluation grids differ by comparison; see the evaluation_grid columns and README 'Detailed Result Tables'.
- Yaw static supports ONLY camera minus commanded reference. The yaw acquisition logged no roll/pitch, so no gravity-tilt comparison exists and none is invented.
- Per-hold MAE and RMSE omitted by design (identically |Bias| and sqrt(Bias^2+SD^2) within a dwell).

**Column abbreviations.** `Cam` = camera, `Cmd` = pulse-derived commanded reference, `Grav` = gravity-derived tilt, `Dir` = direction, `SG` = SECONDARY Savitzky-Golay processed rate, `|w|` = magnitude.

Evaluation-grid columns are omitted here because each is constant for its column; see [`../README.md`](../README.md) for the grid of every comparison.

| Hold | run | Dir | Approach | Cmd ref [deg] | Cam mean [deg] | Cam-Cmd Bias | Cam-Cmd SD | Cam-Cmd MaxAbs | Cam-Cmd N |
|---|---|---|---|---|---|---|---|---|---|
| 0 | Run01 | CW | initial | 0.000000 | -0.021285 | -0.021285 | 0.024157 | 0.165108 | 1286 |
| 1 | Run01 | CW | ascending | 5.006250 | 5.120222 | 0.113972 | 0.024938 | 0.170523 | 1275 |
| 2 | Run01 | CW | ascending | 9.998438 | 10.163104 | 0.164667 | 0.047442 | 0.336437 | 1289 |
| 3 | Run01 | CW | ascending | 19.996875 | 19.931827 | -0.065048 | 0.036591 | 0.173584 | 1303 |
| 4 | Run01 | CW | ascending | 29.995312 | 30.107716 | 0.112403 | 0.028142 | 0.177957 | 1318 |
| 5 | Run01 | CW | ascending | 60.004688 | 60.157540 | 0.152852 | 0.102927 | 0.351599 | 1333 |
| 6 | Run01 | CW | ascending | 90.000000 | 90.037017 | 0.037017 | 0.027392 | 0.106697 | 1349 |
| 7 | Run01 | CW | ascending | 119.995310 | 120.110466 | 0.115154 | 0.023960 | 0.188932 | 1315 |
| 8 | Run01 | CW | ascending | 150.004690 | 149.963193 | -0.041494 | 0.039105 | 0.212193 | 1357 |
| 9 | Run01 | CW | ascending | 180.000000 | 180.059221 | 0.059221 | 0.031437 | 0.286053 | 1329 |
| 10 | Run01 | CW | ascending | 209.995310 | 210.184421 | 0.189109 | 0.031771 | 0.324993 | 1296 |
| 11 | Run01 | CW | ascending | 240.004690 | 240.095310 | 0.090622 | 0.051802 | 0.200136 | 1337 |
| 12 | Run01 | CW | ascending | 270.000000 | 270.078434 | 0.078434 | 0.047920 | 0.281477 | 1309 |
| 13 | Run01 | CW | ascending | 299.995310 | 300.169766 | 0.174454 | 0.021988 | 0.250998 | 1331 |
| 14 | Run01 | CW | ascending | 330.004690 | 329.991519 | -0.013169 | 0.048900 | 0.133119 | 1323 |
| 15 | Run01 | CW | ascending | 360.000000 | 360.040234 | 0.040234 | 0.025116 | 0.118958 | 1357 |
| 16 | Run01 | CW | ascending | 720.000000 | 720.036506 | 0.036506 | 0.024184 | 0.135594 | 1284 |
| 17 | Run02 | CCW | initial | 0.000000 | 0.021114 | 0.021114 | 0.024186 | 0.145371 | 1565 |
| 18 | Run02 | CCW | ascending | 5.006250 | 5.253132 | 0.246882 | 0.067894 | 0.383814 | 1575 |
| 19 | Run02 | CCW | ascending | 9.998438 | 10.283547 | 0.285110 | 0.065122 | 0.462248 | 1557 |
| 20 | Run02 | CCW | ascending | 19.996875 | 20.120301 | 0.123426 | 0.037322 | 0.327826 | 1528 |
| 21 | Run02 | CCW | ascending | 29.995312 | 30.284865 | 0.289553 | 0.051824 | 0.422593 | 1478 |
| 22 | Run02 | CCW | ascending | 60.004688 | 60.234343 | 0.229655 | 0.110182 | 0.466709 | 1508 |
| 23 | Run02 | CCW | ascending | 90.000000 | 90.168112 | 0.168112 | 0.023109 | 0.271232 | 1517 |
| 24 | Run02 | CCW | ascending | 119.995310 | 120.240231 | 0.244918 | 0.024867 | 0.314117 | 1471 |
| 25 | Run02 | CCW | ascending | 150.004690 | 150.188907 | 0.184220 | 0.028426 | 0.278943 | 1569 |
| 26 | Run02 | CCW | ascending | 180.000000 | 180.191645 | 0.191645 | 0.024648 | 0.299608 | 1467 |
| 27 | Run02 | CCW | ascending | 209.995310 | 210.295171 | 0.299859 | 0.044814 | 0.377008 | 1515 |
| 28 | Run02 | CCW | ascending | 240.004690 | 240.163075 | 0.158388 | 0.079443 | 0.328791 | 1488 |
| 29 | Run02 | CCW | ascending | 270.000000 | 270.305913 | 0.305913 | 0.034482 | 0.448167 | 1512 |
| 30 | Run02 | CCW | ascending | 299.995310 | 300.419427 | 0.424114 | 0.028438 | 0.502888 | 1576 |
| 31 | Run02 | CCW | ascending | 330.004690 | 330.457752 | 0.453065 | 0.080172 | 0.677032 | 1510 |
| 32 | Run02 | CCW | ascending | 360.000000 | 360.363287 | 0.363287 | 0.024316 | 0.443008 | 1564 |
| 33 | Run02 | CCW | ascending | 720.000000 | 720.354951 | 0.354951 | 0.023811 | 0.489062 | 1578 |
