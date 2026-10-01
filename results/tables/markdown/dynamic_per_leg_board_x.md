# Dynamic RAW per leg - Board X

One row per constant-rate leg, direction preserved. RAW instantaneous finite-difference derivative.

Machine-readable source: [`../dynamic_per_leg_board_x.csv`](../dynamic_per_leg_board_x.csv)

**Notes**

- Motor quantity = PULSE-DERIVED COMMANDED REFERENCE. Not ground truth, not an actual angle, not a measured motor angle. No encoder-position telemetry was logged.
- N is PER COMPARISON. Evaluation grids differ by comparison; see the evaluation_grid columns and README 'Detailed Result Tables'.
- RAW-rate MaxAbs is a sample extreme and is not interpreted as a deterministic error bound; it scales with sample count.
- Frozen evaluation windows: central 70 % of each constant-rate leg, unchanged from the published headline results.
- camera_mean_rate_dps minus commanded_rate_dps is identically Camera_Cmd_Bias.
- RAW instantaneous finite difference. SG results are in dynamic_sg_SECONDARY_*.csv and are never mixed in here.

**Column abbreviations.** `Cam` = camera, `Cmd` = pulse-derived commanded reference, `Grav` = gravity-derived tilt, `Dir` = direction, `SG` = SECONDARY Savitzky-Golay processed rate, `|w|` = magnitude.

Evaluation-grid columns are omitted here because each is constant for its column; see [`../README.md`](../README.md) for the grid of every comparison.

| Leg | Cmd rate [deg/s] | Dir | Leg [s] | Cam mean rate [deg/s] | Gyro mean rate [deg/s] | Cam-Cmd Bias | Cam-Cmd MAE | Cam-Cmd RMSE | Cam-Cmd MaxAbs | Cam-Cmd N | Gyro-Cmd Bias | Gyro-Cmd MAE | Gyro-Cmd RMSE | Gyro-Cmd MaxAbs | Gyro-Cmd N | Cam-Gyro Bias | Cam-Gyro MAE | Cam-Gyro RMSE | Cam-Gyro MaxAbs | Cam-Gyro N |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 1.000000 | positive | 19.998040 | 1.006295 | 0.993620 | 0.006295 | 0.697028 | 0.886551 | 3.246345 | 1168 | -0.006380 | 0.399220 | 0.495425 | 1.469492 | 3085 | 0.010457 | 0.791616 | 0.982977 | 4.315552 | 3085 |
| 1 | -1.000000 | negative | 19.998048 | -0.998984 | -0.993048 | 0.001016 | 0.708115 | 0.951559 | 3.783710 | 1113 | 0.006952 | 0.390338 | 0.488117 | 1.431123 | 3084 | -0.010726 | 0.775813 | 1.003319 | 4.100168 | 3084 |
| 2 | 1.000000 | positive | 19.998032 | 1.008985 | 0.994569 | 0.008985 | 0.699266 | 0.914303 | 3.592352 | 1135 | -0.005431 | 0.395013 | 0.491240 | 1.443632 | 3086 | 0.013481 | 0.768243 | 0.964627 | 3.760238 | 3086 |
| 3 | -1.000000 | negative | 19.998052 | -1.008771 | -0.997336 | -0.008771 | 0.679535 | 0.894533 | 3.785325 | 1155 | 0.002664 | 0.387865 | 0.484354 | 1.417142 | 3085 | -0.011652 | 0.740521 | 0.935677 | 3.773641 | 3085 |
| 4 | 2.000000 | positive | 9.999600 | 2.013603 | 1.988438 | 0.013603 | 0.705852 | 0.918161 | 3.892171 | 545 | -0.011562 | 0.338163 | 0.440918 | 1.415662 | 1544 | 0.025683 | 0.703711 | 0.919477 | 3.484996 | 1544 |
| 5 | -2.000000 | negative | 9.999612 | -2.013528 | -1.984611 | -0.013528 | 0.696577 | 0.931280 | 3.894278 | 549 | 0.015389 | 0.345570 | 0.446933 | 1.503801 | 1543 | -0.027317 | 0.719464 | 0.947754 | 3.850573 | 1543 |
| 6 | 2.000000 | positive | 9.999608 | 2.029620 | 1.988757 | 0.029620 | 0.718314 | 0.942391 | 3.492661 | 562 | -0.011243 | 0.338088 | 0.439228 | 1.438009 | 1543 | 0.037559 | 0.696429 | 0.911498 | 3.838337 | 1543 |
| 7 | -2.000000 | negative | 9.999612 | -2.013189 | -1.992749 | -0.013189 | 0.753024 | 0.970339 | 3.661026 | 567 | 0.007251 | 0.341076 | 0.444679 | 1.499561 | 1541 | -0.019544 | 0.729950 | 0.939073 | 4.553630 | 1541 |
| 8 | 5.000000 | positive | 4.000532 | 5.040544 | 4.978292 | 0.040544 | 1.432569 | 1.819238 | 4.779577 | 228 | -0.021708 | 1.209196 | 1.478036 | 4.255914 | 615 | 0.063600 | 1.976757 | 2.483906 | 6.922067 | 615 |
| 9 | -5.000000 | negative | 4.000548 | -5.029800 | -4.976305 | -0.029800 | 1.432289 | 1.767953 | 5.015477 | 226 | 0.023695 | 1.205281 | 1.476279 | 4.167743 | 617 | -0.058088 | 1.958602 | 2.419647 | 6.935291 | 617 |
| 10 | 5.000000 | positive | 4.000536 | 5.043018 | 4.966373 | 0.043018 | 1.542509 | 1.903619 | 4.986612 | 229 | -0.033627 | 1.209797 | 1.478878 | 4.103692 | 617 | 0.067839 | 2.003442 | 2.496891 | 6.903805 | 617 |
| 11 | -5.000000 | negative | 4.000556 | -5.059680 | -4.974353 | -0.059680 | 1.540533 | 1.866172 | 5.627732 | 226 | 0.025647 | 1.194233 | 1.468799 | 4.116079 | 618 | -0.071855 | 1.958668 | 2.477143 | 6.854503 | 618 |
| 12 | 10.000000 | positive | 2.000892 | 10.128433 | 9.892578 | 0.128433 | 2.292497 | 2.882941 | 8.401894 | 114 | -0.107422 | 1.943103 | 2.262270 | 5.224475 | 308 | 0.225426 | 3.458045 | 3.970208 | 10.890983 | 308 |
| 13 | -10.000000 | negative | 2.000904 | -10.101781 | -9.868422 | -0.101781 | 2.295282 | 2.853694 | 7.928671 | 113 | 0.131578 | 2.001339 | 2.334129 | 5.085568 | 309 | -0.229270 | 3.421340 | 4.032001 | 10.150118 | 309 |
| 14 | 10.000000 | positive | 2.000892 | 10.100901 | 9.882726 | 0.100901 | 2.467190 | 2.957938 | 8.478117 | 115 | -0.117274 | 2.027111 | 2.350728 | 5.413297 | 308 | 0.191815 | 3.559786 | 4.105760 | 8.835879 | 308 |
| 15 | -10.000000 | negative | 2.000912 | -10.183363 | -9.866476 | -0.183363 | 2.485428 | 3.030082 | 9.517473 | 115 | 0.133524 | 1.977051 | 2.328982 | 5.033625 | 309 | -0.298170 | 3.534841 | 4.146753 | 12.356420 | 309 |
