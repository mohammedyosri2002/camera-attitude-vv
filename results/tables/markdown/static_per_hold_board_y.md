# Static per hold - Board Y

One row per static dwell in acquisition order, with approach direction. Bias / SD / MaxAbs / N per comparison; MAE and RMSE omitted as redundant within a dwell.

Machine-readable source: [`../static_per_hold_board_y.csv`](../static_per_hold_board_y.csv)

**Notes**

- Motor quantity = PULSE-DERIVED COMMANDED REFERENCE. Not ground truth, not an actual angle, not a measured motor angle. No encoder-position telemetry was logged.
- N is PER COMPARISON. Evaluation grids differ by comparison; see the evaluation_grid columns and README 'Detailed Result Tables'.
- Acquisition order preserved. approach_direction is a derived label and changes no residual.
- Per-hold MAE and RMSE are omitted: within one dwell MAE is identically |Bias| and RMSE is identically sqrt(Bias^2+SD^2). They appear on the pooled-by-command table.
- No POOLED_TOTAL here: pooling across different commanded angles is meaningless.

**Column abbreviations.** `Cam` = camera, `Cmd` = pulse-derived commanded reference, `Grav` = gravity-derived tilt, `Dir` = direction, `SG` = SECONDARY Savitzky-Golay processed rate, `|w|` = magnitude.

Evaluation-grid columns are omitted here because each is constant for its column; see [`../README.md`](../README.md) for the grid of every comparison.

| Hold | Approach | Cmd ref [deg] | Pulses | Tags | Cam mean [deg] | Grav mean [deg] | Cam-Cmd Bias | Cam-Cmd SD | Cam-Cmd MaxAbs | Cam-Cmd N | Grav-Cmd Bias | Grav-Cmd SD | Grav-Cmd MaxAbs | Grav-Cmd N | Cam-Grav Bias | Cam-Grav SD | Cam-Grav MaxAbs | Cam-Grav N |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | initial | 0.000000 | 0 | 4 | -0.010289 | -0.000885 | -0.010289 | 0.007303 | 0.034601 | 855 | -0.000885 | 0.129877 | 0.446758 | 1545 | -0.009139 | 0.106529 | 0.333795 | 855 |
| 1 | ascending | 5.006250 | 356 | 4 | 5.250034 | 5.156233 | 0.243784 | 0.051797 | 0.344050 | 826 | 0.149983 | 0.250303 | 1.321930 | 1544 | 0.092337 | 0.226371 | 0.888246 | 826 |
| 2 | ascending | 9.998437 | 711 | 4 | 10.242491 | 10.077846 | 0.244053 | 0.039305 | 0.330278 | 833 | 0.079409 | 0.248578 | 1.367037 | 1547 | 0.165079 | 0.234374 | 1.047606 | 833 |
| 3 | ascending | 15.004688 | 1067 | 4 | 15.304458 | 15.064702 | 0.299771 | 0.018475 | 0.361213 | 824 | 0.060015 | 0.120860 | 0.488237 | 1545 | 0.239961 | 0.106550 | 0.648369 | 824 |
| 4 | ascending | 19.996875 | 1422 | 4 | 20.302996 | 20.006561 | 0.306121 | 0.030243 | 0.396684 | 847 | 0.009686 | 0.193348 | 0.816387 | 1545 | 0.294759 | 0.170415 | 0.959869 | 847 |
| 5 | descending | 15.004688 | 1067 | 4 | 15.306855 | 15.063916 | 0.302168 | 0.018307 | 0.363565 | 848 | 0.059228 | 0.166475 | 0.621024 | 1545 | 0.244176 | 0.141922 | 0.685433 | 848 |
| 6 | descending | 9.998437 | 711 | 4 | 10.204324 | 10.037902 | 0.205886 | 0.036682 | 0.287513 | 845 | 0.039465 | 0.246877 | 1.088543 | 1543 | 0.167991 | 0.232548 | 0.982693 | 845 |
| 7 | descending | 5.006250 | 356 | 4 | 5.211625 | 5.116537 | 0.205375 | 0.054201 | 0.305664 | 836 | 0.110287 | 0.244095 | 2.150652 | 1543 | 0.094501 | 0.219068 | 1.188385 | 836 |
| 8 | descending | 0.000000 | 0 | 4 | 0.232730 | 0.250351 | 0.232730 | 0.025091 | 0.292159 | 849 | 0.250351 | 0.182819 | 1.230955 | 1543 | -0.017741 | 0.159524 | 0.756359 | 849 |
| 9 | descending | -5.006250 | -356 | 4 | -4.691732 | -4.583460 | 0.314518 | 0.009058 | 0.340190 | 840 | 0.422790 | 0.153036 | 0.981742 | 1513 | -0.106930 | 0.129786 | 0.495544 | 840 |
| 10 | descending | -9.998437 | -711 | 3 | -9.711201 | -9.616393 | 0.287236 | 0.041715 | 0.378626 | 854 | 0.382044 | 0.186045 | 1.840165 | 1543 | -0.095742 | 0.171884 | 1.193458 | 854 |
| 11 | descending | -15.004688 | -1067 | 3 | -14.773876 | -14.582321 | 0.230812 | 0.069207 | 0.361323 | 840 | 0.422367 | 0.242302 | 1.498040 | 1543 | -0.191700 | 0.217409 | 1.077860 | 840 |
| 12 | descending | -19.996875 | -1422 | 3 | -19.677161 | -19.417845 | 0.319714 | 0.059614 | 0.421347 | 847 | 0.579030 | 0.201439 | 1.964572 | 1543 | -0.258772 | 0.181712 | 1.191347 | 847 |
| 13 | ascending | -15.004688 | -1067 | 3 | -14.633293 | -14.455366 | 0.371394 | 0.069161 | 0.493925 | 854 | 0.549322 | 0.225979 | 1.565214 | 1543 | -0.178735 | 0.208720 | 0.925715 | 854 |
| 14 | ascending | -9.998437 | -711 | 3 | -9.630439 | -9.548895 | 0.367999 | 0.058056 | 0.476933 | 863 | 0.449543 | 0.245507 | 1.653534 | 1544 | -0.080599 | 0.221564 | 0.917383 | 863 |
| 15 | ascending | -5.006250 | -356 | 4 | -4.614320 | -4.508772 | 0.391930 | 0.017573 | 0.438967 | 847 | 0.497478 | 0.199296 | 1.279385 | 1544 | -0.105320 | 0.186469 | 0.721494 | 847 |
| 16 | ascending | 0.000000 | 0 | 4 | 0.345494 | 0.367256 | 0.345494 | 0.015057 | 0.387177 | 846 | 0.367256 | 0.198555 | 1.544809 | 1543 | -0.023457 | 0.175315 | 0.897607 | 846 |
