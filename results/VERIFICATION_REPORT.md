# Verification Report

Generated: 2026-09-30T20:45:38Z  
Elapsed: 239.3 s  
Result: **PASS** (54/54 gated values within tolerance)

Produced by `python analysis/reproduce_all.py` from the raw data in `data/raw/`.
The motor signal is a **pulse-derived commanded reference** throughout — not
ground truth, not an actual angle, not encoder-verified.

## Checksums

- 49 raw-data files verified against `data/CHECKSUMS.sha256`

## Tolerances

| Quantity | Absolute tolerance |
|---|---|
| static error, 4 dp (deg) | 5e-05 |
| rate error, 4 dp (deg/s) | 5e-05 |
| percentage, 4 dp | 0.0005 |
| percentage, 2 dp (rate-scale range) | 0.005 |
| reprojection, 4 dp (px) | 5e-05 |
| reprojection ratio, 3 dp | 0.0005 |
| candidate separation, 3 dp (deg) | 0.0005 |
| axis direction cosine, 5 dp | 5e-06 |
| frame and sample counts | exact |

Each tolerance is half a unit in the last printed digit of the corresponding
submitted value. The pipeline is deterministic given the same raw data, so the
only legitimate disagreement is the rounding already present in the published
figure. A looser tolerance would hide a real change.

## Gated values

| Metric | Expected | Reproduced | Delta | Status |
|---|---|---|---|---|
| yaw static camera-commanded RMSE (deg) | 0.2206 | 0.2205833403059897 | -1.67e-05 | PASS |
| yaw static camera-commanded MAE (deg) | 0.1801 | 0.1801046429687424 | +4.64e-06 | PASS |
| yaw static camera-commanded MaxAbs (deg) | 0.677 | 0.6770323358753103 | +3.23e-05 | PASS |
| yaw dynamic camera RAW rate RMSE (deg/s) | 5.2471 | 5.247050135553557 | -4.99e-05 | PASS |
| yaw dynamic camera RAW rate MAE (deg/s) | 3.9627 | 3.962678501407934 | -2.15e-05 | PASS |
| board_x static camera-commanded RMSE (deg) | 0.2678 | 0.2678348949902406 | +3.49e-05 | PASS |
| board_x static camera-commanded MAE (deg) | 0.2345 | 0.23450124801574268 | +1.25e-06 | PASS |
| board_x static camera-commanded Bias (deg) | 0.2197 | 0.21971670692861336 | +1.67e-05 | PASS |
| board_x static camera-commanded MaxAbs (deg) | 0.4691 | 0.46909510711499536 | -4.89e-06 | PASS |
| board_x static camera-gravity RMSE (deg) | 0.2894 | 0.28937105173232636 | -2.89e-05 | PASS |
| board_x dynamic camera RAW rate RMSE (deg/s) | 1.2512 | 1.2512268941815428 | +2.69e-05 | PASS |
| board_x dynamic gyro RMSE (deg/s) | 0.8535 | 0.8534822034288921 | -1.78e-05 | PASS |
| board_y static camera_vs_commanded Bias | 0.2741 | 0.27412809207870537 | +2.81e-05 | PASS |
| board_y static camera_vs_commanded MAE | 0.2754 | 0.2753893492953745 | -1.07e-05 | PASS |
| board_y static camera_vs_commanded RMSE | 0.2918 | 0.2918193976023065 | +1.94e-05 | PASS |
| board_y static camera_vs_commanded MaxAbs | 0.4939 | 0.49392494523983466 | +2.49e-05 | PASS |
| board_y static camera_vs_commanded N | 14354 | 14354 | +0.00e+00 | PASS |
| board_y static gravity_vs_commanded RMSE | 0.3879 | 0.3878706909624641 | -2.93e-05 | PASS |
| board_y static camera_vs_gravity Bias | 0.0128 | 0.012767975104227397 | -3.20e-05 | PASS |
| board_y static camera_vs_gravity MAE | 0.1949 | 0.19490699745201523 | +7.00e-06 | PASS |
| board_y static camera_vs_gravity RMSE | 0.2474 | 0.2473890105863721 | -1.10e-05 | PASS |
| board_y static camera_vs_gravity MaxAbs | 1.1935 | 1.193458174654463 | -4.18e-05 | PASS |
| board_y static camera_vs_gravity N | 14354 | 14354 | +0.00e+00 | PASS |
| board_y dynamic camera_raw_rate_vs_commanded Bias | 0.0031 | 0.0030824912712969312 | -1.75e-05 | PASS |
| board_y dynamic camera_raw_rate_vs_commanded MAE | 0.9414 | 0.9414398555916966 | +3.99e-05 | PASS |
| board_y dynamic camera_raw_rate_vs_commanded RMSE | 1.3095 | 1.3095294865958675 | +2.95e-05 | PASS |
| board_y dynamic camera_raw_rate_vs_commanded MaxAbs | 11.6562 | 11.656237325570658 | +3.73e-05 | PASS |
| board_y dynamic camera_raw_rate_vs_commanded N | 12175 | 12175 | +0.00e+00 | PASS |
| board_y dynamic gyro_vs_commanded Bias | 0.0044 | 0.004364463409234353 | -3.55e-05 | PASS |
| board_y dynamic gyro_vs_commanded MAE | 0.5891 | 0.5890557277029311 | -4.43e-05 | PASS |
| board_y dynamic gyro_vs_commanded RMSE | 0.8759 | 0.8758807344664447 | -1.93e-05 | PASS |
| board_y dynamic gyro_vs_commanded MaxAbs | 6.5295 | 6.529457912743073 | -4.21e-05 | PASS |
| board_y dynamic gyro_vs_commanded N | 22208 | 22208 | +0.00e+00 | PASS |
| board_y dynamic camera_raw_rate_vs_gyro RMSE | 1.2735 | 1.2735495533323078 | +4.96e-05 | PASS |
| board_y rate-scale min (%) | -0.7 | -0.7013530140634927 | -1.35e-03 | PASS |
| board_y rate-scale max (%) | 0.52 | 0.5155185586307098 | -4.48e-03 | PASS |
| board_y n_frames | 67436 | 67436 | +0.00e+00 | PASS |
| board_y valid_pose_pct | 100.0 | 100.0 | +0.00e+00 | PASS |
| board_y branch_flips | 0 | 0 | +0.00e+00 | PASS |
| board_y ambiguous_frames | 0 | 0 | +0.00e+00 | PASS |
| board_y four_tag_visibility_pct | 89.3959 | 89.39587164125986 | -2.84e-05 | PASS |
| board_y reprojection_median_px | 0.262 | 0.2619812357988281 | -1.88e-05 | PASS |
| board_y reprojection_mean_px | 0.2604 | 0.26039168200133944 | -8.32e-06 | PASS |
| board_y candidate_separation_min_deg | 24.197 | 24.197074474162 | +7.45e-05 | PASS |
| board_y candidate_separation_max_deg | 103.492 | 103.4916862861044 | -3.14e-04 | PASS |
| board_y reprojection_ratio_min | 10.931 | 10.93114072198873 | +1.41e-04 | PASS |
| board_y reprojection_ratio_max | 23.73 | 23.729888504360005 | -1.11e-04 | PASS |
| board_y hinge axis x | -0.0351 | -0.035095587820779886 | +4.41e-06 | PASS |
| board_y hinge axis y | 0.99924 | 0.9992401348719484 | +1.35e-07 | PASS |
| board_y hinge axis z | -0.01695 | -0.01695442646049984 | -4.43e-06 | PASS |
| replication axis_a camera-commanded RMSE (deg) | 0.3203 | 0.3203442098502052 | +4.42e-05 | PASS |
| replication axis_a camera-gravity RMSE (deg) | 0.3263 | 0.3262797051482702 | -2.03e-05 | PASS |
| replication run04 camera-commanded RMSE (deg) | 0.5698 | 0.5698399661571184 | +4.00e-05 | PASS |
| replication run04 camera-gravity RMSE (deg) | 0.3042 | 0.3042175353058652 | +1.75e-05 | PASS |

## Methodological note: two pose sources

Board-X runs were acquired with their correct board layout, so their logged
relative attitude is authoritative and is used as-is (`pose_source:
acquisition_logged`). RUN05 was acquired with the stale Board-X layout, so its
logged pose columns are invalid and the pose is re-solved for every frame from
stored raw corner pixels (`pose_source: reprocessed_from_raw_corners`).

This distinction is not cosmetic. Re-solving a Board-X run shifts its
camera-vs-commanded RMSE by about 0.10 deg, because the acquisition established
its zero reference from the first frames of the CAPTURE, which precede the motor
baseline window used here. Reprocessing correctly-processed data would therefore
silently change every Board-X number. Each run's `RUN_INFO.json` records which
source is authoritative and why.

Conditioning diagnostics (validity, flips, ambiguity, reprojection, candidate
separation) are ALWAYS recomputed from raw corner pixels, for every run.

## Scalar attitude convention

Board X uses the active relative Euler component; Board Y uses the projection of
the relative rotation vector onto the calibrated hinge axis. Both are reported,
and `static_alternative_scalar` in the per-run JSON gives the same data under the
other convention. On Board X the two differ by under 0.003 deg RMSE, so the
choice is not load-bearing.
