# REPO_BUILD_REPORT.md

Build of the reproducibility repository for IEEE Aerospace Conference paper 2389,
*Camera-Based Three-Axis Attitude Measurement for Small-Satellite Testbeds: A V&V Approach*.

**Reproduction PASSED - 54/54 gated values. Tests PASSED - 34/34.**

Two audits during the final pass produced one material correction to the Board-Y results.
Sections 5, 6 and 14 give it in full. Board-X and yaw are unaffected throughout.

> The SHA-256 of the distributed archive is in `SHA256SUM.txt` beside it. A hash cannot be
> embedded in the file it hashes, so none appears here.

---

## 1. Source archive inventory

`final_data for papper.zip` (264 MB): the yaw campaign folder plus four nested ZIPs.
Six runs retained, **49 raw files, 544 MB**.

| Source | Contents |
|---|---|
| `result_yaw/` | 2 yaw runs (test1 CW, test2 CCW) + 4 acquisition/firmware files |
| `run01_AXIS_A___.zip` | 1 out-of-plane run + 4 code files; two Teensy records |
| `run01_AXIS_B.zip` | 1 out-of-plane run + 5 code files; two Teensy records |
| `run04_AXIS_B.zip` | 1 out-of-plane run; two Teensy records |
| `1789917316032_RUN05.zip` | 1 out-of-plane run + 3 code files + RUN05 geometry |

## 2. Acquisition code preserved unchanged

Byte-for-byte under `src/original/`: `yaw/basler_apriltag_multitag_FINAL.py`,
`yaw/motor_yaw_FINAL.ino`, `yaw/motor_serial_logger_FINAL.py`,
`out_of_plane/basler_apriltag_pitch_roll_FINAL.py`, `out_of_plane/motor_pitch_roll_FINAL.ino`,
`out_of_plane/motor_serial_logger_pitch_roll_FINAL.py`, and
`teensy41_imu_sd_logger_final.ino` (unchanged for the whole campaign). `src/firmware/` and
`src/acquisition/` are byte-identical copies grouped by role; `src/original/` is authoritative.

## 3. New post-campaign reproducibility code

All of `analysis/` and `tests/`, each with a `CREATED POST-CAMPAIGN` header. None ran on the
testbed.

```
analysis/common/{io, motor_reference, camera_motor_sync, imu_motor_sync,
                 attitude_metrics, geometry_utils, out_of_plane}.py
analysis/yaw/reproduce_yaw.py + yaw_vv_master_analysis.py (adapted, labelled)
analysis/board_x/reproduce_board_x.py + audit_board_x_reference.py
analysis/board_y/reproduce_board_y.py + refit_board_y_geometry.py
analysis/reproduce_all.py, analysis/make_paper_figures.py
tests/ (4 modules, 34 tests)
```

## 4. Physical-axis remapping

Folder labels were not trusted; each run's hinge axis was derived from recorded geometry.

| Canonical axis | Runs | Original labels | Gyro |
|---|---|---|---|
| Yaw (in-plane) | `yaw/run01_CW`, `yaw/run02_CCW` | `result_yaw/test1`, `test2` | Z |
| Board X | `board_x/axis_b_run01` (headline), `axis_a`, `axis_b_run04` | `run01_AXIS_B`, `run01_AXIS_A`, `run04_AXIS_B` | Y, Y, X |
| Board Y | `board_y/run05` (headline) | `RUN05` | X |

Three runs share Board X to within ~1 deg despite an 89.5 deg rotation in the camera frame
(board and IMU rotated with the platform). Only RUN05 moved the board relative to the hinge.
Final Board-Y hinge axis **(-0.03510, +0.99924, -0.01695)**,
2.234 deg from board +Y, ~87 deg from Board X.

## 5. Board-X final audit - the original hypothesis was WRONG

`analysis/board_x/audit_board_x_reference.py` -> `results/board_x_reference_audit.json`.

The acquisition pool was read from the original code (`REF_MIN_FRAMES = 200`, filled in
**capture order**) and identified as frames
**0-199**, motor-clock
87.720-90.239 s, i.e.
**29.080 s before** the motor baseline window. Those same corner
observations were re-solved with the same configured layout, IPPE candidate logic, corner
ordering and relative-rotation definition.

| Variant | RMSE | Bias | MAE | MaxAbs | N |
|---|---|---|---|---|---|
| A logged acquisition pose, Euler | 0.2678 | 0.2197 | 0.2345 | 0.4691 | 9499 |
| B re-solved, acquisition-equivalent reference, Euler | **0.2678** | 0.2197 | 0.2345 | 0.4691 | 9499 |
| C re-solved, acquisition-equivalent reference, axis projection | 0.2708 | 0.2196 | 0.2368 | 0.4801 | 9499 |
| D re-solved, motor-baseline reference, Euler | 0.2678 | 0.2197 | 0.2345 | 0.4691 | 9499 |

Maximum sample-wise difference, logged vs re-solved relative attitude:
**4.442e-10 deg** over 44 591 frames.

**0.2678 deg reproduces from raw corners.** The earlier attribution to the zero-reference
definition is retracted: the reference accounted for only ~0.004 deg of a ~0.093 deg gap.
The cause was the **intrinsic matrix** - the acquisition undistorts each frame and solves
with `newK`, so the logged corners are undistorted and `newK` belongs with them
(fx 1334.3666 versus raw 1443.2497, +8.16 %).

## 6. RUN05 geometry: audit, refit and reprocessing

`analysis/board_y/refit_board_y_geometry.py` -> `results/board_y_geometry_refit.json`.

**Which matrix produced the stored layout?** Refitting with the raw `camera_matrix`
reproduces `board_geometry_boardY_bundle_adjusted.json` to **0.0000 mm**; refitting with
`newK` moves centres by up to **0.4817 mm** and rotations by up to
**0.2358 deg**. Documentary confirmation: the original exporter read
`meta['camera_matrix']` and never called `getOptimalNewCameraMatrix`. **CASE B.**

Refit method, fully deterministic: frame selection 3 frames per static hold, evenly spaced by np.linspace over the window trimmed 1.5 s at each end; 2 frames per rate leg, evenly spaced over the central 70 % of the leg; a frame is kept only if >= 3 expected tags are visible and pose_ok == 1;
**83 frames** (68 four-tag, 15 three-tag);
**2536 residuals**, **507 parameters** (9 geometry + 6 per frame);
gauge tag 0 fixed in position and at rotation_deg = 0 during the fit; centroid of the four centres re-centred to the origin afterwards (pure translation, absorbed by the pose); fixed tag size 0.070 m; initial geometry
`config/board_geometry_boardX.json`; optimiser scipy.optimize.least_squares, trf, x_scale=jac; termination xtol = ftol = gtol = 1e-14,
max_nfev = 600; final RMS reprojection **0.2583 px**
(raw-matrix fit: 0.3559 px).

**Board-Y revision history**

| Revision | Intrinsics for pose | Intrinsics behind the layout | static camera - commanded RMSE [deg] | board reprojection median [px] |
|---|---|---|---|---|
| as submitted | raw | raw | 0.2987 | 0.3676 |
| intermediate | newK | raw | 0.2539 | 0.2815 |
| **final** | **newK** | **newK** | **0.2918** | **0.2620** |

Final Board-Y: camera - commanded Bias 0.2741, MAE
0.2754, MaxAbs 0.4939 deg;
camera - gravity RMSE 0.2474 deg; rate-scale
-0.70 % to +0.52 %; minimum reprojection ratio
10.931. Both the lower reprojection and the higher ratio are
independent evidence that `newK` is correct.

Superseded artefacts retained: `config/board_geometry_boardY_bundle_adjusted_rawK_SUPERSEDED.json`,
and `board_y_AS_SUBMITTED_SUPERSEDED` / `board_y_SUPERSEDED_rev2_rawK_geometry` in
`results/verified_numbers.json`. No expected value was ever edited to force a pass.

## 7. Synchronization algorithms

**Camera<->motor**: affine `t_motor = a*t_camera + b`, least squares on **serial motor-event
timestamps only**, never optimised against attitude error; wall-clock cross-check agrees to
better than 1 ppm. The ppm figure is the signed fractional rate of that map and is always
reported with the coefficient.

**IMU<->motor**: fitted independently in the gyro rate domain, camera never used, constrained
to **>=95 % coverage** of the motor window, which rejects the partial-overlap optimum seen as
a physically implausible `a = 1.033994589` (+33 994.6 ppm).

| Run | camera<->motor a | ppm | residual SD [ms] | events | IMU<->motor a | ppm | residual [deg/s] | coverage [%] |
|---|---|---|---|---|---|---|---|---|
| `board_x/axis_b_run01` | 0.998483425 | -1516.6 | 1.213 | 114 | 0.998433646 | -1566.4 | 0.3242 | 100.00 |
| `board_x/axis_a` | 0.998503709 | -1496.3 | 1.222 | 114 | 0.998424304 | -1575.7 | 0.3481 | 100.00 |
| `board_x/axis_b_run04` | 0.998487897 | -1512.1 | 1.181 | 114 | 0.998627271 | -1372.7 | 0.3478 | 100.00 |
| `board_y/run05` | 0.998498247 | -1501.8 | 1.247 | 114 | 0.998395237 | -1604.8 | 0.3493 | 100.00 |
| `yaw/Run01` | — | -1516.3 | 1.227 | 94 | — | -1516.4 | 0.4084 | — |
| `yaw/Run02` | — | -1513.5 | 1.131 | 94 | — | -1499.0 | 0.4068 | — |

## 8. Excluded stale/duplicate files and proof

| Original | Status | Proof / reason |
|---|---|---|
| `run05_AXIS_B/RUNimu_teensy_0002.CSV` | **excluded** | md5-identical to `board_x/axis_b_run04/imu_teensy_RUN0002.CSV` (`03559774118063f5880f3b09b53e59f6`), an 86.8 s manual-handling record peaking at 225.5 deg/s. The real RUN05 record was identified from gyro motion content as `imu_teensy_RUN0001.CSV`. |
| RUN05 folder's 3 code files | relocated | Acquisition code shipped inside a run folder; preserved under `src/original/out_of_plane/`. |

Stationary, header-only and manual-handling records genuinely part of a run folder are
**retained** and flagged in `RUN_INFO.json`. No other file was removed.

## 9. Checksums

`data/CHECKSUMS.sha256` covers all **49** included raw files; `data/MANIFEST.csv`
carries canonical run, original path, repository path, size, SHA-256, role, status and
exclusion reason. `reproduce_all.py` verifies all checksums first and exits 2 on mismatch.

## 10. Clean-environment test

```
path scan for absolute developer paths   -> none in code, config or data
fresh venv, pip install -r requirements.txt only
python analysis/reproduce_all.py   (run from outside the repository)  -> exit 0
python -m pytest tests/ -q                                            -> 34 passed
python analysis/make_paper_figures.py                                 -> 5 PDFs
extract the ZIP to a clean location, fresh venv, repeat all three     -> all pass
```

A missing `xlsxwriter` dependency was caught by this test and added to `requirements.txt`
and `environment.yml`.

## 11. Unit-test results

**34/34 passed.** `test_motor_reference.py` (pulse arithmetic vs firmware),
`test_synchronization.py` (event-only camera map, coverage constraint, completeness,
monotonicity), `test_geometry_selection.py` (layout pairing refused across axes, planarity,
provenance declared camera-derived, RUN05 layout is the newK refit, `camera_intrinsics`
returns the undistorted matrix with zero distortion), `test_expected_results.py` (gate).

## 12. Numerical reproduction results - all RAW

| | Yaw | Board X | Board Y |
|---|---|---|---|
| Camera - commanded RMSE [deg] | 0.2206 | 0.2678 | 0.2918 |
| Camera - commanded MAE [deg] | 0.1801 | 0.2345 | 0.2754 |
| Camera - commanded Bias [deg] | 0.1706 | 0.2197 | 0.2741 |
| Camera - commanded MaxAbs [deg] | 0.6770 | 0.4691 | 0.4939 |
| Gravity - commanded RMSE [deg] | N/A | 0.4005 | 0.3879 |
| Camera - gravity RMSE [deg] | N/A | 0.2894 | 0.2474 |
| Camera RAW rate - commanded RMSE [deg/s] | 5.2471 | 1.2512 | 1.3095 |
| Gyro - commanded RMSE [deg/s] | 3.8335 | 0.8535 | 0.8759 |
| Camera RAW rate - gyro RMSE [deg/s] | 3.6943 | 1.5362 | 1.2735 |

Board-Y rate scale **-0.70 % to +0.52 %**. Replication:
`axis_a` 0.3203 / 0.3263 deg;
`run04` 0.5698 / 0.3042 deg.

Board-Y conditioning: 67436 frames, 100.000 % valid pose,
0 flips, 0 ambiguous, 89.3959 % four-tag
visibility, reprojection median 0.2620 px, candidate separation
24.197-103.492 deg, ratio
10.931-23.730.

Yaw cells are N/A: that acquisition logged no roll/pitch. Dynamic rows are **not** comparable
across columns (rate ranges 1-30 vs 1-10 deg/s, different travel and frame rates). Signed RAW
rate bias differs by axis - yaw 0.00672 deg/s, Board Y
0.00308 deg/s.

## 13. Figure generation

`python analysis/make_paper_figures.py` -> 5 PDFs in `results/figures/`: F3 synchronization,
F4 Board-Y static, F5 conditioning, F6 rate scale, F7 zero-return drift. Limits come from the
data including outliers; nothing is clipped. Seven CSV tables in `results/tables/`.

## 13b. Filtered-rate audit (SECONDARY)

`analysis/common/rate_estimators.py` + `analysis/rate_audit.py`. ONE frozen Savitzky-Golay
estimator (1.00 s window, polynomial order 2, first derivative,
zero phase, uniform-grid resample), taken unchanged from the established yaw method and
applied identically to Yaw, Board X and Board Y over the SAME constant-rate leg windows used
for the RAW results. Not tuned per axis; not optimised to minimise any error.

| | Yaw | Board X | Board Y |
|---|---|---|---|
| RAW instantaneous derivative RMSE [deg/s] | 5.2471 | 1.2512 | 1.3095 |
| SG rate RMSE [deg/s] (secondary) | 0.0421 | 0.1196 | 0.1241 |
| SG Bias / MAE / MaxAbs [deg/s] | -0.0009 / 0.0290 / 0.4142 | -0.0000 / 0.0497 / 2.2674 | +0.0000 / 0.0433 / 2.3339 |
| SG N | 19888 | 8160 | 12175 |
| Displacement rate scale [%] | -0.25 to +0.05 | +0.27 to +1.61 | -0.70 to +0.52 |
| SG window [samples] @ median fps | 88 @ 88.02 Hz | 81 @ 80.33 Hz | 121 @ 120.77 Hz |

**SG RMSE below 0.1 deg/s?** Yaw 0.0421 **YES**;
Board X 0.1196 **NO**; Board Y 0.1241 **NO**.

Output `results/tables/three_axis_rate_comparison.csv`, columns prefixed `RAW_`,
`SG_SECONDARY_` and `DISPSCALE_`. SG is never labelled a RAW accuracy and RAW results are
unchanged by this audit.

## 13c. Naming cleanup

Paper-facing RUN05 naming removed; RUN05 survives only as provenance metadata
(`original_run_label`, prose describing the configuration, and the raw-data folder
`data/raw/board_y/run05/` which is the run identifier).

| Old | New |
|---|---|
| `analysis/board_y/reproduce_run05.py` | `analysis/board_y/reproduce_board_y.py` |
| `analysis/board_y/refit_run05_geometry.py` | `analysis/board_y/refit_board_y_geometry.py` |
| `results/board_y_run05.json` | `results/board_y.json` (regenerated from the FINAL analysis) |
| `results/tables/run05_rate_scale.csv` | `results/tables/board_y_rate_scale.csv` |
| `config/board_geometry_RUN05_bundle_adjusted.json` | `config/board_geometry_boardY_bundle_adjusted.json` |

`tests/test_expected_results.py` now asserts that no active result file contains a
superseded Board-Y value, that the superseded values remain quarantined under their
explicitly-named keys, that no paper-facing RUN05 filename remains, and that SG never
appears inside a RAW block.

## 14. Remaining limitations

1. **Board-Y figures in the manuscript must be updated** to the final values in section 6.
   This is the one item that changes paper text.
2. **Board geometry is camera-derived** - configured board layout, never mechanically
   measured, never an independent dimensional reference.
3. **Motor is a pulse-derived commanded reference** - not ground truth, not an actual angle;
   no encoder telemetry was logged.
4. **No motion-capture or external optical reference** was used.
5. **IMU not independently precision-calibrated**; residual scale, cross-axis and offset
   uncertainty remains. A tilt-angle error is **not** inferred from the accelerometer
   magnitude at rest.
6. **Platform departure from the commanded reference** (0.23-0.38 deg) is seen independently
   by camera and gravity tilt; reported, never removed, no mechanical cause assigned.
7. **Headline runs are directional tests**, not population statistics.
8. **Raw rate derivatives carry differentiation noise**; the rate-scale metric is the
   tracking-fidelity quantity and is never relabelled an instantaneous RMSE.
9. **Frame rates differ between runs**; tag visibility changes at high tilt and the three-tag
   subset is confounded with angle and sign.
10. **Conditioning is configuration-specific.** The largest camera errors occurred nearest the
    fronto-parallel condition and viewing geometry strongly affects conditioning; no strict
    monotonicity is claimed.
11. `CITATION.cff` retains placeholders only for the future repository URL, DOI and
    unsupplied ORCIDs.

## 15. Exact commands used

```bash
# audits
python analysis/board_x/audit_board_x_reference.py
python analysis/board_y/refit_board_y_geometry.py

# verification
python analysis/reproduce_all.py
python -m pytest tests/ -q
python analysis/make_paper_figures.py

# clean environment
python3 -m venv <venv> && <venv>/bin/pip install -r requirements.txt
cd <elsewhere> && <venv>/bin/python <repo>/analysis/reproduce_all.py

# package and verify
zip -r -q -9 camera-attitude-vv_AEROCONF2027_FINAL.zip camera-attitude-vv \
    -x "*__pycache__*" "*.pyc" "*.pytest_cache*"
unzip -t camera-attitude-vv_AEROCONF2027_FINAL.zip
unzip -q camera-attitude-vv_AEROCONF2027_FINAL.zip -d <tmp>
cd <tmp>/camera-attitude-vv && python analysis/reproduce_all.py && python -m pytest tests/ -q
sha256sum camera-attitude-vv_AEROCONF2027_FINAL.zip > SHA256SUM.txt
```

## 16. Archive

Filename `camera-attitude-vv_AEROCONF2027_FINAL.zip`, top-level directory
`camera-attitude-vv/`, full raw dataset included (49 raw files,
544 MB uncompressed). Size and SHA-256 are in
`SHA256SUM.txt` distributed beside the archive, which is authoritative.

```bash
unzip camera-attitude-vv_AEROCONF2027_FINAL.zip
cd camera-attitude-vv
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python analysis/reproduce_all.py
```
