# Camera-Based Three-Axis Attitude Measurement for Small-Satellite Testbeds: A V&V Approach

Reproducibility repository for **IEEE Aerospace Conference paper 2389**
(Session 13.06, System Verification & Validation and Integration & Test).

Mohammed Shalaby · Shamma Jamali · Abdalla Elshaal · Mohamed Okasha

---

## 1. Purpose

On small-satellite attitude testbeds an overhead camera is often used as the reference for
platform motion, and its output is treated as truth without being checked against
independent sensors. **This work treats the camera as the subsystem under test.** The
repository contains the acquisition code that ran on the testbed, the complete raw data,
and a reproduction pipeline that regenerates every headline number in the paper with one
command.

## 2. Reference definitions — read before using any number

| Reference | What it is | What it is **not** |
|---|---|---|
| **Motor** | **Pulse-derived commanded reference.** 25 600 pulses/rev, 0.0140625 deg/pulse, round-to-nearest quantization ±0.00703125 deg. | **NOT ground truth. NOT an actual angle. NOT a measured motor angle.** No independent encoder-position telemetry was logged. |
| **IMU accelerometer** | Gravity-derived tilt during stationary holds, relative to the run's own baseline. | Not independently precision-calibrated. |
| **IMU gyro** | Raw, with only a stationary bias from the same run's own baseline subtracted. | **No scale factor is fitted**, to the motor or to the camera. |
| **Camera** | Rigid four-tag board pose, IPPE, relative to a stationary zero. | The subsystem under test. |

## 3. Hardware

| Item | Specification |
|---|---|
| Camera | Basler ace acA1920-40um, monochrome USB 3.0, Sony IMX249 |
| Acquisition | 1920 × 1200, exposure 5000 µs, gain 0 dB, auto exposure and auto gain disabled |
| Lens | Ricoh FL-CC0814A-2M, 8 mm fixed focal length, C-mount, F1.4–F16 |
| IMU | SparkFun ISM330DHCX on Teensy 4.1, 3-axis accel + gyro, microSD logging, **configured ODR 208 Hz** |
| IMU cadence | The timestamp-derived **logging cadence** measured from the records differs from the configured ODR; it is reported as a logging cadence, not as a physical ODR |
| Motor | UIROBOT UIM5756PM, STEP/DIR, 25 600 commanded pulses/rev |
| Wiring | DIR = D8, STEP = D9, ENABLE = D7 (ENABLE asserted for the whole session) |

Full detail: [`docs/HARDWARE.md`](docs/HARDWARE.md).

## 4. Physical axis map

**Folder labels from the campaign are NOT spacecraft axes.** The physical hinge axis of
each run was determined from the recorded geometry; the original labels are retained in
`RUN_INFO.json` for provenance only.

| Canonical axis | Runs | Original labels | Gyro channel |
|---|---|---|---|
| **Yaw** (in-plane) | `yaw/run01_CW`, `yaw/run02_CCW` | `result_yaw/test1`, `test2` | Z |
| **Board X** (out-of-plane) | `board_x/axis_b_run01` *(headline)*, `board_x/axis_a`, `board_x/axis_b_run04` | `run01_AXIS_B`, `run01_AXIS_A`, `run04_AXIS_B` | Y, Y, X |
| **Board Y** (out-of-plane) | `board_y/run05` *(headline)* | `RUN05` | X |

The three Board-X runs share the same hinge axis to within about 1 deg despite different
camera-view and IMU-mounting configurations. **Board Y is the genuine second out-of-plane
axis**: reproduced hinge axis
`(-0.03510, +0.99924, -0.01695)`,
2.234 deg from board +Y and about 87.1 deg from Board X.

See [`docs/EXPERIMENT_MAP.md`](docs/EXPERIMENT_MAP.md).

## 5. Geometry warning

```
config/board_geometry_boardX.json                 -> yaw and Board-X runs ONLY
config/board_geometry_boardY_bundle_adjusted.json  -> Board-Y (RUN05) ONLY
```

The tag board was physically re-mounted between configurations, so the two files describe
**different physical boards**. Applying one to the other's runs produces a plausible-looking
wrong answer. `select_geometry()` refuses to do it and `tests/test_geometry_selection.py`
verifies the refusal.

**Both files are camera-derived acquisition/reprocessing geometry models. Neither is
independent dimensional ground truth.** Call them the *configured board layout* or
*camera-derived board geometry* — never a *measured board layout*. See
[`docs/GEOMETRY.md`](docs/GEOMETRY.md).

## 6. Synchronization

Camera host, motor controller and IMU free-run on three separate oscillators.

- **Camera ↔ motor**: affine map `t_motor = a · t_camera + b`, fitted by least squares to
  **serial motor-event timestamps only** — never to camera-versus-motor attitude error, so
  the reported agreement is out of sample. Reproduced coefficients give
  −1496 to −1517 ppm with residual SD 1.18–1.25 ms over 114 events (94 for yaw).
  The ppm figure is the signed fractional rate of this specific map; report the coefficient
  with it rather than paraphrasing it as one clock being "fast".
- **IMU ↔ motor**: fitted independently in the gyro rate domain. **The camera is never
  used.** The fit is constrained to solutions covering **≥ 95 % of the motor experiment
  window**, because the commanded pre-position workflow otherwise admits a false
  partial-overlap optimum — observed in practice as a physically implausible
  +33 995 ppm solution.

See [`docs/SYNCHRONIZATION.md`](docs/SYNCHRONIZATION.md).

## 7. Data directory map

```
data/raw/
├── yaw/       run01_CW/  run02_CCW/
├── board_x/   axis_a/  axis_b_run01/  axis_b_run04/
└── board_y/   run05/
```

Each run holds its camera CSV (all sixteen AprilTag corner pixel coordinates per frame),
camera metadata JSON, motor event log, raw motor serial log, acquisition-time clock fit,
the commanded pre-position record where applicable, the IMU record, and `RUN_INFO.json`.

Provenance, checksums and every excluded file: [`DATA_PROVENANCE.md`](DATA_PROVENANCE.md),
`data/MANIFEST.csv`, `data/CHECKSUMS.sha256`.

## 8. Environment setup

```bash
python -m venv .venv && source .venv/bin/activate     # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```
or `conda env create -f environment.yml && conda activate camera-attitude-vv`.

Python 3.10+. Every path in this repository is repository-relative; nothing outside the
repository is required and no interactive file selection is used.

## 9. ONE-COMMAND REPRODUCTION

```bash
python analysis/reproduce_all.py
```

Verifies checksums, refits both clock maps, re-solves camera pose where required,
computes all RAW static, dynamic, rate-scale and conditioning metrics, writes
`results/reproduced_numbers.json`, compares it against `results/verified_numbers.json`,
writes `results/VERIFICATION_REPORT.md` and the paper tables, and **exits non-zero if any
required headline number falls outside tolerance**. Runtime is a few minutes; add
`--skip-yaw` to skip the slowest step.

## 10. Per-axis reproduction

```bash
python analysis/yaw/reproduce_yaw.py           # yaw, CW + CCW pooled
python analysis/board_x/reproduce_board_x.py   # Board X: headline + two replication runs
python analysis/board_y/reproduce_board_y.py     # Board Y, reprocessed from raw corners
```

## 11. Figures and tests

```bash
python analysis/make_paper_figures.py    # -> results/figures/*.pdf
python analysis/rate_audit.py            # SECONDARY filtered-rate audit (also run by reproduce_all)
python -m pytest tests/ -q               # 27 tests
```

## 12. Expected numerical outputs (RAW)

| | Yaw (in-plane) | Board X | Board Y |
|---|---|---|---|
| Camera − commanded, RMSE [deg] | **0.2206** | **0.2678** | **0.2918** |
| Camera − commanded, MAE [deg] | 0.1801 | 0.2345 | 0.2754 |
| Camera − commanded, Bias [deg] | 0.1706 | 0.2197 | 0.2741 |
| Camera − commanded, MaxAbs [deg] | 0.6770 | 0.4691 | 0.4939 |
| Gravity tilt − commanded, RMSE [deg] | N/A | 0.4005 | 0.3879 |
| Camera − gravity tilt, RMSE [deg] | N/A | **0.2894** | **0.2474** |
| Camera RAW rate − commanded, RMSE [deg/s] | 5.2471 | 1.2512 | 1.3095 |
| Gyro − commanded, RMSE [deg/s] | 3.8335 | 0.8535 | 0.8759 |
| Rate-scale error [%] | N/A | — | -0.70 to +0.52 |

Yaw cells are N/A because the yaw acquisition logged no roll/pitch and produced no
gravity-tilt reference. **The dynamic rows are not comparable across columns**: the rate
ranges (1–30 vs 1–10 deg/s), travel and frame rates differ.

Board-Y conditioning: 67436 frames, 100.000 % valid pose,
0 branch flips, 0 ambiguous frames,
89.3959 % four-tag visibility, board reprojection median
0.2620 px, candidate separation
24.197–103.492 deg,
reprojection ratio 10.931–23.730.

### Board-Y revision history

The Board-Y figures were revised twice during the final audits. Both causes were the same
underlying issue: the acquisition undistorts every frame before detection and solves with
`newK` from `getOptimalNewCameraMatrix`, so the stored corner pixels are in **undistorted**
image coordinates and `newK` — not the raw `camera_matrix` — is the matrix that belongs
with them (fx 1334.3666 versus 1443.2497, a +8.16 % difference).

| Revision | Intrinsics for pose | Intrinsics behind the board layout | static camera − commanded RMSE [deg] | board reprojection median [px] |
|---|---|---|---|---|
| as submitted | raw `camera_matrix` | raw `camera_matrix` | 0.2987 | 0.3676 |
| intermediate | **newK** | raw `camera_matrix` | 0.2539 | 0.2815 |
| **final** | **newK** | **newK** | **0.2918** | **0.2620** |

`analysis/board_y/refit_board_y_geometry.py` established conclusively that the original
layout file had been fitted with the raw matrix: refitting with the raw matrix reproduces
the stored file to **0.0000 mm**, while refitting with `newK` moves tag centres by up to
**0.4817 mm** and rotations by up to
**0.2358 deg**. The corrected refit lowers the bundle RMS
from 0.3559 px to
**0.2583 px**, and the final full-run board reprojection
falls to 0.2620 px with the minimum
second-to-selected reprojection ratio rising to 10.931. Both
improvements are independent evidence that `newK` is the physically correct matrix.

Superseded values are preserved in `results/verified_numbers.json` under
`board_y_AS_SUBMITTED_SUPERSEDED` and `board_y_SUPERSEDED_rev2_rawK_geometry`, and the
superseded layout as `config/board_geometry_boardY_bundle_adjusted_rawK_SUPERSEDED.json`.
**None of them may be used in the paper.** Board-X and yaw are unaffected throughout:
their submitted values come from the acquisition, which used `newK` correctly, and they
reproduce exactly.

### Filtered-rate audit (SECONDARY — never a RAW accuracy)

One frozen Savitzky-Golay estimator is applied identically to all three axes over the same
valid constant-rate leg windows as the RAW result: **1.00 s window, polynomial
order 2, first derivative, zero phase, uniform-grid resample**. Parameters come
unchanged from the established yaw method. **The window is not tuned per axis and not
optimised to minimise any error.**

| | Yaw | Board X | Board Y |
|---|---|---|---|
| RAW instantaneous derivative, RMSE [deg/s] | **5.2471** | **1.2512** | **1.3095** |
| SG processed rate, RMSE [deg/s] *(secondary)* | 0.0421 | 0.1196 | 0.1241 |
| SG processed rate, Bias [deg/s] *(secondary)* | -0.0009 | -0.0000 | +0.0000 |
| SG processed rate, MAE [deg/s] *(secondary)* | 0.0290 | 0.0497 | 0.0433 |
| SG processed rate, MaxAbs [deg/s] *(secondary)* | 0.4142 | 2.2674 | 2.3339 |
| SG N | 19888 | 8160 | 12175 |
| Displacement rate-scale [%] | -0.25 to +0.05 | +0.27 to +1.61 | -0.70 to +0.52 |
| SG window [samples] at median fps | 88 @ 88.02 Hz | 81 @ 80.33 Hz | 121 @ 120.77 Hz |

**Is SG RMSE below 0.1 deg/s?** Computed, not assumed:
**Yaw 0.0421 — YES.
Board X 0.1196 — NO.
Board Y 0.1241 — NO.**

Per-commanded-rate detail: `results/tables/three_axis_rate_comparison.csv`, whose columns
are prefixed `RAW_`, `SG_SECONDARY_` and `DISPSCALE_` so the three views cannot be confused.
The SG columns are a bandwidth-limited estimate, not a RAW accuracy, and the axes are not
comparable with one another: rate ranges, travel and frame rates differ.

**All values above are RAW**: no low-pass, no moving average, no Savitzky-Golay, no Kalman,
no fitted camera or IMU scale, no drift subtraction, no zero-return correction. The yaw
Savitzky-Golay and Kalman rate results exist in `results/yaw.json` under an explicitly
named secondary-diagnostics key and are never substituted into the RAW comparison.

## 13. Known limitations

Summarised here, in full in [`docs/LIMITATIONS.md`](docs/LIMITATIONS.md):

- the motor command is not encoder-verified ground truth;
- both board geometry files are camera-derived, not mechanically measured;
- no motion-capture or external optical reference was used;
- the IMU was not independently precision-calibrated;
- Board-X and Board-Y headline results are directional tests, not population statistics;
- raw camera rate derivatives contain numerical differentiation noise;
- acquisition frame rates differ between runs;
- tag visibility changes at high tilt;
- rate ranges differ between the yaw and out-of-plane experiments.

## 14. Citation

See [`CITATION.cff`](CITATION.cff). Cite a tagged release and the archived dataset DOI, not
`main`. The repository URL and DOI fields are marked for replacement.

## 15. Licence

Author-created code and documentation: see [`LICENSE`](LICENSE). Third-party components
(pypylon, pupil_apriltags, OpenCV, Arduino/Teensy libraries, vendor firmware and SDKs) are
**not** covered by it and remain under their own licences; the repository asserts no rights
over them. Vendor datasheets are not redistributed.
