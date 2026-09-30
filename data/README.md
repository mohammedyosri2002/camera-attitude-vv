# data/

## Contents

The **complete raw campaign data** is included in this repository archive: 49 files,
544 MB, organised by **physical board-frame axis**.

| Canonical run | Original label | Files | Size [MB] |
|---|---|---|---|
| `board_x/axis_a` | run01_AXIS_A | 9 | 130.2 |
| `board_x/axis_b_run01` | run01_AXIS_B | 9 | 73.8 |
| `board_x/axis_b_run04` | run04_AXIS_B | 9 | 98.0 |
| `board_y/run05` | RUN05 / run05_AXIS_B | 8 | 102.2 |
| `yaw/run01_CW` | result_yaw/test1 | 7 | 68.0 |
| `yaw/run02_CCW` | result_yaw/test2 | 7 | 71.6 |

## Per-run contents

| File | Role |
|---|---|
| `camera_*.csv` | Camera log. One row per frame, including **all sixteen AprilTag corner pixel coordinates**, per-tag visibility, decision margin and Hamming, and the acquisition-time pose columns. The corner pixels are what make offline re-solution possible. |
| `camera_*_meta.json` | Acquisition metadata: intrinsics, distortion, exposure, gain, solver settings, the board layout used at acquisition time. |
| `motor_events.csv` | Serial motor-event log. The **only** input to the camera↔motor clock fit. |
| `motor_raw.csv` | Raw serial transcript, including the pre-position transaction. |
| `motor_clockfit.json` | Clock fit produced by the serial logger at run time. Retained for provenance; the pipeline refits from events. |
| `prebias_meta.json` | Commanded pre-position record (out-of-plane runs). |
| `imu_teensy*.CSV` | IMU log: raw 3-axis gyro and accel, die temperature, 64-bit µs timestamp. |
| `RUN_INFO.json` | Run metadata: canonical axis, original label, geometry file, IMU file, gyro channel, pre-position, pose source, and the reason for each. |

## Integrity

- `MANIFEST.csv` — canonical run, original path, repository path, size, SHA-256, role,
  included/excluded status and exclusion reason for every file.
- `CHECKSUMS.sha256` — SHA-256 of every included raw file, repository-relative paths.

Verify:

```bash
cd data && sha256sum -c CHECKSUMS.sha256
```

or let the pipeline do it: `python analysis/reproduce_all.py` verifies all checksums as its
first step and exits 2 on any mismatch.

## Choosing the IMU record

Several run folders contain more than one Teensy record. **The correct file is not chosen
by filename.** It was identified from the data — gyro motion content matching the commanded
profile — and that decision is recorded in each `RUN_INFO.json` so reproduction is
deterministic. `analysis/common/io.py:identify_imu_file()` re-derives the same decision from
the data for anyone who wants to check it.

Records that are not runs: stationary recordings with no commanded motion, a header-only
file with zero data rows, and an 86.8 s manual-handling record with a 225.5 deg/s peak
(the board re-mounting operation). These are retained where they were part of the original
folder, and flagged in `RUN_INFO.json` notes.

## Licence

Raw data: **CC BY 4.0**. See the repository `LICENSE`.

## If you are distributing this repository via git

Do **not** commit `data/raw/` to git history — 544 MB in history makes every clone
expensive and cannot be removed without a rewrite. Deposit the raw data in an archival
repository (Zenodo or similar) to obtain a citable DOI, attach it as a release asset for
convenience, and reference the DOI here. `.gitignore` carries a commented-out
`data/raw/` entry for that purpose.

For **this** deliverable the data is included in the archive by explicit request.
