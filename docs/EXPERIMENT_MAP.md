# Experiment Map

## Physical axes, not folder labels

The campaign's original folder labels (`AXIS_A`, `AXIS_B`, `RUN05`) do **not** denote
different spacecraft axes. The physical hinge axis of each run was determined from the
recorded geometry: the hinge direction in the board frame is the principal direction of the
relative rotation vectors, and expressing the same axis in the camera frame through the
reference pose distinguishes a change of imaging configuration from a change of body axis.

| Canonical axis | Repository path | Original label | Gyro | Role |
|---|---|---|---|---|
| Yaw (in-plane) | `data/raw/yaw/run01_CW` | `result_yaw/test1` | Z | directional run |
| Yaw (in-plane) | `data/raw/yaw/run02_CCW` | `result_yaw/test2` | Z | directional run |
| Board X | `data/raw/board_x/axis_b_run01` | `run01_AXIS_B` | Y | **headline** |
| Board X | `data/raw/board_x/axis_a` | `run01_AXIS_A` | Y | replication |
| Board X | `data/raw/board_x/axis_b_run04` | `run04_AXIS_B` | X | replication |
| Board Y | `data/raw/board_y/run05` | `RUN05` | X | **headline** |

Each run carries a `RUN_INFO.json` recording its original label, geometry file, IMU record,
gyro channel, commanded pre-position, pose source and the reasons for each choice.

## Why three runs share Board X

Between `axis_a` and `axis_b_run01` the assembly was rotated about 89.5 deg **in the camera
frame** — but the tag board and the IMU rotated with the platform, so in both body-fixed
frames the hinge axis is unchanged (to within about 0.2 deg). The imaging configuration
changed; the body axis did not.

Between `axis_b_run01` and `axis_b_run04` the **IMU** was re-mounted about 87 deg (gyro
channel moved from Y to X), but the tag board was not. Again the camera sees the same board
axis.

Only in `RUN05` was the **tag board itself** re-mounted relative to the hinge. Reproduced
board-frame hinge axis `(-0.03299, +0.99931, -0.01695)`, 2.125 deg from board +Y and about
87.1 deg from Board X. That is the genuine second out-of-plane axis.

Consequence: the campaign covers **three distinct board axes** (in-plane yaw, Board X,
Board Y) and exercises **all three gyro channels** (Z, Y, X).

## Commanded profiles

Machine-readable: [`config/profiles/`](../config/profiles/), derived from the archived
event logs.

**Out-of-plane** (both axes, identical): 60 s stationary baseline; 17 static holds in the
sequence `0 +5 +10 +15 +20 +15 +10 +5 0 -5 -10 -15 -20 -15 -10 -5 0` deg with 10 s dwell
and 5 deg/s transits; 20 s intermission; two back-and-forth cycles between -10 and +10 deg
at each of 1, 2, 5 and 10 deg/s (16 legs); commanded return to zero; 60 s baseline.
**114 serial events**, motor span 474.3 s.

The static sequence revisits ±5, ±10 and ±15 deg from opposite approach directions so that
hysteresis is observable; `results/tables/` reports it.

**Yaw**: two directional runs (CW, CCW), 17 static holds 0–720 deg, constant-rate segments
to 30 deg/s, **94 serial events** each.

**Effective commanded values.** Errors are computed against the pulse-derived values, never
the nominal request:

| Nominal | Pulses | Effective commanded reference |
|---|---|---|
| 5 deg | 356 | 5.0062500 deg |
| 10 deg | 711 | 9.9984375 deg |
| 15 deg | 1067 | 15.0046875 deg |
| 20 deg | 1422 | 19.9968750 deg |
| −30 deg (pre-position) | −2133 | −29.9953125 deg |

## Commanded pre-position

The out-of-plane runs begin with a commanded pre-position of −29.9953125 deg (−2133
pulses) from the mechanically balanced start, held by motor torque, which is then redefined
as software zero. This keeps the board away from fronto-parallel across the sweep, where
the planar pose is ill-conditioned.

**This is a pulse-derived commanded pre-position, not an encoder-verified physical angle.**
The pre-position record for each run is archived as `prebias_meta.json`.
