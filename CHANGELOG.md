# Changelog

## [1.0.0-aeroconf2027] — repository as submitted

Reproducibility repository for IEEE Aerospace Conference paper 2389.

### Contents
- Acquisition firmware and software preserved unmodified under `src/original/`.
- Complete raw campaign data for six runs under `data/raw/`, organised by PHYSICAL
  board-frame axis (yaw, Board X, Board Y) rather than by original folder label.
- Reproduction pipeline under `analysis/`, written after the campaign.
- SHA-256 provenance for every included raw file.

### Reproduction status
`python analysis/reproduce_all.py` reproduces 54/54 gated headline values from the raw
data and exits 0. `python -m pytest tests/ -q` passes 27 tests.

### Notes on organisation
- Original folder labels (AXIS_A, AXIS_B, RUN05) are retained only in `RUN_INFO.json`.
  Three of them denote the SAME physical hinge axis (Board X) in different camera-view or
  IMU-mounting configurations; only RUN05 is a distinct board axis (Board Y).
- Two board geometry files exist because the tag board was physically re-mounted between
  configurations. Each run declares which applies; cross-application is refused in code.
- RUN05 was acquired with the stale Board-X layout and is reprocessed from stored raw
  AprilTag corner pixels. Board-X runs were acquired correctly and their logged attitude is
  used as-is; re-solving them would change their zero reference and silently shift every
  number. Each run declares `pose_source` and the reason.

### Correction applied during the final audit
An intrinsics audit found that offline reprocessing had applied the raw `camera_matrix` to
undistorted corner coordinates (7.54 % focal-length error). Fixed by using `newK` from
`getOptimalNewCameraMatrix`, and the zero reference now replicates the acquisition's
first-200-capture-frames construction. Board-X and yaw values are unaffected and reproduce
exactly; Board-Y (RUN05) values are **corrected**, with the superseded originals preserved
in `results/verified_numbers.json` under `board_y_AS_SUBMITTED_SUPERSEDED`.

### Final audit: RUN05 board layout refitted
`analysis/board_y/refit_board_y_geometry.py` established that
`board_geometry_boardY_bundle_adjusted.json` had itself been bundle-adjusted with the raw
`camera_matrix` (refitting with the raw matrix reproduces the stored file to 0.0000 mm).
Refitted with `newK`: tag centres move up to 0.4817 mm, bundle RMS improves
0.3559 -> 0.2583 px. Board-Y results regenerated; final static
camera - commanded RMSE 0.2918 deg. Superseded layout kept as
`board_geometry_boardY_bundle_adjusted_rawK_SUPERSEDED.json`; superseded values kept in
`results/verified_numbers.json`. Board-X and yaw unchanged.

### Final GitHub cleanup and filtered-rate audit
Paper-facing RUN05 naming removed from filenames and code paths (RUN05 survives only as
provenance metadata such as `original_run_label`): `reproduce_board_y.py`,
`refit_board_y_geometry.py`, `results/board_y.json`,
`results/tables/board_y_rate_scale.csv`, `config/board_geometry_boardY_bundle_adjusted.json`.
`results/board_y.json` regenerated from the final newK-intrinsics / newK-layout analysis;
tests now assert that no active result file carries a superseded Board-Y value.

Added a SECONDARY filtered-rate audit: one frozen Savitzky-Golay estimator (1.00 s, order 2,
first derivative, zero phase), unchanged from the established yaw method, applied identically
to all three axes over the same constant-rate leg windows. SG RMSE yaw
0.0421, Board X 0.1196, Board Y
0.1241 deg/s. RAW results, synchronization, geometry and the
motor/IMU reference definitions are unchanged.

### Excluded
- `RUNimu_teensy_0002.CSV` from the original RUN05 folder: byte-identical to the Board-X
  run04 manual-handling record. Documented in `DATA_PROVENANCE.md` and `data/MANIFEST.csv`.
