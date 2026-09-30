# Board Geometry

## Two files, two physical boards

```
config/board_geometry_boardX.json                 -> yaw and Board-X runs ONLY
config/board_geometry_boardY_bundle_adjusted.json  -> Board-Y (RUN05) ONLY
```

The tag board was **physically re-mounted** relative to the hinge between the Board-X and
Board-Y configurations, so these files describe different physical boards. They are not
interchangeable.

Applying the wrong file produces a plausible-looking wrong answer rather than an obvious
failure: with the stale layout, RUN05 reprojects at about 1.21 px instead of 0.37 px and
under-reads a 20 deg command by about 3.5 deg.

`select_geometry()` in `analysis/common/geometry_utils.py` raises on cross-application, and
`tests/test_geometry_selection.py` verifies the refusal in both directions.

## Provenance — neither file is dimensional ground truth

| File | `geometry_source` |
|---|---|
| `board_geometry_boardX.json` | `camera_derived_planar_homography` |
| `board_geometry_boardY_bundle_adjusted.json` | `camera_derived_bundle_adjustment_RUN05` |

**Both are camera-derived acquisition / reprocessing geometry models. Neither was
mechanically measured. Neither is an independent dimensional reference.**

Correct terminology: **configured board layout** or **camera-derived board geometry**.

Incorrect and to be avoided: *measured board layout*, *ground-truth geometry*, *surveyed
board*.

The RUN05 layout was obtained by bundle adjustment of the logged AprilTag corner pixels
with the tag black-square size **held fixed** at 0.070 m, because that size sets the
absolute scale and a monocular camera has no independent length reference. The layout is
therefore self-consistent in shape but inherits its absolute scale from an entered
constant.

Mechanical measurement of the tag centre-to-centre distances and of the printed
black-square edge remains outstanding work. Until it is done, no absolute-accuracy claim
should rest on these files. See [`LIMITATIONS.md`](LIMITATIONS.md).

## Intrinsic matrix: the correction that changed the Board-Y numbers

The acquisition program **undistorts every frame before detection**:

```
newK, _ = cv2.getOptimalNewCameraMatrix(CAMERA_MATRIX, DIST_COEFFS, (w,h), 1, (w,h))
m1, m2  = cv2.initUndistortRectifyMap(CAMERA_MATRIX, DIST_COEFFS, None, newK, ...)
und     = cv2.remap(frame, m1, m2, cv2.INTER_LINEAR)
tags    = detector.detect(und, ...)
sol     = solve_board(O, I, newK, R_prev)
```

The corner pixels in the camera CSV are therefore in **undistorted image coordinates**, and
the matrix that belongs with them is `newK` - **not** the raw `camera_matrix` stored in the
metadata. On this rig that is a **7.54 % focal-length difference**: fx 1443.25 versus
1334.37.

The wrong matrix does not fail loudly: the pose still solves on 100 % of frames with zero
branch flips, and board reprojection rises only from about 0.26 px to 0.37 px.

`analysis/board_x/audit_board_x_reference.py` isolated it on Board X, where with `newK` and
the acquisition reference construction the re-solved attitude matches the logged
acquisition attitude to **0.000000 deg**. `analysis/board_y/refit_board_y_geometry.py` then
showed the RUN05 **board layout itself** had been fitted with the raw matrix and refitted
it correctly.

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

## Object-point convention

Object points are built per tag, in the detector's corner order:

```
local corners (un-rotated) : (-h,-h), (+h,-h), (+h,+h), (-h,+h)     h = size/2
p = Rz(rotation_deg) @ local + (center_x_mm, center_y_mm)/1000
z = 0 for every corner
```

This matches `build_object_points()` in the production acquisition code exactly, so the
exported layout numbers need no re-interpretation.

## Pose solution

Rigid multi-tag board pose from all simultaneously visible corners, using
`solvePnPGeneric` with `SOLVEPNP_IPPE`:

1. both planar candidates are returned;
2. candidates with non-positive depth are rejected as physically invalid;
3. both reprojection errors are retained and logged;
4. the lower-error candidate is taken **unless** the second is within a factor of 1.5, in
   which case temporal continuity selects the candidate closer to the previous pose;
5. frame-to-frame jumps beyond 20 deg are counted as branch flips.

Continuity alone is not safe — a continuity-first tracker can follow the mirror branch
through a sign reversal. The reprojection gate is what prevents it.

Relative attitude is `R_rel = R_ref^T R_board`, with `R_ref` the chordal mean of the first
200 valid frames in the stationary baseline. A frame-by-frame subtraction of absolute Euler
angles is never used, and a constant camera mounting orientation cancels exactly.

## Two pose sources, and why

**Every out-of-plane run is reprocessed from stored raw corner pixels**
(`pose_source: reprocessed_from_raw_corners`), using its own configured layout, the
undistorted intrinsic matrix `newK`, and the acquisition reference construction. For
Board X this reproduces the logged acquisition attitude to 0.000000 deg, so the choice is
not load-bearing there; for Board Y it is mandatory, because RUN05 was acquired with the
stale Board-X layout and its logged pose columns are invalid.

The zero reference replicates the acquisition exactly: the chordal mean of the **first 200
valid-pose frames of the capture, in capture order**, with no reference to motor time. The
camera starts before the motor, so these frames precede the motor baseline window. Using
the motor baseline window instead shifts the Board-X static RMSE by about 0.004 deg.

Each run's `RUN_INFO.json` records `pose_source`, `scalar_attitude` and the reason.

## Scalar attitude convention

Board X uses the active relative Euler component; Board Y uses the projection of the
relative rotation vector onto the calibrated hinge axis. The calibrated axis is the
principal direction of the per-hold mean rotation vectors — estimated from camera data
alone, with the commanded reference supplying only the sign, never a scale.

Both conventions are reported: `static_alternative_scalar` in each run's JSON gives the same
data under the other convention. On Board X they differ by under 0.003 deg RMSE, so the
choice is not load-bearing.
