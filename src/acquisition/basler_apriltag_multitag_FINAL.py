#!/usr/bin/env python3
# ============================================================
# basler_apriltag_multitag_FINAL.py
# ------------------------------------------------------------
# FINAL PRODUCTION ACQUISITION -- IEEE Aerospace yaw campaign.
# Ubuntu / Linux.
#
#   python3 basler_apriltag_multitag_FINAL.py \
#       --run 1 --dir CW --duration 840 --out runs
#
# writes to   runs/run01_CW/camera_run01_CW_<stamp>.csv (+ .xlsx)
#
# ------------------------------------------------------------
# THE TESTED YAW ALGORITHM IS UNCHANGED.
#   * rotation_matrix_to_euler_xyz          : unchanged
#   * yaw_wrapped = -yaw                    : unchanged (sign kept
#     as tested; it is inverted for this rig but negating offline
#     is exact, and changing it mid-campaign would split the data
#     across two conventions)
#   * unwrap_angle_deg                      : unchanged
#   * first-seen per-tag yaw zero reference : unchanged
#   * decision-margin weighted fusion       : unchanged
#   * OUTLIER_THRESHOLD_DEG = 12.0          : unchanged
#   * CAMERA_MATRIX / DIST_COEFFS           : unchanged
#   * EXPOSURE_US = 5000, GAIN_DB = 0       : unchanged
#   * online low-pass filter                : PERMANENTLY OFF
# Alternative fusion is evaluated OFFLINE from the per-tag columns.
#
# ADDED (purely additive raw columns, nothing is overwritten):
#   pc_wall_time_ns, pc_perf_counter_ns   (common PC clocks)
#   tag_<id>_roll_deg      absolute, per tag, unprocessed
#   tag_<id>_pitch_deg     absolute, per tag, unprocessed
#   tag_<id>_yaw_abs_deg   unwrapped yaw BEFORE zero-referencing
#   tag_<id>_pose_err      detector pose residual
#   tag_<id>_hamming       decode correction count
#   tag_<id>_tz_m          pose_t[2], tag range
#
# Roll and pitch are NOT fused -- the mounting geometry does not
# justify it for these yaw runs. Raw per-tag values only.
#
# PRE-FLIGHT: a preview window opens first so the image and tags can
# be checked. After confirmation the preview is destroyed and the
# precision recording runs with no imshow, no overlays and no video.
# ============================================================

import argparse
import json
import math
import os
import time

import cv2
import numpy as np
import pandas as pd
from pypylon import pylon
from pupil_apriltags import Detector


# ============================================================
# FIXED ACQUISITION CONFIGURATION -- identical for all ten runs
# ============================================================
TAG_FAMILY = "tag36h11"
TAG_SIZE_M = 0.07                 # black square edge, metres

EXPOSURE_US = 5000.0
GAIN_DB = 0.0

OUTLIER_THRESHOLD_DEG = 12.0      # multi-tag median gate (unchanged)
USE_FILTER = False                # online filtering permanently OFF
SAVE_VIDEO = False                # precision runs
DRAW_OVERLAYS = False             # precision runs
LOG_BRIGHTNESS = True

EXPECTED_TAG_IDS = [0, 1, 2, 3]
BASELINE_CHECK_S = 60.0           # start baseline window for the tag check
STATUS_PERIOD_S = 1.0

CAMERA_MATRIX = np.array([
    [1443.2496917062335, 0.0, 963.2151010468531],
    [0.0, 1443.737705496836, 580.3332937596235],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

DIST_COEFFS = np.array([
    -0.1575447, 0.16190211, -0.00044674, -0.00022088, -0.11840788
], dtype=np.float64)


# ============================================================
# HELPER FUNCTIONS  (unchanged maths)
# ============================================================
def rotation_matrix_to_euler_xyz(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def unwrap_angle_deg(current_wrapped, previous_unwrapped):
    if previous_unwrapped is None:
        return current_wrapped
    delta = current_wrapped - previous_unwrapped
    while delta > 180.0:
        current_wrapped -= 360.0
        delta = current_wrapped - previous_unwrapped
    while delta < -180.0:
        current_wrapped += 360.0
        delta = current_wrapped - previous_unwrapped
    return current_wrapped


def fuse_tag_measurements(tag_measurements):
    if len(tag_measurements) == 0:
        return None, 0, []
    yaws = np.array([m[1] for m in tag_measurements], dtype=np.float64)
    weights = np.array([max(m[2], 1.0) for m in tag_measurements], dtype=np.float64)
    ids = [m[0] for m in tag_measurements]
    if len(yaws) == 1:
        return float(yaws[0]), 1, ids
    median_yaw = np.median(yaws)
    keep_mask = np.abs(yaws - median_yaw) <= OUTLIER_THRESHOLD_DEG
    kept_yaws = yaws[keep_mask]
    kept_weights = weights[keep_mask]
    kept_ids = [ids[i] for i in range(len(ids)) if keep_mask[i]]
    if len(kept_yaws) == 0:
        return float(median_yaw), 1, []
    fused_yaw = float(np.sum(kept_yaws * kept_weights) / np.sum(kept_weights))
    return fused_yaw, len(kept_yaws), kept_ids


# ============================================================
# MAIN
# ============================================================
def main():
    ap = argparse.ArgumentParser(
        description="Final Basler/AprilTag yaw acquisition.")
    ap.add_argument("--run", type=int, required=True, choices=range(1, 11),
                    metavar="1-10")
    ap.add_argument("--dir", required=True, choices=["CW", "CCW"])
    ap.add_argument("--duration", type=float, default=840.0,
                    help="recording duration in seconds (default 840)")
    ap.add_argument("--out", default="runs")
    ap.add_argument("--no-xlsx", action="store_true",
                    help="write CSV only (xlsx of ~35k rows is slow)")
    ap.add_argument("--skip-preflight", action="store_true")
    args = ap.parse_args()

    run_folder = os.path.join(args.out, f"run{args.run:02d}_{args.dir}")
    os.makedirs(run_folder, exist_ok=True)

    stamp = f"run{args.run:02d}_{args.dir}_" + time.strftime("%Y%m%d_%H%M%S")
    CSV_FILE = os.path.join(run_folder, f"camera_{stamp}.csv")
    XLSX_FILE = os.path.join(run_folder, f"camera_{stamp}.xlsx")
    META_FILE = os.path.join(run_folder, f"camera_{stamp}_meta.json")

    print("=" * 66)
    print(f"FINAL CAMERA ACQUISITION   run {args.run:02d}  {args.dir}")
    print(f"  duration : {args.duration:.1f} s")
    print(f"  folder   : {run_folder}")
    print(f"  filter   : {'ON' if USE_FILTER else 'OFF (raw fusion)'}")
    print(f"  video    : {SAVE_VIDEO}   overlays: {DRAW_OVERLAYS}")
    print("=" * 66)

    # --------------------------------------------------------
    # CAMERA SETUP
    # --------------------------------------------------------
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()

    try:
        camera.UserSetSelector.SetValue("Default")
        camera.UserSetLoad.Execute()
    except Exception:
        pass

    for setter in [
        lambda: camera.PixelFormat.SetValue("Mono8"),
        lambda: camera.TriggerMode.SetValue("Off"),
        lambda: camera.AcquisitionMode.SetValue("Continuous"),
        lambda: camera.ExposureAuto.SetValue("Off"),
        lambda: camera.GainAuto.SetValue("Off"),
    ]:
        try:
            setter()
        except Exception:
            pass

    try:
        camera.ExposureTime.SetValue(EXPOSURE_US)
    except Exception:
        pass
    try:
        camera.Gain.SetValue(GAIN_DB)
    except Exception:
        pass

    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_Mono8
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    width = camera.Width.Value
    height = camera.Height.Value

    try:
        cam_max_fps = float(camera.ResultingFrameRate.GetValue())
        print(f"Camera ResultingFrameRate (sensor max): {cam_max_fps:.1f} fps")
    except Exception:
        cam_max_fps = None

    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        CAMERA_MATRIX, DIST_COEFFS, (width, height), 1, (width, height)
    )
    map1, map2 = cv2.initUndistortRectifyMap(
        CAMERA_MATRIX, DIST_COEFFS, None, new_camera_matrix,
        (width, height), cv2.CV_16SC2
    )

    fx_u = float(new_camera_matrix[0, 0])
    fy_u = float(new_camera_matrix[1, 1])
    cx_u = float(new_camera_matrix[0, 2])
    cy_u = float(new_camera_matrix[1, 2])

    detector = Detector(
        families=TAG_FAMILY, nthreads=4, quad_decimate=1.0, quad_sigma=0.0,
        refine_edges=1, decode_sharpening=0.25, debug=0
    )

    def grab_detect():
        """One frame: grab, undistort, detect. Returns (undistorted, tags)."""
        gr = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if not gr.GrabSucceeded():
            gr.Release()
            return None, None
        img = converter.Convert(gr)
        frame = img.GetArray()
        gr.Release()
        und = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
        tags = detector.detect(
            und, estimate_tag_pose=True,
            camera_params=[fx_u, fy_u, cx_u, cy_u], tag_size=TAG_SIZE_M
        )
        return und, tags

    # --------------------------------------------------------
    # PRE-FLIGHT PREVIEW
    # Runs BEFORE any zero reference is created, so it cannot
    # affect the recorded measurements.
    # --------------------------------------------------------
    if not args.skip_preflight:
        print("\nPRE-FLIGHT PREVIEW")
        print("  check the image and that tags "
              f"{EXPECTED_TAG_IDS} are all visible")
        print("  press  g  to accept and start the precision recording")
        print("  press  q  or ESC to abort\n")

        win = "PRE-FLIGHT  (g = go, q/ESC = abort)"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1280, 720)

        accepted = False
        while True:
            und, tags = grab_detect()
            if und is None:
                continue

            disp = cv2.cvtColor(und, cv2.COLOR_GRAY2BGR)
            seen = []
            for tag in tags:
                tid = int(tag.tag_id)
                seen.append(tid)
                corners = tag.corners.astype(np.int32)
                cv2.polylines(disp, [corners.reshape(-1, 1, 2)], True,
                              (0, 255, 0), 2, cv2.LINE_AA)
                top_idx = int(np.argmin(corners[:, 1]))
                tx, ty = corners[top_idx]
                cv2.putText(disp, f"ID {tid}  dm={tag.decision_margin:.0f}",
                            (int(tx) - 10, int(ty) - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                            cv2.LINE_AA)

            seen_sorted = sorted(seen)
            missing = [t for t in EXPECTED_TAG_IDS if t not in seen_sorted]
            txt = [
                f"run {args.run:02d} {args.dir}",
                f"visible: {seen_sorted}",
                ("ALL EXPECTED TAGS VISIBLE" if not missing
                 else f"MISSING: {missing}"),
                "g = go    q/ESC = abort",
            ]
            for i, t in enumerate(txt):
                y = 40 + i * 36
                cv2.putText(disp, t, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(disp, t, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255) if missing else (255, 255, 255),
                            2, cv2.LINE_AA)

            cv2.imshow(win, cv2.resize(disp, (1280, 720)))
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('g'), ord('G')):
                accepted = True
                break
            if k in (ord('q'), ord('Q'), 27):
                break

        cv2.destroyWindow(win)
        cv2.destroyAllWindows()
        for _ in range(5):
            cv2.waitKey(1)

        if not accepted:
            camera.StopGrabbing()
            camera.Close()
            print("Aborted at pre-flight. Nothing was recorded.")
            return

    # --------------------------------------------------------
    # PRECISION RECORDING
    # No imshow, no overlays, no video writer, no waitKey.
    # --------------------------------------------------------
    rows = []
    yaw_zero_ref = {}
    yaw_prev_unwrapped = {}
    prev_elapsed = None
    prev_fused_yaw = None

    baseline_frames = 0
    baseline_incomplete = 0
    baseline_warned = False
    last_status = 0.0
    status_last_count = 0
    status_last_t = 0.0

    start_time = time.time()
    start_wall_ms = int(round(start_time * 1000.0))
    start_perf_ns = time.perf_counter_ns()

    print(f"\nRECORDING for {args.duration:.1f} s ...\n")

    try:
        while camera.IsGrabbing():
            now = time.time()
            if (now - start_time) >= args.duration:
                break

            grab_result = camera.RetrieveResult(
                5000, pylon.TimeoutHandling_ThrowException)

            if grab_result.GrabSucceeded():
                # --- timestamps AT capture, before any processing ---
                cap_now = time.time()
                pc_perf_counter_ns = time.perf_counter_ns()
                pc_wall_time_ns = time.time_ns()
                wall_time_ms = int(round(cap_now * 1000.0))
                elapsed_s = cap_now - start_time
                elapsed_time_ms = wall_time_ms - start_wall_ms

                image = converter.Convert(grab_result)
                frame = image.GetArray()

                frame_brightness = (float(np.mean(frame))
                                    if LOG_BRIGHTNESS else None)

                undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

                tags = detector.detect(
                    undistorted, estimate_tag_pose=True,
                    camera_params=[fx_u, fy_u, cx_u, cy_u],
                    tag_size=TAG_SIZE_M
                )

                visible_ids = []
                fused_yaw_deg = None
                fused_yaw_rate_deg_s = None
                used_ids = []
                used_count = 0
                tag_measurements = []
                tag_extra = {}

                for tag in tags:
                    tag_id = int(tag.tag_id)

                    if not hasattr(tag, "pose_R"):
                        continue

                    decision_margin = float(tag.decision_margin)

                    # ---- UNCHANGED yaw path -------------------
                    roll_abs, pitch_abs, yaw_wrapped = \
                        rotation_matrix_to_euler_xyz(tag.pose_R)
                    yaw_wrapped = -yaw_wrapped

                    prev_unwrapped = yaw_prev_unwrapped.get(tag_id, None)
                    yaw_unwrapped = unwrap_angle_deg(yaw_wrapped, prev_unwrapped)
                    yaw_prev_unwrapped[tag_id] = yaw_unwrapped

                    if tag_id not in yaw_zero_ref:
                        yaw_zero_ref[tag_id] = yaw_unwrapped

                    yaw_relative = yaw_unwrapped - yaw_zero_ref[tag_id]
                    visible_ids.append(tag_id)
                    tag_measurements.append(
                        (tag_id, yaw_relative, decision_margin))
                    # ---- end UNCHANGED yaw path ---------------

                    # ---- additive raw diagnostics -------------
                    try:
                        pose_err = float(getattr(tag, "pose_err", np.nan))
                    except Exception:
                        pose_err = float("nan")
                    try:
                        hamming = int(getattr(tag, "hamming", -1))
                    except Exception:
                        hamming = -1
                    try:
                        tz = float(np.asarray(tag.pose_t).ravel()[2])
                    except Exception:
                        tz = float("nan")

                    tag_extra[tag_id] = dict(
                        roll_abs=roll_abs,
                        pitch_abs=pitch_abs,
                        yaw_abs=yaw_unwrapped,
                        pose_err=pose_err,
                        hamming=hamming,
                        tz=tz,
                    )

                visible_count = len(tag_measurements)

                if visible_count > 0:
                    fused_yaw_deg, used_count, used_ids = \
                        fuse_tag_measurements(tag_measurements)
                    # No online filtering: USE_FILTER is fixed False for the
                    # whole campaign, so fused_yaw_deg is the raw fusion output.
                    if prev_elapsed is not None and prev_fused_yaw is not None:
                        dt = elapsed_s - prev_elapsed
                        if dt > 0:
                            fused_yaw_rate_deg_s = \
                                (fused_yaw_deg - prev_fused_yaw) / dt
                    prev_fused_yaw = fused_yaw_deg
                    prev_elapsed = elapsed_s
                else:
                    prev_fused_yaw = None
                    prev_elapsed = elapsed_s

                visible_ids_text = (",".join(map(str, visible_ids))
                                    if visible_ids else "N/A")
                used_ids_text = (",".join(map(str, used_ids))
                                 if used_ids else "N/A")

                row = {
                    "pc_wall_time_ns": pc_wall_time_ns,
                    "pc_perf_counter_ns": pc_perf_counter_ns,
                    "wall_time_ms": wall_time_ms,
                    "elapsed_time_ms": elapsed_time_ms,
                    "elapsed_time_s": elapsed_s,
                    "visible_count": visible_count,
                    "visible_ids": visible_ids_text,
                    "used_count_after_outlier_rejection": used_count,
                    "used_ids": used_ids_text,
                    "fused_yaw_deg": fused_yaw_deg,
                    "fused_yaw_rate_deg_s": fused_yaw_rate_deg_s,
                    "filter_enabled": int(USE_FILTER),
                }
                if LOG_BRIGHTNESS:
                    row["frame_brightness"] = frame_brightness

                for (tag_id, yaw_relative, decision_margin) in tag_measurements:
                    ex = tag_extra.get(tag_id, {})
                    row[f"tag_{tag_id}_yaw_deg"] = yaw_relative
                    row[f"tag_{tag_id}_decision_margin"] = decision_margin
                    row[f"tag_{tag_id}_roll_deg"] = ex.get("roll_abs")
                    row[f"tag_{tag_id}_pitch_deg"] = ex.get("pitch_abs")
                    row[f"tag_{tag_id}_yaw_abs_deg"] = ex.get("yaw_abs")
                    row[f"tag_{tag_id}_pose_err"] = ex.get("pose_err")
                    row[f"tag_{tag_id}_hamming"] = ex.get("hamming")
                    row[f"tag_{tag_id}_tz_m"] = ex.get("tz")

                rows.append(row)

                # ---- start-baseline tag completeness check ----
                if elapsed_s <= BASELINE_CHECK_S:
                    baseline_frames += 1
                    if sorted(visible_ids) != sorted(EXPECTED_TAG_IDS):
                        baseline_incomplete += 1
                elif not baseline_warned:
                    baseline_warned = True
                    if baseline_incomplete > 0:
                        pct = 100.0 * baseline_incomplete / max(baseline_frames, 1)
                        print("\n" + "!" * 66)
                        print(f"!! WARNING: during the {BASELINE_CHECK_S:.0f} s "
                              f"start baseline, {baseline_incomplete} of "
                              f"{baseline_frames} frames ({pct:.2f}%)")
                        print(f"!! did NOT show all expected tags "
                              f"{EXPECTED_TAG_IDS}.")
                        print("!! A tag first seen after motion starts gets a "
                              "wrong zero reference.")
                        print("!! Consider aborting and restarting this run.")
                        print("!" * 66 + "\n")
                    else:
                        print(f"  start baseline OK: all tags "
                              f"{EXPECTED_TAG_IDS} visible in "
                              f"{baseline_frames}/{baseline_frames} frames")

                # ---- lightweight 1 Hz liveness print ----------
                if elapsed_s - last_status >= STATUS_PERIOD_S:
                    n = len(rows)
                    if status_last_t > 0.0:
                        inst_fps = ((n - status_last_count)
                                    / (elapsed_s - status_last_t))
                    else:
                        inst_fps = n / elapsed_s if elapsed_s > 0 else 0.0
                    print(f"elapsed={elapsed_s:7.1f} s | "
                          f"visible=[{visible_ids_text}] | "
                          f"FPS={inst_fps:5.2f} | frames={n}")
                    last_status = elapsed_s
                    status_last_count = n
                    status_last_t = elapsed_s

            grab_result.Release()

    except KeyboardInterrupt:
        print("\nInterrupted by operator.")
    finally:
        if camera.IsGrabbing():
            camera.StopGrabbing()
        camera.Close()

    # --------------------------------------------------------
    # SAVE -- raw rows only, nothing filtered or removed
    # --------------------------------------------------------
    df = pd.DataFrame(rows)
    df.to_csv(CSV_FILE, index=False)
    print(f"\nSaved {CSV_FILE}  ({len(df)} rows, {len(df.columns)} columns)")

    if not args.no_xlsx:
        try:
            df.to_excel(XLSX_FILE, index=False)
            print(f"Saved {XLSX_FILE}")
        except Exception as e:
            print(f"XLSX write failed ({e}); CSV is the authoritative file.")

    real_fps = float("nan")
    if len(df) > 1:
        dur = df["elapsed_time_s"].iloc[-1] - df["elapsed_time_s"].iloc[0]
        if dur > 0:
            real_fps = (len(df) - 1) / dur
            print(f"Measured average capture rate: {real_fps:.2f} fps "
                  f"over {dur:.2f} s")

    meta = dict(
        script="basler_apriltag_multitag_FINAL.py",
        run=args.run, direction=args.dir,
        requested_duration_s=args.duration,
        n_frames=int(len(df)),
        measured_fps=real_fps,
        width=int(width), height=int(height),
        exposure_us=EXPOSURE_US, gain_db=GAIN_DB,
        tag_family=TAG_FAMILY, tag_size_m=TAG_SIZE_M,
        expected_tag_ids=EXPECTED_TAG_IDS,
        outlier_threshold_deg=OUTLIER_THRESHOLD_DEG,
        filter_enabled=int(USE_FILTER),
        save_video=SAVE_VIDEO, draw_overlays=DRAW_OVERLAYS,
        yaw_sign_negated_in_acquisition=True,
        roll_pitch_fused=False,
        camera_matrix=CAMERA_MATRIX.tolist(),
        dist_coeffs=DIST_COEFFS.tolist(),
        start_wall_time_ns=int(start_time * 1e9),
        start_perf_counter_ns=int(start_perf_ns),
        baseline_check_s=BASELINE_CHECK_S,
        baseline_frames=int(baseline_frames),
        baseline_frames_incomplete=int(baseline_incomplete),
        zero_reference_tag_ids=sorted(yaw_zero_ref.keys()),
        csv_file=os.path.basename(CSV_FILE),
    )
    with open(META_FILE, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"Saved {META_FILE}")

    if baseline_incomplete > 0:
        print("\n!! This run had incomplete tag visibility during the start "
              "baseline. Review before counting it as one of the ten.")

    print("\nDone.")


if __name__ == "__main__":
    main()
