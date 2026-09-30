#!/usr/bin/env python3
"""
analysis/reproduce_all.py
=========================
ONE-COMMAND REPRODUCTION. CREATED POST-CAMPAIGN.

    python analysis/reproduce_all.py

Steps:
  1. locate the included raw data (repository-relative paths only)
  2. verify SHA-256 checksums against data/CHECKSUMS.sha256
  3. refit camera<->motor synchronisation from serial motor events only
  4. select the IMU record declared in each RUN_INFO.json, and re-derive that choice
     from the data as a cross-check
  5. re-solve camera pose from raw corner pixels (conditioning always; attitude where
     the acquisition geometry was wrong)
  6. static RAW metrics      7. dynamic RAW metrics
  8. displacement rate-scale 9. conditioning metrics
 10. write results/reproduced_numbers.json
 11. compare against results/verified_numbers.json
 12. write results/VERIFICATION_REPORT.md and the paper tables
 13. exit non-zero if any required headline number fails tolerance

Exit codes: 0 pass, 1 headline mismatch, 2 checksum/integrity failure.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common import io as cio                                   # noqa: E402
from common.out_of_plane import process                        # noqa: E402

# --------------------------------------------------------------------------------------
# TOLERANCES
# --------------------------------------------------------------------------------------
# Absolute tolerances on the headline numbers. These are set at the last printed digit of
# the submitted values (4 decimals for degrees and deg/s, so 5e-5 is half a unit in the
# last place). They are deliberately tight: this pipeline is deterministic given the same
# raw data, so anything larger would hide a real change. Counts must match exactly.
# Each tolerance is HALF A UNIT IN THE LAST PRINTED DIGIT of the corresponding submitted
# value. The pipeline is deterministic given the same raw data, so the only legitimate
# disagreement is the rounding already present in the published figure. A tolerance looser
# than the published precision would hide a real change; one tighter than it would fail on
# arithmetic that is actually identical.
TOL = {
    "deg":    5e-5,   # 4 dp, e.g. 0.2987 deg
    "dps":    5e-5,   # 4 dp, e.g. 1.3117 deg/s
    "pct":    5e-4,   # 4 dp, e.g. 89.3959 %
    "pct2":   5e-3,   # 2 dp, e.g. the rate-scale range -0.47 % .. +0.75 %
    "px":     5e-5,   # 4 dp, e.g. 0.3676 px
    "ratio3": 5e-4,   # 3 dp, e.g. reprojection ratio 7.517
    "deg3":   5e-4,   # 3 dp, e.g. candidate separation 24.139 deg
    "axis":   5e-6,   # 5 dp direction cosines
    "count":  0,      # frame and sample counts: exact
}


def sha256(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for c in iter(lambda: f.read(chunk), b""):
            h.update(c)
    return h.hexdigest()


def verify_checksums():
    p = os.path.join(cio.REPO_ROOT, "data", "CHECKSUMS.sha256")
    if not os.path.exists(p):
        return None, ["data/CHECKSUMS.sha256 not found"]
    bad, n = [], 0
    for line in open(p):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        digest, rel = line.split(None, 1)
        rel = rel.lstrip("*").strip()
        full = os.path.join(cio.REPO_ROOT, rel)
        if not os.path.exists(full):
            bad.append(f"missing: {rel}")
            continue
        if sha256(full) != digest:
            bad.append(f"checksum mismatch: {rel}")
        n += 1
    return n, bad


def get(d, *path, default=None):
    for k in path:
        if d is None:
            return default
        d = d.get(k) if isinstance(d, dict) else None
    return default if d is None else d


def compare(name, got, exp, kind):
    if exp is None:
        return dict(metric=name, expected=None, reproduced=got, status="NOT-GATED")
    if got is None:
        return dict(metric=name, expected=exp, reproduced=None, delta=None, status="FAIL")
    tol = TOL[kind]
    delta = float(got) - float(exp)
    ok = abs(delta) <= tol if kind != "count" else int(got) == int(exp)
    return dict(metric=name, expected=exp, reproduced=got, delta=delta,
                tolerance=tol, status="PASS" if ok else "FAIL")


def main():
    ap = argparse.ArgumentParser(description="Reproduce every headline result.")
    ap.add_argument("--skip-checksums", action="store_true")
    ap.add_argument("--skip-yaw", action="store_true",
                    help="skip the yaw module (it is the slowest step)")
    a = ap.parse_args()

    t_start = time.time()
    print("=" * 78)
    print("REPRODUCING IEEE Aerospace paper 2389 headline results")
    print(f"repository root: {cio.REPO_ROOT}")
    print("=" * 78)

    # ---- 2. checksums --------------------------------------------------------
    if a.skip_checksums:
        print("\n[1/6] checksums SKIPPED by request")
        ck = (None, [])
    else:
        print("\n[1/6] verifying raw-data checksums ...")
        n, bad = verify_checksums()
        ck = (n, bad)
        if bad:
            for b in bad[:20]:
                print("   " + b)
            print(f"   {len(bad)} problem(s)")
            sys.exit(2)
        print(f"   {n} files verified")

    # ---- 3-9. per-axis reproduction ------------------------------------------
    print("\n[2/6] Board Y (RUN05) — reprocessed from raw corner pixels ...")
    by = process("board_y/run05")

    print("\n[3/6] Board X (axis_b_run01 headline, axis_a and run04 replication) ...")
    bx = {k: process(k) for k in ("board_x/axis_b_run01", "board_x/axis_a",
                                 "board_x/axis_b_run04")}

    yaw = None
    if not a.skip_yaw:
        print("\n[4/6] Yaw (in-plane, CW + CCW pooled) ...")
        sys.path.insert(0, os.path.join(cio.REPO_ROOT, "analysis", "yaw"))
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "repro_yaw", os.path.join(cio.REPO_ROOT, "analysis", "yaw",
                                      "reproduce_yaw.py"))
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        yaw = m.main()
    else:
        print("\n[4/6] Yaw SKIPPED by request")
        p = os.path.join(cio.RESULTS, "yaw.json")
        if os.path.exists(p):
            yaw = json.load(open(p))

    # ---- 10. reproduced_numbers.json -----------------------------------------
    print("\n[5/6] assembling reproduced_numbers.json ...")
    repro = dict(
        generated_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        reference_terminology=("Motor = PULSE-DERIVED COMMANDED REFERENCE. "
                               "Not ground truth, not an actual angle, not encoder-verified."),
        yaw=yaw, board_x=bx, board_y=by)
    os.makedirs(cio.RESULTS, exist_ok=True)
    json.dump(repro, open(os.path.join(cio.RESULTS, "reproduced_numbers.json"), "w"),
              indent=2, default=float)

    # ---- 11. gate ------------------------------------------------------------
    print("\n[6/6] comparing against results/verified_numbers.json ...")
    vp = os.path.join(cio.RESULTS, "verified_numbers.json")
    V = json.load(open(vp))
    bx1 = bx["board_x/axis_b_run01"]
    rows = []

    def C(n, g, e, k):
        rows.append(compare(n, g, e, k))

    # --- yaw ---
    if yaw:
        ys = get(yaw, "static", "camera_vs_commanded")
        yd = get(yaw, "dynamic", "camera_raw_rate_vs_commanded")
        C("yaw static camera-commanded RMSE (deg)", get(ys, "RMSE"),
          get(V, "yaw", "static_camera_vs_commanded", "RMSE"), "deg")
        C("yaw static camera-commanded MAE (deg)", get(ys, "MAE"),
          get(V, "yaw", "static_camera_vs_commanded", "MAE"), "deg")
        C("yaw static camera-commanded MaxAbs (deg)", get(ys, "MaxAbs"),
          get(V, "yaw", "static_camera_vs_commanded", "MaxAbs"), "deg")
        C("yaw dynamic camera RAW rate RMSE (deg/s)", get(yd, "RMSE"),
          get(V, "yaw", "dynamic_camera_raw_vs_commanded", "RMSE"), "dps")
        C("yaw dynamic camera RAW rate MAE (deg/s)", get(yd, "MAE"),
          get(V, "yaw", "dynamic_camera_raw_vs_commanded", "MAE"), "dps")

    # --- board X headline ---
    for lbl, path, kind in (
            ("RMSE", ("static", "camera_vs_commanded", "RMSE"), "deg"),
            ("MAE", ("static", "camera_vs_commanded", "MAE"), "deg"),
            ("Bias", ("static", "camera_vs_commanded", "Bias"), "deg"),
            ("MaxAbs", ("static", "camera_vs_commanded", "MaxAbs"), "deg")):
        C(f"board_x static camera-commanded {lbl} (deg)", get(bx1, *path),
          get(V, "board_x_headline", "static_camera_vs_commanded", lbl), kind)
    C("board_x static camera-gravity RMSE (deg)",
      get(bx1, "static", "camera_vs_gravity", "RMSE"),
      get(V, "board_x_headline", "static_camera_vs_gravity", "RMSE"), "deg")
    C("board_x dynamic camera RAW rate RMSE (deg/s)",
      get(bx1, "dynamic", "camera_raw_rate_vs_commanded", "RMSE"),
      get(V, "board_x_headline", "dynamic_camera_raw_vs_commanded", "RMSE"), "dps")
    C("board_x dynamic gyro RMSE (deg/s)",
      get(bx1, "dynamic", "gyro_vs_commanded", "RMSE"),
      get(V, "board_x_headline", "dynamic_gyro_vs_commanded", "RMSE"), "dps")

    # --- board Y ---
    for grp, vkey in (("camera_vs_commanded", "static_camera_vs_commanded"),
                      ("gravity_vs_commanded", "static_gravity_vs_commanded"),
                      ("camera_vs_gravity", "static_camera_vs_gravity")):
        for lbl, kind in (("Bias", "deg"), ("MAE", "deg"), ("RMSE", "deg"),
                          ("MaxAbs", "deg"), ("N", "count")):
            e = get(V, "board_y", vkey, lbl)
            if e is not None:
                C(f"board_y static {grp} {lbl}", get(by, "static", grp, lbl), e, kind)
    for grp, vkey in (("camera_raw_rate_vs_commanded", "dynamic_camera_raw_vs_commanded"),
                      ("gyro_vs_commanded", "dynamic_gyro_vs_commanded"),
                      ("camera_raw_rate_vs_gyro", "dynamic_camera_raw_vs_gyro")):
        for lbl, kind in (("Bias", "dps"), ("MAE", "dps"), ("RMSE", "dps"),
                          ("MaxAbs", "dps"), ("N", "count")):
            e = get(V, "board_y", vkey, lbl)
            if e is not None:
                C(f"board_y dynamic {grp} {lbl}", get(by, "dynamic", grp, lbl), e, kind)
    C("board_y rate-scale min (%)", get(by, "rate_scale_pct", "min"),
      get(V, "board_y", "rate_scale_pct", "min"), "pct2")
    C("board_y rate-scale max (%)", get(by, "rate_scale_pct", "max"),
      get(V, "board_y", "rate_scale_pct", "max"), "pct2")
    for lbl, path, kind in (
            ("n_frames", ("conditioning", "n_frames"), "count"),
            ("valid_pose_pct", ("conditioning", "valid_pose_pct"), "pct"),
            ("branch_flips", ("conditioning", "branch_flips"), "count"),
            ("ambiguous_frames", ("conditioning", "ambiguous_frames"), "count"),
            ("four_tag_visibility_pct", ("conditioning", "four_tag_visibility_pct"), "pct"),
            ("reprojection_median_px", ("conditioning", "board_reprojection_px", "median"), "px"),
            ("reprojection_mean_px", ("conditioning", "board_reprojection_px", "mean"), "px"),
            ("candidate_separation_min_deg", ("conditioning", "candidate_separation_deg", "min"), "deg3"),
            ("candidate_separation_max_deg", ("conditioning", "candidate_separation_deg", "max"), "deg3"),
            ("reprojection_ratio_min", ("conditioning", "reprojection_ratio", "min"), "ratio3"),
            ("reprojection_ratio_max", ("conditioning", "reprojection_ratio", "max"), "ratio3")):
        C(f"board_y {lbl}", get(by, *path), get(V, "board_y", "conditioning", lbl), kind)
    for i, comp in enumerate("xyz"):
        C(f"board_y hinge axis {comp}", by["hinge_axis_board_frame"][i],
          get(V, "board_y", "hinge_axis_board_frame", default=[None] * 3)[i], "axis")

    # --- replication ---
    for key, vk in (("board_x/axis_a", "axis_a"), ("board_x/axis_b_run04", "run04")):
        C(f"replication {vk} camera-commanded RMSE (deg)",
          get(bx[key], "static", "camera_vs_commanded", "RMSE"),
          get(V, "replication", vk, "static_camera_vs_commanded_RMSE"), "deg")
        C(f"replication {vk} camera-gravity RMSE (deg)",
          get(bx[key], "static", "camera_vs_gravity", "RMSE"),
          get(V, "replication", vk, "static_camera_vs_gravity_RMSE"), "deg")

    # SECONDARY: three-axis filtered-rate audit. Never alters a RAW result.
    try:
        import importlib.util as _il
        _sp = _il.spec_from_file_location(
            "rate_audit", os.path.join(cio.REPO_ROOT, "analysis", "rate_audit.py"))
        _m = _il.module_from_spec(_sp)
        _sp.loader.exec_module(_m)
        print("\n[6b/6] three-axis filtered-rate audit (SECONDARY) ...")
        _m.main()
    except Exception as _e:
        print(f"   [note] rate audit skipped: {_e}")

    gated = [r for r in rows if r["status"] != "NOT-GATED"]
    failed = [r for r in gated if r["status"] == "FAIL"]
    write_report(rows, ck, repro, failed, time.time() - t_start)
    write_tables(yaw, bx, by)

    print("\n" + "=" * 78)
    for r in gated:
        if r["status"] == "FAIL":
            print(f"  FAIL  {r['metric']}: expected {r['expected']}, "
                  f"reproduced {r['reproduced']}, delta {r['delta']}")
    print(f"  {len(gated) - len(failed)}/{len(gated)} gated values PASS")
    print("=" * 78)
    if failed:
        print("REPRODUCTION FAILED — see results/VERIFICATION_REPORT.md")
        sys.exit(1)
    print("REPRODUCTION PASSED")
    print(f"elapsed {time.time() - t_start:.1f} s")
    sys.exit(0)


def write_report(rows, ck, repro, failed, elapsed):
    p = os.path.join(cio.RESULTS, "VERIFICATION_REPORT.md")
    # SECONDARY: three-axis filtered-rate audit. Never alters a RAW result.
    try:
        import importlib.util as _il
        _sp = _il.spec_from_file_location(
            "rate_audit", os.path.join(cio.REPO_ROOT, "analysis", "rate_audit.py"))
        _m = _il.module_from_spec(_sp)
        _sp.loader.exec_module(_m)
        print("\n[6b/6] three-axis filtered-rate audit (SECONDARY) ...")
        _m.main()
    except Exception as _e:
        print(f"   [note] rate audit skipped: {_e}")

    gated = [r for r in rows if r["status"] != "NOT-GATED"]
    L = ["# Verification Report",
         "",
         f"Generated: {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}  ",
         f"Elapsed: {elapsed:.1f} s  ",
         f"Result: **{'FAIL' if failed else 'PASS'}** "
         f"({len(gated) - len(failed)}/{len(gated)} gated values within tolerance)",
         "",
         "Produced by `python analysis/reproduce_all.py` from the raw data in `data/raw/`.",
         "The motor signal is a **pulse-derived commanded reference** throughout — not",
         "ground truth, not an actual angle, not encoder-verified.",
         "",
         "## Checksums", ""]
    L += [f"- {ck[0]} raw-data files verified against `data/CHECKSUMS.sha256`"
          if ck[0] else "- checksum verification skipped", ""]
    L += ["## Tolerances", "",
          "| Quantity | Absolute tolerance |", "|---|---|",
          f"| static error, 4 dp (deg) | {TOL['deg']} |",
          f"| rate error, 4 dp (deg/s) | {TOL['dps']} |",
          f"| percentage, 4 dp | {TOL['pct']} |",
          f"| percentage, 2 dp (rate-scale range) | {TOL['pct2']} |",
          f"| reprojection, 4 dp (px) | {TOL['px']} |",
          f"| reprojection ratio, 3 dp | {TOL['ratio3']} |",
          f"| candidate separation, 3 dp (deg) | {TOL['deg3']} |",
          f"| axis direction cosine, 5 dp | {TOL['axis']} |",
          "| frame and sample counts | exact |", "",
          "Each tolerance is half a unit in the last printed digit of the corresponding",
          "submitted value. The pipeline is deterministic given the same raw data, so the",
          "only legitimate disagreement is the rounding already present in the published",
          "figure. A looser tolerance would hide a real change.", "",
          "## Gated values", "",
          "| Metric | Expected | Reproduced | Delta | Status |", "|---|---|---|---|---|"]
    for r in gated:
        d = "" if r.get("delta") is None else f"{r['delta']:+.2e}"
        L.append(f"| {r['metric']} | {r['expected']} | {r['reproduced']} | {d} | "
                 f"{r['status']} |")
    L += ["", "## Methodological note: two pose sources", "",
          "Board-X runs were acquired with their correct board layout, so their logged",
          "relative attitude is authoritative and is used as-is (`pose_source:",
          "acquisition_logged`). RUN05 was acquired with the stale Board-X layout, so its",
          "logged pose columns are invalid and the pose is re-solved for every frame from",
          "stored raw corner pixels (`pose_source: reprocessed_from_raw_corners`).",
          "",
          "This distinction is not cosmetic. Re-solving a Board-X run shifts its",
          "camera-vs-commanded RMSE by about 0.10 deg, because the acquisition established",
          "its zero reference from the first frames of the CAPTURE, which precede the motor",
          "baseline window used here. Reprocessing correctly-processed data would therefore",
          "silently change every Board-X number. Each run's `RUN_INFO.json` records which",
          "source is authoritative and why.",
          "",
          "Conditioning diagnostics (validity, flips, ambiguity, reprojection, candidate",
          "separation) are ALWAYS recomputed from raw corner pixels, for every run.",
          "",
          "## Scalar attitude convention", "",
          "Board X uses the active relative Euler component; Board Y uses the projection of",
          "the relative rotation vector onto the calibrated hinge axis. Both are reported,",
          "and `static_alternative_scalar` in the per-run JSON gives the same data under the",
          "other convention. On Board X the two differ by under 0.003 deg RMSE, so the",
          "choice is not load-bearing.", ""]
    if failed:
        L += ["## Failures", ""]
        for r in failed:
            L.append(f"- **{r['metric']}**: expected `{r['expected']}`, reproduced "
                     f"`{r['reproduced']}`, delta `{r['delta']}`")
        L.append("")
    open(p, "w").write("\n".join(L))
    print(f"   -> {os.path.relpath(p, cio.REPO_ROOT)}")


def write_tables(yaw, bx, by):
    import csv
    T = os.path.join(cio.RESULTS, "tables")
    os.makedirs(T, exist_ok=True)
    bx1 = bx["board_x/axis_b_run01"]

    def w(name, header, rows):
        with open(os.path.join(T, name), "w", newline="") as f:
            c = csv.writer(f)
            c.writerow(header)
            c.writerows(rows)

    ys = get(yaw, "static", "camera_vs_commanded") if yaw else None
    w("three_axis_static.csv",
      ["metric", "yaw_in_plane", "board_x", "board_y"],
      [["camera_vs_commanded_Bias_deg", get(ys, "Bias"),
        bx1["static"]["camera_vs_commanded"]["Bias"], by["static"]["camera_vs_commanded"]["Bias"]],
       ["camera_vs_commanded_MAE_deg", get(ys, "MAE"),
        bx1["static"]["camera_vs_commanded"]["MAE"], by["static"]["camera_vs_commanded"]["MAE"]],
       ["camera_vs_commanded_RMSE_deg", get(ys, "RMSE"),
        bx1["static"]["camera_vs_commanded"]["RMSE"], by["static"]["camera_vs_commanded"]["RMSE"]],
       ["camera_vs_commanded_MaxAbs_deg", get(ys, "MaxAbs"),
        bx1["static"]["camera_vs_commanded"]["MaxAbs"], by["static"]["camera_vs_commanded"]["MaxAbs"]],
       ["camera_vs_commanded_N", get(ys, "N"),
        bx1["static"]["camera_vs_commanded"]["N"], by["static"]["camera_vs_commanded"]["N"]],
       ["gravity_vs_commanded_RMSE_deg", "N/A",
        bx1["static"]["gravity_vs_commanded"]["RMSE"], by["static"]["gravity_vs_commanded"]["RMSE"]],
       ["camera_vs_gravity_RMSE_deg", "N/A",
        bx1["static"]["camera_vs_gravity"]["RMSE"], by["static"]["camera_vs_gravity"]["RMSE"]],
       ["camera_vs_gravity_Bias_deg", "N/A",
        bx1["static"]["camera_vs_gravity"]["Bias"], by["static"]["camera_vs_gravity"]["Bias"]],
       ["gyro_channel", "Z", "Y", "X"]])

    yd = get(yaw, "dynamic") if yaw else None
    w("three_axis_raw_rate.csv",
      ["metric", "yaw_in_plane", "board_x", "board_y"],
      [["commanded_rate_range_dps", "1-30", "1-10", "1-10"],
       ["camera_raw_rate_vs_commanded_MAE_dps",
        get(yd, "camera_raw_rate_vs_commanded", "MAE"),
        bx1["dynamic"]["camera_raw_rate_vs_commanded"]["MAE"],
        by["dynamic"]["camera_raw_rate_vs_commanded"]["MAE"]],
       ["camera_raw_rate_vs_commanded_RMSE_dps",
        get(yd, "camera_raw_rate_vs_commanded", "RMSE"),
        bx1["dynamic"]["camera_raw_rate_vs_commanded"]["RMSE"],
        by["dynamic"]["camera_raw_rate_vs_commanded"]["RMSE"]],
       ["camera_raw_rate_vs_commanded_Bias_dps",
        get(yd, "camera_raw_rate_vs_commanded", "Bias"),
        bx1["dynamic"]["camera_raw_rate_vs_commanded"]["Bias"],
        by["dynamic"]["camera_raw_rate_vs_commanded"]["Bias"]],
       ["gyro_vs_commanded_RMSE_dps", get(yd, "gyro_vs_commanded", "RMSE"),
        bx1["dynamic"]["gyro_vs_commanded"]["RMSE"],
        by["dynamic"]["gyro_vs_commanded"]["RMSE"]],
       ["camera_raw_rate_vs_gyro_RMSE_dps", get(yd, "camera_raw_rate_vs_gyro", "RMSE"),
        bx1["dynamic"]["camera_raw_rate_vs_gyro"]["RMSE"],
        by["dynamic"]["camera_raw_rate_vs_gyro"]["RMSE"]],
       ["camera_fps_median_Hz", "82.7 / 95.2",
        bx1["conditioning"]["camera_fps_median"], by["conditioning"]["camera_fps_median"]],
       ["NOTE", "columns are NOT comparable: different rate ranges, travel and frame rates",
        "", ""]])

    w("board_y_rate_scale.csv",
      ["leg", "commanded_rate_dps", "direction", "camera_displacement_rate_dps",
       "error_dps", "error_pct"],
      [[r["leg"], r["commanded_rate_dps"], r["direction"],
        r["camera_displacement_rate_dps"], r["error_dps"], r["error_pct"]]
       for r in by["tables"]["rate_scale"]])

    sync = []
    for k, r in list(bx.items()) + [("board_y/run05", by)]:
        s = r["sync"]
        sync.append([k, r["original_run_label"], s["camera_motor"]["a"],
                     s["camera_motor"]["ppm"], s["camera_motor"]["residual_sd_ms"],
                     s["camera_motor"]["n_events"], s["imu_motor"]["a"],
                     s["imu_motor"]["ppm"], s["imu_motor"]["residual_dps_rms"],
                     s["imu_motor"]["coverage_pct"]])
    if yaw:
        for rec in yaw["sync"]:
            sync.append([f"yaw/{rec.get('Run')}", rec.get("Direction"), "",
                         rec.get("Motor_ArduinoPC_ppm"),
                         rec.get("Motor_ArduinoPC_Residual_SD_ms"),
                         rec.get("Motor_Sync_Event_Count"), "",
                         rec.get("IMU_Motor_ppm"),
                         rec.get("IMU_Motor_Residual_dps_RMS"), ""])
    w("synchronization_summary.csv",
      ["run", "original_label", "camera_motor_a", "camera_motor_ppm",
       "camera_motor_residual_sd_ms", "motor_events", "imu_motor_a", "imu_motor_ppm",
       "imu_motor_residual_dps", "imu_coverage_pct"], sync)

    w("replication_runs.csv",
      ["run", "original_label", "camera_fps_median_Hz", "gyro_channel",
       "camera_vs_commanded_RMSE_deg", "camera_vs_gravity_RMSE_deg",
       "camera_raw_rate_RMSE_dps", "gyro_RMSE_dps"],
      [[k, r["original_run_label"], r["conditioning"]["camera_fps_median"],
        r["gyro_reference"]["axis"] and cio.run_info(k)["gyro_channel"],
        r["static"]["camera_vs_commanded"]["RMSE"],
        r["static"]["camera_vs_gravity"]["RMSE"],
        r["dynamic"]["camera_raw_rate_vs_commanded"]["RMSE"],
        r["dynamic"]["gyro_vs_commanded"]["RMSE"]]
       for k, r in bx.items()])

    w("conditioning_summary.csv",
      ["run", "n_frames", "valid_pose_pct", "branch_flips", "ambiguous_frames",
       "four_tag_visibility_pct", "reproj_median_px", "reproj_mean_px", "reproj_p95_px",
       "cand_sep_min_deg", "cand_sep_max_deg", "reproj_ratio_min", "reproj_ratio_max",
       "board_tilt_min_deg", "board_tilt_max_deg"],
      [[k, c["n_frames"], c["valid_pose_pct"], c["branch_flips"], c["ambiguous_frames"],
        c["four_tag_visibility_pct"], c["board_reprojection_px"]["median"],
        c["board_reprojection_px"]["mean"], c["board_reprojection_px"]["p95"],
        c["candidate_separation_deg"]["min"], c["candidate_separation_deg"]["max"],
        c["reprojection_ratio"]["min"], c["reprojection_ratio"]["max"],
        c["board_tilt_from_fronto_parallel_deg"]["min"],
        c["board_tilt_from_fronto_parallel_deg"]["max"]]
       for k, c in [(k, r["conditioning"]) for k, r in
                    list(bx.items()) + [("board_y/run05", by)]]])

    w("zero_return_drift.csv",
      ["run", "t_motor_s", "camera_deg", "gravity_tilt_deg", "difference_deg"],
      [[k, z["t_motor_s"], z["camera_deg"], z["gravity_tilt_deg"], z["difference_deg"]]
       for k, r in list(bx.items()) + [("board_y/run05", by)] for z in r["zero_return"]])

    print(f"   -> {os.path.relpath(T, cio.REPO_ROOT)}/ (7 CSV tables)")


if __name__ == "__main__":
    main()
