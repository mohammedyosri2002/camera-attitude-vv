#!/usr/bin/env python3
"""
analysis/rate_audit.py
======================
THREE-AXIS FILTERED-RATE AUDIT. CREATED POST-CAMPAIGN.

    python analysis/rate_audit.py        (run after analysis/reproduce_all.py)

Applies ONE FROZEN Savitzky-Golay rate estimator identically to Yaw, Board X and Board Y,
over the SAME valid constant-rate leg windows already used for the RAW results, and puts
the three rate views side by side:

    RAW instantaneous derivative   plain finite difference, unsmoothed   -- PRIMARY
    SG processed rate estimate     frozen 1.00 s / order 2 / zero phase  -- SECONDARY
    displacement-based rate scale  total travel / elapsed time per leg   -- SCALE metric

NOTHING HERE REPLACES A RAW RESULT. The SG columns are labelled SECONDARY everywhere they
appear, are never called a RAW accuracy, and are never substituted into the RAW comparison.

The estimator is frozen at the established yaw parameters and is NOT tuned per axis and NOT
optimised to minimise any error. See analysis/common/rate_estimators.py.

Writes results/tables/three_axis_rate_comparison.csv and
results/three_axis_rate_audit.json.
"""
from __future__ import annotations

import csv
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common import io as cio                                  # noqa: E402
from common import rate_estimators as rest                    # noqa: E402
from common.attitude_metrics import metrics, pooled           # noqa: E402

SG_RMSE_QUESTION_THRESHOLD_DPS = 0.1     # the question asked of the audit


def yaw_sg_over_legs():
    """Apply the frozen SG estimator to the yaw runs over their constant-rate segments.

    The yaw acquisition pre-dates the rigid-board estimator, so the per-sample series comes
    from the synchronized samples exported by analysis/yaw/reproduce_yaw.py. The camera
    grid is used (the gyro grid is for IMU pairings), and only segment_type == 'rate' rows
    are taken, which are exactly the windows the RAW yaw result uses.
    """
    files = sorted(glob.glob(os.path.join(cio.RESULTS,
                                          "yaw_synchronized_samples_run*.csv")))
    if not files:
        return None
    raw, sg, disp, by_rate, n_win = [], [], [], {}, 0
    win_samples, fs_all = [], []
    for f in files:
        d = pd.read_csv(f)
        d = d[(d["grid"] == "camera") & (d["segment_type"] == "rate")]
        for sid, seg in d.groupby("segment_id"):
            seg = seg.sort_values("common_time_s")
            t = seg["common_time_s"].to_numpy(float)
            y = seg["camera_raw_yaw_deg"].to_numpy(float)
            r = float(seg["motor_rate_dps"].iloc[0])
            m = np.isfinite(t) & np.isfinite(y)
            if m.sum() < 50 or not np.isfinite(r) or abs(r) < 1e-9:
                continue
            t, y = t[m], y[m]
            der, n_sg, fs = rest.sg_rate(t, y)
            if der is None:
                continue
            win_samples.append(n_sg); fs_all.append(fs)
            e_raw = seg["camera_raw_rate_dps"].to_numpy(float)[m] - r
            e_sg = der - r
            raw.append(e_raw); sg.append(e_sg)
            dr = (y[-1] - y[0]) / (t[-1] - t[0])
            disp.append(100.0 * (dr - r) / r)
            by_rate.setdefault(abs(round(r, 3)), {"raw": [], "sg": [], "disp": []})
            by_rate[abs(round(r, 3))]["raw"].append(e_raw)
            by_rate[abs(round(r, 3))]["sg"].append(e_sg)
            by_rate[abs(round(r, 3))]["disp"].append(100.0 * (dr - r) / r)
            n_win += 1
    if not sg:
        return None
    return dict(axis="yaw", n_windows=n_win,
                sg_window_samples=int(np.median(win_samples)),
                sample_rate_hz=float(np.median(fs_all)),
                raw=pooled(raw), sg=pooled(sg),
                disp_pct=dict(min=float(min(disp)), max=float(max(disp))),
                per_rate={str(k): dict(raw=pooled(v["raw"]), sg=pooled(v["sg"]),
                                       disp_pct_min=float(min(v["disp"])),
                                       disp_pct_max=float(max(v["disp"])))
                          for k, v in sorted(by_rate.items())})


def out_of_plane_from_results(res, axis):
    """Assemble the three rate views for an out-of-plane axis from its reproduced result."""
    sgb = res["dynamic_sg_SECONDARY"]
    legs = res["tables"]["legs"]
    scale = res["tables"]["rate_scale"]
    by = {}
    for L, S in zip(legs, scale):
        k = abs(round(L["commanded_rate_dps"], 3))
        by.setdefault(k, {"raw": [], "sg": [], "disp": []})
        by[k]["raw"].append(L)
        by[k]["sg"].append(L)
        by[k]["disp"].append(S["error_pct"])

    def _pool(rows, prefix):
        n = sum(r[f"{prefix}N"] for r in rows)
        if n == 0:
            return metrics([])
        bias = sum(r[f"{prefix}Bias"] * r[f"{prefix}N"] for r in rows) / n
        mae = sum(r[f"{prefix}MAE"] * r[f"{prefix}N"] for r in rows) / n
        rmse = float(np.sqrt(sum((r[f"{prefix}RMSE"] ** 2) * r[f"{prefix}N"]
                                 for r in rows) / n))
        return dict(Bias=bias, MAE=mae, RMSE=rmse, STD=float("nan"),
                    MaxAbs=max(r[f"{prefix}MaxAbs"] for r in rows), N=int(n))

    return dict(axis=axis, n_windows=len(legs),
                sg_window_samples=int(sgb["sg_window_samples"]),
                sample_rate_hz=float(sgb["sample_rate_hz"]),
                raw=res["dynamic"]["camera_raw_rate_vs_commanded"],
                sg=sgb["camera_sg_rate_vs_commanded"],
                disp_pct=res["rate_scale_pct"],
                per_rate={str(k): dict(raw=_pool(v["raw"], "Camera_Motor_"),
                                       sg=_pool(v["sg"], "SG_Camera_Motor_"),
                                       disp_pct_min=float(min(v["disp"])),
                                       disp_pct_max=float(max(v["disp"])))
                          for k, v in sorted(by.items())})


def main():
    p = os.path.join(cio.RESULTS, "reproduced_numbers.json")
    if not os.path.exists(p):
        sys.exit("run `python analysis/reproduce_all.py` first")
    R = json.load(open(p))

    print("=" * 78)
    print("THREE-AXIS FILTERED-RATE AUDIT")
    print("=" * 78)
    e = rest.describe()
    print(f"  estimator : {e['estimator']}")
    print(f"  window    : {e['window_s']:.2f} s, polynomial order {e['polyorder']}, "
          f"first derivative")
    print(f"  frozen    : {e['frozen']}   tuned per axis: {e['tuned_per_axis']}   "
          f"optimised to minimise error: {e['optimised_to_minimise_error']}")
    print(f"  status    : {e['status']}")

    axes = {}
    y = yaw_sg_over_legs()
    if y:
        axes["yaw"] = y
    axes["board_x"] = out_of_plane_from_results(
        R["board_x"]["board_x/axis_b_run01"], "board_x")
    axes["board_y"] = out_of_plane_from_results(R["board_y"], "board_y")

    print(f"\n  {'axis':<9} {'windows':>8} {'SG win':>8} {'fs [Hz]':>9} | "
          f"{'RAW RMSE':>10} {'SG RMSE':>10} {'SG Bias':>9} {'SG MAE':>9} "
          f"{'SG MaxAbs':>10} {'SG N':>8} | {'disp scale %':>16}")
    for k, a in axes.items():
        print(f"  {k:<9} {a['n_windows']:8d} {a['sg_window_samples']:8d} "
              f"{a['sample_rate_hz']:9.2f} | {a['raw']['RMSE']:10.4f} "
              f"{a['sg']['RMSE']:10.4f} {a['sg']['Bias']:9.4f} {a['sg']['MAE']:9.4f} "
              f"{a['sg']['MaxAbs']:10.4f} {a['sg']['N']:8d} | "
              f"{a['disp_pct']['min']:+7.2f} to {a['disp_pct']['max']:+6.2f}")

    print(f"\n  IS SG RMSE BELOW {SG_RMSE_QUESTION_THRESHOLD_DPS} deg/s?")
    answer = {}
    for k, a in axes.items():
        below = bool(a["sg"]["RMSE"] < SG_RMSE_QUESTION_THRESHOLD_DPS)
        answer[k] = dict(sg_rmse_dps=a["sg"]["RMSE"], below_threshold=below)
        print(f"    {k:<9} SG RMSE = {a['sg']['RMSE']:.4f} deg/s   -> "
              f"{'YES' if below else 'NO'}")

    # ---- per-commanded-rate table -------------------------------------------
    T = os.path.join(cio.RESULTS, "tables")
    os.makedirs(T, exist_ok=True)
    out = os.path.join(T, "three_axis_rate_comparison.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["# ALL camera-vs-commanded-reference rate errors. The motor is a "
                    "PULSE-DERIVED COMMANDED REFERENCE, not ground truth."])
        w.writerow([f"# RAW_* : primary, plain finite difference, unsmoothed"])
        w.writerow([f"# SG_*  : SECONDARY DIAGNOSTIC ONLY, frozen Savitzky-Golay "
                    f"{e['window_s']:.2f} s / order {e['polyorder']} / zero phase, "
                    f"identical for all axes, not tuned per axis. NEVER a RAW accuracy."])
        w.writerow(["# DISPSCALE_* : rate-SCALE metric (total travel / elapsed time per "
                    "leg). NOT an instantaneous rate RMSE."])
        w.writerow(["# Axes are NOT comparable with each other: rate ranges, travel and "
                    "frame rates differ."])
        w.writerow(["axis", "commanded_rate_dps", "n_windows",
                    "RAW_Bias_dps", "RAW_MAE_dps", "RAW_RMSE_dps", "RAW_MaxAbs_dps",
                    "RAW_N",
                    "SG_SECONDARY_Bias_dps", "SG_SECONDARY_MAE_dps",
                    "SG_SECONDARY_RMSE_dps", "SG_SECONDARY_MaxAbs_dps",
                    "SG_SECONDARY_N",
                    "DISPSCALE_min_pct", "DISPSCALE_max_pct",
                    "sg_window_s", "sg_polyorder", "sg_window_samples",
                    "camera_fps_median_Hz"])
        for k, a in axes.items():
            for r, v in a["per_rate"].items():
                w.writerow([k, r, len(v["raw"]) if isinstance(v["raw"], list) else "",
                            f"{v['raw']['Bias']:.6f}", f"{v['raw']['MAE']:.6f}",
                            f"{v['raw']['RMSE']:.6f}", f"{v['raw']['MaxAbs']:.6f}",
                            v["raw"]["N"],
                            f"{v['sg']['Bias']:.6f}", f"{v['sg']['MAE']:.6f}",
                            f"{v['sg']['RMSE']:.6f}", f"{v['sg']['MaxAbs']:.6f}",
                            v["sg"]["N"],
                            f"{v['disp_pct_min']:.4f}", f"{v['disp_pct_max']:.4f}",
                            e["window_s"], e["polyorder"], a["sg_window_samples"],
                            f"{a['sample_rate_hz']:.2f}"])
            w.writerow([k, "ALL", a["n_windows"],
                        f"{a['raw']['Bias']:.6f}", f"{a['raw']['MAE']:.6f}",
                        f"{a['raw']['RMSE']:.6f}", f"{a['raw']['MaxAbs']:.6f}",
                        a["raw"]["N"],
                        f"{a['sg']['Bias']:.6f}", f"{a['sg']['MAE']:.6f}",
                        f"{a['sg']['RMSE']:.6f}", f"{a['sg']['MaxAbs']:.6f}",
                        a["sg"]["N"],
                        f"{a['disp_pct']['min']:.4f}", f"{a['disp_pct']['max']:.4f}",
                        e["window_s"], e["polyorder"], a["sg_window_samples"],
                        f"{a['sample_rate_hz']:.2f}"])
    print(f"\n  -> {os.path.relpath(out, cio.REPO_ROOT)}")

    rep = dict(estimator=e,
               threshold_question_dps=SG_RMSE_QUESTION_THRESHOLD_DPS,
               sg_rmse_below_threshold=answer,
               axes=axes,
               policy=("RAW finite-difference results are primary and unchanged. SG is a "
                       "SECONDARY DIAGNOSTIC and is never labelled a RAW accuracy, never "
                       "substituted into the RAW comparison, and never compared across "
                       "axes as a ranking."))
    q = os.path.join(cio.RESULTS, "three_axis_rate_audit.json")
    json.dump(rep, open(q, "w"), indent=2, default=float)
    print(f"  -> {os.path.relpath(q, cio.REPO_ROOT)}")
    return rep


if __name__ == "__main__":
    main()
