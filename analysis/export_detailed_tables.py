#!/usr/bin/env python3
"""
analysis/export_detailed_tables.py
==================================
FULL PAIRWISE RESULT TABLES. CREATED POST-CAMPAIGN.

    python analysis/export_detailed_tables.py     (run after analysis/reproduce_all.py)

Exposes the complete pairwise statistics behind the pooled headline values. The exporter
uses the frozen pipeline outputs and synchronized samples without changing any segmentation,
gating, synchronization, reference definition, or estimator setting. It may re-form residual
arrays from those frozen synchronized samples for tabulation, but it does not change the
headline analysis definitions.

TERMINOLOGY. The motor quantity is the PULSE-DERIVED COMMANDED REFERENCE. It is never
ground truth, never an actual angle, never a measured motor angle, and no independent
encoder-position telemetry was logged for this campaign.

THREE RATE CONCEPTS ARE KEPT IN SEPARATE FILES AND NEVER MIXED
  RAW instantaneous      dynamic_per_leg_*.csv, dynamic_pooled_by_rate_*.csv
  SECONDARY SG           dynamic_sg_SECONDARY_*.csv
  displacement scale     dynamic_displacement_rate_scale_by_rate.csv

SAMPLE-LEVEL POOLING. Pooled rows use the exact N-weighted identities, which are
algebraically identical to recomputing from the concatenated residual samples:

    Bias   = sum(Bias_i * N_i) / sum(N_i)
    MAE    = sum(MAE_i  * N_i) / sum(N_i)
    RMSE   = sqrt( sum(RMSE_i^2 * N_i) / sum(N_i) )
    MaxAbs = max(MaxAbs_i)

Per-command and per-rate RMSE values are never averaged to form a total.

PER-HOLD MAE AND RMSE ARE OMITTED BY DESIGN. Within one static dwell the residual rarely
changes sign, so MAE is identically |Bias| and RMSE is identically sqrt(Bias^2 + SD^2);
both were verified to machine precision on the frozen data. The independent per-hold
quantities are Bias, SD, MaxAbs and N. MAE and RMSE reappear on the pooled-by-command
tables, where repeated visits and sign changes make them informative.

EVALUATION GRIDS DIFFER BY COMPARISON AND ARE DECLARED IN EVERY TABLE
  static  camera_commanded   camera grid
  static  gravity_commanded  IMU grid (stationary-gated)
  static  camera_gravity     camera grid, raw gravity tilt interpolated onto camera times
  dynamic camera_commanded   camera grid
  dynamic gyro_commanded     IMU grid (the gyro is never resampled)
  dynamic camera_gyro        IMU grid, raw camera rate interpolated onto gyro times
N is therefore reported PER COMPARISON, never as one generic count.
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

from common import io as cio                              # noqa: E402
from common import rate_estimators as rest                # noqa: E402

T = os.path.join(cio.RESULTS, "tables")

GRID = {
    ("STATIC", "camera_commanded"): "camera",
    ("STATIC", "gravity_commanded"): "imu",
    ("STATIC", "camera_gravity"): "camera (gravity tilt interpolated onto camera times)",
    ("RAW_DYNAMIC", "camera_commanded"): "camera",
    ("RAW_DYNAMIC", "gyro_commanded"): "imu",
    ("RAW_DYNAMIC", "camera_gyro"): "imu (camera rate interpolated onto gyro times)",
    ("SG_SECONDARY", "camera_commanded"): "camera",
    ("DISPLACEMENT_SCALE", "camera_commanded"): "camera",
}

HDR_REF = ("# Motor quantity = PULSE-DERIVED COMMANDED REFERENCE. Not ground truth, not an "
           "actual angle, not a measured motor angle. No encoder-position telemetry was "
           "logged.")
HDR_GRID = ("# N is PER COMPARISON. Evaluation grids differ by comparison; see the "
            "evaluation_grid columns and README 'Detailed Result Tables'.")
HDR_MAXABS = ("# RAW-rate MaxAbs is a sample extreme and is not interpreted as a "
              "deterministic error bound; it scales with sample count.")
HDR_SG = ("# SECONDARY bandwidth-limited processed-rate diagnostic. Savitzky-Golay; "
          "uniform-time resampling; 1.00 s window; polynomial order 2; first derivative; "
          "zero-phase offline. Same frozen estimator for all axes. Not tuned per axis. "
          "Not optimized against an error metric. NOT RAW camera-rate accuracy and never "
          "a substitute for the RAW tables.")
HDR_SCALE = ("# Displacement-over-leg tracking metric: total camera angular travel divided "
             "by elapsed time. This is a rate SCALE metric and is NOT instantaneous rate "
             "accuracy.")
HDR_NOBIAS = ("# Signed bias is reported direction-separated only. No pooled signed-bias "
              "column exists, because pooling opposite directions cancels it.")

LONG = []          # tidy long-form accumulator


# Registry of everything written, used to build the Markdown copies and
# results/tables/README.md. Notes that used to be written as "#" lines inside the CSV
# now live here and are rendered into the README and above each Markdown table, so that
# every .csv under results/tables/ is PURE CSV whose FIRST LINE is the column header and
# renders as a table on GitHub.
REGISTRY = []

# Markdown column abbreviations. Values are never altered, only the labels.
ABBREV = [
    ("commanded_rate_magnitude_dps", "Cmd rate |w| [deg/s]"),
    ("commanded_reference_deg", "Cmd ref [deg]"),
    ("camera_displacement_rate_dps", "Cam disp rate [deg/s]"),
    ("scale_error_percent", "Scale err [%]"),
    ("scale_error_dps", "Scale err [deg/s]"),
    ("camera_mean_rate_dps", "Cam mean rate [deg/s]"),
    ("gyro_mean_rate_dps", "Gyro mean rate [deg/s]"),
    ("gravity_tilt_mean_deg", "Grav mean [deg]"),
    ("camera_mean_deg", "Cam mean [deg]"),
    ("commanded_rate_dps", "Cmd rate [deg/s]"),
    ("approach_direction", "Approach"),
    ("commanded_pulses", "Pulses"),
    ("leg_duration_s", "Leg [s]"),
    ("tags_visible", "Tags"),
    ("hold_index", "Hold"),
    ("leg_index", "Leg"),
    ("n_visits", "Visits"),
    ("n_legs_pos", "Legs +"),
    ("n_legs_neg", "Legs -"),
    ("n_legs_CW", "Legs CW"),
    ("n_legs_CCW", "Legs CCW"),
    ("camera_fps_median_Hz", "fps [Hz]"),
    ("sg_window_samples", "SG win [smp]"),
    ("sg_polyorder", "SG order"),
    ("sg_window_s", "SG win [s]"),
    ("SG_Camera_Cmd", "SG Cam-Cmd"),
    ("Camera_Gravity", "Cam-Grav"),
    ("Gravity_Cmd", "Grav-Cmd"),
    ("Camera_Gyro", "Cam-Gyro"),
    ("Camera_Cmd", "Cam-Cmd"),
    ("Gyro_Cmd", "Gyro-Cmd"),
    ("direction", "Dir"),
    ("_MaxAbs", " MaxAbs"), ("_RMSE", " RMSE"), ("_MAE", " MAE"),
    ("_Bias_Pos", " Bias+"), ("_Bias_Neg", " Bias-"),
    ("_Bias_CW", " Bias CW"), ("_Bias_CCW", " Bias CCW"),
    ("_Bias", " Bias"), ("_SD", " SD"), ("_N", " N"),
]


def md_label(col):
    out = col
    for a, b in ABBREV:
        out = out.replace(a, b)
    return out.replace("_", " ").strip()


PURPOSE = {
    "static_per_hold_": ("Static per hold", "One row per static dwell in acquisition "
                         "order, with approach direction. Bias / SD / MaxAbs / N per "
                         "comparison; MAE and RMSE omitted as redundant within a dwell."),
    "static_pooled_by_command_": ("Static pooled by commanded angle", "Residual samples "
                                  "pooled at each effective commanded angle, plus "
                                  "POOLED_TOTAL."),
    "dynamic_per_leg_": ("Dynamic RAW per leg", "One row per constant-rate leg, direction "
                         "preserved. RAW instantaneous finite-difference derivative."),
    "dynamic_pooled_by_rate_": ("Dynamic RAW pooled by rate magnitude", "RAW MAE / RMSE / "
                                "MaxAbs pooled by rate magnitude; signed bias kept "
                                "direction-separated."),
    "dynamic_sg_SECONDARY_": ("SECONDARY Savitzky-Golay processed rate", "SECONDARY "
                              "bandwidth-limited diagnostic. NOT RAW accuracy."),
    "dynamic_displacement_rate_scale_by_rate": ("Displacement rate scale", "Total camera "
                                                "angular travel over elapsed time per leg. "
                                                "A rate SCALE metric, not instantaneous "
                                                "rate accuracy."),
    "result_metrics_long": ("Tidy long-form companion", "Every metric as one row. "
                            "Machine-readable only."),
}
AXIS_NAME = {"yaw": "Yaw", "board_x": "Board X", "board_y": "Board Y"}


def describe_table(base):
    for k, (t, p_) in PURPOSE.items():
        if base.startswith(k):
            ax = base[len(k):]
            return (f"{t} - {AXIS_NAME[ax]}" if ax in AXIS_NAME else t), p_
    return base, ""


def w(path, notes, fieldnames, rows, markdown=True, title=None, purpose=""):
    """Write a PURE CSV: first line is the column header, no comment or prose lines.

    `notes` is documentation. It is NOT written into the CSV; it is carried in REGISTRY
    and rendered into results/tables/README.md and above the Markdown copy.
    """
    with open(path, "w", newline="") as f:
        c = csv.DictWriter(f, fieldnames=fieldnames)
        c.writeheader()
        for r in rows:
            c.writerow({k: r.get(k, "") for k in fieldnames})
    base = os.path.basename(path)[:-4]
    _t, _p = describe_table(base)
    REGISTRY.append(dict(csv_name=os.path.basename(path),
                         title=title or _t,
                         purpose=purpose or _p, notes=list(notes),
                         fieldnames=list(fieldnames), rows=rows,
                         markdown=markdown, n_rows=len(rows)))
    print(f"   {os.path.relpath(path, cio.REPO_ROOT)}  ({len(rows)} rows)")


def write_markdown_copies():
    """Human-readable Markdown copies under results/tables/markdown/.

    Grid columns are omitted from the Markdown: each is a constant string for its
    column and is documented in the legend instead. No comparison is dropped.
    result_metrics_long.csv is deliberately excluded (machine-readable only).
    """
    d = os.path.join(T, "markdown")
    os.makedirs(d, exist_ok=True)
    n = 0
    for e in REGISTRY:
        if not e["markdown"]:
            continue
        cols = [c for c in e["fieldnames"] if not c.endswith("_grid")]
        dropped = [c for c in e["fieldnames"] if c.endswith("_grid")]
        L = [f"# {e['title']}", ""]
        if e["purpose"]:
            L += [e["purpose"], ""]
        L += [f"Machine-readable source: [`../{e['csv_name']}`](../{e['csv_name']})", ""]
        L += ["**Notes**", ""] + [f"- {x.lstrip('# ').strip()}" for x in e["notes"]] + [""]
        L += ["**Column abbreviations.** `Cam` = camera, `Cmd` = pulse-derived commanded "
              "reference, `Grav` = gravity-derived tilt, `Dir` = direction, "
              "`SG` = SECONDARY Savitzky-Golay processed rate, `|w|` = magnitude.", ""]
        if dropped:
            L += ["Evaluation-grid columns are omitted here because each is constant for "
                  "its column; see [`../README.md`](../README.md) for the grid of every "
                  "comparison.", ""]
        L += ["| " + " | ".join(md_label(c) for c in cols) + " |",
              "|" + "|".join(["---"] * len(cols)) + "|"]
        for r in e["rows"]:
            L.append("| " + " | ".join(str(r.get(c, "")) for c in cols) + " |")
        L.append("")
        open(os.path.join(d, e["csv_name"][:-4] + ".md"), "w").write("\n".join(L))
        n += 1
    print(f"   {os.path.relpath(d, cio.REPO_ROOT)}/  ({n} Markdown tables)")
    return n


def write_tables_readme():
    """results/tables/README.md: all metadata that used to live inside the CSV files."""
    e = rest.describe()
    L = ["# Detailed Result Tables", "",
         "Full pairwise statistics behind every pooled headline value in IEEE Aerospace",
         "Conference paper 2389. Generated by `python analysis/export_detailed_tables.py`",
         "(also run automatically by `analysis/reproduce_all.py`) from the frozen pipeline",
         "outputs, so they cannot move a headline number.", "",
         "Every `.csv` in this directory is **pure CSV**: the first line is the column",
         "header, there are no comment or prose lines, and GitHub renders each file as a",
         "table. All documentation lives in this README and in the Markdown copies under",
         "[`markdown/`](markdown/).", "",
         "## Reference definition", "",
         "The motor quantity is the **pulse-derived commanded reference**.", "",
         "- It is **not** ground truth.",
         "- It is **not** an actual or measured angle.",
         "- **No independent encoder-position telemetry was logged** for this campaign.",
         "",
         "Resolution is 25 600 pulses/revolution = 0.0140625 deg/pulse, with",
         "round-to-nearest command quantization of +/-0.00703125 deg.", "",
         "## Evaluation grids, and why N differs per comparison", "",
         "The three comparisons are evaluated on different grids, so",
         "**N is reported per comparison** and never as a single generic sample count.",
         "Every CSV carries explicit `*_grid` columns.", "",
         "| Level | Comparison | Evaluation grid |", "|---|---|---|",
         "| Static | Camera - Command | camera grid |",
         "| Static | Gravity - Command | IMU grid (stationary-gated) |",
         "| Static | Camera - Gravity | camera grid, gravity tilt interpolated/aligned onto camera times |",
         "| Dynamic | Camera - Command | camera grid |",
         "| Dynamic | Gyro - Command | IMU grid (the gyro is never resampled) |",
         "| Dynamic | Camera - Gyro | IMU grid, camera rate interpolated onto IMU times |",
         "",
         "The cross-sensor comparison sits on the camera grid for static work and the IMU",
         "grid for dynamic work. That asymmetry is deliberate: the stationary gravity",
         "reference is gated on IMU samples, while the gyro is the rate reference and is",
         "never resampled to make counts match.", "",
         "Because all three static residuals share one reference, the Camera-Gravity",
         "**Bias** is approximately the difference of the other two biases. Its **RMSE** is",
         "not: common-mode platform departure cancels in the cross-sensor comparison,",
         "which is why Camera-Gravity RMSE can fall below both comparisons against the",
         "commanded reference.", "",
         "## Pooling rules", "",
         "Pooled rows use the exact N-weighted identities, algebraically identical to",
         "recomputing from the concatenated residual samples:", "",
         "```", "Bias   = sum(Bias_i * N_i) / sum(N_i)",
         "MAE    = sum(MAE_i  * N_i) / sum(N_i)",
         "RMSE   = sqrt( sum(RMSE_i^2 * N_i) / sum(N_i) )",
         "MaxAbs = max(MaxAbs_i)", "```", "",
         "Per-command and per-rate RMSE values are **never averaged** to form a total.",
         "",
         "## Why static per-hold tables omit MAE and RMSE", "",
         "Within a single static dwell the residual rarely changes sign, so MAE is",
         "identically |Bias| and RMSE is identically sqrt(Bias^2 + SD^2); both were",
         "verified to machine precision on this data. Publishing them per hold would",
         "present three columns as independent evidence when only two are. The",
         "independent per-hold quantities are **Bias, SD, MaxAbs, N**. MAE and RMSE",
         "reappear on the pooled-by-command tables, where repeated visits and sign changes",
         "make them informative.", "",
         "## Why signed bias stays direction-separated", "",
         "Pooling opposite directions cancels signed bias. On Board X at 10 deg/s the",
         "positive and negative legs are of opposite sign and pool to a value an order of",
         "magnitude smaller than either. **No pooled signed-bias column is exported.**",
         "Bias appears as `*_Bias_Pos` / `*_Bias_Neg` for Board X and Board Y, and as",
         "`*_Bias_CW` / `*_Bias_CCW` for yaw, where direction is the **run** rather than",
         "the leg sign (yaw rate segments are logged as magnitudes within each",
         "single-direction run; no negative yaw legs are fabricated).", "",
         "## RAW MaxAbs caveat", "",
         "**RAW-rate MaxAbs is a sample extreme, not a deterministic error bound.** It",
         "scales with sample count: it rises with frame rate and falls with harder leg",
         "trimming, neither of which is a change in accuracy. For static holds MaxAbs does",
         "bound a real excursion and needs no such caveat.", "",
         "Separately, `camera_mean_rate_dps` minus the commanded rate is identically",
         "`Camera_Cmd_Bias`. The mean-rate columns are a readability check, not independent",
         "evidence.", "",
         "## The three rate concepts are kept strictly separate", "",
         "| Concept | Files | Status |", "|---|---|---|",
         "| RAW instantaneous derivative | `dynamic_per_leg_*.csv`, `dynamic_pooled_by_rate_*.csv` | **PRIMARY** |",
         "| SECONDARY processed rate | `dynamic_sg_SECONDARY_*.csv` | diagnostic only |",
         "| Displacement rate scale | `dynamic_displacement_rate_scale_by_rate.csv` | scale metric |",
         "", "### Savitzky-Golay definition", "",
         f"- SECONDARY bandwidth-limited processed-rate **diagnostic only**",
         f"- uniform-time resampling at the median sample interval",
         f"- Savitzky-Golay, **{e['window_s']:.2f} s window**, **polynomial order "
         f"{e['polyorder']}**, **first derivative**",
         f"- zero-phase, offline",
         f"- the **same frozen estimator for every axis**",
         f"- **not tuned per axis**, **not optimized against an error metric**",
         "- **NOT RAW camera-rate accuracy** and never a substitute for the RAW tables",
         "",
         "For yaw the frozen shared estimator is applied to the raw yaw angle per segment,",
         "exactly as `analysis/rate_audit.py` does. The `camera_sg_rate_dps` column in the",
         "yaw synchronized samples is the legacy yaw-pipeline estimator and is deliberately",
         "**not** used here; it would report about 0.0352 deg/s instead of the frozen",
         "0.0421 deg/s.", "",
         "### Displacement rate-scale definition", "",
         "Total camera angular travel over a constant-rate leg divided by elapsed time,",
         "compared with the commanded rate. It carries no frame-to-frame numerical",
         "differentiation, so it measures how faithfully the camera tracks commanded",
         "angular travel. It is a rate **scale** / tracking-fidelity metric and is **not**",
         "instantaneous rate accuracy.", "",
         "## Yaw static", "",
         "Yaw static supports **only Camera - Commanded reference**. The yaw acquisition",
         "logged no roll/pitch, so no gravity-tilt comparison exists; no empty gravity",
         "columns are emitted and no IMU absolute-yaw comparison is invented.", "",
         "## Table index", "",
         "| Table | CSV | Markdown | Purpose |", "|---|---|---|---|"]
    for r in REGISTRY:
        base = r["csv_name"][:-4]
        md = (f"[view](markdown/{base}.md)" if r["markdown"] else "n/a (machine-readable)")
        L.append(f"| `{base}` | [{r['csv_name']}]({r['csv_name']}) | {md} | "
                 f"{r['purpose']} |")
    L += ["", "`result_metrics_long.csv` is the tidy long-form companion: every metric as",
          "one row with `axis, run, original_run_label, segment_type, segment_id,",
          "commanded_value, direction, comparison, processing_level, metric, value, n,",
          "evaluation_grid`. It is intentionally not rendered as Markdown.", ""]
    p_ = os.path.join(T, "README.md")
    open(p_, "w").write("\n".join(L))
    print(f"   {os.path.relpath(p_, cio.REPO_ROOT)}")


def f6(x):
    return "" if x is None or (isinstance(x, float) and not np.isfinite(x)) else f"{x:.6f}"


def pool(parts):
    """Exact sample-level pooling from per-segment (Bias, MAE, RMSE, MaxAbs, N)."""
    parts = [p for p in parts if p and p.get("N")]
    n = sum(p["N"] for p in parts)
    if n == 0:
        return dict(Bias=None, MAE=None, RMSE=None, MaxAbs=None, N=0)
    return dict(
        Bias=sum(p["Bias"] * p["N"] for p in parts) / n,
        MAE=sum(p["MAE"] * p["N"] for p in parts) / n,
        RMSE=float(np.sqrt(sum((p["RMSE"] ** 2) * p["N"] for p in parts) / n)),
        MaxAbs=max(p["MaxAbs"] for p in parts), N=int(n))


def weighted_mean(rows, value_key, n_key):
    """Sample-weighted mean over repeated visits on the relevant evaluation grid."""
    good = [(float(r[value_key]), int(r[n_key])) for r in rows
            if r.get(value_key) is not None and r.get(n_key)]
    n = sum(nn for _, nn in good)
    return None if n == 0 else sum(v * nn for v, nn in good) / n


def add_long(axis, run, label, seg_type, seg_id, cmd, direction, comparison,
             level, stats, grid):
    for m in ("Bias", "MAE", "RMSE", "MaxAbs", "SD"):
        if m in stats and stats[m] is not None and np.isfinite(stats[m]):
            LONG.append(dict(axis=axis, run=run, original_run_label=label,
                             segment_type=seg_type, segment_id=seg_id,
                             commanded_value=cmd, direction=direction,
                             comparison=comparison, processing_level=level,
                             metric=m, value=f"{stats[m]:.6f}", n=stats.get("N", ""),
                             evaluation_grid=grid))


# =====================================================================================
# OUT-OF-PLANE (Board X, Board Y)
# =====================================================================================
OOP = {"board_x": ("results/board_x.json", "board_x/axis_b_run01"),
       "board_y": ("results/board_y.json", None)}

STATIC_HOLD_COLS = ["hold_index", "approach_direction", "commanded_reference_deg",
                    "commanded_pulses", "tags_visible",
                    "camera_mean_deg", "gravity_tilt_mean_deg",
                    "Camera_Cmd_Bias", "Camera_Cmd_SD", "Camera_Cmd_MaxAbs",
                    "Camera_Cmd_N", "Camera_Cmd_grid",
                    "Gravity_Cmd_Bias", "Gravity_Cmd_SD", "Gravity_Cmd_MaxAbs",
                    "Gravity_Cmd_N", "Gravity_Cmd_grid",
                    "Camera_Gravity_Bias", "Camera_Gravity_SD", "Camera_Gravity_MaxAbs",
                    "Camera_Gravity_N", "Camera_Gravity_grid"]

STATIC_POOL_COLS = (["commanded_reference_deg", "n_visits",
                     "camera_mean_deg", "gravity_tilt_mean_deg"]
                    + [f"{p}_{m}" for p in ("Camera_Cmd", "Gravity_Cmd",
                                            "Camera_Gravity")
                       for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N", "grid")])

PRE = {"Camera_Cmd": "Camera_Motor_", "Gravity_Cmd": "IMUgrav_Motor_",
       "Camera_Gravity": "Camera_IMUgrav_"}
CMP = {"Camera_Cmd": "camera_commanded", "Gravity_Cmd": "gravity_commanded",
       "Camera_Gravity": "camera_gravity"}


def out_of_plane_static(axis, res):
    label = res["original_run_label"]
    holds = res["tables"]["holds"]
    rows = []
    for h in holds:
        r = dict(hold_index=h["hold"], approach_direction=h["approach_direction"],
                 commanded_reference_deg=f6(h["commanded_reference_deg"]),
                 commanded_pulses=h["pulses"], tags_visible=h["tags"],
                 camera_mean_deg=f6(h["camera_mean_deg"]),
                 gravity_tilt_mean_deg=f6(h["gravity_tilt_mean_deg"]))
        for pub, pre in PRE.items():
            st = {m: h[pre + m] for m in ("Bias", "STD", "MaxAbs", "N")}
            g = GRID[("STATIC", CMP[pub])]
            r.update({f"{pub}_Bias": f6(st["Bias"]), f"{pub}_SD": f6(st["STD"]),
                      f"{pub}_MaxAbs": f6(st["MaxAbs"]), f"{pub}_N": st["N"],
                      f"{pub}_grid": g})
            add_long(axis, res["run_key"], label, "hold", h["hold"],
                     f6(h["commanded_reference_deg"]), h["approach_direction"],
                     CMP[pub], "STATIC",
                     dict(Bias=st["Bias"], SD=st["STD"], MaxAbs=st["MaxAbs"], N=st["N"]), g)
        rows.append(r)
    w(os.path.join(T, f"static_per_hold_{axis}.csv"),
      [HDR_REF, HDR_GRID,
       "# Acquisition order preserved. approach_direction is a derived label and changes "
       "no residual.",
       "# Per-hold MAE and RMSE are omitted: within one dwell MAE is identically |Bias| "
       "and RMSE is identically sqrt(Bias^2+SD^2). They appear on the pooled-by-command "
       "table.",
       "# No POOLED_TOTAL here: pooling across different commanded angles is meaningless."],
      STATIC_HOLD_COLS, rows)

    by = {}
    for h in holds:
        by.setdefault(round(h["commanded_reference_deg"], 6), []).append(h)
    prows = []
    for q in sorted(by, key=lambda z: (0 if abs(z) < 1e-9 else (1 if z > 0 else 2), abs(z))):
        hs = by[q]
        r = dict(commanded_reference_deg=f6(q), n_visits=len(hs),
                 camera_mean_deg=f6(weighted_mean(
                     hs, "camera_mean_deg", "Camera_Motor_N")),
                 gravity_tilt_mean_deg=f6(weighted_mean(
                     hs, "gravity_tilt_mean_deg", "IMUgrav_Motor_N")))
        for pub, pre in PRE.items():
            p = pool([{m: x[pre + m] for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")}
                      for x in hs])
            g = GRID[("STATIC", CMP[pub])]
            r.update({f"{pub}_{m}": (f6(p[m]) if m != "N" else p[m])
                      for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")})
            r[f"{pub}_grid"] = g
            add_long(axis, res["run_key"], label, "pooled_by_command", f6(q), f6(q),
                     "all_visits", CMP[pub], "STATIC", p, g)
        prows.append(r)
    tot = dict(commanded_reference_deg="POOLED_TOTAL", n_visits=len(holds),
               camera_mean_deg="", gravity_tilt_mean_deg="")
    for pub, pre in PRE.items():
        p = pool([{m: x[pre + m] for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")}
                  for x in holds])
        g = GRID[("STATIC", CMP[pub])]
        tot.update({f"{pub}_{m}": (f6(p[m]) if m != "N" else p[m])
                    for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")})
        tot[f"{pub}_grid"] = g
        add_long(axis, res["run_key"], label, "pooled_total", "ALL", "", "all",
                 CMP[pub], "STATIC", p, g)
    prows.append(tot)
    w(os.path.join(T, f"static_pooled_by_command_{axis}.csv"),
      [HDR_REF, HDR_GRID,
       "# Samples pooled at sample level via exact N-weighted identities; RMSE is never "
       "averaged across commands.",
       "# Repeated visits to one command are pooled here; the per-hold table preserves "
       "approach direction and hysteresis."],
      STATIC_POOL_COLS, prows)


LEG_PRE = {"Camera_Cmd": "Camera_Motor_", "Gyro_Cmd": "IMU_Motor_",
           "Camera_Gyro": "Camera_IMU_"}
LEG_CMP = {"Camera_Cmd": "camera_commanded", "Gyro_Cmd": "gyro_commanded",
           "Camera_Gyro": "camera_gyro"}
DYN_LEG_COLS = (["leg_index", "commanded_rate_dps", "direction", "leg_duration_s",
                 "camera_mean_rate_dps", "gyro_mean_rate_dps"]
                + [f"{p}_{m}" for p in ("Camera_Cmd", "Gyro_Cmd", "Camera_Gyro")
                   for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N", "grid")])


def out_of_plane_dynamic(axis, res):
    label = res["original_run_label"]
    legs = res["tables"]["legs"]
    rows = []
    for L in legs:
        r = dict(leg_index=L["leg"], commanded_rate_dps=f6(L["commanded_rate_dps"]),
                 direction=L["direction"], leg_duration_s=f6(L["leg_duration_s"]),
                 camera_mean_rate_dps=f6(L["camera_mean_rate_dps"]),
                 gyro_mean_rate_dps=f6(L["gyro_mean_rate_dps"]))
        for pub, pre in LEG_PRE.items():
            g = GRID[("RAW_DYNAMIC", LEG_CMP[pub])]
            st = {m: L[pre + m] for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")}
            r.update({f"{pub}_{m}": (f6(st[m]) if m != "N" else st[m])
                      for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")})
            r[f"{pub}_grid"] = g
            add_long(axis, res["run_key"], label, "rate_leg", L["leg"],
                     f6(L["commanded_rate_dps"]), L["direction"], LEG_CMP[pub],
                     "RAW_DYNAMIC", st, g)
        rows.append(r)
    w(os.path.join(T, f"dynamic_per_leg_{axis}.csv"),
      [HDR_REF, HDR_GRID, HDR_MAXABS,
       "# Frozen evaluation windows: central 70 % of each constant-rate leg, unchanged "
       "from the published headline results.",
       "# camera_mean_rate_dps minus commanded_rate_dps is identically Camera_Cmd_Bias.",
       "# RAW instantaneous finite difference. SG results are in "
       "dynamic_sg_SECONDARY_*.csv and are never mixed in here."],
      DYN_LEG_COLS, rows)

    mags = sorted({abs(round(L["commanded_rate_dps"], 6)) for L in legs})
    cols = ["commanded_rate_magnitude_dps", "n_legs_pos", "n_legs_neg"]
    for p in ("Camera_Cmd", "Gyro_Cmd", "Camera_Gyro"):
        cols += [f"{p}_Bias_Pos", f"{p}_Bias_Neg", f"{p}_MAE", f"{p}_RMSE",
                 f"{p}_MaxAbs", f"{p}_N", f"{p}_grid"]
    prows = []
    for mag in mags:
        sel = [L for L in legs if abs(round(L["commanded_rate_dps"], 6)) == mag]
        pos = [L for L in sel if L["commanded_rate_dps"] > 0]
        neg = [L for L in sel if L["commanded_rate_dps"] < 0]
        r = dict(commanded_rate_magnitude_dps=f6(mag), n_legs_pos=len(pos),
                 n_legs_neg=len(neg))
        for pub, pre in LEG_PRE.items():
            g = GRID[("RAW_DYNAMIC", LEG_CMP[pub])]
            allp = pool([{m: x[pre + m] for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")}
                         for x in sel])
            bp = pool([{m: x[pre + m] for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")}
                       for x in pos])
            bn = pool([{m: x[pre + m] for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")}
                       for x in neg])
            r.update({f"{pub}_Bias_Pos": f6(bp["Bias"]),
                      f"{pub}_Bias_Neg": f6(bn["Bias"]),
                      f"{pub}_MAE": f6(allp["MAE"]), f"{pub}_RMSE": f6(allp["RMSE"]),
                      f"{pub}_MaxAbs": f6(allp["MaxAbs"]), f"{pub}_N": allp["N"],
                      f"{pub}_grid": g})
            add_long(axis, res["run_key"], label, "pooled_by_rate", "", f6(mag),
                     "magnitude", LEG_CMP[pub], "RAW_DYNAMIC",
                     dict(MAE=allp["MAE"], RMSE=allp["RMSE"], MaxAbs=allp["MaxAbs"],
                          N=allp["N"]), g)
            add_long(axis, res["run_key"], label, "pooled_by_rate", "", f6(mag), "positive",
                     LEG_CMP[pub], "RAW_DYNAMIC", dict(Bias=bp["Bias"], N=bp["N"]), g)
            add_long(axis, res["run_key"], label, "pooled_by_rate", "", f6(mag), "negative",
                     LEG_CMP[pub], "RAW_DYNAMIC", dict(Bias=bn["Bias"], N=bn["N"]), g)
        prows.append(r)
    tot = dict(commanded_rate_magnitude_dps="POOLED_TOTAL",
               n_legs_pos=sum(1 for L in legs if L["commanded_rate_dps"] > 0),
               n_legs_neg=sum(1 for L in legs if L["commanded_rate_dps"] < 0))
    for pub, pre in LEG_PRE.items():
        g = GRID[("RAW_DYNAMIC", LEG_CMP[pub])]
        allp = pool([{m: x[pre + m] for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")}
                     for x in legs])
        bp = pool([{m: x[pre + m] for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")}
                   for x in legs if x["commanded_rate_dps"] > 0])
        bn = pool([{m: x[pre + m] for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")}
                   for x in legs if x["commanded_rate_dps"] < 0])
        tot.update({f"{pub}_Bias_Pos": f6(bp["Bias"]), f"{pub}_Bias_Neg": f6(bn["Bias"]),
                    f"{pub}_MAE": f6(allp["MAE"]), f"{pub}_RMSE": f6(allp["RMSE"]),
                    f"{pub}_MaxAbs": f6(allp["MaxAbs"]), f"{pub}_N": allp["N"],
                    f"{pub}_grid": g})
        add_long(axis, res["run_key"], label, "pooled_total", "ALL", "ALL",
                 "all_magnitudes", LEG_CMP[pub], "RAW_DYNAMIC",
                 dict(MAE=allp["MAE"], RMSE=allp["RMSE"], MaxAbs=allp["MaxAbs"],
                      N=allp["N"]), g)
    prows.append(tot)
    w(os.path.join(T, f"dynamic_pooled_by_rate_{axis}.csv"),
      [HDR_REF, HDR_GRID, HDR_MAXABS, HDR_NOBIAS,
       "# MAE and RMSE pooled at sample level by rate MAGNITUDE; signed bias stays "
       "direction-separated in *_Bias_Pos / *_Bias_Neg.",
       "# POOLED_TOTAL is sample-level over all legs of this axis."],
      cols, prows)


def out_of_plane_sg(axis, res):
    label = res["original_run_label"]
    legs = res["tables"]["legs"]
    sgb = res["dynamic_sg_SECONDARY"]
    e = sgb["estimator"]
    mags = sorted({abs(round(L["commanded_rate_dps"], 6)) for L in legs})
    cols = ["commanded_rate_magnitude_dps", "SG_Camera_Cmd_Bias_Pos",
            "SG_Camera_Cmd_Bias_Neg", "SG_Camera_Cmd_MAE", "SG_Camera_Cmd_RMSE",
            "SG_Camera_Cmd_MaxAbs", "SG_Camera_Cmd_N", "SG_Camera_Cmd_grid",
            "sg_window_s", "sg_polyorder", "sg_window_samples", "camera_fps_median_Hz"]
    g = GRID[("SG_SECONDARY", "camera_commanded")]
    rows = []
    for mag in mags + ["POOLED_TOTAL"]:
        sel = legs if mag == "POOLED_TOTAL" else [
            L for L in legs if abs(round(L["commanded_rate_dps"], 6)) == mag]
        st = lambda S: [{m: x["SG_Camera_Motor_" + m]
                         for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")} for x in S]
        allp = pool(st(sel))
        bp = pool(st([L for L in sel if L["commanded_rate_dps"] > 0]))
        bn = pool(st([L for L in sel if L["commanded_rate_dps"] < 0]))
        rows.append(dict(
            commanded_rate_magnitude_dps=(mag if isinstance(mag, str) else f6(mag)),
            SG_Camera_Cmd_Bias_Pos=f6(bp["Bias"]), SG_Camera_Cmd_Bias_Neg=f6(bn["Bias"]),
            SG_Camera_Cmd_MAE=f6(allp["MAE"]), SG_Camera_Cmd_RMSE=f6(allp["RMSE"]),
            SG_Camera_Cmd_MaxAbs=f6(allp["MaxAbs"]), SG_Camera_Cmd_N=allp["N"],
            SG_Camera_Cmd_grid=g, sg_window_s=e["window_s"], sg_polyorder=e["polyorder"],
            sg_window_samples=sgb["sg_window_samples"],
            camera_fps_median_Hz=f"{sgb['sample_rate_hz']:.2f}"))
        add_long(axis, res["run_key"], label,
                 "pooled_total" if isinstance(mag, str) else "pooled_by_rate", "",
                 "ALL" if isinstance(mag, str) else f6(mag), "magnitude",
                 "camera_commanded", "SG_SECONDARY",
                 dict(MAE=allp["MAE"], RMSE=allp["RMSE"], MaxAbs=allp["MaxAbs"],
                      N=allp["N"]), g)
    w(os.path.join(T, f"dynamic_sg_SECONDARY_{axis}.csv"), [HDR_SG, HDR_REF, HDR_GRID],
      cols, rows)


# =====================================================================================
# YAW  (camera-commanded static only; no gravity-tilt comparison exists)
# =====================================================================================
def yaw_frames():
    fs = sorted(glob.glob(os.path.join(cio.RESULTS,
                                       "yaw_synchronized_samples_run*.csv")))
    if not fs:
        return None
    out = []
    for f in fs:
        d = pd.read_csv(f)
        run = str(d["run"].iloc[0]) if "run" in d else os.path.basename(f)
        direction = str(d["direction"].iloc[0]) if "direction" in d else "?"
        out.append((run, direction, d))
    return out


def stats(e):
    e = np.asarray(e, float)
    e = e[np.isfinite(e)]
    if e.size == 0:
        return dict(Bias=None, MAE=None, RMSE=None, MaxAbs=None, SD=None, N=0)
    return dict(Bias=float(e.mean()), MAE=float(np.abs(e).mean()),
                RMSE=float(np.sqrt((e ** 2).mean())),
                MaxAbs=float(np.abs(e).max()),
                SD=float(e.std(ddof=1)) if e.size > 1 else None, N=int(e.size))


def yaw_tables():
    FR = yaw_frames()
    if not FR:
        print("   [skip] yaw synchronized samples not found; run reproduce_yaw first")
        return
    gcam = GRID[("STATIC", "camera_commanded")]

    # ---- static per hold -------------------------------------------------------
    rows, pooled_by, hold_i = [], {}, 0
    for run, direction, d in FR:
        h = d[(d["grid"] == "camera") & (d["segment_type"] == "hold")]
        prev = None
        for sid, seg in h.groupby("segment_id", sort=True):
            cmd = float(seg["effective_motor_angle_deg"].iloc[0])
            e = seg["camera_motor_error_deg"].to_numpy(float)
            s = stats(e)
            ap = ("initial" if prev is None else
                  "ascending" if cmd > prev + 1e-9 else
                  "descending" if cmd < prev - 1e-9 else "repeat")
            prev = cmd
            rows.append(dict(hold_index=hold_i, run=run, direction=direction,
                             approach_direction=ap, commanded_reference_deg=f6(cmd),
                             camera_mean_deg=f6(float(
                                 seg["camera_raw_yaw_deg"].mean())),
                             Camera_Cmd_Bias=f6(s["Bias"]), Camera_Cmd_SD=f6(s["SD"]),
                             Camera_Cmd_MaxAbs=f6(s["MaxAbs"]), Camera_Cmd_N=s["N"],
                             Camera_Cmd_grid=gcam))
            add_long("yaw", run, f"result_yaw/{run}", "hold", hold_i, f6(cmd), ap,
                     "camera_commanded", "STATIC", s, gcam)
            pooled_by.setdefault(round(cmd, 6), []).append(e)
            hold_i += 1
    w(os.path.join(T, "static_per_hold_yaw.csv"),
      [HDR_REF, HDR_GRID,
       "# Yaw static supports ONLY camera minus commanded reference. The yaw acquisition "
       "logged no roll/pitch, so no gravity-tilt comparison exists and none is invented.",
       "# Per-hold MAE and RMSE omitted by design (identically |Bias| and "
       "sqrt(Bias^2+SD^2) within a dwell)."],
      ["hold_index", "run", "direction", "approach_direction",
       "commanded_reference_deg", "camera_mean_deg", "Camera_Cmd_Bias", "Camera_Cmd_SD",
       "Camera_Cmd_MaxAbs", "Camera_Cmd_N", "Camera_Cmd_grid"], rows)

    prows, allE = [], []
    for q in sorted(pooled_by):
        E = np.concatenate(pooled_by[q])
        allE.append(E)
        s = stats(E)
        prows.append(dict(commanded_reference_deg=f6(q), n_visits=len(pooled_by[q]),
                          camera_mean_deg=f6(q + s["Bias"]),
                          Camera_Cmd_Bias=f6(s["Bias"]), Camera_Cmd_MAE=f6(s["MAE"]),
                          Camera_Cmd_RMSE=f6(s["RMSE"]),
                          Camera_Cmd_MaxAbs=f6(s["MaxAbs"]), Camera_Cmd_N=s["N"],
                          Camera_Cmd_grid=gcam))
        add_long("yaw", "run01+run02", "result_yaw/test1+test2", "pooled_by_command",
                 f6(q), f6(q), "all_visits", "camera_commanded", "STATIC", s, gcam)
    s = stats(np.concatenate(allE))
    prows.append(dict(commanded_reference_deg="POOLED_TOTAL",
                      n_visits=sum(len(v) for v in pooled_by.values()),
                      camera_mean_deg="",
                      Camera_Cmd_Bias=f6(s["Bias"]), Camera_Cmd_MAE=f6(s["MAE"]),
                      Camera_Cmd_RMSE=f6(s["RMSE"]),
                      Camera_Cmd_MaxAbs=f6(s["MaxAbs"]), Camera_Cmd_N=s["N"],
                      Camera_Cmd_grid=gcam))
    add_long("yaw", "run01+run02", "result_yaw/test1+test2", "pooled_total", "ALL", "",
             "all", "camera_commanded", "STATIC", s, gcam)
    w(os.path.join(T, "static_pooled_by_command_yaw.csv"),
      [HDR_REF, HDR_GRID,
       "# Yaw static: camera minus commanded reference ONLY. No gravity-tilt comparison "
       "exists for yaw.",
       "# POOLED_TOTAL pools both directional runs at sample level."],
      ["commanded_reference_deg", "n_visits", "camera_mean_deg",
       "Camera_Cmd_Bias", "Camera_Cmd_MAE", "Camera_Cmd_RMSE",
       "Camera_Cmd_MaxAbs", "Camera_Cmd_N", "Camera_Cmd_grid"], prows)

    # ---- dynamic: per leg, per rate, SG, displacement ---------------------------
    leg_rows, by_rate, scale_rows, sg_by = [], {}, [], {}
    gc = GRID[("RAW_DYNAMIC", "camera_commanded")]
    gg = GRID[("RAW_DYNAMIC", "gyro_commanded")]
    gcg = GRID[("RAW_DYNAMIC", "camera_gyro")]
    li = 0
    for run, direction, d in FR:
        cam = d[(d["grid"] == "camera") & (d["segment_type"] == "rate")]
        imu = d[(d["grid"] == "imu") & (d["segment_type"] == "rate")]
        for sid in sorted(cam["segment_id"].unique()):
            sc = cam[cam["segment_id"] == sid].sort_values("common_time_s")
            si = imu[imu["segment_id"] == sid].sort_values("common_time_s")
            r = float(sc["motor_rate_dps"].iloc[0])
            if abs(r) < 1e-9:
                continue
            e_cm = sc["camera_raw_rate_dps"].to_numpy(float) - r
            e_gm = si["imu_raw_rate_dps"].to_numpy(float) - r
            e_cg = (si["camera_raw_rate_dps"].to_numpy(float)
                    - si["imu_raw_rate_dps"].to_numpy(float))
            # SECONDARY SG: apply the FROZEN shared estimator to the raw yaw angle,
            # exactly as analysis/rate_audit.py does. The camera_sg_rate_dps column in
            # the synchronized samples is the legacy yaw-pipeline estimator and is NOT
            # the frozen cross-axis estimator; using it would report ~0.0352 deg/s
            # instead of the frozen 0.0421 deg/s.
            _t = sc["common_time_s"].to_numpy(float)
            _y = sc["camera_raw_yaw_deg"].to_numpy(float)
            _m = np.isfinite(_t) & np.isfinite(_y)
            _der, _, _ = rest.sg_rate(_t[_m], _y[_m])
            e_sg = (_der - r) if _der is not None else np.array([])
            S = {"Camera_Cmd": (stats(e_cm), gc, "camera_commanded"),
                 "Gyro_Cmd": (stats(e_gm), gg, "gyro_commanded"),
                 "Camera_Gyro": (stats(e_cg), gcg, "camera_gyro")}
            row = dict(leg_index=li, run=run, direction=direction,
                       commanded_rate_dps=f6(r),
                       leg_duration_s=f6(float(sc["common_time_s"].iloc[-1]
                                               - sc["common_time_s"].iloc[0])),
                       camera_mean_rate_dps=f6(float(
                           sc["camera_raw_rate_dps"].mean())),
                       gyro_mean_rate_dps=f6(float(si["imu_raw_rate_dps"].mean())))
            for pub, (st_, g_, cmpname) in S.items():
                row.update({f"{pub}_{m}": (f6(st_[m]) if m != "N" else st_[m])
                            for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N")})
                row[f"{pub}_grid"] = g_
                add_long("yaw", run, f"result_yaw/{run}", "rate_leg", li, f6(r),
                         direction, cmpname, "RAW_DYNAMIC", st_, g_)
                by_rate.setdefault(abs(round(r, 6)), {}).setdefault(
                    pub, {}).setdefault(direction, []).append(st_)
            leg_rows.append(row)
            sg_by.setdefault(abs(round(r, 6)), {}).setdefault(
                direction, []).append(stats(e_sg))
            y = sc["camera_raw_yaw_deg"].to_numpy(float)
            t = sc["common_time_s"].to_numpy(float)
            dr = (y[-1] - y[0]) / (t[-1] - t[0])
            scale_rows.append(dict(axis="yaw", run=run, commanded_rate_dps=f6(r),
                                   direction=direction,
                                   camera_displacement_rate_dps=f6(dr),
                                   scale_error_dps=f6(dr - r),
                                   scale_error_percent=f"{100.0 * (dr - r) / r:.4f}"))
            li += 1
    w(os.path.join(T, "dynamic_per_leg_yaw.csv"),
      [HDR_REF, HDR_GRID, HDR_MAXABS,
       "# Yaw direction is the RUN (CW / CCW); yaw rate segments are logged as "
       "magnitudes within each single-direction run. No negative legs are fabricated."],
      ["leg_index", "run", "direction", "commanded_rate_dps", "leg_duration_s",
       "camera_mean_rate_dps", "gyro_mean_rate_dps"]
      + [f"{p}_{m}" for p in ("Camera_Cmd", "Gyro_Cmd", "Camera_Gyro")
         for m in ("Bias", "MAE", "RMSE", "MaxAbs", "N", "grid")], leg_rows)

    dirs = sorted({d for _, d, _ in FR})
    cw = next((x for x in dirs if "C" in x.upper() and "CC" not in x.upper()), dirs[0])
    ccw = next((x for x in dirs if "CC" in x.upper()), dirs[-1])
    cols = ["commanded_rate_magnitude_dps", "n_legs_CW", "n_legs_CCW"]
    for p in ("Camera_Cmd", "Gyro_Cmd", "Camera_Gyro"):
        cols += [f"{p}_Bias_CW", f"{p}_Bias_CCW", f"{p}_MAE", f"{p}_RMSE",
                 f"{p}_MaxAbs", f"{p}_N", f"{p}_grid"]
    prows = []
    for mag in sorted(by_rate) + ["POOLED_TOTAL"]:
        keys = sorted(by_rate) if mag == "POOLED_TOTAL" else [mag]
        r = dict(commanded_rate_magnitude_dps=(mag if isinstance(mag, str) else f6(mag)),
                 n_legs_CW=sum(len(by_rate[k].get("Camera_Cmd", {}).get(cw, []))
                               for k in keys),
                 n_legs_CCW=sum(len(by_rate[k].get("Camera_Cmd", {}).get(ccw, []))
                                for k in keys))
        for pub in ("Camera_Cmd", "Gyro_Cmd", "Camera_Gyro"):
            g_ = {"Camera_Cmd": gc, "Gyro_Cmd": gg, "Camera_Gyro": gcg}[pub]
            allst = [s for k in keys for dd in by_rate[k].get(pub, {}).values()
                     for s in dd]
            bcw = [s for k in keys for s in by_rate[k].get(pub, {}).get(cw, [])]
            bccw = [s for k in keys for s in by_rate[k].get(pub, {}).get(ccw, [])]
            a, c1, c2 = pool(allst), pool(bcw), pool(bccw)
            r.update({f"{pub}_Bias_CW": f6(c1["Bias"]),
                      f"{pub}_Bias_CCW": f6(c2["Bias"]), f"{pub}_MAE": f6(a["MAE"]),
                      f"{pub}_RMSE": f6(a["RMSE"]), f"{pub}_MaxAbs": f6(a["MaxAbs"]),
                      f"{pub}_N": a["N"], f"{pub}_grid": g_})
            add_long("yaw", "run01+run02", "result_yaw/test1+test2",
                     "pooled_total" if isinstance(mag, str) else "pooled_by_rate", "",
                     "ALL" if isinstance(mag, str) else f6(mag), "magnitude",
                     {"Camera_Cmd": "camera_commanded", "Gyro_Cmd": "gyro_commanded",
                      "Camera_Gyro": "camera_gyro"}[pub], "RAW_DYNAMIC",
                     dict(MAE=a["MAE"], RMSE=a["RMSE"], MaxAbs=a["MaxAbs"], N=a["N"]), g_)
        prows.append(r)
    w(os.path.join(T, "dynamic_pooled_by_rate_yaw.csv"),
      [HDR_REF, HDR_GRID, HDR_MAXABS, HDR_NOBIAS,
       f"# Yaw direction is by RUN: CW = {cw}, CCW = {ccw}. Signed bias is reported "
       "per run and never pooled across directions."], cols, prows)

    gsg = GRID[("SG_SECONDARY", "camera_commanded")]
    e = rest.describe()
    cols = ["commanded_rate_magnitude_dps", "SG_Camera_Cmd_Bias_CW",
            "SG_Camera_Cmd_Bias_CCW", "SG_Camera_Cmd_MAE", "SG_Camera_Cmd_RMSE",
            "SG_Camera_Cmd_MaxAbs", "SG_Camera_Cmd_N", "SG_Camera_Cmd_grid",
            "sg_window_s", "sg_polyorder"]
    rows = []
    for mag in sorted(sg_by) + ["POOLED_TOTAL"]:
        keys = sorted(sg_by) if mag == "POOLED_TOTAL" else [mag]
        allst = [s for k in keys for dd in sg_by[k].values() for s in dd]
        a = pool(allst)
        c1 = pool([s for k in keys for s in sg_by[k].get(cw, [])])
        c2 = pool([s for k in keys for s in sg_by[k].get(ccw, [])])
        rows.append(dict(
            commanded_rate_magnitude_dps=(mag if isinstance(mag, str) else f6(mag)),
            SG_Camera_Cmd_Bias_CW=f6(c1["Bias"]), SG_Camera_Cmd_Bias_CCW=f6(c2["Bias"]),
            SG_Camera_Cmd_MAE=f6(a["MAE"]), SG_Camera_Cmd_RMSE=f6(a["RMSE"]),
            SG_Camera_Cmd_MaxAbs=f6(a["MaxAbs"]), SG_Camera_Cmd_N=a["N"],
            SG_Camera_Cmd_grid=gsg, sg_window_s=e["window_s"],
            sg_polyorder=e["polyorder"]))
        add_long("yaw", "run01+run02", "result_yaw/test1+test2",
                 "pooled_total" if isinstance(mag, str) else "pooled_by_rate", "",
                 "ALL" if isinstance(mag, str) else f6(mag), "magnitude",
                 "camera_commanded", "SG_SECONDARY",
                 dict(MAE=a["MAE"], RMSE=a["RMSE"], MaxAbs=a["MaxAbs"], N=a["N"]), gsg)
    w(os.path.join(T, "dynamic_sg_SECONDARY_yaw.csv"), [HDR_SG, HDR_REF, HDR_GRID],
      cols, rows)
    return scale_rows


# =====================================================================================
def main(board_x_data=None, board_y_data=None):
    LONG.clear()
    os.makedirs(T, exist_ok=True)
    print("=" * 78)
    print("DETAILED RESULT TABLES  (full pairwise statistics)")
    print("=" * 78)
    print("  motor quantity = PULSE-DERIVED COMMANDED REFERENCE (not ground truth)")

    print("\n[1/4] out-of-plane static ...")
    res = {}
    supplied = {"board_x": board_x_data, "board_y": board_y_data}
    for axis, (path, key) in OOP.items():
        d = supplied[axis]
        if d is None:
            d = json.load(open(os.path.join(cio.REPO_ROOT, path)))
        res[axis] = d[key] if key else d
        out_of_plane_static(axis, res[axis])

    print("\n[2/4] out-of-plane dynamic RAW + SECONDARY SG ...")
    scale_all = []
    for axis in OOP:
        out_of_plane_dynamic(axis, res[axis])
        out_of_plane_sg(axis, res[axis])
        for s in res[axis]["tables"]["rate_scale"]:
            scale_all.append(dict(axis=axis, run=res[axis]["run_key"],
                                  commanded_rate_dps=f6(s["commanded_rate_dps"]),
                                  direction=s["direction"],
                                  camera_displacement_rate_dps=f6(
                                      s["camera_displacement_rate_dps"]),
                                  scale_error_dps=f6(s["error_dps"]),
                                  scale_error_percent=f"{s['error_pct']:.4f}"))
            LONG.append(dict(axis=axis, run=res[axis]["run_key"],
                             original_run_label=res[axis]["original_run_label"],
                             segment_type="rate_leg", segment_id=s["leg"],
                             commanded_value=f6(s["commanded_rate_dps"]),
                             direction=s["direction"], comparison="camera_commanded",
                             processing_level="DISPLACEMENT_SCALE",
                             metric="scale_error_percent",
                             value=f"{s['error_pct']:.4f}", n="",
                             evaluation_grid=GRID[("DISPLACEMENT_SCALE",
                                                   "camera_commanded")]))

    print("\n[3/4] yaw ...")
    ys = yaw_tables()
    if ys:
        for s in ys:
            scale_all.append(dict(axis="yaw", run=s["run"],
                                  commanded_rate_dps=s["commanded_rate_dps"],
                                  direction=s["direction"],
                                  camera_displacement_rate_dps=s[
                                      "camera_displacement_rate_dps"],
                                  scale_error_dps=s["scale_error_dps"],
                                  scale_error_percent=s["scale_error_percent"]))
            LONG.append(dict(axis="yaw", run=s["run"],
                             original_run_label=f"result_yaw/{s['run']}",
                             segment_type="rate_leg", segment_id="",
                             commanded_value=s["commanded_rate_dps"],
                             direction=s["direction"], comparison="camera_commanded",
                             processing_level="DISPLACEMENT_SCALE",
                             metric="scale_error_percent",
                             value=s["scale_error_percent"], n="",
                             evaluation_grid=GRID[("DISPLACEMENT_SCALE",
                                                   "camera_commanded")]))
    w(os.path.join(T, "dynamic_displacement_rate_scale_by_rate.csv"),
      [HDR_SCALE, HDR_REF],
      ["axis", "run", "commanded_rate_dps", "direction",
       "camera_displacement_rate_dps", "scale_error_dps", "scale_error_percent"],
      scale_all)

    print("\n[4/4] tidy long-form companion ...")
    w(os.path.join(T, "result_metrics_long.csv"),
      [HDR_REF, HDR_GRID,
       "# Generated automatically from the same calculations as the wide tables.",
       "# processing_level: STATIC | RAW_DYNAMIC | SG_SECONDARY | DISPLACEMENT_SCALE.",
       "# SG_SECONDARY rows are a bandwidth-limited diagnostic and are NOT RAW accuracy."],
      ["axis", "run", "original_run_label", "segment_type", "segment_id",
       "commanded_value", "direction", "comparison", "processing_level", "metric",
       "value", "n", "evaluation_grid"], LONG, markdown=False)
    print("\n[5/5] Markdown copies and results/tables/README.md ...")
    write_markdown_copies()
    write_tables_readme()
    print(f"\nDone. {len(LONG)} long-form metric rows; "
          f"{len(REGISTRY)} CSV tables, all pure CSV (header on line 1).")


if __name__ == "__main__":
    main()
