#!/usr/bin/env python3
"""
analysis/make_paper_figures.py
==============================
Publication figures from the reproduced RAW results. CREATED POST-CAMPAIGN.

    python analysis/make_paper_figures.py

Run `python analysis/reproduce_all.py` first. Headline figures use RAW results only.
Axis limits are set from the data including outliers; nothing is clipped to look tidy.
"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import io as cio           # noqa: E402

DPI = 200
FIG = os.path.join(cio.RESULTS, "figures")


def _load():
    p = os.path.join(cio.RESULTS, "reproduced_numbers.json")
    if not os.path.exists(p):
        sys.exit("run `python analysis/reproduce_all.py` first")
    return json.load(open(p))


def fig_sync(R):
    """F3: camera<->motor clock-fit residual per run."""
    fig, ax = plt.subplots(figsize=(7.2, 3.4))
    runs = [("board_x/axis_a", R["board_x"]["board_x/axis_a"]),
            ("board_x/axis_b_run01", R["board_x"]["board_x/axis_b_run01"]),
            ("board_x/axis_b_run04", R["board_x"]["board_x/axis_b_run04"]),
            ("board_y/run05", R["board_y"])]
    names = [k.split("/")[-1] for k, _ in runs]
    sd = [r["sync"]["camera_motor"]["residual_sd_ms"] for _, r in runs]
    ppm = [r["sync"]["camera_motor"]["ppm"] for _, r in runs]
    b = ax.bar(names, sd, color="#3b6ea5")
    for r_, p_ in zip(b, ppm):
        ax.text(r_.get_x() + r_.get_width() / 2, r_.get_height() + 0.02,
                f"{p_:+.1f} ppm", ha="center", fontsize=8)
    ax.set_ylabel("clock-fit residual SD [ms]")
    ax.set_ylim(0, max(sd) * 1.35)
    ax.set_title("Camera$\\leftrightarrow$motor alignment, fitted to serial motor events only\n"
                 "(114 events per out-of-plane run)", fontsize=10)
    ax.grid(axis="y", alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "F3_synchronization.pdf")); plt.close(fig)


def fig_static(R):
    """F4: commanded vs measured, Board Y, both references."""
    by = R["board_y"]
    h = by["tables"]["holds"]
    cmd = np.array([r["commanded_reference_deg"] for r in h])
    cam = np.array([r["camera_mean_deg"] for r in h])
    gt = np.array([r["gravity_tilt_mean_deg"] for r in h])
    fig, ax = plt.subplots(1, 2, figsize=(9.6, 3.9))
    lim = [cmd.min() - 2, cmd.max() + 2]
    ax[0].plot(lim, lim, "k--", lw=.8, label="ideal")
    ax[0].plot(cmd, cam, "o", ms=5, label="camera (RAW)")
    ax[0].plot(cmd, gt, "x", ms=6, label="IMU gravity tilt (RAW)")
    ax[0].set_xlabel("pulse-derived commanded reference [deg]")
    ax[0].set_ylabel("measured [deg]"); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
    ax[0].set_title("Board Y: measured vs commanded reference", fontsize=10)
    ax[1].axhline(0, color="k", lw=.8)
    ax[1].plot(cmd, cam - cmd, "o", ms=5, label="camera $-$ commanded")
    ax[1].plot(cmd, gt - cmd, "x", ms=6, label="gravity tilt $-$ commanded")
    ax[1].plot(cmd, cam - gt, "s", ms=4, mfc="none", label="camera $-$ gravity tilt")
    ax[1].set_xlabel("pulse-derived commanded reference [deg]")
    ax[1].set_ylabel("error [deg]"); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)
    ax[1].set_title("RAW residuals (no drift removal)", fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "F4_static_board_y.pdf")); plt.close(fig)


def fig_rate(R):
    """F6: rate-scale per leg, the tracking-fidelity metric."""
    by = R["board_y"]
    rs = by["tables"]["rate_scale"]
    fig, ax = plt.subplots(figsize=(7.2, 3.6))
    for d, mk, c in (("positive", "o", "#3b6ea5"), ("negative", "s", "#a5433b")):
        sel = [r for r in rs if r["direction"] == d]
        ax.plot([abs(r["commanded_rate_dps"]) for r in sel],
                [r["error_pct"] for r in sel], mk, ms=6, color=c,
                label=f"{d} legs")
    ax.axhline(0, color="k", lw=.8)
    ax.set_xscale("log"); ax.set_xticks([1, 2, 5, 10]); ax.set_xticklabels([1, 2, 5, 10])
    ax.set_xlabel("commanded rate magnitude [deg/s]")
    ax.set_ylabel("rate-scale error [%]")
    ax.set_title("Board Y rate scale: camera displacement per leg / commanded rate\n"
                 "(NOT an instantaneous rate RMSE)", fontsize=10)
    ax.legend(fontsize=8); ax.grid(alpha=.3, which="both")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "F6_rate_scale.pdf")); plt.close(fig)


def fig_conditioning(R):
    """F5: pose conditioning across the sweep, both out-of-plane axes."""
    fig, ax = plt.subplots(figsize=(7.4, 4.0))
    for lbl, r, mk in (("Board X (axis_b_run01)", R["board_x"]["board_x/axis_b_run01"], "s"),
                       ("Board Y (run05)", R["board_y"], "o")):
        h = r["tables"]["holds"]
        cmd = [x["commanded_reference_deg"] for x in h]
        err = [x["Camera_Motor_Bias"] for x in h]
        ax.plot(cmd, err, mk, ms=5, label=lbl)
    ax.axhline(0, color="k", lw=.8)
    ax.set_xlabel("pulse-derived commanded reference [deg]")
    ax.set_ylabel("camera $-$ commanded, per-hold mean [deg]")
    c1 = R["board_y"]["conditioning"]
    ax.set_title("Per-hold RAW camera error, both out-of-plane axes\n"
                 f"Board Y: candidate separation "
                 f"{c1['candidate_separation_deg']['min']:.1f}"
                 f"-{c1['candidate_separation_deg']['max']:.1f} deg, "
                 f"reprojection ratio {c1['reprojection_ratio']['min']:.1f}"
                 f"-{c1['reprojection_ratio']['max']:.1f}", fontsize=9)
    ax.legend(fontsize=8); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "F5_conditioning.pdf")); plt.close(fig)


def fig_drift(R):
    """F7: zero-return departure seen by two independent sensors."""
    fig, ax = plt.subplots(figsize=(7.0, 3.6))
    for lbl, r, mk in (("Board X (axis_b_run01)", R["board_x"]["board_x/axis_b_run01"], "s"),
                       ("Board Y (run05)", R["board_y"], "o")):
        z = r["zero_return"]
        i = np.arange(1, len(z) + 1)
        ax.plot(i, [x["camera_deg"] for x in z], mk + "-", ms=6, label=f"{lbl}: camera")
        ax.plot(i, [x["gravity_tilt_deg"] for x in z], mk + "--", ms=6, mfc="none",
                label=f"{lbl}: gravity tilt")
    ax.axhline(0, color="k", lw=.8)
    ax.set_xticks([1, 2, 3]); ax.set_xlabel("visit to the commanded zero")
    ax.set_ylabel("measured attitude at commanded zero [deg]")
    ax.set_title("Two independent sensors observe the same departure from the\n"
                 "commanded reference: platform motion, not camera error", fontsize=9)
    ax.legend(fontsize=7); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "F7_zero_return_drift.pdf")); plt.close(fig)


def main():
    os.makedirs(FIG, exist_ok=True)
    R = _load()
    for f in (fig_sync, fig_static, fig_rate, fig_conditioning, fig_drift):
        f(R)
        print(f"   {f.__doc__.splitlines()[0]}")
    print(f"-> {os.path.relpath(FIG, cio.REPO_ROOT)}/")


if __name__ == "__main__":
    main()
