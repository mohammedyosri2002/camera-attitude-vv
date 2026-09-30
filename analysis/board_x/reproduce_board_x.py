#!/usr/bin/env python3
"""
analysis/board_x/reproduce_board_x.py
=====================================
BOARD-X runs. CREATED POST-CAMPAIGN.

Three runs share the Board-X hinge axis to within about 1 deg despite different camera-view
and IMU-mounting configurations. Original folder labels (AXIS_A, AXIS_B run01, AXIS_B
run04) are retained for provenance only; they do NOT denote different spacecraft axes.

  axis_b_run01  HEADLINE Board-X result
  axis_a        replication
  axis_b_run04  replication (IMU re-mounted; gyro channel X)

All three use config/board_geometry_boardX.json. The RUN05 layout must NEVER be applied
to them -- select_geometry() enforces this.

Usage (from the repository root):
    python analysis/board_x/reproduce_board_x.py
"""
import json, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common import io as cio                       # noqa: E402
from common.out_of_plane import process            # noqa: E402

KEYS = ["board_x/axis_b_run01", "board_x/axis_a", "board_x/axis_b_run04"]

def main():
    out = {}
    for k in KEYS:
        res = process(k)
        s, d = res["static"], res["dynamic"]
        tag = "HEADLINE" if cio.run_info(k)["headline_run"] else "replication"
        print(f"  [{tag}] camera vs commanded RMSE {s['camera_vs_commanded']['RMSE']:.4f} deg | "
              f"camera vs gravity RMSE {s['camera_vs_gravity']['RMSE']:.4f} deg | "
              f"camera RAW rate RMSE {d['camera_raw_rate_vs_commanded']['RMSE']:.4f} deg/s | "
              f"gyro RMSE {d['gyro_vs_commanded']['RMSE']:.4f} deg/s")
        out[k] = res
    os.makedirs(cio.RESULTS, exist_ok=True)
    p = os.path.join(cio.RESULTS, "board_x.json")
    json.dump(out, open(p, "w"), indent=2, default=float)
    print(f"  -> {os.path.relpath(p, cio.REPO_ROOT)}")
    return out

if __name__ == "__main__":
    main()
