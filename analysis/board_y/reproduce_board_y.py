#!/usr/bin/env python3
"""
analysis/board_y/reproduce_board_y.py
===================================
BOARD-Y headline run (original label RUN05). CREATED POST-CAMPAIGN.

RUN05 is the genuine SECOND out-of-plane board axis: the tag board was physically
re-mounted ~87 deg relative to the hinge. It was ACQUIRED with the stale Board-X layout
and is REPROCESSED here from the stored raw AprilTag corner pixels using
config/board_geometry_boardY_bundle_adjusted.json.

Usage (from the repository root):
    python analysis/board_y/reproduce_board_y.py
"""
import json, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common import io as cio                       # noqa: E402
from common.out_of_plane import process            # noqa: E402

KEY = "board_y/run05"

def main():
    res = process(KEY)
    s, d, c = res["static"], res["dynamic"], res["conditioning"]
    print(f"\n  STATIC  camera vs commanded reference : "
          f"RMSE {s['camera_vs_commanded']['RMSE']:.4f}  "
          f"MAE {s['camera_vs_commanded']['MAE']:.4f}  "
          f"Bias {s['camera_vs_commanded']['Bias']:.4f}  N {s['camera_vs_commanded']['N']}")
    print(f"  STATIC  gravity tilt vs commanded     : "
          f"RMSE {s['gravity_vs_commanded']['RMSE']:.4f}")
    print(f"  STATIC  camera vs gravity tilt        : "
          f"RMSE {s['camera_vs_gravity']['RMSE']:.4f}  "
          f"Bias {s['camera_vs_gravity']['Bias']:.4f}")
    print(f"  DYNAMIC camera RAW rate vs commanded  : "
          f"RMSE {d['camera_raw_rate_vs_commanded']['RMSE']:.4f} deg/s")
    print(f"  DYNAMIC gyro vs commanded             : "
          f"RMSE {d['gyro_vs_commanded']['RMSE']:.4f} deg/s")
    print(f"  RATE SCALE (displacement/leg)         : "
          f"{res['rate_scale_pct']['min']:+.2f} % to {res['rate_scale_pct']['max']:+.2f} %")
    print(f"  CONDITIONING: valid {c['valid_pose_pct']:.4f} %  flips {c['branch_flips']}  "
          f"ambiguous {c['ambiguous_frames']}  4-tag {c['four_tag_visibility_pct']:.4f} %")
    out = os.path.join(cio.RESULTS, "board_y.json")
    os.makedirs(cio.RESULTS, exist_ok=True)
    json.dump(res, open(out, "w"), indent=2, default=float)
    print(f"  -> {os.path.relpath(out, cio.REPO_ROOT)}")
    return res

if __name__ == "__main__":
    main()
