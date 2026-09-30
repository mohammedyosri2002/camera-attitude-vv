#!/usr/bin/env python3
"""
analysis/yaw/reproduce_yaw.py
=============================
YAW (in-plane) reproduction. CREATED POST-CAMPAIGN.

Two directional runs (CW, CCW) pooled at sample level. The headline numbers are RAW:
camera vs the pulse-derived commanded reference. Savitzky-Golay and Kalman results are
computed but retained only as clearly labelled SECONDARY DIAGNOSTICS.

Usage (from the repository root):
    python analysis/yaw/reproduce_yaw.py
"""
import importlib.util, json, os, sys, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common import io as cio                     # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def _load_master():
    spec = importlib.util.spec_from_file_location(
        "yaw_master", os.path.join(HERE, "yaw_vv_master_analysis.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def main():
    import pandas as pd
    M = _load_master()
    r1 = os.path.join(cio.DATA_RAW, "yaw", "run01_CW")
    r2 = os.path.join(cio.DATA_RAW, "yaw", "run02_CCW")
    tmp = tempfile.mkdtemp(prefix="yaw_repro_")
    print("\n=== yaw (in-plane), runs run01_CW + run02_CCW ===")
    sys.argv = ["yaw", "--run1", r1, "--run2", r2, "--out", tmp]
    M.main()

    # export the synchronized per-sample camera/motor series so the frozen SG estimator
    # can be applied to yaw over the SAME constant-rate leg windows as the RAW result
    import shutil, glob as _g
    for f in _g.glob(os.path.join(tmp, "synchronized_samples_run*.csv")):
        shutil.copy(f, os.path.join(cio.RESULTS, "yaw_" + os.path.basename(f)))

    xl = os.path.join(tmp, "Yaw_VV_MASTER_FULL_RESULTS.xlsx")
    A = pd.read_excel(xl, sheet_name="08_ANGLES_COMBINED_MASTER", header=2)
    R = pd.read_excel(xl, sheet_name="11_RATES_COMBINED_MASTER", header=2)
    CW = pd.read_excel(xl, sheet_name="06_ANGLES_CW_FULL", header=2)
    CC = pd.read_excel(xl, sheet_name="07_ANGLES_CCW_FULL", header=2)
    S = pd.read_excel(xl, sheet_name="02_SYNC_FULL", header=2)
    a, q = A.iloc[-1], R.iloc[-1]
    mx = max(CW[CW["Requested_Angle_deg"].astype(str) != "POOLED_TOTAL"]
             ["RAW_Camera_Motor_MaxAbs_deg"].max(),
             CC[CC["Requested_Angle_deg"].astype(str) != "POOLED_TOTAL"]
             ["RAW_Camera_Motor_MaxAbs_deg"].max())

    res = dict(
        run_key="yaw", canonical_axis="yaw",
        original_run_label="result_yaw/test1 + result_yaw/test2",
        pose_source="acquisition_logged",
        scalar_attitude="fused_in_plane_yaw (per-tag fusion; pre-dates the rigid-board estimator)",
        static=dict(camera_vs_commanded=dict(
            Bias=float(a["Combined_RAW_CM_Bias_deg"]),
            MAE=float(a["Combined_RAW_CM_MAE_deg"]),
            RMSE=float(a["Combined_RAW_CM_RMSE_deg"]),
            MaxAbs=float(mx), N=int(a["Combined_RAW_CM_N"])
            if "Combined_RAW_CM_N" in a else None),
            gravity_vs_commanded=None, camera_vs_gravity=None,
            note="yaw acquisition logged no roll/pitch: no gravity-tilt reference exists"),
        dynamic=dict(
            camera_raw_rate_vs_commanded=dict(
                Bias=float(q["COMBINED_RAW_CM_Bias"]), MAE=float(q["COMBINED_RAW_CM_MAE"]),
                RMSE=float(q["COMBINED_RAW_CM_RMSE"]), MaxAbs=float(q["COMBINED_RAW_CM_MaxAbs"])),
            gyro_vs_commanded=dict(
                Bias=float(q["COMBINED_IMU_M_Bias"]), MAE=float(q["COMBINED_IMU_M_MAE"]),
                RMSE=float(q["COMBINED_IMU_M_RMSE"]), MaxAbs=float(q["COMBINED_IMU_M_MaxAbs"])),
            camera_raw_rate_vs_gyro=dict(
                RMSE=float(q["COMBINED_RAW_CI_RMSE"]), MAE=float(q["COMBINED_RAW_CI_MAE"]))),
        secondary_filtered_diagnostics_DO_NOT_COMPARE_WITH_RAW=dict(
            savitzky_golay_rate_RMSE_dps=float(q["COMBINED_SG_CM_RMSE"]),
            kalman_rate_RMSE_dps=float(q["COMBINED_KF_CM_RMSE"]),
            note=("Secondary only. Never substituted into the three-axis RAW comparison "
                  "and never compared with a RAW out-of-plane derivative.")),
        sync=S.to_dict("records"))
    os.makedirs(cio.RESULTS, exist_ok=True)
    p = os.path.join(cio.RESULTS, "yaw.json")
    json.dump(res, open(p, "w"), indent=2, default=float)
    s, d = res["static"]["camera_vs_commanded"], res["dynamic"]
    print(f"\n  STATIC  camera vs commanded reference : RMSE {s['RMSE']:.4f}  "
          f"MAE {s['MAE']:.4f}  MaxAbs {s['MaxAbs']:.4f}")
    print(f"  DYNAMIC camera RAW rate vs commanded  : "
          f"RMSE {d['camera_raw_rate_vs_commanded']['RMSE']:.4f}  "
          f"MAE {d['camera_raw_rate_vs_commanded']['MAE']:.4f} deg/s")
    print(f"  DYNAMIC gyro vs commanded             : "
          f"RMSE {d['gyro_vs_commanded']['RMSE']:.4f} deg/s")
    print(f"  [secondary, not for the RAW table] SG {res['secondary_filtered_diagnostics_DO_NOT_COMPARE_WITH_RAW']['savitzky_golay_rate_RMSE_dps']:.4f} deg/s")
    print(f"  -> {os.path.relpath(p, cio.REPO_ROOT)}")
    return res


if __name__ == "__main__":
    main()
