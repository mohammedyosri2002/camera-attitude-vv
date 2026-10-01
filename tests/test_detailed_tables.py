"""Detailed pairwise result tables: existence, structure and headline reproduction.

Run `python analysis/reproduce_all.py` then
`python analysis/export_detailed_tables.py` first.
"""
import os
import sys

import pandas as pd
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                "analysis"))
from common import io as cio                                        # noqa: E402

T = os.path.join(cio.RESULTS, "tables")
AXES = ["yaw", "board_x", "board_y"]
OOP = ["board_x", "board_y"]

EXPECTED = {   # frozen headline values; this test must fail if any of them moves
    "yaw": dict(static=0.2206, raw_cm=5.2471, raw_gm=3.8335, raw_cg=3.6943, sg=0.0421),
    "board_x": dict(static=0.2678, cam_grav=0.2894, raw_cm=1.2512, raw_gm=0.8535,
                    raw_cg=1.5362, sg=0.1196),
    "board_y": dict(static=0.2918, cam_grav=0.2474, raw_cm=1.3095, raw_gm=0.8759,
                    raw_cg=1.2735, sg=0.1241),
}
RATES = {"yaw": [1, 5, 10, 15, 20, 25, 30], "board_x": [1, 2, 5, 10],
         "board_y": [1, 2, 5, 10]}
YAW_STATIC_N = 48369
TOL = 5e-5

FILES = ([f"static_per_hold_{a}.csv" for a in AXES]
         + [f"static_pooled_by_command_{a}.csv" for a in AXES]
         + [f"dynamic_per_leg_{a}.csv" for a in AXES]
         + [f"dynamic_pooled_by_rate_{a}.csv" for a in AXES]
         + [f"dynamic_sg_SECONDARY_{a}.csv" for a in AXES]
         + ["dynamic_displacement_rate_scale_by_rate.csv", "result_metrics_long.csv"])

pytestmark = pytest.mark.skipif(
    not os.path.exists(os.path.join(T, "result_metrics_long.csv")),
    reason="run `python analysis/export_detailed_tables.py` first")


def rd(name):
    return pd.read_csv(os.path.join(T, name), comment="#")


def raw(name):
    return open(os.path.join(T, name)).read()


def total(df, col):
    m = df[df.iloc[:, 0].astype(str) == "POOLED_TOTAL"]
    assert len(m) == 1, "exactly one POOLED_TOTAL row expected"
    return float(m.iloc[0][col])


# ---- 1. every table exists -----------------------------------------------------
@pytest.mark.parametrize("name", FILES)
def test_table_exists_and_is_non_empty(name):
    p = os.path.join(T, name)
    assert os.path.exists(p), f"missing table {name}"
    assert len(rd(name)) > 0


# ---- 2. expected rate sets ------------------------------------------------------
@pytest.mark.parametrize("axis", AXES)
def test_expected_rate_set_present(axis):
    d = rd(f"dynamic_pooled_by_rate_{axis}.csv")
    got = sorted(round(float(x)) for x in d["commanded_rate_magnitude_dps"]
                 if str(x) != "POOLED_TOTAL")
    assert got == RATES[axis], (axis, got)


# ---- 3. sample-level pooled totals reproduce the frozen headline values ---------
@pytest.mark.parametrize("axis", AXES)
def test_pooled_totals_reproduce_headline(axis):
    e = EXPECTED[axis]
    assert abs(total(rd(f"static_pooled_by_command_{axis}.csv"),
                     "Camera_Cmd_RMSE") - e["static"]) < TOL
    if "cam_grav" in e:
        assert abs(total(rd(f"static_pooled_by_command_{axis}.csv"),
                         "Camera_Gravity_RMSE") - e["cam_grav"]) < TOL
    d = rd(f"dynamic_pooled_by_rate_{axis}.csv")
    assert abs(total(d, "Camera_Cmd_RMSE") - e["raw_cm"]) < TOL
    assert abs(total(d, "Gyro_Cmd_RMSE") - e["raw_gm"]) < TOL
    assert abs(total(d, "Camera_Gyro_RMSE") - e["raw_cg"]) < TOL
    assert abs(total(rd(f"dynamic_sg_SECONDARY_{axis}.csv"),
                     "SG_Camera_Cmd_RMSE") - e["sg"]) < TOL


def test_yaw_sg_is_the_frozen_estimator_not_the_legacy_value():
    """Guards against regressing to the legacy yaw-only SG (~0.0352 deg/s)."""
    v = total(rd("dynamic_sg_SECONDARY_yaw.csv"), "SG_Camera_Cmd_RMSE")
    assert abs(v - 0.0421) < TOL
    assert abs(v - 0.0352) > 1e-3


def test_yaw_static_pooled_n_is_preserved():
    d = rd("static_pooled_by_command_yaw.csv")
    m = d[d["commanded_reference_deg"].astype(str) == "POOLED_TOTAL"].iloc[0]
    assert int(m["Camera_Cmd_N"]) == YAW_STATIC_N


# ---- 4. no pooled signed-direction bias anywhere ---------------------------------
@pytest.mark.parametrize("axis", AXES)
def test_no_direction_cancelling_pooled_bias(axis):
    for f in (f"dynamic_pooled_by_rate_{axis}.csv",
              f"dynamic_sg_SECONDARY_{axis}.csv"):
        bad = [c for c in rd(f).columns if c.endswith("_Bias")]
        assert not bad, f"{f} exports a direction-pooled signed bias: {bad}"


@pytest.mark.parametrize("axis", OOP)
def test_direction_separated_bias_present(axis):
    c = rd(f"dynamic_pooled_by_rate_{axis}.csv").columns
    for p in ("Camera_Cmd", "Gyro_Cmd", "Camera_Gyro"):
        assert f"{p}_Bias_Pos" in c and f"{p}_Bias_Neg" in c


def test_yaw_bias_is_by_run_not_by_leg_sign():
    c = rd("dynamic_pooled_by_rate_yaw.csv").columns
    for p in ("Camera_Cmd", "Gyro_Cmd", "Camera_Gyro"):
        assert f"{p}_Bias_CW" in c and f"{p}_Bias_CCW" in c
        assert f"{p}_Bias_Pos" not in c and f"{p}_Bias_Neg" not in c


# ---- 5. yaw static has no gravity comparison ------------------------------------
@pytest.mark.parametrize("name", ["static_per_hold_yaw.csv",
                                  "static_pooled_by_command_yaw.csv"])
def test_yaw_static_has_no_gravity_comparison(name):
    cols = rd(name).columns
    assert not [c for c in cols if "Gravity" in c or "gravity" in c], \
        "yaw logged no roll/pitch; a gravity-tilt comparison must not be invented"
    long = rd("result_metrics_long.csv")
    ys = long[(long["axis"] == "yaw") & (long["processing_level"] == "STATIC")]
    assert set(ys["comparison"]) == {"camera_commanded"}


# ---- 6. SG tables are marked SECONDARY and kept out of the RAW tables ------------
@pytest.mark.parametrize("axis", AXES)
def test_sg_tables_marked_secondary_and_separate(axis):
    h = raw(f"dynamic_sg_SECONDARY_{axis}.csv")
    assert "SECONDARY" in h
    assert "1.00 s window" in h and "order 2" in h
    assert "NOT RAW camera-rate accuracy" in h
    assert "Not tuned per axis" in h
    assert "Not optimized against an error metric" in h
    for f in (f"dynamic_per_leg_{axis}.csv", f"dynamic_pooled_by_rate_{axis}.csv"):
        assert not [c for c in rd(f).columns if c.upper().startswith("SG")], \
            f"SG columns must not appear in the RAW table {f}"


# ---- 7. camera-gyro exists for dynamic RAW --------------------------------------
@pytest.mark.parametrize("axis", AXES)
def test_camera_gyro_present_in_raw_dynamic(axis):
    for f in (f"dynamic_per_leg_{axis}.csv", f"dynamic_pooled_by_rate_{axis}.csv"):
        c = rd(f).columns
        assert "Camera_Gyro_RMSE" in c and "Camera_Gyro_N" in c


# ---- 8. N is per comparison, non-null, and grids are declared --------------------
@pytest.mark.parametrize("axis", OOP)
def test_n_is_per_comparison_and_grids_declared(axis):
    d = rd(f"static_pooled_by_command_{axis}.csv")
    for c in ("Camera_Cmd_N", "Gravity_Cmd_N", "Camera_Gravity_N"):
        assert c in d.columns and d[c].notna().all() and (d[c] > 0).all()
    t = d[d["commanded_reference_deg"].astype(str) == "POOLED_TOTAL"].iloc[0]
    assert int(t["Camera_Cmd_N"]) != int(t["Gravity_Cmd_N"]), \
        "camera and IMU grids differ; identical N would indicate a resampling error"
    for c in ("Camera_Cmd_grid", "Gravity_Cmd_grid", "Camera_Gravity_grid"):
        assert c in d.columns and d[c].notna().all()


def test_per_hold_omits_redundant_mae_rmse():
    for axis in AXES:
        c = rd(f"static_per_hold_{axis}.csv").columns
        assert not [x for x in c if "MAE" in x or "RMSE" in x]
        assert any(x.endswith("_SD") for x in c)


def test_per_hold_preserves_approach_direction():
    for axis in OOP:
        d = rd(f"static_per_hold_{axis}.csv")
        assert "approach_direction" in d.columns
        assert set(d["approach_direction"]) <= {"initial", "ascending", "descending",
                                                "repeat"}
        assert d["hold_index"].tolist() == sorted(d["hold_index"].tolist())


def test_long_form_is_complete_and_labelled():
    d = rd("result_metrics_long.csv")
    assert set(d["processing_level"]) == {"STATIC", "RAW_DYNAMIC", "SG_SECONDARY",
                                          "DISPLACEMENT_SCALE"}
    assert set(d["axis"]) == {"yaw", "board_x", "board_y"}
    assert d["evaluation_grid"].notna().all()
    assert {"camera_commanded", "gravity_commanded", "camera_gravity",
            "gyro_commanded", "camera_gyro"} <= set(d["comparison"])


# ---- 9. reference terminology ----------------------------------------------------
DISCLAIMER = ("motor quantity = pulse-derived commanded reference. not ground truth, "
              "not an actual angle, not a measured motor angle. no encoder-position "
              "telemetry was logged.")


@pytest.mark.parametrize("name", FILES)
def test_no_forbidden_reference_terminology(name):
    """Forbidden terms may appear ONLY inside the standard negating disclaimer."""
    t = raw(name).lower()
    assert DISCLAIMER in t, f"{name} is missing the commanded-reference disclaimer"
    rest_of_file = t.replace(DISCLAIMER, "")
    for bad in ("ground truth", "actual angle", "measured motor angle", "encoder"):
        assert bad not in rest_of_file, \
            f"{name} uses forbidden term '{bad}' outside the disclaimer"
