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
    # PURE CSV: no comment= needed. If a "#" line ever reappears before the header this
    # read will produce a one-column frame and the structural tests below will fail.
    return pd.read_csv(os.path.join(T, name))


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


# ---- 6. SG stays SECONDARY, documented in README, kept out of the RAW tables ------
@pytest.mark.parametrize("axis", AXES)
def test_sg_tables_separate_and_marked(axis):
    # the filename itself carries SECONDARY, and the columns are SG_-prefixed
    assert "SECONDARY" in f"dynamic_sg_SECONDARY_{axis}.csv"
    assert [c for c in rd(f"dynamic_sg_SECONDARY_{axis}.csv").columns
            if c.startswith("SG_")]
    for f in (f"dynamic_per_leg_{axis}.csv", f"dynamic_pooled_by_rate_{axis}.csv"):
        assert not [c for c in rd(f).columns if c.upper().startswith("SG")], \
            f"SG columns must not appear in the RAW table {f}"


def test_sg_definition_documented_in_tables_readme():
    h = open(os.path.join(T, "README.md")).read()
    for frag in ("SECONDARY", "1.00 s window", "polynomial order 2", "first derivative",
                 "zero-phase", "not tuned per axis",
                 "NOT RAW camera-rate accuracy", "uniform-time resampling"):
        assert frag in h, f"results/tables/README.md is missing: {frag}"


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


# ---- 9. pure CSV, and reference terminology documented in the README ---------------
@pytest.mark.parametrize("name", FILES)
def test_csv_is_pure_and_renders_on_github(name):
    """First line must be the column header; no comment/prose lines; rectangular."""
    import csv as _csv
    rows = list(_csv.reader(open(os.path.join(T, name))))
    assert rows, f"{name} is empty"
    assert not rows[0][0].lstrip().startswith("#"), \
        f"{name} starts with a comment line; GitHub will not render it as a table"
    width = len(rows[0])
    assert width > 1, f"{name} header parsed as a single column"
    ragged = [i for i, r in enumerate(rows) if len(r) != width]
    assert not ragged, f"{name} has ragged rows at {ragged[:5]}"


def test_all_tables_dir_csvs_are_pure():
    """Covers every CSV in the directory, including those written by other modules."""
    import csv as _csv, glob as _g
    for p in sorted(_g.glob(os.path.join(T, "*.csv"))):
        rows = list(_csv.reader(open(p)))
        assert not rows[0][0].lstrip().startswith("#"), \
            f"{os.path.basename(p)} still has a comment line before the header"
        w_ = len(rows[0])
        assert not [i for i, r in enumerate(rows) if len(r) != w_], \
            f"{os.path.basename(p)} is ragged"


def test_reference_terminology_documented_in_tables_readme():
    h = open(os.path.join(T, "README.md")).read().lower()
    assert "pulse-derived commanded reference" in h
    for frag in ("not** ground truth", "no independent encoder-position telemetry"):
        assert frag.replace("**", "") in h.replace("**", ""), f"README missing: {frag}"


def test_no_forbidden_terminology_in_csv_payload(name=None):
    """No CSV may contain the forbidden terms at all now that prose has been removed."""
    import glob as _g
    for p in sorted(_g.glob(os.path.join(T, "*.csv"))):
        t = open(p).read().lower()
        for bad in ("ground truth", "actual angle", "measured motor angle", "encoder"):
            assert bad not in t, f"{os.path.basename(p)} contains '{bad}'"


# ---- 10. Markdown copies and navigation README -----------------------------------
MD_EXPECTED = ([f"static_per_hold_{a}" for a in AXES]
               + [f"static_pooled_by_command_{a}" for a in AXES]
               + [f"dynamic_per_leg_{a}" for a in AXES]
               + [f"dynamic_pooled_by_rate_{a}" for a in AXES]
               + [f"dynamic_sg_SECONDARY_{a}" for a in AXES]
               + ["dynamic_displacement_rate_scale_by_rate"])


@pytest.mark.parametrize("base", MD_EXPECTED)
def test_markdown_copy_exists_and_is_a_table(base):
    p = os.path.join(T, "markdown", f"{base}.md")
    assert os.path.exists(p), f"missing Markdown copy for {base}"
    body = open(p).read()
    assert "|---" in body, f"{base}.md has no Markdown table separator"
    assert body.lstrip().startswith("# "), f"{base}.md has no heading"


def test_long_form_has_no_markdown_copy():
    assert not os.path.exists(os.path.join(T, "markdown", "result_metrics_long.md")), \
        "result_metrics_long must stay machine-readable only"


def test_markdown_preserves_values():
    """Spot-check: Board-Y pooled RMSE must appear unaltered in the Markdown copy."""
    body = open(os.path.join(T, "markdown",
                             "static_pooled_by_command_board_y.md")).read()
    assert "0.291" in body


def test_tables_readme_navigation_links_every_table():
    h = open(os.path.join(T, "README.md")).read()
    for name in FILES:
        assert f"({name})" in h, f"results/tables/README.md does not link {name}"
    for base in MD_EXPECTED:
        assert f"markdown/{base}.md" in h, f"README does not link markdown/{base}.md"


def test_tables_readme_documents_grids_and_caveats():
    h = open(os.path.join(T, "README.md")).read()
    for frag in ("Evaluation grids", "N is reported per comparison",
                 "MaxAbs is a sample extreme", "identically |Bias|",
                 "direction-separated", "Displacement rate-scale definition",
                 "camera grid", "IMU grid"):
        assert frag in h, f"results/tables/README.md is missing: {frag}"
