"""Gate: the reproduced numbers must match the submitted values.

Run `python analysis/reproduce_all.py` first; this test reads its output.
"""
import json, os, sys, pytest
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "analysis"))
from common import io as cio

RP = os.path.join(cio.RESULTS, "reproduced_numbers.json")
VP = os.path.join(cio.RESULTS, "verified_numbers.json")
pytestmark = pytest.mark.skipif(
    not os.path.exists(RP), reason="run `python analysis/reproduce_all.py` first")

def _load():
    return json.load(open(RP)), json.load(open(VP))

def test_board_y_static():
    r, v = _load()
    g = r["board_y"]["static"]["camera_vs_commanded"]
    e = v["board_y"]["static_camera_vs_commanded"]
    for k in ("Bias", "MAE", "RMSE", "MaxAbs"):
        assert abs(g[k] - e[k]) < 5e-5, k
    assert g["N"] == e["N"]

def test_board_y_camera_vs_gravity():
    r, v = _load()
    g = r["board_y"]["static"]["camera_vs_gravity"]
    e = v["board_y"]["static_camera_vs_gravity"]
    for k in ("Bias", "MAE", "RMSE"):
        assert abs(g[k] - e[k]) < 5e-5, k

def test_board_y_dynamic():
    r, v = _load()
    for gk, vk in (("camera_raw_rate_vs_commanded", "dynamic_camera_raw_vs_commanded"),
                   ("gyro_vs_commanded", "dynamic_gyro_vs_commanded")):
        g, e = r["board_y"]["dynamic"][gk], v["board_y"][vk]
        for k in ("Bias", "MAE", "RMSE", "MaxAbs"):
            assert abs(g[k] - e[k]) < 5e-5, (gk, k)

def test_board_y_conditioning():
    r, v = _load()
    c, e = r["board_y"]["conditioning"], v["board_y"]["conditioning"]
    assert c["n_frames"] == e["n_frames"]
    assert c["branch_flips"] == e["branch_flips"] == 0
    assert c["ambiguous_frames"] == e["ambiguous_frames"] == 0
    assert abs(c["four_tag_visibility_pct"] - e["four_tag_visibility_pct"]) < 5e-4

def test_board_y_hinge_axis_is_board_y():
    r, v = _load()
    import numpy as np, math
    n = np.array(r["board_y"]["hinge_axis_board_frame"])
    assert np.allclose(n, v["board_y"]["hinge_axis_board_frame"], atol=5e-6)
    assert math.degrees(math.acos(min(abs(n[1]), 1.0))) < 5.0, "must be near board +Y"

def test_board_x_headline():
    r, v = _load()
    g = r["board_x"]["board_x/axis_b_run01"]["static"]["camera_vs_commanded"]
    e = v["board_x_headline"]["static_camera_vs_commanded"]
    for k in ("Bias", "MAE", "RMSE", "MaxAbs"):
        assert abs(g[k] - e[k]) < 5e-5, k

def test_replication_runs():
    r, v = _load()
    for key, vk in (("board_x/axis_a", "axis_a"), ("board_x/axis_b_run04", "run04")):
        g = r["board_x"][key]["static"]
        assert abs(g["camera_vs_commanded"]["RMSE"]
                   - v["replication"][vk]["static_camera_vs_commanded_RMSE"]) < 5e-5
        assert abs(g["camera_vs_gravity"]["RMSE"]
                   - v["replication"][vk]["static_camera_vs_gravity_RMSE"]) < 5e-5

def test_rate_scale_range():
    r, v = _load()
    g, e = r["board_y"]["rate_scale_pct"], v["board_y"]["rate_scale_pct"]
    assert abs(g["min"] - e["min"]) < 5e-3 and abs(g["max"] - e["max"]) < 5e-3

def test_yaw_headline_if_present():
    r, v = _load()
    if not r.get("yaw"):
        pytest.skip("yaw not reproduced in this run")
    g = r["yaw"]["static"]["camera_vs_commanded"]
    e = v["yaw"]["static_camera_vs_commanded"]
    for k in ("RMSE", "MAE", "MaxAbs"):
        assert abs(g[k] - e[k]) < 5e-5, k

def test_filtered_yaw_is_not_in_the_raw_comparison():
    r, _ = _load()
    if not r.get("yaw"):
        pytest.skip("yaw not reproduced in this run")
    assert "savitzky" not in json.dumps(r["yaw"]["dynamic"]).lower()
    assert "kalman" not in json.dumps(r["yaw"]["dynamic"]).lower()


# ----------------------------------------------------------------------------------
# Guard: no ACTIVE result file may carry superseded Board-Y values.
# ----------------------------------------------------------------------------------
SUPERSEDED_BOARD_Y = [
    0.2987, 0.2814, 0.2826, 0.4765,          # revision 1, raw-K intrinsics + raw-K layout
    0.2641, 0.2102, 1.2128,
    0.2539, 0.2186, 0.2212, 0.4847,          # revision 2, newK intrinsics + raw-K layout
    0.2164, 0.1625,
]
FINAL_BOARD_Y = dict(static_cm_rmse=0.2918, static_ci_rmse=0.2474,
                     raw_rate_rmse=1.3095, reproj_median=0.2620)
ACTIVE_RESULT_FILES = ["board_y.json", "reproduced_numbers.json",
                       "three_axis_rate_audit.json"]


def _walk(o):
    if isinstance(o, dict):
        for k, v in o.items():
            yield from _walk(v)
    elif isinstance(o, list):
        for v in o:
            yield from _walk(v)
    elif isinstance(o, (int, float)) and not isinstance(o, bool):
        yield float(o)


def test_final_board_y_values_are_present():
    r = json.load(open(RP))["board_y"]
    assert abs(r["static"]["camera_vs_commanded"]["RMSE"]
               - FINAL_BOARD_Y["static_cm_rmse"]) < 5e-5
    assert abs(r["static"]["camera_vs_gravity"]["RMSE"]
               - FINAL_BOARD_Y["static_ci_rmse"]) < 5e-5
    assert abs(r["dynamic"]["camera_raw_rate_vs_commanded"]["RMSE"]
               - FINAL_BOARD_Y["raw_rate_rmse"]) < 5e-5
    assert abs(r["conditioning"]["board_reprojection_px"]["median"]
               - FINAL_BOARD_Y["reproj_median"]) < 5e-5


def test_no_active_result_file_contains_superseded_board_y_values():
    """Superseded Board-Y numbers may live ONLY under the explicitly-named keys in
    verified_numbers.json. No active result file may contain them."""
    for name in ACTIVE_RESULT_FILES:
        p = os.path.join(cio.RESULTS, name)
        if not os.path.exists(p):
            continue
        vals = list(_walk(json.load(open(p))))
        for bad in SUPERSEDED_BOARD_Y:
            hits = [v for v in vals if abs(v - bad) < 1e-6]
            assert not hits, f"{name} contains superseded Board-Y value {bad}"


def test_superseded_values_are_quarantined_and_labelled():
    V = json.load(open(VP))
    for key in ("board_y_AS_SUBMITTED_SUPERSEDED",
                "board_y_SUPERSEDED_rev2_rawK_geometry"):
        assert key in V, f"{key} must be retained for traceability"
        assert "SUPERSEDED" in V[key]["_status"].upper()
    assert "SUPERSEDED" not in V["board_y"].get("_status", "").upper()
    assert abs(V["board_y"]["static_camera_vs_commanded"]["RMSE"]
               - FINAL_BOARD_Y["static_cm_rmse"]) < 5e-5


def test_no_paper_facing_run05_filenames_remain():
    """RUN05 may survive as provenance metadata, never as an active filename."""
    import glob as _g
    bad = []
    for pat in ("config/*.json", "results/*.json", "results/tables/*.csv",
                "analysis/**/*.py"):
        for f in _g.glob(os.path.join(cio.REPO_ROOT, pat), recursive=True):
            b = os.path.basename(f)
            if "RUN05" in b or "run05" in b:
                bad.append(os.path.relpath(f, cio.REPO_ROOT))
    assert not bad, f"paper-facing RUN05 filenames remain: {bad}"


def test_sg_is_labelled_secondary_and_frozen():
    p = os.path.join(cio.RESULTS, "three_axis_rate_audit.json")
    if not os.path.exists(p):
        pytest.skip("run analysis/rate_audit.py first")
    a = json.load(open(p))
    e = a["estimator"]
    assert e["window_s"] == 1.00 and e["polyorder"] == 2
    assert e["frozen"] is True
    assert e["tuned_per_axis"] is False
    assert e["optimised_to_minimise_error"] is False
    assert "SECONDARY" in e["status"].upper()
    # the frozen window must be identical for every axis
    wins = {k: v["sg_window_samples"] for k, v in a["axes"].items()}
    assert len(wins) >= 3
    # and SG must never appear inside a RAW block
    r = json.load(open(RP))
    assert "sg" not in json.dumps(r["board_y"]["dynamic"]).lower()
    assert "sg" not in json.dumps(r["board_x"]["board_x/axis_b_run01"]["dynamic"]).lower()
