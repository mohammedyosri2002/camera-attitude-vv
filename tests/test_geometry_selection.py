"""The RUN05 layout must never reach a Board-X run, and vice versa."""
import os, sys, pytest
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "analysis"))
from common import io as cio, geometry_utils as geo

BOARD_X = ["board_x/axis_a", "board_x/axis_b_run01", "board_x/axis_b_run04"]

def test_board_x_uses_board_x_layout():
    for k in BOARD_X:
        g, name = geo.select_geometry(k)
        assert name == "board_geometry_boardX.json"
        assert g["geometry_source"] == "camera_derived_planar_homography"

def test_board_y_uses_run05_layout():
    g, name = geo.select_geometry("board_y/run05")
    assert name == "board_geometry_boardY_bundle_adjusted.json"
    assert g["geometry_source"] == "camera_derived_bundle_adjustment_RUN05"

def test_layouts_are_physically_different_boards():
    a, _ = geo.select_geometry("board_x/axis_b_run01")
    b, _ = geo.select_geometry("board_y/run05")
    assert a["tags"] != b["tags"]

def test_cross_application_is_refused():
    info_path = os.path.join(cio.run_dir("board_x/axis_a"), "RUN_INFO.json")
    import json
    d = json.load(open(info_path))
    orig = d["geometry_file"]
    try:
        d["geometry_file"] = "board_geometry_boardY_bundle_adjusted.json"
        json.dump(d, open(info_path, "w"), indent=2)
        with pytest.raises(RuntimeError):
            geo.select_geometry("board_x/axis_a")
    finally:
        d["geometry_file"] = orig
        json.dump(d, open(info_path, "w"), indent=2)

def test_object_points_are_planar_and_correct_size():
    for k in BOARD_X[:1] + ["board_y/run05"]:
        g, _ = geo.select_geometry(k)
        obj = geo.build_object_points(g)
        size = g["tag_black_square_size_m"]
        import numpy as np
        for t, c in obj.items():
            assert np.all(c[:, 2] == 0.0)
            edges = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
            assert np.allclose(edges, size, atol=1e-9)

def test_geometry_provenance_is_declared_camera_derived():
    for name in ("board_geometry_boardX.json",
                 "board_geometry_boardY_bundle_adjusted.json"):
        g = cio.load_json(cio.config_path(name))
        assert "camera_derived" in g["geometry_source"], \
            "geometry must be declared camera-derived, never dimensional ground truth"


def test_run05_layout_was_refitted_with_undistorted_intrinsics():
    """The canonical RUN05 layout must be the newK refit, not the superseded raw-K fit."""
    g = cio.load_json(cio.config_path("board_geometry_boardY_bundle_adjusted.json"))
    assert g["geometry_source"] == "camera_derived_bundle_adjustment_RUN05"
    assert "SUPERSEDED" not in g["geometry_source"]
    sup = cio.load_json(cio.config_path(
        "board_geometry_boardY_bundle_adjusted_rawK_SUPERSEDED.json"))
    assert "SUPERSEDED" in sup["geometry_source"]
    assert g["tags"] != sup["tags"], "canonical layout must differ from the superseded fit"


def test_camera_intrinsics_returns_undistorted_matrix():
    """camera_intrinsics must return newK, not the raw camera_matrix."""
    import numpy as np
    for k in ("board_x/axis_b_run01", "board_y/run05"):
        K, D, meta = cio.camera_intrinsics(k)
        raw = np.array(meta["camera_matrix"], float)
        assert K[0, 0] < raw[0, 0] * 0.99, "must be the undistorted matrix"
        assert np.allclose(D, 0.0), "distortion already removed; coefficients must be zero"
