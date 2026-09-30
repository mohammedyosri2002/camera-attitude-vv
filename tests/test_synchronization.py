"""Clock alignment: event-only camera map, coverage-constrained IMU map."""
import os, sys, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "analysis"))
from common import io as cio, motor_reference as mref, camera_motor_sync as cms
from common import imu_motor_sync as ims

ALL = ["yaw/run01_CW", "yaw/run02_CCW", "board_x/axis_a", "board_x/axis_b_run01",
       "board_x/axis_b_run04", "board_y/run05"]

def test_camera_motor_fit_is_plausible_on_every_run():
    for k in ALL:
        ev, _ = cio.load_motor(k)
        f = cms.fit_from_events(ev)
        assert -2000 < f["ppm"] < -1000, (k, f["ppm"])
        assert f["residual_sd_ms"] < 5.0, (k, f["residual_sd_ms"])
        assert f["n_events"] in (94, 114), (k, f["n_events"])

def test_camera_motor_fit_uses_events_only():
    assert "never to attitude residuals" in \
        cms.fit_from_events(cio.load_motor("board_y/run05")[0])["fitted_to"]

def test_wallclock_and_monotonic_fits_agree():
    for k in ALL:
        f = cms.fit_from_events(cio.load_motor(k)[0])
        assert abs(f["ppm"] - f["ppm_wallclock"]) < 1.0, k

def test_imu_fit_meets_coverage_constraint():
    for k in ["board_x/axis_b_run01", "board_y/run05"]:
        ev, _ = cio.load_motor(k)
        _, _, motion, _, T0, T1 = mref.segment_profile(ev)
        imu, _ = cio.load_imu(k)
        f = ims.fit(imu, motion, T0, T1, cio.run_info(k)["gyro_channel"])
        assert f["coverage_pct"] >= 95.0, (k, f["coverage_pct"])
        assert -2500 < f["ppm"] < -500, (k, f["ppm"])   # rejects the +33995 ppm artefact
        assert f["residual_dps_rms"] < 1.0, (k, f["residual_dps_rms"])
        assert "camera never used" in f["fitted_to"]

def test_coverage_constraint_is_active():
    assert ims.MIN_COVERAGE >= 0.95

def test_event_completeness():
    for k in ALL:
        ev, _ = cio.load_motor(k)
        assert (ev["event"] == "RUN_END").sum() == 1, k
        t = ev["t_arduino_us"].to_numpy(float)
        assert np.all(np.diff(t) > 0), f"{k}: motor timestamps not monotonic"
