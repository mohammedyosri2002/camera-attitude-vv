"""
analysis/common/camera_motor_sync.py
====================================
CAMERA <-> MOTOR clock alignment. CREATED POST-CAMPAIGN.

The camera host and the motor controller free-run on separate oscillators. The affine map

    t_motor = a * t_camera + b

is fitted by ordinary least squares to the SERIAL MOTOR-EVENT TIMESTAMPS ONLY: each
profile transition is stamped with the controller's micros() and with the host's
perf_counter_ns() on receipt.

CRITICAL: this map is NEVER optimised against camera-versus-motor attitude error. The
camera-motor agreement reported in the paper is therefore OUT OF SAMPLE with respect to
the synchronisation.

SIGN CONVENTION. The reported ppm is (a - 1) * 1e6 for the map above, i.e. the fractional
rate of the controller clock expressed in the host time base. A negative value means the
controller's micros() advances slightly slower per host second under this map. Do not
paraphrase it as "the Arduino is N ppm fast" without restating the map; report the
coefficient and the signed ppm together.
"""
from __future__ import annotations
import numpy as np


def fit_from_events(ev):
    """Refit the affine map from the archived event timestamps.

    Returns a dict with a, b, ppm, residual SD (ms), event count and span.
    """
    t_ard = ev["t_arduino_us"].to_numpy(float) / 1e6
    t_perf = ev["t_perf_ns"].to_numpy(float) / 1e9
    t_wall = ev["t_wall_ns"].to_numpy(float) / 1e9

    def _fit(x, y):
        x0, y0 = x[0], y[0]
        xs, ys = x - x0, y - y0
        a, b = np.polyfit(xs, ys, 1)
        r = ys - (a * xs + b)
        return float(a), float(b), float(r.std(ddof=1)) if len(r) > 1 else float("nan")

    a_p, b_p, sd_p = _fit(t_perf, t_ard)
    a_w, b_w, sd_w = _fit(t_wall, t_ard)
    return dict(
        map="t_motor = a * t_camera_host + b   (Arduino micros in host perf_counter base)",
        a=a_p, b=b_p, ppm=(a_p - 1.0) * 1e6, residual_sd_ms=sd_p * 1000.0,
        a_wallclock=a_w, ppm_wallclock=(a_w - 1.0) * 1e6,
        residual_sd_ms_wallclock=sd_w * 1000.0,
        n_events=int(len(ev)), span_s=float(t_perf[-1] - t_perf[0]),
        t_perf_first_ns=float(ev["t_perf_ns"].iloc[0]),
        t_arduino_first_us=float(ev["t_arduino_us"].iloc[0]),
        fitted_to="serial motor events only; never to attitude residuals")


def camera_times_to_motor_clock(cam, fit):
    """Map camera frame timestamps into the Arduino clock using the event-derived map."""
    a = float(fit["a"]); b = float(fit["b"])
    p0 = float(fit["t_perf_first_ns"]) / 1e9
    a0 = float(fit["t_arduino_first_us"]) / 1e6
    return a * (cam["pc_perf_counter_ns"].to_numpy(float) / 1e9 - p0) + b + a0
