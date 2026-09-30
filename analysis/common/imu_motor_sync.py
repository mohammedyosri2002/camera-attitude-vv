"""
analysis/common/imu_motor_sync.py
=================================
IMU <-> MOTOR clock alignment, fitted in the GYRO RATE DOMAIN. CREATED POST-CAMPAIGN.

The camera is NEVER used to define this map. The objective matches the smoothed magnitude
of the dominant gyro channel against the |commanded rate| profile; the smoothing exists
only inside this objective and touches no reported measurement.

WHY THE COVERAGE CONSTRAINT EXISTS
----------------------------------
The commanded pre-position (PREBIAS) workflow introduces a long and variable delay between
controller reset and the start of the profile, which moves the IMU-to-motor offset far from
any naive search window. Without a constraint the optimiser can select a PARTIAL-OVERLAP
minimum in which only a stationary tail of the IMU record overlaps a stationary part of the
motor profile: the residual is tiny and the solution is nonsense. This was observed in
practice as a physically implausible a = 1.033994589 (+33 994.6 ppm).

The fit is therefore restricted to solutions whose mapped IMU record covers at least
MIN_COVERAGE of the motor experiment window. With the constraint the same data give
a = 0.998395237 (-1604.8 ppm) for RUN05 at 100 % coverage.
"""
from __future__ import annotations
import numpy as np
from scipy.optimize import minimize
from scipy.signal import savgol_filter

MIN_COVERAGE = 0.95        # >= 95 % of the motor window must be covered
SMOOTH_S = 0.30            # objective-internal only; never applied to a reported signal
COARSE_STEP_S = 0.25
FINE_STEP_S = 0.005


def fit(imu, motion, T0, T1, dominant_axis=None):
    ti = imu["teensy_time_us"].to_numpy(float) * 1e-6
    G = imu[["gyro_x_dps", "gyro_y_dps", "gyro_z_dps"]].to_numpy(float)

    if dominant_axis is None:
        dev = [np.percentile(np.abs(G[:, i] - np.median(G[:, i])), 99.9) for i in range(3)]
        dom = int(np.argmax(dev))
    else:
        dom = "xyz".index(dominant_axis.lower())

    fs = 1.0 / np.median(np.diff(ti))
    w = int(round(SMOOTH_S * fs)) | 1
    gs = savgol_filter(np.abs(G[:, dom] - np.median(G[:, dom])), w, 1)

    span = T1 - T0

    def ref(t):
        t = np.atleast_1d(np.asarray(t, float)); o = np.zeros_like(t)
        for t0, t1, r in motion:
            o[(t >= t0) & (t < t1)] = r
        return o

    def cost(p):
        a, b = p
        t = a * ti + b
        m = (t >= T0) & (t <= T1)
        if m.sum() < 1000:
            return 1e12
        if (t[m].max() - t[m].min()) / span < MIN_COVERAGE:   # <-- the constraint
            return 1e12
        return float(np.mean((gs[m] - ref(t[m])) ** 2))

    cands = [(cost((1.0, b0)), b0)
             for b0 in np.arange(T0 - ti[-1], T1 - ti[0], COARSE_STEP_S)]
    cands = [c for c in cands if c[0] < 1e11]
    if not cands:
        raise RuntimeError("no offset satisfies the coverage constraint")
    b_coarse = min(cands)[1]

    grid = b_coarse + np.arange(-0.5, 0.5, FINE_STEP_S)
    b_fine = float(grid[int(np.argmin([cost((1.0, z)) for z in grid]))])
    r1 = minimize(lambda p: cost((1.0, p[0])), [b_fine], method="Nelder-Mead",
                  options=dict(xatol=1e-7, fatol=1e-12))
    r2 = minimize(cost, [1.0, r1.x[0]], method="Nelder-Mead",
                  options=dict(xatol=1e-11, fatol=1e-14, maxiter=40000, maxfev=40000))
    a, b = float(r2.x[0]), float(r2.x[1])
    t = a * ti + b
    m = (t >= T0) & (t <= T1)
    cov = 100.0 * (t[m].max() - t[m].min()) / span
    return dict(map="t_motor = a * t_imu + b", a=a, b=b, ppm=(a - 1.0) * 1e6,
                residual_dps_rms=float(np.sqrt(r2.fun)),
                offset_only_b=float(r1.x[0]),
                offset_only_residual_dps_rms=float(np.sqrt(r1.fun)),
                coverage_pct=float(cov), min_coverage_pct=100.0 * MIN_COVERAGE,
                dominant_gyro_axis="XYZ"[dom],
                n_valid_offsets_searched=int(len(cands)),
                fitted_to="gyro rate domain vs commanded rate; camera never used")


def apply(imu, f):
    return float(f["a"]) * imu["teensy_time_us"].to_numpy(float) * 1e-6 + float(f["b"])
