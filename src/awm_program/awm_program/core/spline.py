"""Time parameterisation of D09 trajectory segments (G1.5; DES-001 Rev 0.5 §4.3 step 5, A47, A62).

Every segment carries the positions, velocities and accelerations of **one clamped C2 cubic spline** through its
points (velocity 0 at both ends = the segment starts and ends at rest, DES-001 §6.2). A quintic interpolation from
(p, v, a) at both ends of an interval — what the ROS 2 joint trajectory controller does when all three are given —
reproduces that cubic exactly, so the limits checked here (:func:`max_abs`: acceleration at the knots, velocity at
the knots and at the interior extrema) are the limits of the commanded motion, not only of the stored points.

* :func:`weld_times` — times along the seam arc length s for a trapezoidal speed profile rest → travel speed →
  rest (ramp acceleration = parameter, Assumption). The constant-speed part runs exactly at the WPS travel speed
  relative to the part; the travel speed is never changed here.
* :func:`rest_to_rest` — air / approach / retract timing inside *scaled* joint limits (scaling ≤ 1, never above
  the limits); the duration is stretched until the spline is inside them.

Pure numpy; all values in the caller's units (degrees, mm, seconds).
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

from awm_program.timing import TimingError, constant_speed_times, trapezoid_times

__all__ = ['TimingError', 'clamped_cubic', 'evaluate', 'max_abs', 'weld_times', 'rest_to_rest']


def _check_t(t: np.ndarray) -> np.ndarray:
    t = np.asarray(t, dtype=float)
    if t.ndim != 1 or len(t) < 2 or np.any(np.diff(t) <= 0.0):
        raise TimingError('spline knots need at least 2 strictly increasing times')
    return t


def clamped_cubic(t: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Velocities and accelerations at the knots of the C2 cubic spline through ``y`` (n, m) at times ``t`` (n,)
    with zero velocity at both ends (tridiagonal solve for the knot accelerations)."""
    t = _check_t(t)
    y = np.asarray(y, dtype=float).reshape(len(t), -1)
    n = len(t)
    h = np.diff(t)
    d = np.diff(y, axis=0) / h[:, None]
    lower = np.zeros(n)
    diag = np.zeros(n)
    upper = np.zeros(n)
    rhs = np.zeros_like(y)
    diag[0], upper[0], rhs[0] = 2.0 * h[0], h[0], 6.0 * d[0]
    diag[-1], lower[-1], rhs[-1] = 2.0 * h[-1], h[-1], -6.0 * d[-1]
    for i in range(1, n - 1):
        lower[i], diag[i], upper[i] = h[i - 1], 2.0 * (h[i - 1] + h[i]), h[i]
        rhs[i] = 6.0 * (d[i] - d[i - 1])
    # Thomas algorithm (diagonally dominant system)
    c = np.zeros(n)
    r = np.zeros_like(y)
    c[0] = upper[0] / diag[0]
    r[0] = rhs[0] / diag[0]
    for i in range(1, n):
        m = diag[i] - lower[i] * c[i - 1]
        c[i] = upper[i] / m if i < n - 1 else 0.0
        r[i] = (rhs[i] - lower[i] * r[i - 1]) / m
    acc = np.zeros_like(y)
    acc[-1] = r[-1]
    for i in range(n - 2, -1, -1):
        acc[i] = r[i] - c[i] * acc[i + 1]
    vel = np.zeros_like(y)
    vel[:-1] = d - h[:, None] * (2.0 * acc[:-1] + acc[1:]) / 6.0
    vel[0] = 0.0
    vel[-1] = 0.0
    return vel, acc


def evaluate(t: np.ndarray, y: np.ndarray, v: np.ndarray, a: np.ndarray, tq: np.ndarray
             ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Position, velocity, acceleration of the spline at times ``tq`` (inside [t0, tn])."""
    t = _check_t(t)
    y = np.asarray(y, dtype=float).reshape(len(t), -1)
    v = np.asarray(v, dtype=float).reshape(y.shape)
    a = np.asarray(a, dtype=float).reshape(y.shape)
    tq = np.asarray(tq, dtype=float)
    k = np.clip(np.searchsorted(t, tq, side='right') - 1, 0, len(t) - 2)
    h = (t[k + 1] - t[k])[:, None]
    tau = (tq - t[k])[:, None]
    jerk = (a[k + 1] - a[k]) / h
    p = y[k] + v[k] * tau + a[k] * tau ** 2 / 2.0 + jerk * tau ** 3 / 6.0
    return p, v[k] + a[k] * tau + jerk * tau ** 2 / 2.0, a[k] + jerk * tau


def max_abs(t: np.ndarray, y: np.ndarray, v: np.ndarray, a: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Exact max |velocity| and max |acceleration| per joint of the spline (acceleration is linear per interval,
    velocity quadratic with its extremum where the acceleration crosses zero)."""
    t = _check_t(t)
    v = np.asarray(v, dtype=float).reshape(len(t), -1)
    a = np.asarray(a, dtype=float).reshape(v.shape)
    vmax = np.abs(v).max(axis=0)
    amax = np.abs(a).max(axis=0)
    h = np.diff(t)[:, None]
    a0, a1 = a[:-1], a[1:]
    da = a1 - a0
    with np.errstate(divide='ignore', invalid='ignore'):
        tau = np.where(np.abs(da) > 0.0, -a0 * h / da, -1.0)
    inside = (tau > 0.0) & (tau < h)
    vt = v[:-1] + a0 * tau + da * tau ** 2 / (2.0 * h)
    vt = np.where(inside, np.abs(vt), 0.0)
    return np.maximum(vmax, vt.max(axis=0)), amax


def weld_times(s_mm: np.ndarray, speed_mm_s: float, ramp_accel_mm_s2: float) -> Tuple[np.ndarray, float]:
    """Times (s) of the seam points ``s_mm`` for rest → ``speed_mm_s`` → rest with ramp acceleration
    ``ramp_accel_mm_s2``; returns ``(t, ramp_length_mm)``. Refuses an arc too short for a constant-speed part."""
    s = np.asarray(s_mm, dtype=float)
    if speed_mm_s <= 0.0 or ramp_accel_mm_s2 <= 0.0:
        raise TimingError('travel speed and ramp acceleration must be > 0')
    x = s - s[0]
    ramp = speed_mm_s ** 2 / (2.0 * ramp_accel_mm_s2)
    if 2.0 * ramp >= x[-1]:
        raise TimingError(f'arc of {x[-1]:.3f} mm has no constant-speed part: the speed ramps need '
                          f'2 × {ramp:.3f} mm at {speed_mm_s} mm/s and {ramp_accel_mm_s2} mm/s²')
    return constant_speed_times(x, speed_mm_s, ramp_accel_mm_s2), float(ramp)


def rest_to_rest(q: np.ndarray, vlim: np.ndarray, alim: np.ndarray, scaling: float
                 ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """``(t, v, a)`` for the dense joint path ``q`` (n, m) from rest to rest within ``scaling`` × the limits."""
    if not 0.0 < scaling <= 1.0:
        raise TimingError('limit scaling must be in (0, 1]: limits are never raised')
    q = np.asarray(q, dtype=float)
    if len(q) < 2 or np.any(np.abs(np.diff(q, axis=0)).max(axis=1) <= 1e-12):
        raise TimingError('rest-to-rest path needs at least 2 points and no repeated point')
    vs = np.asarray(vlim, dtype=float) * scaling
    acs = np.asarray(alim, dtype=float) * scaling
    t, _ = trapezoid_times(q, vs, acs)
    for _ in range(4):
        v, a = clamped_cubic(t, q)
        vmax, amax = max_abs(t, q, v, a)
        k = max(1.0, float((vmax / vs).max()), float(np.sqrt((amax / acs).max())))
        if k <= 1.0 + 1e-12:
            return t, v, a
        t = t * (k * (1.0 + 1e-9))   # stretching time by k scales v by 1/k and a by 1/k² (exact for a spline)
    v, a = clamped_cubic(t, q)
    return t, v, a
