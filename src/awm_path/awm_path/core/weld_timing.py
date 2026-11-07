"""Weld-segment timing model and the acceleration-aware maximum travel speed (G1.9; A47; DES-001 Rev 0.5 §4.3
step 5, §6.2; CP-59 follow-up of 2026-11-06).

This module is the one source of the weld timing model: the program generator (M6, ``awm_program``, G1.12) calls
these functions to time a weld segment and to find its maximum speed, so a path the planner reports as feasible is
accepted by the program and the maximum the planner shows is the maximum the program accepts (the equality is tested
in ``awm_program/test/test_core_shared_timing.py``):

* times along the seam arc length s for a trapezoidal speed profile rest → travel speed → rest with the weld ramp
  acceleration (:func:`weld_times`; the constant-speed part runs exactly at the travel speed relative to the part);
* one clamped C2 cubic spline through the 7-joint states at those times, velocity 0 at both ends
  (:func:`clamped_cubic`);
* exact maxima of |velocity| and |acceleration| of that spline (:func:`max_abs`), checked against the joint and
  positioner velocity **and** acceleration limits (:func:`within_limits`).

:func:`speed_report` gives ``speed_within_limits`` at the requested speed and the largest feasible travel speed
(bisection on (0, √(a_ramp · L)) — above that the ramps leave no constant-speed part — rounded down to the
resolution). The travel speed is an Engineer input: it is never changed here, and no limit is scaled or raised.

Feasibility is **not** always monotonic in the travel speed: the knot times of the spline change with v and the
ramps move along the seam, so a path can be refused at one speed and accepted at a higher one (review minor 1:
about 1 % of random paths). :func:`max_feasible_speed` therefore scans ``SCAN_POINTS`` speeds on (0, √(a_ramp · L))
first and bisects only between the highest feasible scan speed and the next one; a feasible request is never reported
above the maximum. Remaining case (Assumption, named): a feasible island narrower than the scan step
√(a_ramp · L) / ``SCAN_POINTS`` above the highest feasible scan speed can be missed — the reported maximum is then
too low (conservative), never infeasible; the program generator uses the same search, so it reports the same value.

Pure numpy (B-06); degrees, mm, seconds.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

__all__ = ['SCAN_POINTS', 'TimingError', 'constant_speed_times', 'weld_times', 'clamped_cubic', 'max_abs', 'weld_spline_maxima',
           'within_limits', 'max_feasible_speed', 'speed_report']

_REL = 1e-9   # relative tolerance of a limit check (same as the program generator)
#: Coarse scan points of :func:`max_feasible_speed` on (0, √(a_ramp · L)) (review minor 1).
SCAN_POINTS = 256


class TimingError(ValueError):
    """A weld segment cannot be timed (non-positive speed / ramp, zero length, ramps longer than the arc)."""


def constant_speed_times(s: np.ndarray, speed: float, accel: float) -> np.ndarray:
    """Time at each arc length for a trapezoidal speed profile (rest → speed → rest)."""
    if speed <= 0 or accel <= 0:
        raise TimingError('speed and acceleration must be positive')
    s = np.asarray(s, float)
    total = float(s[-1])
    if total <= 0:
        raise TimingError('zero-length weld segment')
    d_acc = speed ** 2 / (2 * accel)
    if 2 * d_acc > total:
        d_acc = total / 2
        speed = math.sqrt(accel * total)
    t_acc = speed / accel
    t_total = 2 * t_acc + (total - 2 * d_acc) / speed
    out = np.empty_like(s)
    for i, si in enumerate(s):
        if si <= d_acc:
            out[i] = math.sqrt(2 * si / accel)
        elif si >= total - d_acc:
            out[i] = t_total - math.sqrt(2 * max(total - si, 0.0) / accel)
        else:
            out[i] = t_acc + (si - d_acc) / speed
    if np.any(np.diff(out) <= 0):
        raise TimingError('weld samples are not strictly increasing in time (duplicate points in the weld segment)')
    return out


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


def max_abs(t: np.ndarray, v: np.ndarray, a: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
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


def weld_spline_maxima(s_mm: np.ndarray, q7_deg: np.ndarray, speed_mm_s: float, ramp_accel_mm_s2: float
                       ) -> Tuple[np.ndarray, np.ndarray]:
    """Max |velocity| (deg/s) and max |acceleration| (deg/s²) per joint of the timed weld spline."""
    q7 = np.asarray(q7_deg, float)
    t, _ = weld_times(s_mm, speed_mm_s, ramp_accel_mm_s2)
    v, a = clamped_cubic(t, q7)
    return max_abs(t, v, a)


def _limits(vlim, alim, m: int) -> Tuple[np.ndarray, np.ndarray]:
    vlim = np.asarray(vlim, float).ravel()
    alim = np.asarray(alim, float).ravel()
    if len(vlim) != m or len(alim) != m:
        raise ValueError(f'{m} velocity and {m} acceleration limits are needed, got {len(vlim)} / {len(alim)}')
    if np.any(vlim <= 0.0) or np.any(alim <= 0.0):
        raise ValueError('velocity and acceleration limits must be > 0')
    return vlim, alim


def _ok(vmax: np.ndarray, amax: np.ndarray, vlim: np.ndarray, alim: np.ndarray) -> bool:
    return bool(np.all(vmax <= vlim * (1 + _REL)) and np.all(amax <= alim * (1 + _REL)))


def within_limits(s_mm: np.ndarray, q7_deg: np.ndarray, speed_mm_s: float, vlim: Sequence[float],
                  alim: Sequence[float], ramp_accel_mm_s2: float) -> bool:
    """True if the weld spline at ``speed_mm_s`` keeps every joint inside its velocity and acceleration limit (an
    arc too short for the speed ramps is not feasible)."""
    q7 = np.asarray(q7_deg, float)
    vlim, alim = _limits(vlim, alim, q7.shape[1])
    try:
        vmax, amax = weld_spline_maxima(s_mm, q7, speed_mm_s, ramp_accel_mm_s2)
    except TimingError:
        return False
    return _ok(vmax, amax, vlim, alim)


def max_feasible_speed(s_mm: np.ndarray, q7_deg: np.ndarray, vlim: Sequence[float], alim: Sequence[float],
                       ramp_accel_mm_s2: float, resolution_mm_s: float, request_mm_s: Optional[float] = None) -> float:
    """Largest travel speed (rounded down to ``resolution_mm_s``) whose weld spline is inside the velocity and
    acceleration limits; 0.0 if none ≥ the resolution is found. Coarse scan of ``SCAN_POINTS`` speeds, then bisection
    between the highest feasible scan speed and the next scan speed (module docstring: non-monotonic paths). If
    ``request_mm_s`` is feasible, the result is at least the request rounded down to the resolution (when that is
    feasible too), so a record never says "within limits" with a maximum below the request."""
    if resolution_mm_s <= 0.0:
        raise ValueError('speed search resolution must be > 0')
    s = np.asarray(s_mm, float)
    length = float(s[-1] - s[0])
    if length <= 0.0 or ramp_accel_mm_s2 <= 0.0:
        return 0.0

    def ok(v: float) -> bool:
        return v > 0.0 and within_limits(s, q7_deg, v, vlim, alim, ramp_accel_mm_s2)

    top = math.sqrt(ramp_accel_mm_s2 * length)            # ramps leave no constant-speed part at ≥ top
    step = top / SCAN_POINTS
    feasible = [k for k in range(1, SCAN_POINTS) if ok(k * step)]
    k0 = max(feasible) if feasible else 0
    lo, hi = k0 * step, (k0 + 1) * step
    while hi - lo > resolution_mm_s / 4.0:
        mid = 0.5 * (lo + hi)
        if ok(mid):
            lo = mid
        else:
            hi = mid
    best = math.floor(lo / resolution_mm_s + 1e-9) * resolution_mm_s
    while best > 0.0 and not ok(best):
        best = round(best - resolution_mm_s, 12)        # never report an infeasible maximum
    if request_mm_s is not None and ok(float(request_mm_s)):
        floor_req = math.floor(float(request_mm_s) / resolution_mm_s + 1e-9) * resolution_mm_s
        if floor_req > best and ok(floor_req):
            best = floor_req
    return float(max(best, 0.0))


def _limited_by(vmax, amax, vlim, alim) -> str:
    rv, ra = vmax / vlim, amax / alim
    j_v, j_a = int(np.argmax(rv)), int(np.argmax(ra))
    return f'joint {j_v + 1} velocity' if rv[j_v] >= ra[j_a] else f'joint {j_a + 1} acceleration'


def speed_report(s_mm: np.ndarray, q7_deg: np.ndarray, speed_mm_s: float, vlim: Sequence[float],
                 alim: Sequence[float], ramp_accel_mm_s2: float, resolution_mm_s: float) -> Dict:
    """``speed_within_limits`` at ``speed_mm_s``, the largest feasible travel speed ``max_travel_speed_mm_s``, the
    spline velocity maxima at the requested speed ``max_joint_speed_deg_s`` (7) and ``limited_by`` (the joint and
    quantity closest to its limit at the requested speed; 'speed ramps longer than the arc' if it cannot be timed).
    Joint numbers: 1–6 robot, 7 positioner."""
    q7 = np.asarray(q7_deg, float)
    vlim, alim = _limits(vlim, alim, q7.shape[1])
    if speed_mm_s <= 0.0:
        raise ValueError('travel speed must be > 0')
    try:
        vmax, amax = weld_spline_maxima(s_mm, q7, speed_mm_s, ramp_accel_mm_s2)
        ok = _ok(vmax, amax, vlim, alim)
        limited = _limited_by(vmax, amax, vlim, alim)
    except TimingError:
        s = np.asarray(s_mm, float)
        t = (s - s[0]) / speed_mm_s
        vmax = (np.abs(np.diff(q7, axis=0)) / np.diff(t)[:, None]).max(axis=0)   # reported only (constant speed)
        ok, limited = False, 'speed ramps longer than the arc'
    return {'max_joint_speed_deg_s': [float(x) for x in vmax], 'speed_within_limits': bool(ok),
            'max_travel_speed_mm_s': max_feasible_speed(s_mm, q7, vlim, alim, ramp_accel_mm_s2, resolution_mm_s,
                                                        speed_mm_s if ok else None),
            'limited_by': limited}
