"""Time parameterisation of D09 trajectory segments (G1.5, G1.12; DES-001 Rev 0.5 §4.3 step 5, A47, A62).

Every segment carries the positions, velocities and accelerations of **one clamped C2 cubic spline** through its
points (velocity 0 at both ends = the segment starts and ends at rest, DES-001 §6.2). A quintic interpolation from
(p, v, a) at both ends of an interval — what the ROS 2 joint trajectory controller does when all three are given —
reproduces that cubic exactly, so the limits checked here (:func:`max_abs`: acceleration at the knots, velocity at
the knots and at the interior extrema) are the limits of the commanded motion, not only of the stored points.

One source (G1.12, decision Truong 2026-11-07): the weld timing model — :func:`weld_times` (rest → travel speed →
rest along the seam arc length with the weld ramp; the constant-speed part runs exactly at the travel speed relative
to the part, never changed here), :func:`clamped_cubic` and the spline maxima — is the planner's
``awm_path.core.weld_timing``; this module re-exports it (``max_abs`` keeps its program signature as a thin wrapper),
so the program and the D07 speed record cannot diverge. Program-only helpers:

* :func:`evaluate` — position, velocity, acceleration of the spline at given times;
* :func:`rest_to_rest` — air / approach / retract timing inside *scaled* joint limits (scaling ≤ 1, never above
  the limits); the duration is stretched until the spline is inside them.

Pure numpy; all values in the caller's units (degrees, mm, seconds).
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

from awm_path.core import weld_timing as _wt
from awm_path.core.weld_timing import TimingError, clamped_cubic, weld_times
from awm_program.timing import trapezoid_times

__all__ = ['TimingError', 'clamped_cubic', 'evaluate', 'max_abs', 'weld_times', 'rest_to_rest']


def _check_t(t: np.ndarray) -> np.ndarray:
    t = np.asarray(t, dtype=float)
    if t.ndim != 1 or len(t) < 2 or np.any(np.diff(t) <= 0.0):
        raise TimingError('spline knots need at least 2 strictly increasing times')
    return t


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
    """Exact max |velocity| and max |acceleration| per joint of the spline: the planner's
    :func:`awm_path.core.weld_timing.max_abs` (``y`` is accepted for the program signature and not needed)."""
    return _wt.max_abs(t, v, a)


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
