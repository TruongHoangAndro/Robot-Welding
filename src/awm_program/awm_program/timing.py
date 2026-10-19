"""Time parameterisation helpers (pure numpy).

The weld segment runs at the WPS travel speed: time follows the TCP arc length with a short
acceleration ramp at both ends, so the segment starts and ends at rest (fixes demo finding F12).
"""

from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np


class TimingError(ValueError):
    """A trajectory cannot be timed within the limits."""


def arc_length(points: np.ndarray) -> np.ndarray:
    """Cumulative arc length of a polyline [N,3]."""
    steps = np.linalg.norm(np.diff(np.asarray(points, float), axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(steps)])


def constant_speed_times(s: np.ndarray, speed: float, accel: float) -> np.ndarray:
    """Time at each arc length for a trapezoidal speed profile (rest -> speed -> rest)."""
    if speed <= 0 or accel <= 0:
        raise TimingError('speed and acceleration must be positive')
    s = np.asarray(s, float)
    total = float(s[-1])
    if total <= 0:
        raise TimingError('zero-length weld segment')
    d_acc = speed ** 2 / (2 * accel)
    if 2 * d_acc > total:  # triangular profile: never reaches the travel speed
        d_acc = total / 2
        speed = np.sqrt(accel * total)
    t_acc = speed / accel
    t_total = 2 * t_acc + (total - 2 * d_acc) / speed
    out = np.empty_like(s)
    for i, si in enumerate(s):
        if si <= d_acc:
            out[i] = np.sqrt(2 * si / accel)
        elif si >= total - d_acc:
            out[i] = t_total - np.sqrt(2 * max(total - si, 0.0) / accel)
        else:
            out[i] = t_acc + (si - d_acc) / speed
    if np.any(np.diff(out) <= 0):
        raise TimingError('weld samples are not strictly increasing in time '
                          '(duplicate points in the weld segment)')
    return out


def joint_velocities(q: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Finite-difference joint velocities [N,6], zero at both ends (segment starts / ends at rest)."""
    q = np.asarray(q, float)
    t = np.asarray(t, float)
    v = np.zeros_like(q)
    if len(t) > 2:
        v[1:-1] = (q[2:] - q[:-2]) / (t[2:] - t[:-2])[:, None]
    return v


def max_velocity_ratio(q: np.ndarray, t: np.ndarray, vmax: Sequence[float]) -> Tuple[float, int]:
    """Largest |dq/dt| / vmax over consecutive points, and the joint index where it happens."""
    q = np.asarray(q, float)
    dt = np.diff(np.asarray(t, float))
    if np.any(dt <= 0):
        return float('inf'), -1
    rates = np.abs(np.diff(q, axis=0)) / dt[:, None] / np.asarray(vmax, float)
    idx = np.unravel_index(int(np.argmax(rates)), rates.shape)
    return float(rates[idx]), int(idx[1])


def trapezoid_times(q: np.ndarray, vmax: Sequence[float],
                    amax: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    """Synchronised rest-to-rest timing of a dense joint path (all joints within vmax / amax).

    Path parameter s = cumulative max_j |dq_j| / vmax_j (time at full speed); a trapezoidal
    profile on s with ds/dt <= 1 keeps every joint at or below its velocity limit.
    Returns (times of the kept points, mask of kept points: repeated points are dropped).
    """
    q = np.asarray(q, float)
    vmax = np.asarray(vmax, float)
    amax = np.asarray(amax, float)
    ds = np.max(np.abs(np.diff(q, axis=0)) / vmax, axis=1)
    s = np.concatenate([[0.0], np.cumsum(ds)])
    if s[-1] <= 0:
        raise TimingError('zero-length joint path')
    accel = float(np.min(amax / vmax))  # d2s/dt2 limit so every joint stays within amax
    keep = np.concatenate([[True], ds > 1e-12])
    return constant_speed_times(s[keep], 1.0, accel), keep
