"""The single unit adapter of the project (ADR-14, design rule 7).

Project data (JSON schemas, ``awm_interfaces`` messages, UI, reports, the public API of ``awm_path.core``) is in
mm, degrees and seconds. Third-party ROS packages (URDF, TF, MoveIt2, ur_robot_driver, ur-analytic-ik) use m and rad
(REP 103). Every conversion between the two goes through this module; no ad-hoc ``* 1000`` / ``/ 1000`` /
``radians()`` anywhere else.

Scalars in give floats out; array-likes in give new ``np.ndarray`` out (inputs are never modified).
"""

from __future__ import annotations

import math
from typing import Union

import numpy as np

MM_PER_M = 1000.0
DEG_PER_RAD = 180.0 / math.pi

Number = Union[float, int]
ArrayLike = Union[Number, 'np.ndarray', list, tuple]


def _scale(x: ArrayLike, factor: float) -> Union[float, np.ndarray]:
    """Multiply a scalar or an array-like by ``factor``; scalars stay Python floats."""
    if np.ndim(x) == 0:
        return float(x) * factor
    return np.asarray(x, dtype=float) * factor


def mm_to_m(x: ArrayLike) -> Union[float, np.ndarray]:
    """Length(s) in mm -> m."""
    return _scale(x, 1.0 / MM_PER_M)


def m_to_mm(x: ArrayLike) -> Union[float, np.ndarray]:
    """Length(s) in m -> mm."""
    return _scale(x, MM_PER_M)


def deg_to_rad(x: ArrayLike) -> Union[float, np.ndarray]:
    """Angle(s) in degrees -> radians."""
    return _scale(x, 1.0 / DEG_PER_RAD)


def rad_to_deg(x: ArrayLike) -> Union[float, np.ndarray]:
    """Angle(s) in radians -> degrees."""
    return _scale(x, DEG_PER_RAD)


def _check_4x4(t: np.ndarray) -> np.ndarray:
    a = np.asarray(t, dtype=float)
    if a.shape != (4, 4):
        raise ValueError(f'expected a 4x4 homogeneous transform, got shape {a.shape}')
    return a


def transform_mm_to_m(t: np.ndarray) -> np.ndarray:
    """4x4 homogeneous transform with translation in mm -> new transform with translation in m (rotation unchanged)."""
    out = _check_4x4(t).copy()
    out[:3, 3] = out[:3, 3] / MM_PER_M
    return out


def transform_m_to_mm(t: np.ndarray) -> np.ndarray:
    """4x4 homogeneous transform with translation in m -> new transform with translation in mm (rotation unchanged)."""
    out = _check_4x4(t).copy()
    out[:3, 3] = out[:3, 3] * MM_PER_M
    return out
