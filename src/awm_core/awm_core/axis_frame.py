"""Positioner-axis frame shared by the cell and the job import (CP-60, DES-001 §5.3).

Rule: origin = picked circle centre, x = axis direction, z = world up (the file up axis rotated to +Z) made
orthogonal to x, y = z × x. θ = 0 is the positioner pose in the file (decided 2026-11-06). Lengths in mm.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

from awm_core import frames, units

#: Fixed rotation per file up axis that brings it to world +Z (right-handed): (axis, angle deg).
UP_TO_Z = {'+Z': ([1, 0, 0], 0.0), '-Z': ([1, 0, 0], 180.0), '+Y': ([1, 0, 0], 90.0),
           '-Y': ([1, 0, 0], -90.0), '+X': ([0, 1, 0], -90.0), '-X': ([0, 1, 0], 90.0)}
VERTICAL_TOL = 1e-6


def world_R_file(up_axis: str) -> np.ndarray:
    """Rotation file → world for a file whose up axis is ``up_axis`` (raises ValueError if unknown)."""
    if up_axis not in UP_TO_Z:
        raise ValueError(f'up axis {up_axis!r} not in {sorted(UP_TO_Z)}')
    axis, deg = UP_TO_Z[up_axis]
    rot = frames.axis_angle_to_matrix(axis, units.deg_to_rad(deg))
    return np.where(np.abs(rot) < 1e-15, 0.0, rot)


def file_T_axis(center_file_mm: Sequence[float], axis_file: Sequence[float], up_axis: str) -> np.ndarray:
    """4x4 pose (mm) of the positioner-axis frame at θ = 0, expressed in the file frame."""
    rot = world_R_file(up_axis)
    direction = np.asarray(axis_file, dtype=float)
    norm = np.linalg.norm(direction)
    if direction.shape != (3,) or not np.isfinite(direction).all() or norm < 1e-12:
        raise ValueError(f'axis direction {list(axis_file)} is not a finite non-zero 3-vector')
    x = rot @ (direction / norm)
    up = np.array([0.0, 0.0, 1.0])
    z = up - (up @ x) * x
    if np.linalg.norm(z) < VERTICAL_TOL:
        raise ValueError('positioner axis is vertical: flange frame z (world up) undefined')
    z /= np.linalg.norm(z)
    world_r = np.column_stack([x, np.cross(z, x), z])
    return frames.make_transform(np.asarray(center_file_mm, dtype=float), rot.T @ world_r)
