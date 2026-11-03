"""Minimal forward kinematics on a parsed URDF (test helper, numpy only; m / rad as in the URDF)."""

from __future__ import annotations

import math
from typing import Dict, Optional

import numpy as np


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    """URDF fixed-axis roll / pitch / yaw -> 3x3 rotation (Rz(y) Ry(p) Rx(r))."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, angle: float) -> np.ndarray:
    """Rotation matrix about a unit axis (Rodrigues)."""
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * kx @ kx


def origin_matrix(origin) -> np.ndarray:
    """4x4 transform of a URDF <origin> (None = identity)."""
    t = np.eye(4)
    if origin is not None:
        t[:3, :3] = rpy_matrix(*(origin.rpy or [0, 0, 0]))
        t[:3, 3] = origin.xyz or [0, 0, 0]
    return t


def fk(robot, link: str, q: Optional[Dict[str, float]] = None, root: Optional[str] = None) -> np.ndarray:
    """root_T_link for joint values q (missing joints = 0)."""
    q = q or {}
    root = root or robot.get_root()
    t = np.eye(4)
    while link != root:
        joint = robot.joint_map[robot.parent_map[link][0]]
        local = origin_matrix(joint.origin)
        if joint.type in ('revolute', 'continuous'):
            rot = np.eye(4)
            rot[:3, :3] = axis_angle(joint.axis, q.get(joint.name, 0.0))
            local = local @ rot
        t = local @ t
        link = joint.parent
    return t
