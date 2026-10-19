"""Rigid-body frame math (SI units, quaternions in x, y, z, w order).

Frame chain (DES-001 §5.3): ``base_T_tcp = base_T_fixture · fixture_T_part · part_T_pose``.
A transform ``a_T_b`` maps coordinates expressed in frame b into frame a.
"""

from __future__ import annotations

from typing import Any, Dict, Sequence

import numpy as np

_EPS = 1e-12


def quat_to_matrix(q_xyzw: Sequence[float]) -> np.ndarray:
    """Return the 3x3 rotation matrix of a (normalised here) quaternion."""
    x, y, z, w = (float(v) for v in q_xyzw)
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < _EPS:
        raise ValueError('zero-length quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(rot: np.ndarray) -> np.ndarray:
    """Return the quaternion (x, y, z, w) with w >= 0 of a 3x3 rotation matrix."""
    m = np.asarray(rot, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return q


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF convention: R = Rz(yaw) · Ry(pitch) · Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_rpy(rot: np.ndarray) -> np.ndarray:
    """Inverse of :func:`rpy_to_matrix` (roll, pitch, yaw)."""
    m = np.asarray(rot, dtype=float)
    pitch = np.arcsin(-np.clip(m[2, 0], -1.0, 1.0))
    if abs(np.cos(pitch)) > 1e-9:
        roll = np.arctan2(m[2, 1], m[2, 2])
        yaw = np.arctan2(m[1, 0], m[0, 0])
    else:
        roll = 0.0
        yaw = np.arctan2(-m[0, 1], m[1, 1])
    return np.array([roll, pitch, yaw])


def axis_angle_to_matrix(axis: Sequence[float], angle: float) -> np.ndarray:
    """Rodrigues rotation about a unit axis."""
    a = np.asarray(axis, dtype=float)
    n = np.linalg.norm(a)
    if n < _EPS:
        raise ValueError('zero-length rotation axis')
    a = a / n
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


def make_transform(position: Sequence[float], rot: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous transform."""
    t = np.eye(4)
    t[:3, :3] = np.asarray(rot, dtype=float)
    t[:3, 3] = np.asarray(position, dtype=float)
    return t


def pose_to_matrix(pose: Dict[str, Any]) -> np.ndarray:
    """JSON pose ``{position_m, quat_xyzw}`` → 4x4 transform."""
    return make_transform(pose['position_m'], quat_to_matrix(pose['quat_xyzw']))


def matrix_to_pose(t: np.ndarray) -> Dict[str, list]:
    """4x4 transform → JSON pose ``{position_m, quat_xyzw}``."""
    t = np.asarray(t, dtype=float)
    return {'position_m': [float(v) for v in t[:3, 3]],
            'quat_xyzw': [float(v) for v in matrix_to_quat(t[:3, :3])]}


def xyz_rpy_to_matrix(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    """URDF-style origin → 4x4 transform."""
    return make_transform(xyz, rpy_to_matrix(*rpy))


def invert(t: np.ndarray) -> np.ndarray:
    """Inverse of a rigid transform."""
    t = np.asarray(t, dtype=float)
    inv = np.eye(4)
    inv[:3, :3] = t[:3, :3].T
    inv[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return inv


def compose(*transforms: np.ndarray) -> np.ndarray:
    """Left-to-right product of 4x4 transforms."""
    out = np.eye(4)
    for t in transforms:
        out = out @ np.asarray(t, dtype=float)
    return out


def frame_from_z_x(z_axis: Sequence[float], x_hint: Sequence[float]) -> np.ndarray:
    """Right-handed rotation whose z axis is ``z_axis`` and x axis is ``x_hint`` made orthogonal."""
    z = np.asarray(z_axis, dtype=float)
    nz = np.linalg.norm(z)
    if nz < _EPS:
        raise ValueError('zero-length z axis')
    z = z / nz
    x = np.asarray(x_hint, dtype=float)
    x = x - np.dot(x, z) * z
    nx = np.linalg.norm(x)
    if nx < 1e-9:
        raise ValueError('x hint is parallel to the z axis')
    x = x / nx
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def rotation_angle_between(r_a: np.ndarray, r_b: np.ndarray) -> float:
    """Angle (rad) of the relative rotation between two rotation matrices."""
    r = np.asarray(r_a).T @ np.asarray(r_b)
    c = (np.trace(r) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))
