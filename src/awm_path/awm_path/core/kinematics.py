"""KinematicsBackend: analytic UR5e forward / inverse kinematics with all IK branches (ADR-12, WP-S S.2).

Public values are mm and degrees (ADR-14). Poses are 4x4 homogeneous transforms ``base_link_T_tool0`` with the
translation in mm. Joint order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 (6 values, the
positioner joint is not part of the robot kinematics). Conversion to m / rad happens only through
``awm_core.units``. This module never imports rclpy or ROS messages (B-06).

Frames: ``base_link`` -> ``tool0`` of ``ur_description`` (the frames MoveIt uses). The library's own base frame is
related to ``base_link`` by the fixed transform ``LIB_BASE_T_BASE_LINK``, absorbed in ``fk`` / ``ik``.
"""

from __future__ import annotations

import importlib.metadata
from typing import List, Protocol, Sequence, Tuple, runtime_checkable

import numpy as np

from awm_core import units

#: Fixed transform library base -> ``base_link`` (set by the URDF check of Task 4).
LIB_BASE_T_BASE_LINK = np.eye(4)

_RIGID_TOL = 1e-9
#: An IK solution is returned only if its FK reproduces the target this closely (filters degenerate solutions at
#: singularities); stricter than the S.2 acceptance (0.01 mm, 1e-6 rad).
_IK_CHECK_POS_MM = 1e-3
_IK_CHECK_ROT_RAD = 1e-7


@runtime_checkable
class KinematicsBackend(Protocol):
    """Robot kinematics in mm / degrees, frames ``base_link`` -> ``tool0``."""

    name: str

    def fk(self, q_deg: Sequence[float]) -> np.ndarray:
        """Return the 4x4 ``base_link_T_tool0`` (translation in mm) for 6 joint angles in degrees."""
        ...

    def ik(self, base_link_T_tool0: np.ndarray) -> List[np.ndarray]:
        """Return all IK solutions (each shape (6,), degrees in (-180, 180]); ``[]`` if unreachable."""
        ...


def wrap_deg(q_deg: np.ndarray) -> np.ndarray:
    """Wrap angles in degrees into (-180, 180]."""
    q = np.asarray(q_deg, dtype=float)
    return q - 360.0 * np.ceil((q - 180.0) / 360.0)


def check_rigid(t: np.ndarray) -> np.ndarray:
    """Return ``t`` as a float 4x4 array or raise ``ValueError`` if it is not a rigid homogeneous transform."""
    a = np.asarray(t, dtype=float)
    if a.shape != (4, 4):
        raise ValueError(f'expected a 4x4 transform, got shape {a.shape}')
    if not np.all(np.isfinite(a)):
        raise ValueError('transform contains non-finite values')
    if not np.allclose(a[3], [0.0, 0.0, 0.0, 1.0], atol=_RIGID_TOL, rtol=0.0):
        raise ValueError(f'last row of the transform is {a[3]}, expected [0, 0, 0, 1]')
    r = a[:3, :3]
    if abs(np.linalg.det(r) - 1.0) >= _RIGID_TOL or np.linalg.norm(r.T @ r - np.eye(3)) >= _RIGID_TOL:
        raise ValueError('rotation part of the transform is not orthonormal with det +1')
    return a


def _check_q(q_deg: Sequence[float]) -> np.ndarray:
    q = np.asarray(q_deg, dtype=float)
    if q.shape != (6,):
        raise ValueError(f'expected 6 joint angles, got shape {q.shape}')
    if not np.all(np.isfinite(q)):
        raise ValueError(f'joint angles not finite: {q}')
    return q


def pose_error(a: np.ndarray, b: np.ndarray) -> Tuple[float, float]:
    """Return (position error mm, rotation error rad) between two 4x4 transforms in mm."""
    dp = float(np.linalg.norm(np.asarray(a)[:3, 3] - np.asarray(b)[:3, 3]))
    r = np.asarray(a)[:3, :3].T @ np.asarray(b)[:3, :3]
    # atan2 form: accurate for small angles (arccos of the trace is not).
    s = 0.5 * np.linalg.norm([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    c = 0.5 * (np.trace(r) - 1.0)
    return dp, float(np.arctan2(s, c))


def same_state_deg(a: Sequence[float], b: Sequence[float], tol_deg: float = 1e-6) -> bool:
    """True if two joint states are equal modulo 360 deg on every joint."""
    d = wrap_deg(np.asarray(a, dtype=float) - np.asarray(b, dtype=float))
    return bool(np.all(np.minimum(np.abs(d), np.abs(360.0 - np.abs(d))) <= tol_deg))


class UrAnalyticKinematics:
    """UR5e kinematics from ``ur-analytic-ik`` (closed form, up to 8 solutions)."""

    def __init__(self) -> None:
        from ur_analytic_ik import ur5e  # imported here only: the library is a venv package (Task 1)

        self._lib = ur5e
        self.name = f"ur_analytic_ik-{importlib.metadata.version('ur_analytic_ik')}/ur5e"
        self._base_link_T_lib = np.linalg.inv(LIB_BASE_T_BASE_LINK)

    def fk(self, q_deg: Sequence[float]) -> np.ndarray:
        q_rad = units.deg_to_rad(_check_q(q_deg))
        lib_T_tool0 = units.transform_m_to_mm(np.asarray(self._lib.forward_kinematics(*map(float, q_rad))))
        return self._base_link_T_lib @ lib_T_tool0

    def ik(self, base_link_T_tool0: np.ndarray) -> List[np.ndarray]:
        target = check_rigid(base_link_T_tool0)
        lib_T_tool0 = units.transform_mm_to_m(LIB_BASE_T_BASE_LINK @ target)
        sols: List[np.ndarray] = []
        for raw in self._lib.inverse_kinematics(np.ascontiguousarray(lib_T_tool0)):
            q = np.asarray(raw, dtype=float)
            if q.shape != (6,) or not np.all(np.isfinite(q)):
                continue
            q_deg = wrap_deg(units.rad_to_deg(q))
            dp, dr = pose_error(self.fk(q_deg), target)
            if dp < _IK_CHECK_POS_MM and dr < _IK_CHECK_ROT_RAD:
                sols.append(q_deg)
        return sols


def fk_tcp(kin: KinematicsBackend, q_deg: Sequence[float], tool0_T_tcp: np.ndarray) -> np.ndarray:
    """Return ``base_link_T_tcp`` (mm) for joint angles in degrees and a fixed ``tool0_T_tcp`` (mm)."""
    return kin.fk(q_deg) @ check_rigid(tool0_T_tcp)


def ik_tcp(kin: KinematicsBackend, base_link_T_tcp: np.ndarray, tool0_T_tcp: np.ndarray) -> List[np.ndarray]:
    """Return all joint solutions (degrees) that put the TCP at ``base_link_T_tcp`` (mm)."""
    return kin.ik(check_rigid(base_link_T_tcp) @ np.linalg.inv(check_rigid(tool0_T_tcp)))
