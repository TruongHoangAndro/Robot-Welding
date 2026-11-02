"""Frame check (WP-S S.2): KinematicsBackend FK equals the ur_description URDF chain base_link -> tool0.

The reference FK below is computed from the URDF joint origins, independently of ur-analytic-ik (test-only code).
"""
import subprocess
from typing import List, Tuple

import numpy as np
import pytest
from urdf_parser_py.urdf import URDF

from awm_core import units
from awm_path.core.kinematics import UrAnalyticKinematics, pose_error

POS_TOL_MM = 0.01
ROT_TOL_RAD = 1e-6
SEED = 20261102
N_STATES = 200


def _rpy_matrix(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _axis_angle(axis, angle_rad: float) -> np.ndarray:
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle_rad) * kx + (1 - np.cos(angle_rad)) * kx @ kx


def _origin_mm(joint) -> np.ndarray:
    t = np.eye(4)
    if joint.origin is not None:
        t[:3, :3] = _rpy_matrix(joint.origin.rpy or [0.0, 0.0, 0.0])
        t[:3, 3] = units.m_to_mm(np.asarray(joint.origin.xyz or [0.0, 0.0, 0.0], dtype=float))
    return t


@pytest.fixture(scope='module')
def chain() -> List[Tuple[object, bool]]:
    prefix = subprocess.run(['ros2', 'pkg', 'prefix', 'ur_description'], check=True, capture_output=True,
                            text=True).stdout.strip()
    xml = subprocess.run(['xacro', f'{prefix}/share/ur_description/urdf/ur.urdf.xacro', 'ur_type:=ur5e', 'name:=ur'],
                         check=True, capture_output=True, text=True).stdout
    robot = URDF.from_xml_string(xml)
    joint_names = robot.get_chain('base_link', 'tool0', joints=True, links=False)
    joints = {j.name: j for j in robot.joints}
    out = [(joints[n], joints[n].type in ('revolute', 'continuous')) for n in joint_names]
    assert sum(movable for _, movable in out) == 6
    return out


def urdf_fk(chain, q_deg) -> np.ndarray:
    """base_link_T_tool0 (mm) composed from URDF joint origins and revolute rotations."""
    t = np.eye(4)
    q_rad = iter(units.deg_to_rad(np.asarray(q_deg, dtype=float)))
    for joint, movable in chain:
        t = t @ _origin_mm(joint)
        if movable:
            rot = np.eye(4)
            rot[:3, :3] = _axis_angle(joint.axis, next(q_rad))
            t = t @ rot
    return t


def test_fk_matches_ur_description_urdf(chain) -> None:
    kin = UrAnalyticKinematics()
    rng = np.random.default_rng(SEED)
    for q in rng.uniform(-180.0, 180.0, size=(N_STATES, 6)):
        dp, dr = pose_error(kin.fk(q), urdf_fk(chain, q))
        assert dp < POS_TOL_MM and dr < ROT_TOL_RAD, (q, dp, dr)
