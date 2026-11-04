"""Tests of the M4 core KinematicsBackend (WP-S S.2): analytic UR5e IK vs FK, mm / degrees."""
import subprocess
import sys

import numpy as np
import pytest

from awm_core import units
from awm_path.core.kinematics import (UrAnalyticKinematics, fk_tcp, ik_tcp, pose_error,
                                      same_state_deg)

POS_TOL_MM = 0.01
ROT_TOL_RAD = 1e-6
SEED = 20261102
N_STATES = 500


def _sample_states(n: int, seed: int = SEED) -> np.ndarray:
    """Seeded joint states in [-180, 180) deg; wrist_2 rejected near the wrist singularity (|sin| < 0.05)."""
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < n:
        q = rng.uniform(-180.0, 180.0, size=6)
        if abs(np.sin(units.deg_to_rad(q[4]))) < 0.05:
            continue
        out.append(q)
    return np.array(out)


@pytest.fixture(scope='module')
def kin() -> UrAnalyticKinematics:
    return UrAnalyticKinematics()


@pytest.fixture(scope='module')
def states() -> np.ndarray:
    return _sample_states(N_STATES)


def _rot_x(deg: float) -> np.ndarray:
    a = units.deg_to_rad(deg)
    c, s = np.cos(a), np.sin(a)
    t = np.eye(4)
    t[1:3, 1:3] = [[c, -s], [s, c]]
    return t


def test_backend_name_records_library_version(kin: UrAnalyticKinematics) -> None:
    assert kin.name == 'ur_analytic_ik-0.1.0.post3/ur5e'


def test_ik_solutions_reproduce_pose(kin: UrAnalyticKinematics, states: np.ndarray) -> None:
    for q in states:
        target = kin.fk(q)
        sols = kin.ik(target)
        assert sols, f'no IK solution for reachable state {q}'
        for sol in sols:
            assert sol.shape == (6,)
            assert np.all(sol > -180.0) and np.all(sol <= 180.0)
            dp, dr = pose_error(kin.fk(sol), target)
            assert dp < POS_TOL_MM and dr < ROT_TOL_RAD, (q, sol, dp, dr)


def test_ik_contains_original_state_modulo_360(kin: UrAnalyticKinematics, states: np.ndarray) -> None:
    fixed = np.array([10.0, -80.0, 70.0, -100.0, -60.0, 350.0])
    for q in list(states) + [fixed]:
        sols = kin.ik(kin.fk(q))
        assert any(same_state_deg(sol, q) for sol in sols), (q, sols)


def test_ik_returns_at_most_8_solutions(kin: UrAnalyticKinematics, states: np.ndarray) -> None:
    for q in states:
        assert 1 <= len(kin.ik(kin.fk(q))) <= 8


def test_ik_unreachable_returns_empty(kin: UrAnalyticKinematics) -> None:
    target = np.eye(4)
    target[:3, 3] = [2000.0, 0.0, 0.0]
    assert kin.ik(target) == []


def test_ik_at_wrist_singularity_returns_only_valid_solutions(kin: UrAnalyticKinematics) -> None:
    q = np.array([0.0, -90.0, 90.0, -90.0, 0.0, 0.0])
    target = kin.fk(q)
    for sol in kin.ik(target):
        assert np.all(np.isfinite(sol))
        dp, _ = pose_error(kin.fk(sol), target)
        assert dp < POS_TOL_MM


def test_ik_rejects_non_rigid_transform(kin: UrAnalyticKinematics) -> None:
    t = kin.fk([0.0, -90.0, 90.0, -90.0, -90.0, 0.0])
    scaled = t.copy()
    scaled[:3, :3] *= 1.001
    with pytest.raises(ValueError):
        kin.ik(scaled)
    with pytest.raises(ValueError):
        kin.ik(t[:3, :])


def test_tcp_round_trip(kin: UrAnalyticKinematics, states: np.ndarray) -> None:
    tool0_T_tcp = _rot_x(15.0)
    tool0_T_tcp[:3, 3] = [0.0, 0.0, 250.0]
    for q in states[:100]:
        target = fk_tcp(kin, q, tool0_T_tcp)
        sols = ik_tcp(kin, target, tool0_T_tcp)
        assert sols
        for sol in sols:
            dp, dr = pose_error(fk_tcp(kin, sol, tool0_T_tcp), target)
            assert dp < POS_TOL_MM and dr < ROT_TOL_RAD


def test_core_does_not_import_rclpy() -> None:
    code = ("import awm_path.core.kinematics, sys; "
            "assert not any(m == 'rclpy' or m.startswith(('rclpy.', 'moveit_msgs')) for m in sys.modules)")
    assert subprocess.run([sys.executable, '-c', code]).returncode == 0


def test_fk_tcp_composition_order(kin: UrAnalyticKinematics, states: np.ndarray) -> None:
    """S.2 follow-up R4: ``fk_tcp = fk @ tool0_T_tcp`` (tool offset applied in the tool0 frame, not the base)."""
    shift = np.eye(4)
    shift[:3, 3] = [0.0, 0.0, 250.0]
    tool = _rot_x(15.0)
    tool[:3, 3] = [5.0, -7.0, 250.0]
    for q in states[:50]:
        f = kin.fk(q)
        got = fk_tcp(kin, q, shift)
        np.testing.assert_allclose(got[:3, 3], f[:3, 3] + 250.0 * f[:3, 2], atol=1e-6, rtol=0.0)
        np.testing.assert_allclose(got[:3, :3], f[:3, :3], atol=1e-12, rtol=0.0)
        np.testing.assert_allclose(fk_tcp(kin, q, tool), f @ tool, atol=1e-6, rtol=0.0)


def test_ik_at_q5_zero_documents_library_behaviour(kin: UrAnalyticKinematics) -> None:
    """S.2 follow-up R2: exactly at q5 = 0 (wrist singularity) the library may return ``[]`` for a reachable pose.

    Every returned solution must be valid. For the documented seed ``SEED + 5`` (= 20261107, 200 poses,
    ur_analytic_ik 0.1.0.post3) the library returns ``[]`` for 5 poses; the test asserts that at least one empty
    result occurs, so the behaviour the planner relies on (``no_ik_near_singular``) stays covered. The planner therefore reports ``[]`` inside the reach envelope as
    ``no_ik_near_singular``, not ``no_ik_unreachable`` (``awm_path.core.sweep``).
    """
    rng = np.random.default_rng(SEED + 5)
    n, empty = 200, 0
    for _ in range(n):
        q = rng.uniform(-180.0, 180.0, size=6)
        q[4] = 0.0
        target = kin.fk(q)
        sols = kin.ik(target)
        if not sols:
            empty += 1
        for sol in sols:
            dp, dr = pose_error(kin.fk(sol), target)
            assert dp < POS_TOL_MM and dr < ROT_TOL_RAD
    print(f'q5 = 0: ik() == [] for {empty} / {n} reachable poses')
    assert empty > 0, 'seeded set no longer produces an empty IK result at q5 = 0'
