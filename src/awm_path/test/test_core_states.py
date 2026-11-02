"""Tests of configuration labels, unwrapping and singularity metrics (WP-S S.3, TST-002 F2, F6, S.2 R1)."""
import itertools
import math
import subprocess
import sys

import numpy as np
import pytest

from awm_core import units
from awm_path.core.kinematics import LIB_BASE_T_BASE_LINK, UrAnalyticKinematics
from awm_path.core.states import (LABELS, UR5E_GEOMETRY, SingularityMetrics, SingularityThresholds,
                                  config_label, ik_existence, nearest_representative, singular_reason,
                                  singularity_metrics, unwrapped_variants, wrist_centre_lib_mm)

SEED = 20261103
UR_LIMITS = ((-360.0, 360.0), (-360.0, 360.0), (-180.0, 180.0), (-360.0, 360.0), (-360.0, 360.0), (-360.0, 360.0))
THRESH = SingularityThresholds(wrist_deg=8.6, elbow_deg=5.0, shoulder_mm=50.0)


@pytest.fixture(scope='module')
def kin() -> UrAnalyticKinematics:
    return UrAnalyticKinematics()


def _sin_deg(x: float) -> float:
    return math.sin(units.deg_to_rad(x))


def _non_singular_states(n: int, seed: int = SEED) -> np.ndarray:
    """Seeded states clear of the three F6 singularities (margins well above the IK filter)."""
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < n:
        q = rng.uniform(-180.0, 180.0, size=6)
        m = singularity_metrics(q)
        if m.sin_q5 < 0.1 or m.sin_q3 < 0.1 or m.wrist_centre_mm < 30.0:
            continue
        out.append(q)
    return np.array(out)


def test_eight_labels_in_fixed_order() -> None:
    assert len(LABELS) == 8 and len(set(LABELS)) == 8
    assert LABELS[0] == 'S+E+W+' and LABELS[-1] == 'S-E-W-'


def test_wrist_centre_matches_backend_fk(kin: UrAnalyticKinematics) -> None:
    """DH wrist centre (origin of frame 5) = tool0 position - d6 * tool0 Z, in the UR ``base`` frame."""
    for q in _non_singular_states(100, seed=SEED + 1):
        lib_T_tool0 = LIB_BASE_T_BASE_LINK @ kin.fk(q)
        expected = lib_T_tool0[:3, 3] - UR5E_GEOMETRY.d6 * lib_T_tool0[:3, 2]
        np.testing.assert_allclose(wrist_centre_lib_mm(q), expected, atol=1e-6, rtol=0.0)


def test_ik_solutions_of_non_singular_poses_carry_distinct_labels(kin: UrAnalyticKinematics) -> None:
    full = 0
    for q in _non_singular_states(500):
        sols = kin.ik(kin.fk(q))
        labels = [config_label(s) for s in sols]
        assert 1 <= len(sols) <= 8
        assert len(set(labels)) == len(labels), (q, labels)
        assert config_label(q) in labels
        full += len(sols) == 8
    assert full > 0


def test_label_invariant_under_full_turns() -> None:
    rng = np.random.default_rng(SEED + 2)
    for q in _non_singular_states(200, seed=SEED + 3):
        k = rng.integers(-2, 3, size=6)
        assert config_label(q + 360.0 * k) == config_label(q)


def test_label_boundaries_are_deterministic() -> None:
    q = np.array([0.0, -90.0, 0.0, -90.0, 0.0, 0.0])  # elbow and wrist exactly singular
    assert config_label(q)[2:] == 'E+W+'
    assert config_label(q + 360.0) == config_label(q)
    q2 = np.array([0.0, -90.0, 180.0, -90.0, -180.0, 0.0])
    assert config_label(q2)[2:] == 'E+W+'


def test_unwrapped_variants_count_and_order() -> None:
    q = np.array([10.0, -20.0, 30.0, -40.0, 50.0, -60.0])
    v = unwrapped_variants(q, UR_LIMITS)
    assert len(v) == 2 ** 5  # elbow +-180 -> 1, all others +-360 -> 2
    for a in v:
        assert np.allclose(np.mod(a - q, 360.0) * (np.mod(a - q, 360.0) < 359.0), 0.0)
        assert all(lo <= x <= hi for x, (lo, hi) in zip(a, UR_LIMITS))
    assert [tuple(a) for a in v] == sorted({tuple(a) for a in v})
    assert len(unwrapped_variants(q, [(-180.0, 180.0)] * 6)) == 1
    assert len(unwrapped_variants(np.zeros(6), UR_LIMITS)) == 3 ** 5
    assert unwrapped_variants(np.array([0, 0, 0, 0, 0, 200.0]), [(-180.0, 180.0)] * 6)[0][5] == -160.0
    assert unwrapped_variants(np.zeros(6), [(10.0, 20.0)] + [(-180.0, 180.0)] * 5) == []


def test_nearest_representative_moves_each_joint_at_most_half_a_turn() -> None:
    rng = np.random.default_rng(SEED + 4)
    for _ in range(1000):
        prev = rng.uniform(-540.0, 540.0, size=6)
        q = rng.uniform(-180.0, 180.0, size=6)
        r = nearest_representative(q, prev)
        assert np.all(np.abs(r - prev) <= 180.0 + 1e-9)
        d = np.mod(r - q, 360.0)
        assert np.all(np.minimum(d, 360.0 - d) < 1e-9)
    np.testing.assert_allclose(nearest_representative([179.0] * 6, [-179.0] * 6), [-181.0] * 6)


def test_singularity_metrics_definition() -> None:
    q = np.array([0.0, -60.0, 70.0, -100.0, 30.0, 0.0])
    m = singularity_metrics(q)
    assert m.sin_q5 == pytest.approx(0.5) and m.sin_q3 == pytest.approx(_sin_deg(70.0))
    c = wrist_centre_lib_mm(q)
    v = c[0] * _sin_deg(0.0) - c[1] * math.cos(0.0)
    assert m.wrist_centre_mm == pytest.approx(math.sqrt(c[0] ** 2 + c[1] ** 2 - v ** 2))
    assert m.wrist_centre_mm == pytest.approx(abs(c[0]))


def test_singular_reason_thresholds_at_the_boundaries() -> None:
    ok = SingularityMetrics(sin_q5=1.0, sin_q3=1.0, wrist_centre_mm=500.0)
    assert singular_reason(ok, THRESH) is None
    sw, se = _sin_deg(8.6), _sin_deg(5.0)
    assert singular_reason(SingularityMetrics(sw, 1.0, 500.0), THRESH) is None
    assert singular_reason(SingularityMetrics(sw * (1 - 1e-9), 1.0, 500.0), THRESH) == 'singular_wrist'
    assert singular_reason(SingularityMetrics(1.0, se, 500.0), THRESH) is None
    assert singular_reason(SingularityMetrics(1.0, se * (1 - 1e-9), 500.0), THRESH) == 'singular_elbow'
    assert singular_reason(SingularityMetrics(1.0, 1.0, 50.0), THRESH) is None
    assert singular_reason(SingularityMetrics(1.0, 1.0, 49.999), THRESH) == 'singular_shoulder'
    # Precedence wrist > elbow > shoulder.
    assert singular_reason(SingularityMetrics(0.0, 0.0, 0.0), THRESH) == 'singular_wrist'
    assert singular_reason(SingularityMetrics(1.0, 0.0, 0.0), THRESH) == 'singular_elbow'


def test_ik_existence_classifies_unreachable_and_singular(kin: UrAnalyticKinematics) -> None:
    far = np.eye(4)
    far[:3, 3] = [2000.0, 0.0, 0.0]
    assert ik_existence(far) == 'unreachable'
    near_axis = np.eye(4)
    near_axis[:3, 3] = [0.0, 0.0, 400.0]
    assert ik_existence(near_axis) == 'unreachable'  # wrist centre inside the d4 cylinder
    for q in _non_singular_states(50, seed=SEED + 5):
        assert ik_existence(kin.fk(q)) == 'reachable'
    q0 = np.array([20.0, -70.0, 80.0, -60.0, 0.0, 30.0])
    assert ik_existence(kin.fk(q0)) == 'singular'
    stretched = np.array([20.0, -90.0, 0.0, -60.0, 70.0, 30.0])
    assert ik_existence(kin.fk(stretched)) == 'singular'


def test_ik_existence_agrees_with_backend(kin: UrAnalyticKinematics) -> None:
    """Every pose the backend solves is 'reachable' or 'singular'; poses classed 'unreachable' have no solution."""
    rng = np.random.default_rng(SEED + 6)
    for _ in range(300):
        t = kin.fk(rng.uniform(-180.0, 180.0, size=6))
        t[:3, 3] += rng.uniform(-400.0, 400.0, size=3)
        cls = ik_existence(t)
        if kin.ik(t):
            assert cls in ('reachable', 'singular'), t
        else:
            assert cls in ('unreachable', 'singular'), t


def test_states_module_is_pure() -> None:
    code = ("import awm_path.core.states, sys; "
            "assert not any(m == 'rclpy' or m.startswith(('rclpy.', 'moveit_msgs')) for m in sys.modules)")
    assert subprocess.run([sys.executable, '-c', code]).returncode == 0


def test_labels_cover_product() -> None:
    assert set(LABELS) == {f'S{a}E{b}W{c}' for a, b, c in itertools.product('+-', repeat=3)}
