"""Tests of the tool pose per ψ and the kinematic candidate sweep (WP-S S.3, TST-002 F1, F2, F6)."""
import filecmp
import math
from typing import List

import numpy as np
import pytest

from awm_path.core.kinematics import UrAnalyticKinematics, fk_tcp, pose_error
from awm_path.core.states import config_label
from awm_path.core.sweep import (CSV_COLUMNS, PlannerParams, pitch_points, required_points, sweep, tcp_pose,
                                 write_candidates_csv)

R_MM = 13.3
N_POINTS = 72
TOOL_Z_MM = 200.0  # synthetic tool0 -> tcp offset of the test only


def circle_frames(centre, axis, r: float, n: int) -> List[np.ndarray]:
    """Seam frames of a synthetic closed circle (butt joint between tubes): Z radial out, X = axis × Z, Y = axis."""
    a = np.asarray(axis, dtype=float)
    a /= np.linalg.norm(a)
    e1 = np.cross(a, [0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.cross(a, [1.0, 0.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(a, e1)
    out = []
    for k in range(n):
        th = 2.0 * math.pi * k / n
        z = math.cos(th) * e1 + math.sin(th) * e2
        t = np.eye(4)
        t[:3, 0] = np.cross(a, z)
        t[:3, 1] = a
        t[:3, 2] = z
        t[:3, 3] = np.asarray(centre, dtype=float) + r * z
        out.append(t)
    return out


#: Reachable placement in front of the robot (base_link frame), tube axis horizontal.
CENTRE = (0.0, -450.0, 250.0)
AXIS = (0.0, 1.0, 0.0)


def tool() -> np.ndarray:
    t = np.eye(4)
    t[2, 3] = TOOL_Z_MM
    return t


@pytest.fixture(scope='module')
def kin() -> UrAnalyticKinematics:
    return UrAnalyticKinematics()


@pytest.fixture(scope='module')
def params() -> PlannerParams:
    return PlannerParams()


@pytest.fixture(scope='module')
def frames() -> List[np.ndarray]:
    return circle_frames(CENTRE, AXIS, R_MM, N_POINTS)


@pytest.fixture(scope='module')
def layers(frames, kin, params):
    return sweep(frames, tool(), kin, params, params.psi_free)


def test_params_defaults_follow_tst002(params: PlannerParams) -> None:
    assert params.pitch_max_mm == 3.0 and params.dq_max_deg == 5.0
    assert params.psi_free == tuple(float(x) for x in range(0, 360, 10))
    assert params.psi_wire == (-15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0)
    assert (params.singular_wrist_deg, params.singular_elbow_deg, params.singular_shoulder_mm) == (8.6, 5.0, 50.0)
    assert (params.wrist3_abs_deg, params.wrist3_travel_deg) == (270.0, 180.0)
    assert params.effective_limits_deg()[5] == (-270.0, 270.0)


def test_tcp_pose_beam_axis_and_psi() -> None:
    f = circle_frames(CENTRE, AXIS, R_MM, 8)[3]
    for psi in (0.0, 10.0, 125.0, 350.0):
        t = tcp_pose(f, psi)
        np.testing.assert_allclose(t[:3, 2], -f[:3, 2], atol=1e-12)
        np.testing.assert_allclose(t[:3, 3], f[:3, 3], atol=1e-12)
        cos_psi = float(t[:3, 0] @ f[:3, 0])
        assert cos_psi == pytest.approx(math.cos(math.radians(psi)), abs=1e-12)
    np.testing.assert_allclose(tcp_pose(f, 0.0)[:3, 0], f[:3, 0], atol=1e-12)


def test_pitch_rule_f1(frames, params) -> None:
    rate = 360.0 / (2.0 * math.pi * R_MM)
    assert params.dq_max_deg / rate == pytest.approx(1.1606, abs=1e-4)
    assert required_points(2.0 * math.pi * R_MM, rate, params, closed=True) == N_POINTS
    chk = pitch_points(frames, params, closed=True)
    assert chk.ok and len(chk.step_mm) == N_POINTS
    assert max(chk.rotation_deg) == pytest.approx(5.0, abs=1e-9)
    coarse = pitch_points(circle_frames(CENTRE, AXIS, R_MM, 60), params, closed=True)
    assert not coarse.ok and len(coarse.failing_steps) == 60
    line = []
    for k in range(4):
        t = np.eye(4)
        t[0, 3] = 3.5 * k
        line.append(t)
    chk_line = pitch_points(line, params, closed=False)
    assert not chk_line.ok and chk_line.limit_mm == (3.0, 3.0, 3.0)


def test_every_point_has_valid_candidates(layers) -> None:
    assert len(layers) == N_POINTS
    for layer in layers:
        assert any(c.reason is None for c in layer)


def test_candidates_put_tcp_on_the_seam(layers, frames, kin) -> None:
    for layer in layers[::6]:
        for c in layer:
            if c.q_deg is None:
                continue
            target = tcp_pose(frames[c.point], c.psi_deg)
            got = fk_tcp(kin, c.q_deg, tool())
            dp, _ = pose_error(got, target)
            assert dp < 0.01
            np.testing.assert_allclose(got[:3, 2], -frames[c.point][:3, 2], atol=1e-9)
            assert c.label == config_label(c.q_deg)


def test_layer_order_is_psi_then_label(layers) -> None:
    from awm_path.core.states import LABEL_ORDER
    for layer in layers[:3]:
        keys = [(c.psi_index, LABEL_ORDER[c.label] if c.label else -1) for c in layer]
        assert keys == sorted(keys)
        for j in range(36):
            labels = [c.label for c in layer if c.psi_index == j]
            assert len(labels) == len(set(labels))


def test_far_seam_is_unreachable(kin, params) -> None:
    far = circle_frames((2000.0, 0.0, 300.0), AXIS, R_MM, 6)
    out = sweep(far, tool(), kin, params, params.psi_free)
    assert all(c.reason == 'no_ik_unreachable' and c.q_deg is None for layer in out for c in layer)
    assert all(len(layer) == 36 for layer in out)


def test_wrist_singular_frame(kin, params) -> None:
    q = np.array([20.0, -100.0, 100.0, -60.0, 3.0, 30.0])
    frame = fk_tcp(kin, q, tool()) @ np.diag([1.0, -1.0, -1.0, 1.0])  # tcp = frame · Rx(180)
    out = sweep([frame], tool(), kin, params, (0.0,))
    match = [c for c in out[0] if c.q_deg is not None and np.allclose(c.q_deg, q, atol=1e-6)]
    assert len(match) == 1 and match[0].reason == 'singular_wrist'
    assert match[0].metrics.sin_q5 == pytest.approx(math.sin(math.radians(3.0)))


def test_joint_limit_reason(kin) -> None:
    tight = PlannerParams(joint_limits_deg=((-10.0, 10.0),) + ((-360.0, 360.0),) * 5)
    f = circle_frames(CENTRE, AXIS, R_MM, 4)
    out = sweep(f, tool(), kin, tight, (0.0,))
    reasons = {c.reason for layer in out for c in layer}
    assert 'joint_limit' in reasons
    for layer in out:
        for c in layer:
            if c.reason != 'joint_limit' and c.q_deg is not None:
                assert -10.0 <= c.q_deg[0] <= 10.0


def test_sweep_and_csv_are_deterministic(tmp_path, frames, kin, params, layers) -> None:
    again = sweep(frames, tool(), kin, params, params.psi_free)
    assert again == layers
    a, b = tmp_path / 'a.csv', tmp_path / 'b.csv'
    extra = {'valid': [[True] * len(layer) for layer in layers]}
    write_candidates_csv(str(a), layers, extra)
    write_candidates_csv(str(b), again, extra)
    assert filecmp.cmp(a, b, shallow=False)
    header = a.read_text().splitlines()[0].split(',')
    assert header == list(CSV_COLUMNS) + ['valid']
    with pytest.raises(ValueError):
        write_candidates_csv(str(a), layers, {'bad': [[1]]})
