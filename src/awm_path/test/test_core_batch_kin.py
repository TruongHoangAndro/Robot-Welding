"""Vectorised UR5e kinematics (G1.3): batch FK / labels / F6 metrics equal the S.2 / S.3 scalar functions, and the
batch IK returns one verified solution per configuration label."""
import numpy as np
import pytest

from awm_core import units
from awm_path.core.batch_kin import LABEL_COUNT, batch_fk, batch_label_metrics, ik_by_label
from awm_path.core.kinematics import UrAnalyticKinematics, pose_error
from awm_path.core.states import LABEL_ORDER, label_and_metrics

SEED = 20261106


@pytest.fixture(scope='module')
def kin():
    return UrAnalyticKinematics()


def _states(n=300):
    rng = np.random.default_rng(SEED)
    q = rng.uniform(-360.0, 360.0, size=(n, 6))
    q[:, 2] = rng.uniform(-179.0, 179.0, size=n)
    return q


def test_batch_fk_equals_backend_fk(kin):
    q = _states()
    t = batch_fk(q)
    for k in range(len(q)):
        dp, dr = pose_error(t[k], kin.fk(q[k]))
        assert dp < 1e-6 and dr < 1e-9


def test_batch_labels_and_metrics_equal_scalar():
    q = _states()
    lab, s5, s3, u = batch_label_metrics(q)
    for k in range(len(q)):
        label, m = label_and_metrics(q[k])
        assert lab[k] == LABEL_ORDER[label]
        assert s5[k] == pytest.approx(m.sin_q5, abs=1e-12)
        assert s3[k] == pytest.approx(m.sin_q3, abs=1e-12)
        assert u[k] == pytest.approx(m.wrist_centre_mm, abs=1e-9)


def test_ik_by_label_verified_and_unique(kin):
    q = _states(60)
    q = q[np.abs(np.sin(units.deg_to_rad(q[:, 4]))) > 0.2]
    targets = batch_fk(q)
    sol, ok = ik_by_label(kin, targets)
    assert sol.shape == (len(q), LABEL_COUNT, 6) and ok.shape == (len(q), LABEL_COUNT)
    assert ok.any(axis=1).all()
    lab, _, _, _ = batch_label_metrics(sol[ok])
    rows = np.nonzero(ok)
    assert np.array_equal(lab, rows[1])               # stored under its own label
    t_back = batch_fk(sol[ok])
    for k, row in enumerate(rows[0]):
        dp, dr = pose_error(t_back[k], targets[row])
        assert dp < 1e-3 and dr < 1e-7
    # the original state is among the solutions (same label, equal modulo 360 deg)
    lab0, _, _, _ = batch_label_metrics(q)
    for k in range(len(q)):
        assert ok[k, lab0[k]]
        d = (sol[k, lab0[k]] - q[k] + 180.0) % 360.0 - 180.0
        assert np.abs(d).max() < 1e-6


def test_ik_unreachable_gives_no_solution(kin):
    far = np.eye(4)
    far[:3, 3] = [5000.0, 0.0, 0.0]
    sol, ok = ik_by_label(kin, far[None])
    assert not ok.any()
    assert np.isnan(sol).all()
