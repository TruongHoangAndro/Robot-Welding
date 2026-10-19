"""Frame math."""

import numpy as np
import pytest

from awm_core import frames


def _random_rot(rng):
    q = rng.normal(size=4)
    return frames.quat_to_matrix(q / np.linalg.norm(q))


def test_quat_matrix_roundtrip():
    rng = np.random.default_rng(1)
    for _ in range(200):
        r = _random_rot(rng)
        assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)
        assert np.isclose(np.linalg.det(r), 1.0)
        q = frames.matrix_to_quat(r)
        assert q[3] >= 0
        assert np.allclose(frames.quat_to_matrix(q), r, atol=1e-12)


def test_rpy_roundtrip_and_convention():
    r = frames.rpy_to_matrix(0.0, 0.0, np.pi / 2)
    assert np.allclose(r @ [1, 0, 0], [0, 1, 0])
    rng = np.random.default_rng(2)
    for _ in range(100):
        rpy = rng.uniform(-1.4, 1.4, size=3)
        assert np.allclose(frames.matrix_to_rpy(frames.rpy_to_matrix(*rpy)), rpy)


def test_compose_invert():
    rng = np.random.default_rng(3)
    a = frames.make_transform(rng.normal(size=3), _random_rot(rng))
    b = frames.make_transform(rng.normal(size=3), _random_rot(rng))
    assert np.allclose(frames.compose(a, frames.invert(a)), np.eye(4))
    p = np.array([0.1, 0.2, 0.3, 1.0])
    assert np.allclose(frames.compose(a, b) @ p, a @ (b @ p))


def test_pose_dict_roundtrip():
    t = frames.xyz_rpy_to_matrix([0.4, -0.1, 0.02], [0.1, 0.2, 0.3])
    assert np.allclose(frames.pose_to_matrix(frames.matrix_to_pose(t)), t)


def test_frame_from_z_x():
    r = frames.frame_from_z_x([0, 0, -2], [1, 0, 0.5])
    assert np.allclose(r[:, 2], [0, 0, -1])
    assert np.allclose(r[:, 0], [1, 0, 0])
    assert np.isclose(np.linalg.det(r), 1.0)
    with pytest.raises(ValueError):
        frames.frame_from_z_x([0, 0, 1], [0, 0, 3])


def test_axis_angle():
    r = frames.axis_angle_to_matrix([0, 0, 1], np.pi / 2)
    assert np.allclose(r @ [1, 0, 0], [0, 1, 0])
    assert np.isclose(frames.rotation_angle_between(np.eye(3), r), np.pi / 2)
