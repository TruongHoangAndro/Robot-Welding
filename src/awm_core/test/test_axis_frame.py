"""Positioner-axis frame rule shared by the cell and the job import (CP-60)."""
import numpy as np
import pytest

from awm_core.axis_frame import file_T_axis, world_R_file


def test_axis_frame_matches_scene_rule():
    t = file_T_axis([1, 2, 3], [1, 0, 0], '+Y')
    np.testing.assert_allclose(t[:3, 3], [1, 2, 3])
    np.testing.assert_allclose(t[:3, 0], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(world_R_file('+Y') @ t[:3, 2], [0, 0, 1], atol=1e-12)
    np.testing.assert_allclose(t[:3, :3] @ t[:3, :3].T, np.eye(3), atol=1e-12)
    assert np.linalg.det(t[:3, :3]) == pytest.approx(1.0)


def test_axis_frame_tilted_axis_keeps_z_up():
    axis = np.array([1.0, 0.0, 1.0]) / np.sqrt(2.0)          # +Z-up file, axis 45 deg up
    t = file_T_axis([0, 0, 0], axis, '+Z')
    np.testing.assert_allclose(t[:3, 0], axis, atol=1e-12)
    assert t[2, 2] > 0 and abs(t[:3, 2] @ axis) < 1e-12


def test_axis_frame_normalises_direction():
    np.testing.assert_allclose(file_T_axis([0, 0, 0], [2, 0, 0], '+Z')[:3, 0], [1, 0, 0])


def test_axis_frame_vertical_refused():
    with pytest.raises(ValueError, match='vertical'):
        file_T_axis([0, 0, 0], [0, 1, 0], '+Y')


def test_unknown_up_axis_refused():
    with pytest.raises(ValueError, match='up axis'):
        world_R_file('Y')
