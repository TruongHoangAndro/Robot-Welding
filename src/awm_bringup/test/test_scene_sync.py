"""Checker-side guard: a validity request must use the positioner angle the cell scene has applied."""

import json

import pytest

from awm_bringup.scene_sync import PositionerMismatch, check_positioner, trajectory_duration_s
from awm_core import units


def _status(theta_deg, loaded=True, revision=4):
    return json.dumps({'loaded': loaded, 'revision': revision, 'theta_deg': theta_deg})


NAMES = ['shoulder_pan_joint', 'positioner_joint']


def test_matching_angle_returns_revision():
    assert check_positioner(NAMES, [0.0, units.deg_to_rad(90.005)], _status(90.0)) == 4


@pytest.mark.parametrize('theta_req_deg', [90.02, 0.0, -90.0])
def test_mismatch_refused(theta_req_deg):
    with pytest.raises(PositionerMismatch):
        check_positioner(NAMES, [0.0, units.deg_to_rad(theta_req_deg)], _status(90.0))


def test_missing_joint_or_scene_refused():
    with pytest.raises(PositionerMismatch, match='positioner_joint'):
        check_positioner(['shoulder_pan_joint'], [0.0], _status(0.0))
    with pytest.raises(PositionerMismatch, match='not loaded'):
        check_positioner(NAMES, [0.0, 0.0], _status(None, loaded=False, revision=0))
    with pytest.raises(PositionerMismatch, match='not loaded'):
        check_positioner(NAMES, [0.0, 0.0], None)


def test_trajectory_duration_respects_limits():
    lim = {'max_velocity_deg_s': 30.0, 'max_acceleration_deg_s2': 50.0}
    t = trajectory_duration_s(90.0, lim)
    # cubic segment: peak speed 1.5 d / T, peak acceleration 6 d / T^2 -- both within the limits
    assert 1.5 * 90.0 / t <= 30.0 and 6 * 90.0 / t ** 2 <= 50.0
    assert trajectory_duration_s(0.0, lim) > 0
