"""ChainParams defaults are the D16 file values (one effective definition)."""

from dataclasses import asdict
from pathlib import Path

import pytest

from awm_path.params import ChainParams, from_yaml


def _params_file() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('awm_bringup')) / 'config' / 'awm_params.yaml'


def test_yaml_matches_defaults():
    loaded = asdict(from_yaml(_params_file()))
    defaults = asdict(ChainParams())
    assert set(loaded) == set(defaults)
    for key, value in defaults.items():
        assert loaded[key] == pytest.approx(value) if isinstance(value, (float, list)) \
            else loaded[key] == value, key


def test_unknown_key_rejected(tmp_path):
    bad = tmp_path / 'p.yaml'
    bad.write_text('/**:\n  ros__parameters:\n    speed_mm_s: 3\n')
    with pytest.raises(KeyError, match='speed_mm_s'):
        from_yaml(bad)


SPLIT_ARM = '/joint_trajectory_controller/follow_joint_trajectory'
SEVEN = '/robot_positioner_trajectory_controller/follow_joint_trajectory'


def test_controller_action_present_passes():
    """G1.13 request 1: the configured action is running (sim_mock trajectory_controllers:=split)."""
    from awm_path.params import check_controller_action
    check_controller_action(SPLIT_ARM, [SPLIT_ARM, '/positioner_trajectory_controller/follow_joint_trajectory'])


def test_controller_action_missing_in_robot_positioner_mode_names_the_launch_option():
    """With trajectory_controllers:=robot_positioner only the 7-joint action exists: fail at once, name the option."""
    from awm_path.params import ControllerActionMissing, check_controller_action
    with pytest.raises(ControllerActionMissing) as exc:
        check_controller_action(SPLIT_ARM, [SEVEN])
    msg = str(exc.value)
    assert SPLIT_ARM in msg and 'trajectory_controllers:=robot_positioner' in msg
    assert 'trajectory_controllers:=split' in msg


def test_controller_action_missing_without_any_controller():
    from awm_path.params import ControllerActionMissing, check_controller_action
    with pytest.raises(ControllerActionMissing, match='not running'):
        check_controller_action(SPLIT_ARM, [])
