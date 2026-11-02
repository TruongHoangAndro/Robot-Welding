"""Pure tests of the M4 core SceneBackend types (WP-S S.2, B-06)."""
import dataclasses
import subprocess
import sys

import pytest

from awm_path.core.scene import StateCheck, validate_states


def test_check_rejects_wrong_joint_count() -> None:
    with pytest.raises(ValueError):
        validate_states([[0.0] * 7])
    assert validate_states([]).shape == (0, 6)


def test_validate_states_rejects_non_finite() -> None:
    with pytest.raises(ValueError):
        validate_states([[0.0, 0.0, 0.0, 0.0, float('nan'), 0.0]])


def test_state_check_is_immutable() -> None:
    check = StateCheck(True, ())
    with pytest.raises(dataclasses.FrozenInstanceError):
        check.valid = False


def test_core_scene_does_not_import_rclpy() -> None:
    code = ("import awm_path.core.scene, sys; "
            "assert not any(m == 'rclpy' or m.startswith(('rclpy.', 'moveit_msgs')) for m in sys.modules)")
    assert subprocess.run([sys.executable, '-c', code]).returncode == 0
