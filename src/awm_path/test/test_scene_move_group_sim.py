"""Integration (WP-S S.2): MoveGroupSceneBackend against move_group on mock hardware.

Needs no OCC interpreter. Starts ``sim.launch.py headless:=true`` in a private ROS domain.
"""
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from awm_bringup import params_file  # noqa: E402
from awm_bringup.testing import SimLaunch  # noqa: E402
from awm_core import units  # noqa: E402
from awm_path.backends.move_group import MoveGroupSceneBackend  # noqa: E402
from awm_path.core.scene import StateCheck  # noqa: E402
from awm_path.moveit_client import MoveItClient  # noqa: E402
from awm_path.params import from_yaml  # noqa: E402

SEED = 20261102


@pytest.fixture(scope='module')
def scene(tmp_path_factory):
    tmp = tmp_path_factory.mktemp('scene')
    params = from_yaml(params_file())
    with SimLaunch(log_dir=tmp):
        rclpy.init()
        node = rclpy.create_node('awm_path_scene_test')
        client = MoveItClient(node, params.planning_group, params.tcp_link, params.base_frame,
                              params.joint_names, params.service_timeout_s)
        try:
            client.wait_ready(120.0)
            yield MoveGroupSceneBackend(client), client
        finally:
            client.close()
            node.destroy_node()
            rclpy.shutdown()


def test_home_is_valid(scene) -> None:
    backend, _ = scene
    assert backend.check([[0.0, -90.0, 90.0, -90.0, -90.0, 0.0]]) == [StateCheck(valid=True, contacts=())]


def test_arm_pointing_down_collides(scene) -> None:
    backend, _ = scene
    (result,) = backend.check([[0.0, 90.0, 0.0, 0.0, 0.0, 0.0]])
    assert result.valid is False
    assert result.contacts


def test_matches_move_group_per_state(scene) -> None:
    backend, client = scene
    states = np.random.default_rng(SEED).uniform(-180.0, 180.0, size=(20, 6))
    results = backend.check(states)
    assert len(results) == len(states)
    for q, res in zip(states, results):
        valid, contacts = client.state_valid(units.deg_to_rad(q))
        assert res == StateCheck(valid=valid, contacts=tuple(contacts))


def test_check_rejects_wrong_joint_count(scene) -> None:
    backend, _ = scene
    with pytest.raises(ValueError):
        backend.check([[0.0] * 7])
