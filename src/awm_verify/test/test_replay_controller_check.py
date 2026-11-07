"""G1.14 (review 2.a of G1.12): Replayer.wait_ready names the sim_mock launch option when the configured
controller action is absent because only the 7-joint controller runs (``trajectory_controllers:=robot_positioner``).

No ROS graph: the action client and the graph query are replaced by fakes (fake action-name lists).
"""
import pytest

pytest.importorskip('rclpy', reason='rclpy not available (run inside awm_dev)')

from awm_path.params import SEVEN_JOINT_CONTROLLER_ACTION  # noqa: E402
from awm_verify import replay  # noqa: E402

ARM_ACTION = '/joint_trajectory_controller/follow_joint_trajectory'


class _Client:
    def __init__(self, available: bool) -> None:
        self.available = available
        self.waited = False

    def wait_for_server(self, timeout_sec: float) -> bool:
        self.waited = True
        return self.available


def _replayer(monkeypatch, available: bool, names):
    calls = []

    def fake_names(node):
        calls.append(node)
        return [(n, ['control_msgs/action/FollowJointTrajectory']) for n in names]

    monkeypatch.setattr(replay, 'get_action_names_and_types', fake_names)
    r = replay.Replayer.__new__(replay.Replayer)          # no node, no subscription
    r.node = object()
    r._action_name = ARM_ACTION
    r._client = _Client(available)
    return r, calls


def test_only_the_seven_joint_controller_names_the_launch_option(monkeypatch):
    r, calls = _replayer(monkeypatch, False, [SEVEN_JOINT_CONTROLLER_ACTION])
    with pytest.raises(replay.ReplayError, match=r'trajectory_controllers:=robot_positioner.*trajectory_controllers:=split'):
        r.wait_ready(0.01)
    assert r._client.waited and calls == [r.node]          # graph queried only after the wait timed out


def test_no_controller_at_all_still_refuses_with_a_clear_message(monkeypatch):
    r, _ = _replayer(monkeypatch, False, [])
    with pytest.raises(replay.ReplayError, match=f'{ARM_ACTION} is not running'):
        r.wait_ready(0.01)


def test_listed_but_not_answering_keeps_the_previous_message(monkeypatch):
    r, _ = _replayer(monkeypatch, False, [ARM_ACTION])
    with pytest.raises(replay.ReplayError, match=r'is sim\.launch\.py running\?'):
        r.wait_ready(0.01)


def test_graph_not_queried_when_the_server_answers(monkeypatch):
    r, calls = _replayer(monkeypatch, True, [])
    monkeypatch.setattr(r, 'current', lambda timeout_s=5.0: None)
    r.wait_ready(0.01)
    assert calls == []
