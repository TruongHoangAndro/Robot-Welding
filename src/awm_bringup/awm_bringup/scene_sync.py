"""Keep validity checks and the positioner angle of the cell scene consistent (WP-S S.4 review, finding 1).

The rotating set (holders, jig, part) is a set of planning-scene world objects placed by ``cell_scene_loader`` at the
positioner angle it has *applied* (``/awm/cell_scene/status``: ``theta_deg``, ``revision``). A
``check_state_validity`` request with another ``positioner_joint`` value does not move them, so its answer would be
wrong. Rules for every checker:

1. Move the positioner first: :meth:`SceneSync.set_positioner_and_wait` commands the mock positioner controller and
   returns only when the status reports that angle applied (returns the scene revision).
2. Guard every request: :func:`check_positioner` raises :class:`PositionerMismatch` when the request has no
   ``positioner_joint`` value, the scene is not loaded, or the value differs from the applied angle by more than
   ``TOL_DEG``. A refused request is never reported as valid.

Mock hardware only (ADR-13): the helper sends trajectories to ``positioner_trajectory_controller`` within the limits
of ``positioner_limits.yaml``. That controller exists only with ``sim_mock trajectory_controllers:=split`` (default);
with ``robot_positioner`` (one 7-joint controller, G1.7) :meth:`SceneSync.set_positioner_and_wait` fails at once
with a message (:func:`check_controller_mode`) instead of waiting for a server that is never started.
"""

from __future__ import annotations

import json
import math
import time
from typing import Dict, Optional, Sequence, Union

from awm_core import units

TOL_DEG = 0.01
JOINT = 'positioner_joint'
STATUS_TOPIC = '/awm/cell_scene/status'
CONTROLLER_ACTION = '/positioner_trajectory_controller/follow_joint_trajectory'
SEVEN_JOINT_ACTION = '/robot_positioner_trajectory_controller/follow_joint_trajectory'


class PositionerMismatch(RuntimeError):
    """The request does not match the positioner angle applied to the cell scene."""


def _parse(status: Union[None, str, dict]) -> Optional[dict]:
    if status is None:
        return None
    return json.loads(status) if isinstance(status, str) else status


def check_positioner(names: Sequence[str], positions_rad: Sequence[float], status: Union[None, str, dict],
                     joint: str = JOINT, tol_deg: float = TOL_DEG) -> int:
    """Raise unless ``joint`` in (names, positions) equals the applied angle; return the scene revision."""
    st = _parse(status)
    if not st or not st.get('loaded') or st.get('theta_deg') is None:
        raise PositionerMismatch('cell scene not loaded: no applied positioner angle')
    names = list(names)
    if joint not in names:
        raise PositionerMismatch(f'request has no {joint} value; give it explicitly (applied {st["theta_deg"]} deg)')
    req_deg = units.rad_to_deg(positions_rad[names.index(joint)])
    if abs(req_deg - float(st['theta_deg'])) > tol_deg:
        raise PositionerMismatch(f'request {joint} = {req_deg:.4f} deg, cell scene at {st["theta_deg"]:.4f} deg '
                                 f'(revision {st["revision"]}): move the positioner first')
    return int(st['revision'])


def check_robot_state(robot_state, status: Union[None, str, dict], joint: str = JOINT, tol_deg: float = TOL_DEG
                      ) -> int:
    """:func:`check_positioner` for a ``moveit_msgs/RobotState`` (e.g. ``GetStateValidity.Request.robot_state``)."""
    js = robot_state.joint_state
    return check_positioner(js.name, js.position, status, joint, tol_deg)


def check_controller_mode(action_names_and_types) -> None:
    """Raise when the 7-joint controller runs and the split positioner controller does not (wrong launch mode)."""
    names = {name for name, _types in action_names_and_types}
    if SEVEN_JOINT_ACTION in names and CONTROLLER_ACTION not in names:
        raise RuntimeError(f'{CONTROLLER_ACTION} is not running: sim_mock was launched with '
                           'trajectory_controllers:=robot_positioner (one 7-joint controller). Checkers that move the '
                           'positioner (SceneSync, awm_path move_group backend) need trajectory_controllers:=split')


def trajectory_duration_s(delta_deg: float, lim: Dict[str, float], margin: float = 1.25) -> float:
    """Duration of a one-segment cubic move of |delta_deg| within the velocity / acceleration limits."""
    d = abs(delta_deg)
    t_v = 1.5 * d / lim['max_velocity_deg_s']
    t_a = math.sqrt(6.0 * d / lim['max_acceleration_deg_s2'])
    return max(margin * max(t_v, t_a), 0.2)


class SceneSync:
    """Helper on an rclpy node (the caller spins only through this helper while waiting)."""

    def __init__(self, node, limits: Optional[Dict[str, float]] = None) -> None:
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
        from control_msgs.action import FollowJointTrajectory
        from awm_bringup.cell_config import load_positioner_limits
        self.node = node
        self.limits = limits or load_positioner_limits()
        self.status: Optional[dict] = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, STATUS_TOPIC, self._on_status, latched)
        self.action = ActionClient(node, FollowJointTrajectory, CONTROLLER_ACTION)

    def _on_status(self, msg) -> None:
        self.status = json.loads(msg.data)

    def _spin_until(self, predicate, timeout_s: float, what: str) -> None:
        import rclpy
        deadline = time.monotonic() + timeout_s
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError(f'{what} not reached after {timeout_s:.0f} s (status {self.status})')
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_loaded(self, timeout_s: float = 90.0) -> dict:
        """Block until the cell scene is loaded; return the status."""
        self._spin_until(lambda: bool(self.status and self.status.get('loaded')), timeout_s, 'cell scene loaded')
        return self.status

    def check(self, robot_state) -> int:
        """Guard a request against the current status (raises :class:`PositionerMismatch`)."""
        return check_robot_state(robot_state, self.status)

    def set_positioner_and_wait(self, theta_deg: float, timeout_s: float = 60.0) -> int:
        """Move the mock positioner to theta_deg, wait until the scene applied it, return the scene revision."""
        import rclpy
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        lim = self.limits
        if not lim['lower_deg'] <= theta_deg <= lim['upper_deg']:
            raise ValueError(f'{theta_deg} deg outside the positioner limits {lim["lower_deg"]}..{lim["upper_deg"]}')
        st = self.wait_loaded(timeout_s)
        if abs(theta_deg - st['theta_deg']) > TOL_DEG / 10:
            from rclpy.action import get_action_names_and_types
            check_controller_mode(get_action_names_and_types(self.node))
            if not self.action.wait_for_server(timeout_sec=timeout_s):
                raise TimeoutError(f'{CONTROLLER_ACTION} not available')
            dur = trajectory_duration_s(theta_deg - st['theta_deg'], lim)
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [JOINT]
            sec = int(dur)
            goal.trajectory.points = [JointTrajectoryPoint(
                positions=[units.deg_to_rad(theta_deg)], velocities=[0.0],
                time_from_start=Duration(sec=sec, nanosec=int((dur - sec) * 1e9)))]
            fut = self.action.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout_s)
            handle = fut.result()
            if handle is None or not handle.accepted:
                raise RuntimeError('positioner goal rejected')
            res = handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout_s + dur)
            if res.result() is None or res.result().result.error_code != 0:
                raise RuntimeError(f'positioner move failed: {res.result()}')
        def applied() -> bool:
            th = self.status.get('theta_deg') if self.status else None
            return th is not None and abs(float(th) - theta_deg) <= TOL_DEG

        self._spin_until(applied, timeout_s, f'positioner {theta_deg} deg applied in the cell scene')
        return int(self.status['revision'])
