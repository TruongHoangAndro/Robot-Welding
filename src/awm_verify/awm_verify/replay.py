"""Replay of program segments on ros2_control mock hardware (A71).

Only the simulation controller is ever addressed: the action name comes from D16
(``controller_action``) and sim.launch.py runs mock_components only.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from awm_program.timing import joint_velocities


class ReplayError(RuntimeError):
    """The controller is unavailable or rejected / aborted a segment."""


@dataclass
class SegmentResult:
    segment_id: str
    ok: bool
    detail: str
    start_error_rad: float
    end_error_rad: float
    duration_s: float


def _duration(t: float) -> Duration:
    sec = int(t)
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)))


class Replayer:
    """Sends one segment at a time to the joint trajectory controller and checks the state."""

    def __init__(self, node, action_name: str, joint_names: Sequence[str]) -> None:
        self.node = node
        self.joint_names = list(joint_names)
        self._client = ActionClient(node, FollowJointTrajectory, action_name)
        self._lock = threading.Lock()
        self._state: Optional[np.ndarray] = None
        node.create_subscription(JointState, '/joint_states', self._on_state, 10)

    def _on_state(self, msg: JointState) -> None:
        try:
            q = np.array([msg.position[list(msg.name).index(j)] for j in self.joint_names])
        except ValueError:
            return
        with self._lock:
            self._state = q

    def current(self, timeout_s: float = 5.0) -> np.ndarray:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._state is not None:
                    return self._state.copy()
            time.sleep(0.02)
        raise ReplayError('no /joint_states received')

    def wait_ready(self, timeout_s: float = 60.0) -> None:
        if not self._client.wait_for_server(timeout_sec=timeout_s):
            raise ReplayError(f'trajectory controller action {self._client._action_name} not '
                              'available (is sim.launch.py running?)')
        self.current(timeout_s)

    def _wait(self, future, timeout_s: float):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout_s):
            raise ReplayError('controller did not answer in time')
        return future.result()

    def run_segment(self, segment: Dict, speed_scale: float, start_tol: float,
                    end_tol: float) -> SegmentResult:
        q = np.array([p['q_rad'] for p in segment['points']])
        t = np.array([p['t_s'] for p in segment['points']]) / speed_scale
        start_err = float(np.max(np.abs(self.current() - q[0])))
        if start_err > start_tol:
            return SegmentResult(segment['segment_id'], False,
                                 f'start state differs by {start_err:.4f} rad', start_err,
                                 float('nan'), 0.0)
        v = joint_velocities(q, t)
        traj = JointTrajectory(joint_names=self.joint_names)
        traj.points = [JointTrajectoryPoint(positions=[float(x) for x in qi],
                                            velocities=[float(x) for x in vi],
                                            time_from_start=_duration(float(ti)))
                       for qi, vi, ti in zip(q, v, t)]
        if traj.points[0].time_from_start == _duration(0.0):
            # The first point equals the current state; the controller starts from it.
            traj.points[0].time_from_start = _duration(1e-3)
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        t0 = time.monotonic()
        handle = self._wait(self._client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise ReplayError(f'{segment["segment_id"]}: goal rejected by the controller')
        result = self._wait(handle.get_result_async(), float(t[-1]) + 30.0)
        elapsed = time.monotonic() - t0
        time.sleep(0.1)  # let /joint_states catch up with the final command
        end_err = float(np.max(np.abs(self.current() - q[-1])))
        code = result.result.error_code
        ok = code == FollowJointTrajectory.Result.SUCCESSFUL and end_err <= end_tol
        detail = 'ok' if ok else (f'controller error {code} {result.result.error_string}'
                                  if code != FollowJointTrajectory.Result.SUCCESSFUL
                                  else f'final error {end_err:.4f} rad')
        return SegmentResult(segment['segment_id'], ok, detail, start_err, end_err, elapsed)

    def run_program(self, segments: List[Dict], speed_scale: float, start_tol: float,
                    end_tol: float) -> List[SegmentResult]:
        results = []
        for seg in segments:
            res = self.run_segment(seg, speed_scale, start_tol, end_tol)
            results.append(res)
            if not res.ok:
                break  # never continue after a failed segment
        return results
