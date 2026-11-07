"""Replay a D09 program segment by segment on the 7-joint controller of ``sim_mock`` (G1.7). Mock hardware only.

    ros2 launch awm_bringup sim_mock.launch.py trajectory_controllers:=robot_positioner
    ros2 run awm_bringup replay_program --program <job>/programs/P###/program.json [--config <yaml>]
        [--segments G002-G004]

Before the first segment the controller manager must report only mock hardware components
(``/controller_manager/list_hardware_components``, plugin ``mock_components/GenericSystem``), and the configured
action must be the 7-joint controller; otherwise the CLI refuses (exit 2) and sends nothing.

Per segment: read ``/joint_states``; if any joint differs from the segment start by more than
``start_state_tolerance_deg`` the segment is **not sent** and the CLI stops (exit 2). Otherwise the segment is sent
unchanged (program timing, no scaling) as one ``FollowJointTrajectory`` goal to ``controller_action``; the largest
tracking error from the controller feedback and the end deviation of ``/joint_states`` from the last point are
logged. The end deviation must stay within ``final_tolerance_deg``.

Output: log lines per segment and one machine-readable last line ``replay {json}`` on stdout.
Exit codes: 0 replayed; 2 refused — status ``refused`` when nothing moved, ``partial`` when the refusal (start
state of segment N > 1) came after N − 1 segments were already executed (the refused segment is never sent);
1 error (including a command-line usage error).

The replay sends joint motion only: weld events of the program are listed in the log, never executed (no PLC, no
laser; DES-001 ADR-04).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Dict, List, Optional

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListHardwareComponents
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from awm_bringup import replay

EXIT_OK, EXIT_ERROR, EXIT_REFUSED = 0, 1, 2
HW_SERVICE = '/controller_manager/list_hardware_components'


def _duration(t_s: float) -> Duration:
    sec = int(t_s)
    return Duration(sec=sec, nanosec=int(round((t_s - sec) * 1e9)))


def build_goal(seg: replay.ReplaySegment, names: List[str]) -> FollowJointTrajectory.Goal:
    """One segment as a controller goal (rad; header stamp 0 = start now)."""
    times, pos, vel, acc = seg.trajectory_rad()
    traj = JointTrajectory(joint_names=list(names))
    for t, p, v, a in zip(times, pos, vel, acc):
        traj.points.append(JointTrajectoryPoint(positions=p, velocities=v, accelerations=a,
                                                time_from_start=_duration(t)))
    return FollowJointTrajectory.Goal(trajectory=traj)


class Replayer:
    """rclpy side of the replay: joint state, action client, per-segment check and summary."""

    def __init__(self, node: Node, cfg: replay.ReplayConfig) -> None:
        self.node = node
        self.cfg = cfg
        self.log = node.get_logger()
        self._js: Optional[JointState] = None
        self._js_count = 0
        node.create_subscription(JointState, '/joint_states', self._on_js, 50)
        self.client = ActionClient(node, FollowJointTrajectory, cfg.controller_action)

    def hardware_components(self) -> List[tuple]:
        """(name, plugin) of every hardware component from the controller manager."""
        cli = self.node.create_client(ListHardwareComponents, HW_SERVICE)
        if not cli.wait_for_service(timeout_sec=self.cfg.wait_timeout_s):
            raise replay.ReplayRefused(f'{HW_SERVICE} not available: cannot prove mock hardware')
        fut = cli.call_async(ListHardwareComponents.Request())
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=self.cfg.wait_timeout_s)
        if not fut.done() or fut.result() is None:
            raise replay.ReplayRefused(f'{HW_SERVICE} did not answer: cannot prove mock hardware')
        return [(c.name, c.plugin_name or c.class_type) for c in fut.result().component]

    def _on_js(self, msg: JointState) -> None:
        self._js = msg
        self._js_count += 1

    def _spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    def fresh_state_deg(self) -> Dict[str, float]:
        """A /joint_states message received after this call started, as {joint: deg}."""
        start = self._js_count
        deadline = time.monotonic() + self.cfg.wait_timeout_s
        while self._js_count <= start:
            if time.monotonic() > deadline:
                raise TimeoutError(f'no /joint_states within {self.cfg.wait_timeout_s} s')
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return replay.positions_deg(self._js.name, self._js.position)

    def run_segment(self, seg: replay.ReplaySegment, names: List[str]) -> Dict:
        start_dev = replay.check_start_state(seg, names, self.fresh_state_deg(), self.cfg.start_state_tolerance_deg)
        stats = replay.TrackingStats()
        fb = lambda m: stats.add(list(m.feedback.error.positions))  # noqa: E731
        t0 = time.monotonic()
        fut = self.client.send_goal_async(build_goal(seg, names), feedback_callback=fb)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=self.cfg.wait_timeout_s)
        handle = fut.result() if fut.done() else None
        if handle is None or not handle.accepted:
            raise RuntimeError(f'{seg.segment_id}: goal not accepted by {self.cfg.controller_action}')
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res_fut, timeout_sec=seg.duration_s + self.cfg.result_margin_s)
        if not res_fut.done():
            handle.cancel_goal_async()
            raise RuntimeError(f'{seg.segment_id}: no result within {seg.duration_s + self.cfg.result_margin_s:.1f} s')
        result = res_fut.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'{seg.segment_id}: controller error {result.error_code} {result.error_string!r}')
        elapsed = time.monotonic() - t0
        self._spin_for(self.cfg.settle_s)
        end_dev = replay.deviation_deg(seg.end_deg, names, self.fresh_state_deg())
        rec = {'segment_id': seg.segment_id, 'kind': seg.kind, 'seam_id': seg.seam_id,
               'duration_s': round(seg.duration_s, 6), 'elapsed_s': round(elapsed, 3),
               'start_deviation_deg': round(start_dev, 6), 'end_deviation_deg': round(end_dev, 6),
               'max_tracking_error_deg': round(stats.max_deg, 6), 'feedback_samples': stats.samples}
        self.log.info(f'{seg.segment_id} {seg.kind}{" " + seg.seam_id if seg.seam_id else ""}: '
                      f'{seg.duration_s:.2f} s planned, {elapsed:.2f} s run, max tracking error '
                      f'{stats.max_deg:.4f} deg ({stats.samples} feedback), end deviation {end_dev:.4f} deg')
        if end_dev > self.cfg.final_tolerance_deg:
            raise RuntimeError(f'{seg.segment_id}: end state off by {end_dev:.4f} deg '
                               f'(> final_tolerance_deg {self.cfg.final_tolerance_deg})')
        return rec


def _parse(argv: Optional[List[str]]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(prog='replay_program', description=__doc__.split('\n')[0])
    ap.add_argument('--program', required=True, help='D09 program.json (v2, controller robot_positioner)')
    ap.add_argument('--config', default='', help='replay config YAML (default: awm_bringup config/program_replay.yaml)')
    ap.add_argument('--segments', default='', help='segment id or inclusive range, e.g. G002-G004 (default: all)')
    return ap.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    try:
        args = _parse(rclpy.utilities.remove_ros_args(sys.argv)[1:] if argv is None else argv)
    except SystemExit as exc:      # argparse: usage error (2) -> error exit 1, --help (0) stays 0
        return EXIT_OK if exc.code == 0 else EXIT_ERROR
    summary: Dict = {'program': args.program, 'status': 'error', 'segments': []}
    rclpy.init()
    node = rclpy.create_node('awm_program_replay')
    log = node.get_logger()
    code = EXIT_ERROR
    try:
        cfg = replay.load_config(args.config or None)
        prog = replay.load_program(args.program)
        segs = prog.select(args.segments or None)
        summary.update(program_rev=prog.program_rev, joint_names=prog.joint_names,
                       controller_action=cfg.controller_action)
        r = Replayer(node, cfg)
        components = r.hardware_components()
        replay.check_mock_hardware(components)
        summary['hardware'] = [{'name': n, 'plugin': p} for n, p in components]
        if not r.client.wait_for_server(timeout_sec=cfg.wait_timeout_s):
            raise RuntimeError(f'{cfg.controller_action} not available (launch sim_mock with '
                               'trajectory_controllers:=robot_positioner)')
        log.info(f'replay {prog.program_rev}: {len(segs)} segment(s) on {cfg.controller_action}; weld events are '
                 'not executed (motion only)')
        for seg in segs:
            summary['segments'].append(r.run_segment(seg, prog.joint_names))
        last = segs[-1]
        final = r.fresh_state_deg()
        summary['final_state_deg'] = {j: round(final[j], 6) for j in prog.joint_names}
        summary['final_deviation_deg'] = round(replay.deviation_deg(last.end_deg, prog.joint_names, final), 6)
        summary['max_tracking_error_deg'] = max(s['max_tracking_error_deg'] for s in summary['segments'])
        summary['status'] = 'ok'
        code = EXIT_OK
    except replay.ReplayRefused as exc:
        log.error(f'refused: {exc}')
        summary.update(status=replay.refusal_status(len(summary['segments'])), reason=str(exc))
        code = EXIT_REFUSED
    except Exception as exc:  # noqa: BLE001 - CLI boundary: report and exit 1
        log.error(f'replay failed: {exc}')
        summary.update(status='error', reason=str(exc))
        code = EXIT_ERROR
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print('replay ' + json.dumps(summary, sort_keys=True), flush=True)  # CLI result line (not a node log)
    return code


if __name__ == '__main__':
    sys.exit(main())
