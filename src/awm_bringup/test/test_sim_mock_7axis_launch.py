"""G1.7 launch test: ``sim_mock trajectory_controllers:=robot_positioner`` (DES-001 §6.3 Option A1) + D09 replay CLI.

Mock hardware only. Two launches, each starting at the mock start state (``initial_positions_v2.yaml``):

1. ``test_replay_synthetic`` (always): only the 7-joint controller is active (split pair not loaded); a synthetic
   2-segment program (test data, not part data) is replayed, final ``/joint_states`` = last point; a program that
   does not start at the current state is refused (exit 2) and nothing moves.
2. ``test_replay_golden`` (session fixture ``golden``, ``conftest.py``, G1.22: repository rebuilt from the cell STEP
   by ``scripts/golden_rebuild.py``; skipped with ``AWM_CELL_STEP not set`` otherwise): the J1 + J2 program of the
   rebuild (two seams, weld segments in ``sync``) is replayed synchronised in ``sim_mock repo:=<golden repo>``;
   final ``/joint_states`` = last D09 point within ``final_tolerance_deg``; per-segment tracking error is logged
   (``G17 {json}`` with ``pytest -s``).
"""

import json
import math
import os
import subprocess
from pathlib import Path

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from awm_bringup import replay  # noqa: E402
from awm_bringup.testing import SimLaunch, wait_for_services  # noqa: E402
from awm_core import units  # noqa: E402

ROBOT = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
NAMES = ROBOT + ['positioner_joint']
ARGS = {'trajectory_controllers': 'robot_positioner'}


def _start_deg():
    """Mock start state from awm_description (rad, ros2_control format) in deg."""
    from ament_index_python.packages import get_package_share_directory
    f = Path(get_package_share_directory('awm_description')) / 'config' / 'initial_positions_v2.yaml'
    d = yaml.safe_load(f.read_text())
    return [float(units.rad_to_deg(d[j])) for j in NAMES]


def _smooth_segment(sid, a, b, duration, n=21):
    """Rest-to-rest cosine blend a -> b (deg) with consistent velocities / accelerations."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    pts = []
    for k in range(n):
        t = duration * k / (n - 1)
        s = 0.5 - 0.5 * math.cos(math.pi * t / duration)
        ds = 0.5 * math.pi / duration * math.sin(math.pi * t / duration)
        dds = 0.5 * (math.pi / duration) ** 2 * math.cos(math.pi * t / duration)
        pts.append({'time_from_start_s': t, 'positions_deg': list(a + s * (b - a)),
                    'velocities_deg_s': [0.0] * 7 if k in (0, n - 1) else list(ds * (b - a)),
                    'accelerations_deg_s2': list(dds * (b - a))})
    return {'segment_id': sid, 'controller': 'robot_positioner', 'kind': 'air', 'seam_id': None, 'hold_deg': [],
            'tcp_speed_mm_s': None, 'collision_checked': False, 'points': pts}


def _synthetic_program(start):
    # Small joint moves (a few degrees, positioner 20 deg) at low speed: test data only.
    p1 = [v + d for v, d in zip(start, [3.0, -2.0, 2.0, -2.0, 3.0, 5.0, 20.0])]
    p2 = [v + d for v, d in zip(start, [-2.0, 1.0, -1.5, 1.0, -2.0, -5.0, -10.0])]
    # Header / context from the frozen D09 v2.1 example (contract T0.5); motion data replaced.
    from awm_core import jsonio
    doc = json.loads((jsonio.SCHEMA_DIR / 'v2' / 'examples' / 'program.example.json').read_text())
    doc.update(created_by='test_sim_mock_7axis_launch', program_rev='P001',
               joint_names={'robot': ROBOT, 'positioner': ['positioner_joint']},
               home={'robot_deg': start[:6], 'positioner_deg': start[6]},
               segments=[_smooth_segment('G001', start, p1, 3.0), _smooth_segment('G002', p1, p2, 3.0)],
               events=[], cycle_time_s=6.0)
    jsonio.validate(doc, 'v2/program')
    return doc


def _replay(program: Path, timeout: float, extra=()):
    proc = subprocess.run(['ros2', 'run', 'awm_bringup', 'replay_program', '--program', str(program), *extra],
                          capture_output=True, text=True, timeout=timeout, env=dict(os.environ))
    lines = [ln for ln in proc.stdout.splitlines() if ln.startswith('replay ')]
    summary = json.loads(lines[-1][len('replay '):]) if lines else {}
    return proc.returncode, summary, proc.stderr


def _joint_state_deg(node, timeout=30.0):
    got = []
    sub = node.create_subscription(JointState, '/joint_states', got.append, 10)
    deadline = __import__('time').monotonic() + timeout
    while not got and __import__('time').monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    assert got, 'no /joint_states'
    return replay.positions_deg(got[-1].name, got[-1].position)


def _wait_ready(node, sim, timeout=120.0):
    wait_for_services(node, [('/controller_manager/list_controllers', ListControllers)], timeout_s=timeout)
    cli = node.create_client(ListControllers, '/controller_manager/list_controllers')
    deadline = __import__('time').monotonic() + timeout
    while True:
        fut = cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        states = {c.name: c.state for c in fut.result().controller} if fut.done() else {}
        if states.get('robot_positioner_trajectory_controller') == 'active' and \
                states.get('joint_state_broadcaster') == 'active':
            return states
        if __import__('time').monotonic() > deadline:
            pytest.fail(f'7-joint controller not active: {states}\n' + sim.log_tail())
        rclpy.spin_once(node, timeout_sec=0.5)


def test_replay_synthetic(tmp_path):
    start = _start_deg()
    doc = _synthetic_program(start)
    prog = tmp_path / 'program.json'
    prog.write_text(json.dumps(doc))
    with SimLaunch(log_dir=tmp_path, launch_file='sim_mock.launch.py', launch_args=ARGS) as sim:
        rclpy.init()
        node = rclpy.create_node('awm_g17_synthetic_test')
        try:
            states = _wait_ready(node, sim)
            assert 'joint_trajectory_controller' not in states and 'positioner_trajectory_controller' not in states

            rc, summary, err = _replay(prog, timeout=120.0)
            assert rc == 0, f'{summary}\n{err[-2000:]}\n' + sim.log_tail()
            assert [s['segment_id'] for s in summary['segments']] == ['G001', 'G002']
            last = doc['segments'][-1]['points'][-1]['positions_deg']
            now = _joint_state_deg(node)
            assert max(abs(now[j] - v) for j, v in zip(NAMES, last)) <= 0.05
            assert summary['final_deviation_deg'] <= 0.05

            # Same program again: the robot is at G002's end, not at G001's start -> refused, nothing sent.
            rc, summary, err = _replay(prog, timeout=60.0)
            assert rc == 2, f'{summary}\n{err[-2000:]}'
            assert summary['status'] == 'refused' and 'G001' in summary['reason'] and summary['segments'] == []
            after = _joint_state_deg(node)
            assert max(abs(after[j] - now[j]) for j in NAMES) <= 1e-6
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


def test_replay_golden(tmp_path, golden):
    path = Path(golden['repo']) / golden['program']['file']
    assert path.is_file(), path
    doc = json.loads(path.read_text())
    prog = replay.load_program(path)
    welds = [s for s in doc['segments'] if s['kind'] in ('weld', 'positioner_weld')]
    assert len({s['seam_id'] for s in welds}) >= 2, 'golden program must weld J1 and J2'
    assert prog.segments[0].start_deg == pytest.approx(_start_deg(), abs=0.05), \
        'golden program does not start at the sim_mock start state'
    cycle = sum(s.duration_s for s in prog.segments)
    args = dict(ARGS, repo=golden['repo'])  # the cell the program was planned in (G1.16)
    with SimLaunch(log_dir=tmp_path, launch_file='sim_mock.launch.py', launch_args=args) as sim:
        rclpy.init()
        node = rclpy.create_node('awm_g17_golden_test')
        try:
            _wait_ready(node, sim)
            rc, summary, err = _replay(path, timeout=cycle + 30.0 * len(prog.segments) + 120.0)
            print('G17 ' + json.dumps({'program': str(path), 'cycle_s': round(cycle, 3), **{
                k: summary.get(k) for k in ('status', 'final_deviation_deg', 'max_tracking_error_deg')},
                'segments': summary.get('segments')}, sort_keys=True))
            assert rc == 0, f'{summary}\n{err[-3000:]}\n' + sim.log_tail()
            assert [s['segment_id'] for s in summary['segments']] == [s.segment_id for s in prog.segments]
            assert {s['seam_id'] for s in summary['segments'] if s['seam_id']} == {s['seam_id'] for s in welds}
            last = prog.segments[-1].end_deg
            now = _joint_state_deg(node)
            assert max(abs(now[j] - v) for j, v in zip(NAMES, last)) <= replay.load_config(None).final_tolerance_deg
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
