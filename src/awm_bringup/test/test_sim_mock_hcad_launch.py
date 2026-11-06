"""Smoke test of ``sim_mock.launch.py head_model:=hcad`` (Goal 1 G1.1) on mock hardware only.

Checks: 7 joints in /joint_states; move_group FK of tcp_laser in tool0 equals the value computed from the head.yaml in
use (beam_exit + nozzle_len * beam_axis, quaternion of tool0_T_tcp_mm); a missing head_dir stops the launch with a
clear error. Head folder: AWM_HEAD_DIR (real export) when set, otherwise the synthetic test head of awm_description.
"""

import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from moveit_msgs.msg import RobotState  # noqa: E402
from moveit_msgs.srv import GetPositionFK  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from awm_bringup.testing import SimLaunch, wait_for_services  # noqa: E402
from awm_core import units  # noqa: E402
from awm_core.frames import quat_to_matrix  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'awm_description' / 'test'))
from head_fixture import real_head_dir, write_synthetic_head_dir  # noqa: E402

ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def _call(node, client, req, timeout=30.0):
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    assert fut.done(), 'service call timed out'
    return fut.result()


def test_sim_mock_hcad(tmp_path):
    head_dir = real_head_dir() or write_synthetic_head_dir(tmp_path / 'head')
    head = yaml.safe_load((Path(head_dir) / 'head.yaml').read_text())
    axis = np.asarray(head['beam_axis_tool0'], dtype=float)
    axis /= np.linalg.norm(axis)
    expected_mm = np.asarray(head['beam_exit_tool0_mm'], dtype=float) + float(head['nozzle_len_mm']) * axis
    args = {'head_model': 'hcad', 'head_dir': str(head_dir)}
    with SimLaunch(log_dir=tmp_path, launch_file='sim_mock.launch.py', launch_args=args) as sim:
        rclpy.init()
        node = rclpy.create_node('awm_sim_mock_hcad_smoke_test')
        try:
            states = []
            node.create_subscription(JointState, '/joint_states', states.append, 10)
            try:
                wait_for_services(node, [('/compute_fk', GetPositionFK)], timeout_s=120.0)
            except Exception:
                pytest.fail('move_group not up\n' + sim.log_tail())
            deadline = time.monotonic() + 30.0
            while not states and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            assert states, 'no /joint_states\n' + sim.log_tail()
            assert sorted(states[-1].name) == sorted(ARM + ['positioner_joint'])

            rs = RobotState()
            rs.joint_state.name = list(ARM)
            rs.joint_state.position = [0.0, -1.5708, 1.0, -1.0, -1.5708, 0.0]
            req = GetPositionFK.Request(fk_link_names=['tcp_laser'], robot_state=rs)
            req.header.frame_id = 'tool0'
            res = _call(node, node.create_client(GetPositionFK, '/compute_fk'), req)
            assert res.error_code.val == 1, res.error_code.val
            p = res.pose_stamped[0].pose
            got_mm = np.asarray(units.m_to_mm([p.position.x, p.position.y, p.position.z]))
            assert np.max(np.abs(got_mm - expected_mm)) <= 1e-6, (got_mm, expected_mm)
            rot = quat_to_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
            assert np.allclose(rot, quat_to_matrix(head['tool0_T_tcp_mm']['quat_xyzw']), atol=1e-9)
            assert math.isclose(float(np.dot(rot[:, 2], axis)), 1.0, abs_tol=1e-9)
        finally:
            node.destroy_node()
            rclpy.shutdown()


def test_sim_mock_hcad_without_head_dir_fails(tmp_path):
    """head_model:=hcad without head_dir: the launch stops before any node starts, naming head_dir."""
    proc = subprocess.run(['ros2', 'launch', 'awm_bringup', 'sim_mock.launch.py', 'headless:=true',
                           'head_model:=hcad'], capture_output=True, text=True, timeout=120, check=False)
    assert proc.returncode != 0
    assert 'head_dir' in proc.stdout + proc.stderr
