"""Smoke test of ``sim.launch.py headless:=true``: move_group services and mock controllers come up."""

import time

import pytest

rclpy = pytest.importorskip('rclpy')
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from moveit_msgs.srv import (GetCartesianPath, GetMotionPlan, GetPositionFK,  # noqa: E402
                             GetPositionIK, GetStateValidity)
from rclpy.action import ActionClient  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from awm_bringup.testing import SimLaunch, wait_for_services  # noqa: E402


def test_sim_comes_up(tmp_path):
    with SimLaunch(log_dir=tmp_path) as sim:
        rclpy.init()
        node = rclpy.create_node('awm_sim_smoke_test')
        try:
            wait_for_services(node, [('/compute_ik', GetPositionIK), ('/compute_fk', GetPositionFK),
                                     ('/compute_cartesian_path', GetCartesianPath),
                                     ('/plan_kinematic_path', GetMotionPlan),
                                     ('/check_state_validity', GetStateValidity)])
            action = ActionClient(node, FollowJointTrajectory,
                                  '/joint_trajectory_controller/follow_joint_trajectory')
            assert action.wait_for_server(timeout_sec=60.0), sim.log_tail()
            states = []
            node.create_subscription(JointState, '/joint_states', states.append, 10)
            deadline = time.monotonic() + 30.0
            while not states and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            assert states, 'no /joint_states'
            home = dict(zip(states[-1].name, states[-1].position))
            assert home['elbow_joint'] == pytest.approx(1.5708)
        finally:
            node.destroy_node()
            rclpy.shutdown()
