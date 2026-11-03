"""Smoke test of ``sim_mock.launch.py`` (cell model v2, WP-S S.4 B4) on mock hardware.

Checks: 7 joints in /joint_states; cell scene loaded from a manifest argument; padding 3 mm on every link except
nozzle_tip (loaded planning scene); no ACM entry between a moving link and a cell object; home valid; head pushed
into the Pismo frame invalid; nozzle_tip contact allowed without padding; rotating set follows positioner_joint.
"""

import json
import math
import time

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from moveit_msgs.msg import PlanningSceneComponents, PositionIKRequest, RobotState  # noqa: E402
from moveit_msgs.srv import GetPlanningScene, GetPositionIK, GetStateValidity  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

from awm_bringup.testing import SimLaunch, wait_for_services  # noqa: E402
from awm_core import units  # noqa: E402

ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HOME = [0.0, -1.5708, 1.0, -1.0, -1.5708, 0.0]
HEAD = ['head_mount', 'head_body', 'nozzle_body', 'nozzle_tip', 'conduit']
# Test data (not part data): Pismo box as in the nominal manifest + one rotating box 100 mm off the positioner axis.
MANIFEST = {'objects': [
    {'object_id': 'pismo_frame', 'parent_frame': 'world', 'box_mm': [1200, 800, 800],
     'pose_mm': {'position_mm': [250, 0, -400], 'quat_xyzw': [0, 0, 0, 1]}},
    {'object_id': 'holder_test', 'parent_frame': 'positioner_flange', 'box_mm': [20, 20, 20],
     'pose_mm': {'position_mm': [0, 100, 0], 'quat_xyzw': [0, 0, 0, 1]}},
]}
POSITIONER_MM = (700.0, 0.0, 300.0)


def _call(node, client, req, timeout=30.0):
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    assert fut.done(), 'service call timed out'
    return fut.result()


def _state(names, values):
    rs = RobotState()
    rs.joint_state.name = list(names)
    rs.joint_state.position = [float(v) for v in values]
    return rs


def _valid(node, client, names, values):
    req = GetStateValidity.Request(robot_state=_state(names, values), group_name='laser_arm')
    res = _call(node, client, req)
    return res.valid, [(c.contact_body_1, c.contact_body_2) for c in res.contacts]


def _ik_beam_down(node, ik, xyz_mm):
    """IK (KDL from home) for tcp_laser at xyz_mm with the home orientation (beam axis = -Z world)."""
    req = GetPositionIK.Request()
    r: PositionIKRequest = req.ik_request
    r.group_name = 'laser_arm'
    r.ik_link_name = 'tcp_laser'
    r.avoid_collisions = False
    r.robot_state = _state(ARM, HOME)
    r.timeout = Duration(sec=1)
    ps = PoseStamped()
    ps.header.frame_id = 'world'
    x, y, z = (float(v) for v in units.mm_to_m(xyz_mm))
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
    # tool0 / tcp_laser rotation at home: x -> -Y, y -> -X, z -> -Z (quaternion of that matrix)
    ps.pose.orientation.x, ps.pose.orientation.y = math.sqrt(0.5), -math.sqrt(0.5)
    ps.pose.orientation.z, ps.pose.orientation.w = 0.0, 0.0
    r.pose_stamped = ps
    res = _call(node, ik, req)
    assert res.error_code.val == 1, f'IK failed for {xyz_mm}: {res.error_code.val}'
    js = res.solution.joint_state
    return [js.position[js.name.index(j)] for j in ARM]


def test_sim_mock(tmp_path):
    manifest = tmp_path / 'cell_scene.yaml'
    manifest.write_text(yaml.safe_dump(MANIFEST))
    args = {'cell_manifest': str(manifest), 'positioner_xyz_mm': '%g %g %g' % POSITIONER_MM}
    with SimLaunch(log_dir=tmp_path, launch_file='sim_mock.launch.py', launch_args=args) as sim:
        rclpy.init()
        node = rclpy.create_node('awm_sim_mock_smoke_test')
        try:
            wait_for_services(node, [('/check_state_validity', GetStateValidity), ('/compute_ik', GetPositionIK),
                                     ('/get_planning_scene', GetPlanningScene)], timeout_s=120.0)
            valid = node.create_client(GetStateValidity, '/check_state_validity')
            ik = node.create_client(GetPositionIK, '/compute_ik')
            scene = node.create_client(GetPlanningScene, '/get_planning_scene')

            # 7 joints in /joint_states
            states, status = [], []
            node.create_subscription(JointState, '/joint_states', states.append, 10)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(String, '/awm/cell_scene/status', status.append, latched)
            deadline = time.monotonic() + 90.0
            while not (states and status) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            assert states, 'no /joint_states\n' + sim.log_tail()
            assert status, 'cell scene not loaded\n' + sim.log_tail()
            assert sorted(states[-1].name) == sorted(ARM + ['positioner_joint'])
            info = json.loads(status[-1].data)
            assert [o['object_id'] for o in info['objects']] == ['pismo_frame', 'holder_test']

            # Padding (F4) and ACM (B-04) of the loaded planning scene
            comp = (PlanningSceneComponents.LINK_PADDING_AND_SCALING | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                    | PlanningSceneComponents.WORLD_OBJECT_NAMES)
            ps = _call(node, scene, GetPlanningScene.Request(components=PlanningSceneComponents(components=comp))).scene
            pad = {p.link_name: p.padding for p in ps.link_padding}
            for link in HEAD + ['shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link',
                                'wrist_3_link', 'base_link_inertia']:
                expected = 0.0 if link == 'nozzle_tip' else 0.003
                assert pad[link] == pytest.approx(expected, abs=1e-9), (link, pad[link])
            assert {o.id for o in ps.world.collision_objects} >= {'pismo_frame', 'holder_test'}
            acm = ps.allowed_collision_matrix
            names = list(acm.entry_names)
            for link in names:
                for obj in ('pismo_frame', 'holder_test'):
                    if obj in names and link not in ('pismo_frame', 'holder_test'):
                        allowed = acm.entry_values[names.index(link)].enabled[names.index(obj)]
                        assert allowed == (link == 'base_link_inertia' and obj == 'pismo_frame'), (link, obj)

            # Home valid
            ok, contacts = _valid(node, valid, ARM, HOME)
            assert ok, contacts
            # Head pushed 50 mm into the Pismo frame top -> invalid (head links vs pismo_frame)
            q = _ik_beam_down(node, ik, (430.0, 133.0, -50.0))
            ok, contacts = _valid(node, valid, ARM, q)
            assert not ok and any('pismo_frame' in c for c in contacts), contacts
            # Nozzle tip 0.5 mm into the plate -> invalid; tcp at the surface (contact, tip stops 0.3 mm short)
            # and 1 mm above (tip 1.3 mm < 3 mm padding) -> valid: nozzle_tip is unpadded, nozzle_body is padded.
            for z_mm, expect in ((-0.5, False), (0.0, True), (1.0, True)):
                ok, contacts = _valid(node, valid, ARM, _ik_beam_down(node, ik, (430.0, 133.0, z_mm)))
                assert ok == expect, (z_mm, contacts)

            # Rotating set follows positioner_joint: 90 deg about +X moves holder_test from +Y 100 to +Z 100 mm
            action = ActionClient(node, FollowJointTrajectory,
                                  '/positioner_trajectory_controller/follow_joint_trajectory')
            assert action.wait_for_server(timeout_sec=60.0), sim.log_tail()
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ['positioner_joint']
            goal.trajectory.points = [JointTrajectoryPoint(positions=[units.deg_to_rad(90.0)],
                                                           time_from_start=Duration(sec=4))]  # 22.5 deg/s < limit
            fut = action.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            res_fut = fut.result().get_result_async()
            rclpy.spin_until_future_complete(node, res_fut, timeout_sec=20.0)
            assert res_fut.result().result.error_code == 0
            deadline = time.monotonic() + 10.0
            pos = None
            while time.monotonic() < deadline:
                comp = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                ps = _call(node, scene, GetPlanningScene.Request(
                    components=PlanningSceneComponents(components=comp))).scene
                obj = [o for o in ps.world.collision_objects if o.id == 'holder_test'][0]
                pos = units.m_to_mm([obj.pose.position.x, obj.pose.position.y, obj.pose.position.z])
                if abs(pos[2] - (POSITIONER_MM[2] + 100.0)) < 0.5:
                    break
                time.sleep(0.2)
            assert pos == pytest.approx([POSITIONER_MM[0], POSITIONER_MM[1], POSITIONER_MM[2] + 100.0], abs=0.5)
        finally:
            node.destroy_node()
            rclpy.shutdown()
