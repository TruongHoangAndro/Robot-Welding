"""Smoke test of ``sim_mock.launch.py`` (cell model v2, WP-S S.4 B4) on mock hardware.

Checks: 7 joints in /joint_states; cell scene loaded from a manifest argument; padding 3 mm on every link except
nozzle_tip (loaded planning scene and functionally: 2 mm gap invalid, 4 mm valid); no ACM entry between a moving link
and a cell object; home valid; head pushed into the Pismo frame invalid; nozzle_tip contact allowed without padding;
the rotating set follows positioner_joint and a request at a positioner angle the scene has not applied is refused.
"""

import math
import time

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from builtin_interfaces.msg import Duration  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from moveit_msgs.msg import (CollisionObject, PlanningScene, PlanningSceneComponents,  # noqa: E402
                             PositionIKRequest, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetPlanningScene, GetPositionFK, GetPositionIK,  # noqa: E402
                             GetStateValidity)
from shape_msgs.msg import SolidPrimitive  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from awm_bringup.scene_sync import PositionerMismatch, SceneSync  # noqa: E402
from awm_bringup.testing import SimLaunch, wait_for_services  # noqa: E402
from awm_core import units  # noqa: E402
from awm_core.frames import compose, make_transform, matrix_to_quat, quat_to_matrix  # noqa: E402

ARM = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HOME = [0.0, -1.5708, 1.0, -1.0, -1.5708, 0.0]
HEAD = ['head_mount', 'head_body', 'nozzle_body', 'nozzle_tip', 'conduit']
# Test data (not part data): Pismo box as in the nominal manifest + one rotating box 267 mm off the positioner axis.
# At 0 deg the holder is above the robot, clear; turned 90 deg about +X it sits inside the head body at home.
MANIFEST = {'objects': [
    {'object_id': 'pismo_frame', 'parent_frame': 'world', 'box_mm': [1200, 800, 800],
     'pose_mm': {'position_mm': [250, 0, -400], 'quat_xyzw': [0, 0, 0, 1]}},
    {'object_id': 'holder_test', 'parent_frame': 'positioner_flange', 'box_mm': [40, 40, 40],
     'pose_mm': {'position_mm': [0, 0, 267], 'quat_xyzw': [0, 0, 0, 1]}},
]}
POSITIONER_MM = (430.0, 400.0, 450.0)
HEAD_BODY_HALF_D_MM, HEAD_BODY_MID_Z_MM = 17.5, 250.5   # H-BOX defaults in the head_body (= tool0) frame


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


def _valid(node, client, sync, values, theta_deg):
    """Guarded validity check: arm values + an explicit positioner angle that must equal the applied one."""
    req = GetStateValidity.Request(robot_state=_state(ARM + ['positioner_joint'],
                                                      list(values) + [units.deg_to_rad(theta_deg)]),
                                   group_name='laser_arm')
    sync.check(req.robot_state)
    res = _call(node, client, req)
    return res.valid, [(c.contact_body_1, c.contact_body_2) for c in res.contacts]


def _probe(node, apply, t_world_mm, present=True):
    """Add (or remove) a 20 mm test box at a world pose (mm)."""
    co = CollisionObject(id='padding_probe')
    co.header.frame_id = 'world'
    if present:
        co.operation = CollisionObject.ADD
        co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(units.mm_to_m([20.0, 20.0, 20.0])))]
        pose = PoseStamped().pose
        pose.orientation.w = 1.0
        co.primitive_poses = [pose]
        x, y, z = (float(v) for v in units.mm_to_m(t_world_mm[:3, 3]))
        co.pose.position.x, co.pose.position.y, co.pose.position.z = x, y, z
        qx, qy, qz, qw = (float(v) for v in matrix_to_quat(t_world_mm[:3, :3]))
        co.pose.orientation.x, co.pose.orientation.y, co.pose.orientation.z, co.pose.orientation.w = qx, qy, qz, qw
    else:
        co.operation = CollisionObject.REMOVE
    scene = PlanningScene(is_diff=True)
    scene.world.collision_objects = [co]
    assert _call(node, apply, ApplyPlanningScene.Request(scene=scene)).success


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
            sync = SceneSync(node)
            wait_for_services(node, [('/check_state_validity', GetStateValidity), ('/compute_ik', GetPositionIK),
                                     ('/compute_fk', GetPositionFK), ('/get_planning_scene', GetPlanningScene),
                                     ('/apply_planning_scene', ApplyPlanningScene)], timeout_s=120.0)
            valid = node.create_client(GetStateValidity, '/check_state_validity')
            ik = node.create_client(GetPositionIK, '/compute_ik')
            fk = node.create_client(GetPositionFK, '/compute_fk')
            scene = node.create_client(GetPlanningScene, '/get_planning_scene')
            apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')

            # 7 joints in /joint_states; scene loaded at the current angle (0 deg)
            states = []
            node.create_subscription(JointState, '/joint_states', states.append, 10)
            try:
                info = sync.wait_loaded(90.0)
            except TimeoutError:
                pytest.fail('cell scene not loaded\n' + sim.log_tail())
            deadline = time.monotonic() + 30.0
            while not states and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            assert states, 'no /joint_states\n' + sim.log_tail()
            assert sorted(states[-1].name) == sorted(ARM + ['positioner_joint'])
            assert [o['object_id'] for o in info['objects']] == ['pismo_frame', 'holder_test']
            assert info['revision'] >= 1 and info['theta_deg'] == pytest.approx(0.0, abs=1e-9)

            # Padding (F4) and ACM (B-04) of the loaded planning scene
            comp = (PlanningSceneComponents.LINK_PADDING_AND_SCALING | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                    | PlanningSceneComponents.WORLD_OBJECT_NAMES)
            ps = _call(node, scene, GetPlanningScene.Request(components=PlanningSceneComponents(components=comp))).scene
            pad = {p.link_name: p.padding for p in ps.link_padding}
            for link in HEAD + ['shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link',
                                'wrist_3_link', 'base_link_inertia']:
                expected = units.mm_to_m(0.0 if link == 'nozzle_tip' else 3.0)
                assert pad[link] == pytest.approx(expected, abs=1e-12), (link, pad[link])
            assert {o.id for o in ps.world.collision_objects} >= {'pismo_frame', 'holder_test'}
            acm = ps.allowed_collision_matrix
            names = list(acm.entry_names)
            for link in names:
                for obj in ('pismo_frame', 'holder_test'):
                    if obj in names and link not in ('pismo_frame', 'holder_test'):
                        allowed = acm.entry_values[names.index(link)].enabled[names.index(obj)]
                        assert allowed == (link == 'base_link_inertia' and obj == 'pismo_frame'), (link, obj)

            # Home valid; head pushed 50 mm into the Pismo frame -> invalid
            ok, contacts = _valid(node, valid, sync, HOME, 0.0)
            assert ok, contacts
            q = _ik_beam_down(node, ik, (430.0, 133.0, -50.0))
            ok, contacts = _valid(node, valid, sync, q, 0.0)
            assert not ok and any('pismo_frame' in c for c in contacts), contacts
            # Nozzle tip 0.5 mm into the plate -> invalid; tcp at the surface (contact, tip stops 0.3 mm short)
            # and 1 mm above (tip 1.3 mm < 3 mm padding) -> valid: nozzle_tip is unpadded, nozzle_body is padded.
            for z_mm, expect in ((-0.5, False), (0.0, True), (1.0, True)):
                ok, contacts = _valid(node, valid, sync, _ik_beam_down(node, ik, (430.0, 133.0, z_mm)), 0.0)
                assert ok == expect, (z_mm, contacts)

            # Functional padding: a box 2 mm from the head_body side face -> invalid, 4 mm -> valid (3 mm padding)
            req = GetPositionFK.Request(fk_link_names=['head_body'], robot_state=_state(ARM, HOME))
            req.header.frame_id = 'world'
            p = _call(node, fk, req).pose_stamped[0].pose
            world_t_head = make_transform(
                units.m_to_mm([p.position.x, p.position.y, p.position.z]),
                quat_to_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))
            for gap_mm, expect in ((2.0, False), (4.0, True)):
                local = make_transform([0.0, HEAD_BODY_HALF_D_MM + gap_mm + 10.0, HEAD_BODY_MID_Z_MM], [[1, 0, 0],
                                                                                                         [0, 1, 0],
                                                                                                         [0, 0, 1]])
                _probe(node, apply, compose(world_t_head, local))
                ok, contacts = _valid(node, valid, sync, HOME, 0.0)
                assert ok == expect, (gap_mm, contacts)
                if not expect:
                    assert ('head_body', 'padding_probe') in contacts or ('padding_probe', 'head_body') in contacts
            _probe(node, apply, None, present=False)

            # Positioner: a request at 90 deg before the scene is there is refused, never "valid"
            with pytest.raises(PositionerMismatch):
                _valid(node, valid, sync, HOME, 90.0)
            rev0 = sync.status['revision']
            rev = sync.set_positioner_and_wait(90.0, timeout_s=60.0)
            assert rev > rev0 and sync.status['theta_deg'] == pytest.approx(90.0, abs=0.01)
            comp = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            ps = _call(node, scene, GetPlanningScene.Request(components=PlanningSceneComponents(components=comp))).scene
            obj = [o for o in ps.world.collision_objects if o.id == 'holder_test'][0]
            pos = units.m_to_mm([obj.pose.position.x, obj.pose.position.y, obj.pose.position.z])
            assert pos == pytest.approx([POSITIONER_MM[0], POSITIONER_MM[1] - 267.0, POSITIONER_MM[2]], abs=0.01)
            ok, contacts = _valid(node, valid, sync, HOME, 90.0)
            assert not ok and any('holder_test' in c for c in contacts), contacts
            with pytest.raises(PositionerMismatch):
                _valid(node, valid, sync, HOME, 0.0)
            # and back: valid again at 0 deg
            sync.set_positioner_and_wait(0.0, timeout_s=60.0)
            ok, contacts = _valid(node, valid, sync, HOME, 0.0)
            assert ok, contacts
        finally:
            node.destroy_node()
            rclpy.shutdown()
