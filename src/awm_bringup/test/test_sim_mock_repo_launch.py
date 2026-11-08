"""G1.16 (close-out A2) launch test: ``sim_mock repo:=<CP-60 repo> trajectory_controllers:=robot_positioner``.

Mock hardware only. ``test_sim_mock_synthetic_repo`` (always on) runs the same frame / scene / head checks on the
tiny generated repository of ``test_repo_args.py`` (non-identity base: the CP-60 import puts world at the UR base, so
there world_T_base is identity). ``test_sim_mock_repo``: the golden repository of the session fixture ``golden``
(``conftest.py``, G1.22: CP-60 cell import rebuilt from the cell STEP by ``scripts/golden_rebuild.py``; read only),
skipped with ``AWM_CELL_STEP not set`` otherwise; the always-on argument / refusal tests are in ``test_repo_args.py``.

Checks, with no pose typed by hand:
- TF ``world -> base_link`` and ``world -> positioner_axis`` = ``cell/model/frames.yaml`` (≤ 1e-6 mm, ≤ 1e-6 deg);
  move_group FK of ``positioner_flange`` at θ = 30° = rotation of the axis frame about the frames.yaml ``direction``;
- every ``scene_manifest.json`` object is in the planning scene, world objects at their manifest pose;
- the head is H-CAD: FK of ``tcp_laser`` in ``tool0`` = ``head/head.yaml`` ``tool0_T_tcp_mm``;
- the 7-joint controller is active and a synthetic 2-segment program replays through ``replay_program``.
"""

import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
import tf2_ros  # noqa: E402
from moveit_msgs.msg import PlanningSceneComponents, RobotState  # noqa: E402
from moveit_msgs.srv import GetPlanningScene, GetPositionFK  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from awm_bringup.cell_config import REPO_CONFLICT_ARGS, repo_launch_args  # noqa: E402

from awm_bringup.scene_sync import SceneSync  # noqa: E402
from awm_bringup.testing import SimLaunch, wait_for_services  # noqa: E402
from awm_core import units  # noqa: E402
from awm_core.frames import axis_angle_to_matrix, make_transform, quat_to_matrix  # noqa: E402

from test_repo_args import make_repo  # noqa: E402
from test_sim_mock_7axis_launch import (NAMES, _joint_state_deg, _replay, _start_deg,  # noqa: E402
                                        _synthetic_program, _wait_ready)

TOL_MM, TOL_DEG = 1e-6, 1e-6


def _repo(golden) -> Path:
    repo = Path(golden['repo'])
    assert (repo / 'cell' / 'model' / 'frames.yaml').is_file(), f'{repo}: no cell/model/frames.yaml'
    return repo


def _pose_mm(entry) -> np.ndarray:
    return make_transform(np.asarray(entry['position_mm'], float), quat_to_matrix(entry['quat_xyzw']))


def _msg_pose_mm(position, orientation) -> np.ndarray:
    return make_transform(units.m_to_mm([position.x, position.y, position.z]),
                          quat_to_matrix([orientation.x, orientation.y, orientation.z, orientation.w]))


def _angle_deg(r_a: np.ndarray, r_b: np.ndarray) -> float:
    """Rotation angle between two rotations; atan2 form, accurate near 0 (acos loses ~1e-6 deg there)."""
    r = r_a.T @ r_b
    sin = 0.5 * np.linalg.norm([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    cos = 0.5 * (np.trace(r) - 1.0)
    return float(units.rad_to_deg(math.atan2(sin, cos)))


def _assert_pose(got: np.ndarray, want: np.ndarray, name: str) -> None:
    d_mm = float(np.max(np.abs(got[:3, 3] - want[:3, 3])))
    d_deg = _angle_deg(got[:3, :3], want[:3, :3])
    assert d_mm <= TOL_MM and d_deg <= TOL_DEG, f'{name}: {d_mm:.3g} mm, {d_deg:.3g} deg'


def _lookup(node, buf, target: str, timeout=30.0) -> np.ndarray:
    deadline = time.monotonic() + timeout
    while not buf.can_transform('world', target, rclpy.time.Time()):
        assert time.monotonic() < deadline, f'no TF world -> {target}'
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform('world', target, rclpy.time.Time()).transform
    return _msg_pose_mm(t.translation, t.rotation)


def _urdf_joint_axis(node, joint: str, timeout=30.0) -> np.ndarray:
    """Axis of a joint in the URDF published on /robot_description (latched)."""
    got = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(String, '/robot_description', lambda m: got.append(m.data), qos)
    deadline = time.monotonic() + timeout
    while not got:
        assert time.monotonic() < deadline, 'no /robot_description'
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    for j in ET.fromstring(got[-1]).iter('joint'):
        if j.get('name') == joint:
            return np.asarray([float(v) for v in j.find('axis').get('xyz').split()])
    raise AssertionError(f'{joint} not in the URDF')


def _call(node, client, req, timeout=30.0):
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    assert fut.done(), 'service call timed out'
    return fut.result()


def _fk(node, client, link, frame, names, values):
    rs = RobotState()
    rs.joint_state.name = list(names)
    rs.joint_state.position = [float(v) for v in values]
    req = GetPositionFK.Request(fk_link_names=[link], robot_state=rs)
    req.header.frame_id = frame
    res = _call(node, client, req)
    assert res.error_code.val == 1, (link, res.error_code.val)
    p = res.pose_stamped[0].pose
    return _msg_pose_mm(p.position, p.orientation)


def test_sim_mock_repo(tmp_path, golden):
    _check_repo(_repo(golden), tmp_path, replay=True)


def test_sim_mock_synthetic_repo(tmp_path):
    _check_repo(make_repo(tmp_path / 'repo'), tmp_path, replay=False)


def _check_repo(repo: Path, tmp_path: Path, replay: bool) -> None:
    model = repo / 'cell' / 'model'
    frames = yaml.safe_load((model / 'frames.yaml').read_text())
    manifest = json.loads((model / 'scene_manifest.json').read_text())
    head = yaml.safe_load((model / 'head' / 'head.yaml').read_text())
    start = _start_deg()
    program = tmp_path / 'program.json'
    program.write_text(json.dumps(_synthetic_program(start)))
    args = {'repo': str(repo), 'trajectory_controllers': 'robot_positioner'}
    with SimLaunch(log_dir=tmp_path, launch_file='sim_mock.launch.py', launch_args=args) as sim:
        rclpy.init()
        node = rclpy.create_node('awm_g116_repo_test')
        try:
            sync = SceneSync(node)
            try:
                wait_for_services(node, [('/compute_fk', GetPositionFK), ('/get_planning_scene', GetPlanningScene)],
                                  timeout_s=180.0)
                info = sync.wait_loaded(180.0)
            except Exception as exc:  # noqa: BLE001
                pytest.fail(f'sim_mock not up: {exc}\n' + sim.log_tail())
            fk = node.create_client(GetPositionFK, '/compute_fk')

            # Frames: TF and FK against frames.yaml
            buf = tf2_ros.Buffer()
            tf2_ros.TransformListener(buf, node)
            world_t_axis = _pose_mm(frames['world_T_positioner_axis'])
            _assert_pose(_lookup(node, buf, 'base_link'), _pose_mm(frames['world_T_base']), 'base_link')
            _assert_pose(_lookup(node, buf, 'positioner_axis'), world_t_axis, 'positioner_axis')
            direction = np.asarray(frames['world_T_positioner_axis']['direction'], float)
            direction /= np.linalg.norm(direction)
            # positioner_joint axis in the URDF = the resolver output (a dropped arg would give the xacro default)
            expected_axis = [float(v) for v in repo_launch_args(
                str(repo), dict({n: '' for n in REPO_CONFLICT_ARGS}, head_model=''))['positioner_axis_xyz'].split()]
            assert np.allclose(_urdf_joint_axis(node, 'positioner_joint'), expected_axis, atol=1e-12)
            arm = start[:6]
            for theta in (0.0, 30.0):
                want = world_t_axis.copy()
                want[:3, :3] = axis_angle_to_matrix(direction, units.deg_to_rad(theta)) @ world_t_axis[:3, :3]
                got = _fk(node, fk, 'positioner_flange', 'world', NAMES,
                          [units.deg_to_rad(v) for v in arm] + [units.deg_to_rad(theta)])
                _assert_pose(got, want, f'positioner_flange at {theta} deg')

            # Scene: every manifest object loaded, world objects at their manifest pose
            ids = [o['object_id'] for o in manifest['objects']]
            assert [o['object_id'] for o in info['objects']] == ids
            comp = PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            scene = node.create_client(GetPlanningScene, '/get_planning_scene')
            ps = _call(node, scene, GetPlanningScene.Request(components=PlanningSceneComponents(components=comp)))
            objs = {o.id: o for o in ps.scene.world.collision_objects}
            assert set(ids) <= set(objs), sorted(set(ids) - set(objs))
            # world objects at pose_mm; rotating objects at world_T_flange(theta applied) . pose_mm
            assert info['theta_deg'] == pytest.approx(0.0, abs=1e-9)
            world_t_flange = _lookup(node, buf, 'positioner_flange')
            parents = {'world': np.eye(4), 'positioner_flange': world_t_flange}
            assert {e['parent_frame'] for e in manifest['objects']} == set(parents)
            for entry in manifest['objects']:
                o = objs[entry['object_id']]
                assert o.header.frame_id == 'world'
                _assert_pose(_msg_pose_mm(o.pose.position, o.pose.orientation),
                             parents[entry['parent_frame']] @ _pose_mm(entry['pose_mm']), entry['object_id'])

            # Head: H-CAD tcp_laser in tool0 from head.yaml
            got = _fk(node, fk, 'tcp_laser', 'tool0', NAMES, [units.deg_to_rad(v) for v in start])
            want = make_transform(np.asarray(head['tool0_T_tcp_mm']['xyz'], float),
                                  quat_to_matrix(head['tool0_T_tcp_mm']['quat_xyzw']))
            _assert_pose(got, want, 'tcp_laser in tool0 (H-CAD)')

            if not replay:
                return
            # Replay on the 7-joint controller (mock hardware) still works with repo:=
            _wait_ready(node, sim)
            rc, summary, err = _replay(program, timeout=120.0)
            assert rc == 0, f'{summary}\n{err[-2000:]}\n' + sim.log_tail()
            assert summary['final_deviation_deg'] <= 0.05
            last = json.loads(program.read_text())['segments'][-1]['points'][-1]['positions_deg']
            now = _joint_state_deg(node)
            assert max(abs(now[j] - v) for j, v in zip(NAMES, last)) <= 0.05
            print('G116 ' + json.dumps({'repo': str(repo), 'objects': len(ids), 'head_model': 'hcad',
                                        'replay': summary.get('status'),
                                        'final_deviation_deg': summary.get('final_deviation_deg')}))
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
