"""WP-S S.4 C4: G2 runner logic with a fake scene backend (no ROS) and the launch arguments from frames.yaml."""

import json

import numpy as np
import pytest

from awm_path.core.contact import ContactParams, PartMesh
from awm_path.core.gate import load_pose_specs
from awm_path.core.scene import StateCheck
from awm_path.core.scene_files import JobObject
from awm_path.gate_g2 import launch_args_from_frames, run_gate, write_gate_log


class FakeKin:
    name = 'fake'

    def fk(self, q):
        t = np.eye(4)
        t[:3, 3] = q[:3]
        return t

    def ik(self, target):
        return [np.array([target[0, 3], target[1, 3], target[2, 3], 0.0, 0.0, 0.0])]


class FakeScene:
    """Valid unless the TCP x (= q1 with FakeKin, tool offset 0) is below -30; clearance invalid when x < 10."""

    name = 'fake-scene'

    def __init__(self):
        self.thetas = []

    def world_T_links(self, q, theta, links):
        flange = np.eye(4)
        flange[:3, 3] = [0.0, 0.0, 100.0 + theta]
        out = {'base_link': np.eye(4), 'tool0': np.eye(4), 'tcp_laser': np.eye(4), 'positioner_flange': flange}
        return {k: out[k] for k in links}

    def check(self, states, theta):
        self.thetas.append(theta)
        return [StateCheck(q[0] >= -30, () if q[0] >= -30 else ('nozzle_tip<->part',)) for q in states]

    def check_clearance(self, states, theta, c):
        return [StateCheck(q[0] >= 10, ()) for q in states]


def _plate():
    tris = []
    for i in range(40):
        for j in range(40):
            x, y = -20 + i, -20 + j
            tris += [[[x, y, 0], [x + 1, y, 0], [x + 1, y + 1, 0]], [[x, y, 0], [x + 1, y + 1, 0], [x, y + 1, 0]]]
    tris = np.asarray(tris, float)
    cx = tris[:, :, 0].mean(axis=1)
    return PartMesh(tris, np.where(cx < 0, 0, np.where(cx < 10, 1, 2)), ('A', 'B', 'C'))


def _doc():
    tcp = lambda x, z=0.0: {'frame': 'part', 'position_mm': [x, 0.0, z], 'beam_dir': [0, 0, -1],  # noqa: E731
                            'x_dir': [1, 0, 0]}
    return {'schema': 'awm_g2_poses/0.1', 'seam_faces': {'S': {'n1': ['A'], 'n2': ['B']}}, 'poses': [
        {'id': 'C1', 'kind': 'colliding', 'tcp': tcp(-35.0), 'checks': ['validity'],
         'expect': {'valid': False, 'objects': ['part']}},
        {'id': 'F1', 'kind': 'free', 'tcp': tcp(15.0, 30.0), 'checks': ['validity'], 'expect': {'valid': True}},
        {'id': 'Z1', 'kind': 'contact_zone', 'tcp': tcp(12.0), 'theta_deg': 5.0, 'checks': ['validity', 'contact'],
         'seam': 'S', 'expect': {'valid': False}},
        {'id': 'F2', 'kind': 'free', 'tcp': tcp(-15.0), 'checks': ['validity', 'contact'], 'seam': 'S',
         'expect': {'valid': True}},
    ]}


def _part(parent='positioner_flange'):
    return JobObject('part', 'part', parent, np.eye(4), np.zeros((3, 3)), np.zeros((1, 3), int), 'ab' * 32, True)


def test_run_gate_combines_validity_and_contact():
    poses, seams = load_pose_specs(_doc())
    scene = FakeScene()
    results = run_gate(scene, FakeKin(), poses, seams, _plate(), _part(), ContactParams())
    by = {r.pose.pose_id: r for r in results}
    assert not by['C1'].valid and by['C1'].passed and by['C1'].expected_object_seen
    assert by['F1'].valid and by['F1'].passed
    assert by['F1'].details['clearance_5mm_valid'] is True
    z = by['Z1']
    assert z.details['validity'] is True and z.contact.reason == 'contact_off_seam' and not z.valid and z.passed
    assert z.details['theta_deg'] == 5.0 and 5.0 in scene.thetas
    # world_T_part at theta: flange z = 100 + theta -> the tcp target follows the part
    assert z.details['q_deg'][2] == pytest.approx(105.0)
    assert by['F2'].valid and by['F2'].contact.ok


def test_run_gate_respects_the_parent_frame_of_the_part():
    """A part attached to world does not follow the positioner angle (scene_files.world_T_object)."""
    poses, seams = load_pose_specs(_doc())
    results = run_gate(FakeScene(), FakeKin(), poses, seams, _plate(), _part('world'), ContactParams())
    z = {r.pose.pose_id: r for r in results}['Z1']
    assert z.details['theta_deg'] == 5.0 and z.details['q_deg'][2] == pytest.approx(0.0)


def test_write_gate_log(tmp_path):
    poses, seams = load_pose_specs(_doc())
    results = run_gate(FakeScene(), FakeKin(), poses, seams, _plate(), _part(), ContactParams())
    j, md = write_gate_log(tmp_path / 'g2_log.json', tmp_path / 'g2_log.md', {'commit': 'abc'}, results)
    doc = json.loads(j.read_text())
    assert doc['record']['commit'] == 'abc'
    assert doc['summary']['pass'] is False and doc['summary']['missing']   # 4 poses only
    assert [p['id'] for p in doc['poses']] == ['C1', 'F1', 'Z1', 'F2']
    text = md.read_text()
    assert '| C1 |' in text and 'FAIL' in text


def test_launch_args_from_frames(tmp_path):
    frames = tmp_path / 'frames.yaml'
    frames.write_text(
        'world_T_base:\n  position_mm: [0.0, 0.0, 0.0]\n  quat_xyzw: [0.0, 0.0, 0.0, 1.0]\n'
        'world_T_positioner_axis:\n  position_mm: [-2.0, -475.0, 75.0]\n  quat_xyzw: [0.0, 0.0, 0.7071067811865476,'
        ' 0.7071067811865476]\n  direction: [0.0, 1.0, 0.0]\n')
    args = launch_args_from_frames(frames)
    assert args['positioner_xyz_mm'] == '-2 -475 75'
    assert args['base_xyz_mm'] == '0 0 0'
    # direction in world (0, 1, 0) expressed in the axis frame rotated +90 deg about Z -> (1, 0, 0)
    axis = [float(v) for v in args['positioner_axis_xyz'].split()]
    assert np.allclose(axis, [1.0, 0.0, 0.0], atol=1e-12)
