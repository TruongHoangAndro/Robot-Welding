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
    j, md = write_gate_log(tmp_path / 'g2_log.json', tmp_path / 'g2_log.md', {'commit': 'abc'}, results,
                           notes=['rule (b) covered by Z5'])
    doc = json.loads(j.read_text())
    assert doc['record']['commit'] == 'abc'
    assert doc['summary']['pass'] is False and doc['summary']['missing']   # 4 poses only
    assert [p['id'] for p in doc['poses']] == ['C1', 'F1', 'Z1', 'F2']
    text = md.read_text()
    assert '| C1 |' in text and 'FAIL' in text
    assert '## Notes' in text and '- rule (b) covered by Z5' in text
    assert doc['notes'] == ['rule (b) covered by Z5']


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


def test_parse_head_args() -> None:
    from awm_path.gate_g2 import parse_head_args
    assert parse_head_args(['nozzle_dia_mm=12', 'contact_standoff_mm=0.3']) == {
        'nozzle_dia_mm': '12', 'contact_standoff_mm': '0.3'}
    assert parse_head_args(None) == {}
    for bad in (['nozzle_dia_mm'], ['nozzle_dia_mm=/etc'], ['a=1', 'a=2'], ['X=1']):
        with pytest.raises(ValueError):
            parse_head_args(bad)


def test_run_gate_uses_the_given_tool0_T_tcp() -> None:
    """S.5c Task 0: the runner passes tool0_T_tcp of the robot description; identity = the FK default here, and a
    different offset changes the solved states (it is really used)."""
    poses, seams = load_pose_specs(_doc())
    a = run_gate(FakeScene(), FakeKin(), poses, seams, _plate(), _part(), ContactParams())
    b = run_gate(FakeScene(), FakeKin(), poses, seams, _plate(), _part(), ContactParams(), tool0_T_tcp=np.eye(4))
    assert [r.details['q_deg'] for r in a] == [r.details['q_deg'] for r in b]
    off = np.eye(4)
    off[2, 3] = 7.0
    c = run_gate(FakeScene(), FakeKin(), poses, seams, _plate(), _part(), ContactParams(), tool0_T_tcp=off)
    assert [r.details['q_deg'] for r in a] != [r.details['q_deg'] for r in c]


def test_head_assumption_text_comes_from_the_description_values():
    """Review M2: run records state the nozzle / s_c / TCP of the description in use, not hard-coded H-BOX values."""
    from awm_path.gate_g2 import head_assumption
    head = {'nozzle_geometry': 'cylinder', 'nozzle_radius_mm': 6.0, 'contact_standoff_mm': 0.25,
            'nozzle_tip_length_mm': 5.0, 'conduit_source': 'explicit', 'conduit_source_assumption': False,
            'tool0_T_tcp_mm': {'position_mm': [43.8, 0.0, 279.6], 'quat_xyzw': [0.0, 0.0, 0.0, 1.0]}}
    text = head_assumption('H-CAD', ContactParams(nozzle_radius_mm=6.0, standoff_mm=0.25), head)
    assert 'H-CAD head' in text and 'Ø12 mm' in text and 's_c 0.25 mm' in text and '(43.8, 0, 279.6)' in text
    assert 'H-BOX' not in text and 'Ø20' not in text
    assert 'conduit source explicit' in text


@pytest.mark.parametrize('source', ['parametric', 'rear_estimate'])
def test_head_assumption_marks_estimated_conduit_sources_as_assumptions(source):
    """G1.13 request 2: every record states the conduit source; parametric / rear_estimate are Assumptions."""
    from awm_path.gate_g2 import head_assumption
    head = {'nozzle_geometry': 'cylinder', 'nozzle_radius_mm': 10.0, 'contact_standoff_mm': 0.3,
            'nozzle_tip_length_mm': 4.7, 'conduit_source': source, 'conduit_source_assumption': True,
            'tool0_T_tcp_mm': {'position_mm': [0.0, 0.0, 481.0], 'quat_xyzw': [0.0, 0.0, 0.0, 1.0]}}
    text = head_assumption('H-BOX', ContactParams(), head)
    assert f'conduit source {source} (Assumption' in text


def test_head_assumption_refuses_a_record_without_conduit_source():
    from awm_path.gate_g2 import head_assumption
    head = {'nozzle_geometry': 'cylinder', 'nozzle_radius_mm': 10.0, 'contact_standoff_mm': 0.3,
            'nozzle_tip_length_mm': 4.7,
            'tool0_T_tcp_mm': {'position_mm': [0.0, 0.0, 481.0], 'quat_xyzw': [0.0, 0.0, 0.0, 1.0]}}
    with pytest.raises(ValueError, match='conduit_source'):
        head_assumption('H-BOX', ContactParams(), head)


def test_head_assumption_without_nozzle_states_it():
    """G1.13 request 3: nozzle_len_mm 0 -> no parametric nozzle, F9 not available (no contact params)."""
    from awm_path.gate_g2 import head_assumption
    head = {'nozzle_geometry': 'none', 'nozzle_radius_mm': None, 'contact_standoff_mm': None,
            'nozzle_tip_length_mm': None, 'conduit_source': 'explicit', 'conduit_source_assumption': False,
            'tool0_T_tcp_mm': {'position_mm': [1.0, 2.0, 300.0], 'quat_xyzw': [0.0, 0.0, 0.0, 1.0]}}
    text = head_assumption('H-CAD', None, head)
    assert 'no parametric nozzle' in text and 'F9 not available' in text and '(1, 2, 300)' in text


def test_head_launch_args_pass_the_head_model_and_folder(tmp_path):
    """Review I1: runners pass head_model (and head_dir for H-CAD) to sim_mock, as load_planning_description does."""
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'awm_description' / 'test'))
    pytest.importorskip('awm_bringup.cell_config', reason='awm_bringup not built')
    from head_fixture import write_synthetic_head_dir
    from awm_path.gate_g2 import head_launch_args
    assert head_launch_args('H-BOX', None, tmp_path) == {'head_model': 'hbox'}
    head = write_synthetic_head_dir(tmp_path / 'cell' / 'model' / 'head')
    assert head_launch_args('H-CAD', None, tmp_path) == {'head_model': 'hcad', 'head_dir': str(head.resolve())}
    other = write_synthetic_head_dir(tmp_path / 'other' / 'head')
    assert head_launch_args('hcad', str(other), tmp_path)['head_dir'] == str(other.resolve())
    with pytest.raises(ValueError):
        head_launch_args('H-CAD', None, tmp_path / 'empty')
    with pytest.raises(ValueError, match='head model'):
        head_launch_args('H-XYZ', None, tmp_path)


def test_head_args_refuse_head_model_and_head_dir():
    """Review minor 5: head_model / head_dir belong to --head-model / --head-dir; a duplicate in --head-arg is refused
    with a clear message instead of a TypeError from dict(**a, **b)."""
    from awm_path.gate_g2 import parse_head_args
    for key in ('head_model', 'head_dir'):
        with pytest.raises(ValueError, match='--head-model / --head-dir'):
            parse_head_args([f'{key}=1'])
    assert parse_head_args(['nozzle_dia_mm=12']) == {'nozzle_dia_mm': '12'}
