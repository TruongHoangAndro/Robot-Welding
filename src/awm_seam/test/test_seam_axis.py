"""G1.2 / CP-58 decision 2: seam_axis.json (seam in positioner_flange at theta = 0), pure Python."""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from awm_core import frames, jsonio  # noqa: E402
from awm_seam import joint_seams as js  # noqa: E402
from awm_seam import seam_axis as sa  # noqa: E402
from synthetic import (RING_R_MM, coaxial_pose, fake_sampler, make_job, request,  # noqa: E402
                       write_frames, write_placement)


def _build(job, **kw):
    js.build_seams(job, [js.JointSeamRequest.from_dict(request(**kw))], 1.0, sampler=fake_sampler)
    return sa.build_seam_axis(job)


def test_coaxial_ring_r_constant_and_schema(tmp_path):
    job = make_job(tmp_path)
    doc = _build(job, positioner_mode='C')
    jsonio.validate(doc, 'v2/seam_axis')
    seam = doc['seams'][0]
    rs = [p['r_mm'] for p in seam['points']]
    assert max(rs) - min(rs) <= 0.01
    assert seam['summary']['r_min_mm'] == pytest.approx(RING_R_MM, abs=1e-6)
    assert seam['coaxial']['is_coaxial'] is True
    assert seam['coaxial']['axis_angle_deg'] < 1e-6
    assert abs(seam['summary']['axial_max_mm'] - seam['summary']['axial_min_mm']) < 1e-9
    # unwrapped phi covers one turn
    assert abs(seam['summary']['phi_end_deg'] - seam['summary']['phi_start_deg']) == pytest.approx(360.0, abs=1e-6)
    names = {i['name'] for i in doc['inputs']}
    assert names == {'seams.json', 'placement.json', '../../cell/model/frames.yaml'}


def test_off_axis_ring_is_not_coaxial(tmp_path):
    job = make_job(tmp_path, flange_T_part=coaxial_pose((0.0, 76.6, 0.0)))
    seam = _build(job)['seams'][0]
    assert seam['coaxial']['is_coaxial'] is False
    assert seam['summary']['r_max_mm'] - seam['summary']['r_min_mm'] > 20.0


def test_perpendicular_ring_is_not_coaxial(tmp_path):
    job = make_job(tmp_path, flange_T_part={'position_mm': [0.0, 0.0, 76.6], 'quat_xyzw': [0, 0, 0, 1]})
    seam = _build(job)['seams'][0]
    assert seam['coaxial']['is_coaxial'] is False
    assert seam['coaxial']['axis_angle_deg'] > 80.0


def test_part_axis_part_round_trip(tmp_path):
    pose = {'position_mm': [102.03, 321.58, -593.4], 'quat_xyzw': [0.7071067811865475, 0, 0, 0.7071067811865476]}
    job = make_job(tmp_path, flange_T_part=pose)
    doc = _build(job)
    seams = js.load_seams(job)
    for part_pt, ax_pt in zip(seams['seams'][0]['geometry']['points'], doc['seams'][0]['points']):
        back = sa.axis_to_part(doc, ax_pt)
        assert np.linalg.norm(np.subtract(back['p_mm'], part_pt['p_mm'])) <= 1e-6
        rot = frames.quat_to_matrix(back['quat_xyzw'])
        assert np.linalg.norm(rot[:, 0] - part_pt['t']) <= 1e-9
        z = np.add(part_pt['n1'], part_pt['n2'])
        assert np.linalg.norm(rot[:, 2] - z / np.linalg.norm(z)) <= 1e-9
        # beam = -Z, Y = Z x X
        assert np.allclose(np.cross(rot[:, 2], rot[:, 0]), rot[:, 1])


def test_reversing_the_direction_reverses_phi_order(tmp_path):
    fwd = _build(make_job(tmp_path / 'a'))['seams'][0]['points']
    rev = _build(make_job(tmp_path / 'b'), reversed=True)['seams'][0]['points']
    dphi_f = fwd[-1]['phi_deg'] - fwd[0]['phi_deg']
    dphi_r = rev[-1]['phi_deg'] - rev[0]['phi_deg']
    assert dphi_f == pytest.approx(-dphi_r, abs=1e-6) and abs(dphi_f) > 300
    # same points visited in the opposite order (same start vertex on the closed joint)
    for a, b in zip(fwd, reversed(rev)):
        assert np.allclose(a['p_mm'], b['p_mm'], atol=1e-6)


def test_start_offset_moves_phi_start(tmp_path):
    a = _build(make_job(tmp_path / 'a'))['seams'][0]
    quarter = 0.5 * math.pi * RING_R_MM
    b = _build(make_job(tmp_path / 'b'), start_offset_mm=quarter)['seams'][0]
    shift = (b['points'][0]['phi_deg'] - a['points'][0]['phi_deg']) % 360.0
    assert min(shift, 360.0 - shift) == pytest.approx(90.0, abs=1e-6)


def test_arcs_follow_the_seam(tmp_path):
    seam = _build(make_job(tmp_path), arcs=2, arc_overlap_mm=3.0, positioner_mode='B',
                  positioner_angles_deg=[0.0, 90.0])['seams'][0]
    length = seam['length_mm']
    assert [(a['index'], a['of']) for a in seam['arcs']] == [(1, 2), (2, 2)]
    assert seam['arcs'][0]['s_start_mm'] == 0.0
    assert seam['arcs'][0]['s_end_mm'] == pytest.approx(length / 2 + 3.0)
    assert seam['arcs'][1]['s_start_mm'] == pytest.approx(length / 2)
    assert seam['arcs'][1]['s_end_mm'] == pytest.approx(length + 3.0)  # closed: wraps over the start
    assert [a['positioner_angle_deg'] for a in seam['arcs']] == [0.0, 90.0]


@pytest.mark.parametrize('what', ['seams', 'placement', 'frames'])
def test_stale_file_is_refused(tmp_path, what):
    job = make_job(tmp_path)
    _build(job)
    sa.read_seam_axis(job)  # fresh: accepted
    if what == 'seams':
        js.build_seams(job, [js.JointSeamRequest.from_dict(request(reversed=True))], 1.0, sampler=fake_sampler)
    elif what == 'placement':
        write_placement(job, coaxial_pose((0.0, 1.0, 0.0)))
    else:
        write_frames(tmp_path / 'cell' / 'model' / 'frames.yaml', direction=(1.0, 0.0, 0.0))
        p = tmp_path / 'cell' / 'model' / 'frames.yaml'
        p.write_text(p.read_text() + '# edited\n')
    with pytest.raises(jsonio.StaleInputError):
        sa.read_seam_axis(job)


def test_frames_with_another_axis_convention_are_refused(tmp_path):
    job = make_job(tmp_path)
    js.build_seams(job, [js.JointSeamRequest.from_dict(request())], 1.0, sampler=fake_sampler)
    write_frames(tmp_path / 'cell' / 'model' / 'frames.yaml', direction=(0.0, 1.0, 0.0))
    with pytest.raises(sa.SeamAxisError, match='axis'):
        sa.build_seam_axis(job)


def test_non_unit_quaternion_in_placement_is_refused(tmp_path):
    job = make_job(tmp_path, flange_T_part={'position_mm': [0, 0, 0], 'quat_xyzw': [0, 0, 0, 1.01]})
    js.build_seams(job, [js.JointSeamRequest.from_dict(request())], 1.0, sampler=fake_sampler)
    with pytest.raises(sa.SeamAxisError, match='quaternion'):
        sa.build_seam_axis(job)


def test_tolerances_are_parameters(tmp_path):
    job = make_job(tmp_path, flange_T_part=coaxial_pose((0.0, 0.004, 0.0)))
    js.build_seams(job, [js.JointSeamRequest.from_dict(request())], 1.0, sampler=fake_sampler)
    loose = sa.build_seam_axis(job)['seams'][0]['coaxial']
    tight = sa.build_seam_axis(job, tol_r_spread_mm=0.001)['seams'][0]['coaxial']
    assert loose['is_coaxial'] is True and tight['is_coaxial'] is False
    assert tight['tol_r_spread_mm'] == 0.001
    assert json.loads(job.dir.joinpath('seam_axis.json').read_text())['seams'][0]['coaxial'] == tight
