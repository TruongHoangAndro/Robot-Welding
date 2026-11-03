"""WP-S S.4 C4: entry gate G2 (C1 scene sanity) — pose specs, resolution, verdict (TST-002 Rev 0.3 §5.2 G2).

Synthetic data only; the G2 pose set of the spike lives in docs/generated/spike/ (data, not source)."""

import numpy as np
import pytest

from awm_path.core.gate import (GatePose, GateResult, PoseSpecError, gate_summary, load_pose_specs,
                                resolve_pose, tool_pose)
from awm_path.core.kinematics import UrAnalyticKinematics, fk_tcp, pose_error
from awm_path.core.states import config_label


def _spec(**kw):
    d = {'id': 'P01', 'kind': 'free', 'description': 'test', 'theta_deg': 0.0,
         'tcp': {'frame': 'world', 'position_mm': [400.0, 100.0, 200.0], 'beam_dir': [0, 0, -1],
                 'x_dir': [1, 0, 0]},
         'checks': ['validity'], 'expect': {'valid': True}}
    d.update(kw)
    return d


def test_tool_pose_orthonormal_z_is_beam() -> None:
    t = tool_pose([1.0, 2.0, 3.0], [0.0, 0.0, -2.0], [1.0, 0.0, 0.5])
    assert np.allclose(t[:3, 2], [0, 0, -1])
    assert np.allclose(t[:3, 0], [1, 0, 0])
    assert np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3)) and np.isclose(np.linalg.det(t[:3, :3]), 1.0)
    assert np.allclose(t[:3, 3], [1, 2, 3])
    with pytest.raises(ValueError):
        tool_pose([0, 0, 0], [0, 0, 1], [0, 0, 2])          # x parallel to the beam


def test_load_pose_specs_checks_fields(tmp_path) -> None:
    doc = {'schema': 'awm_g2_poses/0.1', 'seam_faces': {'J2': {'n1': ['F1'], 'n2': ['F2']}},
           'poses': [_spec(), _spec(id='P02', kind='colliding', expect={'valid': False, 'objects': ['jig']}),
                     _spec(id='P03', kind='contact_zone', checks=['validity', 'contact'], seam='J2',
                           expect={'valid': False})]}
    poses, seams = load_pose_specs(doc)
    assert [p.pose_id for p in poses] == ['P01', 'P02', 'P03']
    assert seams == {'J2': (('F1',), ('F2',))}
    assert poses[1].expect_objects == ('jig',)
    for bad in ({'kind': 'other'}, {'checks': ['contact']}, {'id': 'P01'}, {'checks': ['validity', 'contact']},
                {'tcp': None}, {'expect': {}}):
        broken = dict(doc, poses=[_spec(), _spec(**{'id': 'P09', **bad})])
        with pytest.raises(PoseSpecError):
            load_pose_specs(broken)
    with pytest.raises(PoseSpecError, match='schema'):
        load_pose_specs(dict(doc, schema='x'))


def test_joint_spec_is_used_as_is() -> None:
    (p,), _ = load_pose_specs({'schema': 'awm_g2_poses/0.1', 'poses': [
        _spec(tcp=None, joints_deg=[0, -90, 0, -90, 0, 0])]})
    q, info = resolve_pose(p, np.eye(4), np.eye(4), np.eye(4), None)
    assert np.allclose(q, [0, -90, 0, -90, 0, 0]) and info['source'] == 'joints_deg'


def test_resolve_tcp_pose_in_part_frame_with_label() -> None:
    kin = UrAnalyticKinematics()
    tool0_T_tcp = np.eye(4)
    tool0_T_tcp[2, 3] = 481.0
    world_T_part = np.eye(4)
    world_T_part[:3, 3] = [300.0, -200.0, 100.0]
    (p,), _ = load_pose_specs({'schema': 'awm_g2_poses/0.1', 'poses': [
        _spec(tcp={'frame': 'part', 'position_mm': [50.0, -100.0, 0.0], 'beam_dir': [0, 0, -1], 'x_dir': [0, 1, 0]},
              label='S+E+W+')]})
    q, info = resolve_pose(p, np.eye(4), world_T_part, tool0_T_tcp, kin)
    assert config_label(q) == 'S+E+W+' and info['label'] == 'S+E+W+'
    dp, dr = pose_error(fk_tcp(kin, q, tool0_T_tcp), world_T_part @ tool_pose([50, -100, 0], [0, 0, -1], [0, 1, 0]))
    assert dp < 1e-6 and dr < 1e-9
    assert info['solutions'] >= 2


def test_resolve_without_label_takes_first_label_and_reports_missing_ik() -> None:
    kin = UrAnalyticKinematics()
    (p, far), _ = load_pose_specs({'schema': 'awm_g2_poses/0.1', 'poses': [
        _spec(), _spec(id='P02', tcp={'frame': 'world', 'position_mm': [5000.0, 0, 0], 'beam_dir': [0, 0, -1],
                                      'x_dir': [1, 0, 0]})]})
    q, info = resolve_pose(p, np.eye(4), np.eye(4), np.eye(4), kin)
    assert info['label'] == config_label(q)
    with pytest.raises(PoseSpecError, match='no IK'):
        resolve_pose(far, np.eye(4), np.eye(4), np.eye(4), kin)


def _result(pid, kind, valid, expect_valid, objects=(), contacts=()):
    pose = GatePose(pid, kind, 'd', 0.0, None, None, None, ('validity',), None, expect_valid, tuple(objects))
    return GateResult(pose, valid, tuple(contacts), None, None)


def test_gate_summary_counts_and_verdict() -> None:
    res = ([_result(f'C{i}', 'colliding', False, False, ['jig'], ['jig<->head_body']) for i in range(10)]
           + [_result(f'F{i}', 'free', True, True) for i in range(10)]
           + [_result(f'Z{i}', 'contact_zone', False, False) for i in range(3)])
    s = gate_summary(res)
    assert s['pass'] is True
    assert s['counts'] == {'colliding': [10, 10], 'free': [10, 10], 'contact_zone': [3, 3]}
    assert s['expected_objects_seen'] == [10, 10]
    res[3] = _result('C3', 'colliding', True, False, ['jig'])
    s = gate_summary(res)
    assert s['pass'] is False and s['failing'] == ['C3'] and s['counts']['colliding'] == [9, 10]
    s = gate_summary(res[10:])
    assert s['pass'] is False and 'colliding' in s['missing']
