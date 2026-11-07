"""G1.3 pure D07 v2 (MINOR 2.1) builder: schema, part-frame TCP poses, kinematic-only marks, rejected arcs, determinism.

Uses the synthetic cell / ring of ``test_core_sync`` (test data, not part data).
"""
import copy
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from awm_core import frames, jsonio, units  # noqa: E402
from awm_path.core.batch_kin import batch_fk  # noqa: E402
from awm_path.core.d07 import ArcPlan, PlannerInfo, build_path_doc, part_tcp_pose  # noqa: E402
from awm_path.core.kinematics import UrAnalyticKinematics, pose_error  # noqa: E402
from awm_path.core.sync import ArcSpec, SyncParams, plan_arc, rot_x  # noqa: E402
from test_core_sync import _cell, _ring_seam  # noqa: E402

FLANGE_T_PART = frames.make_transform([1.0, -2.0, 0.5], frames.axis_angle_to_matrix([0, 1, 0], 0.4))
HEADER = {'created_by': 'test', 'created_at': '2026-11-06T00:00:00Z',
          'inputs': [{'name': 'seam_axis.json', 'sha256': 'a' * 64}], 'run_id': 'R20261106T000000Z-abcdef'}
CONTEXT = {'cell_id': 'TEST', 'cell_rev': 'A', 'd14_sha256': 'b' * 64, 'cell_config_sha256': 'c' * 64,
           'calibration': {'calibration_id': 'CAL-NOMINAL', 'status': 'NOMINAL', 'sha256': 'd' * 64,
                           'components': {}},
           'robot_serial': 'SIM', 'kinematics_sha256': 'e' * 64, 'controller_calibration_sha256': None,
           'ur_safety_checksum': None, 'ur_installation': None, 'git': {'commit': 'abcdef0', 'dirty': False},
           'image_digest': None}


@pytest.fixture(scope='module')
def kin():
    return UrAnalyticKinematics()


@pytest.fixture(scope='module')
def plans(kin):
    seam = _ring_seam()
    L = seam['length_mm']
    ok = plan_arc(kin, _cell(), seam, ArcSpec(1, 1, 0.0, L + 2.0, None), 'sync', SyncParams())
    cell_far = _cell()
    far = copy.deepcopy(seam)
    for p in far['points']:
        p['p_mm'] = [p['p_mm'][0], p['p_mm'][1] + 2000.0, p['p_mm'][2]]
    bad = plan_arc(kin, cell_far, far, ArcSpec(1, 1, 0.0, L, None), 'sync', SyncParams())
    assert ok.ok and not bad.ok
    return [ArcPlan(ok, 8.0, 2.0), ArcPlan(bad, 8.0, 0.0)]


def _info(kin):
    return PlannerInfo(kinematics_backend=kin.name, head_model='hbox', params={'collision': 'off',
                                                                               'sync': SyncParams().record()})


def _doc(plans, kin):
    return build_path_doc(plans, _cell(), FLANGE_T_PART, _info(kin), HEADER, CONTEXT)


def test_doc_validates_and_is_kinematic_only(plans, kin):
    doc = _doc(plans, kin)
    jsonio.validate(doc, 'v2/path')
    assert doc['schema_version'] == '2.1' and doc['collision_checked'] is False
    assert doc['frame'] == 'part' and doc['tcp'] == 'tcp_laser'
    assert doc['planner']['scene_backend'].startswith('none') and doc['planner']['head_model'] == 'box'
    (seam,) = doc['seams']
    assert seam['mode'] == 'sync' and seam['overlap_mm'] == 2.0
    assert all(p['min_clearance_mm'] is None and p['status'] == 'ok' for p in seam['points'])
    assert seam['timed'][0]['kind'] == 'weld' and len(seam['timed'][0]['points']) == len(seam['points'])
    (rej,) = doc['rejected']
    assert rej['failing_ranges'] and rej['failing_points'] and rej['reason']
    assert 0.0 <= rej['coverage_fraction'] < 1.0


def test_part_frame_pose_and_q_agree_through_the_positioner(plans, kin):
    """Rotation model vs FK through the written D07: base_T_axis · Rx(θ) · flange_T_part · part_T_tcp = FK(q)."""
    doc = _doc(plans, kin)
    cell = _cell()
    for p in doc['seams'][0]['points']:
        pose = frames.make_transform(p['pose_mm']['position_mm'], frames.quat_to_matrix(p['pose_mm']['quat_xyzw']))
        want = cell.base_link_T_axis @ rot_x(p['theta_deg']) @ FLANGE_T_PART @ pose
        got = batch_fk(np.array([p['q_deg']]))[0] @ cell.tool0_T_tcp
        dp, dr = pose_error(got, want)
        assert dp <= 0.01 and units.rad_to_deg(dr) <= 0.01


def test_part_tcp_pose_beam_is_minus_seam_z():
    f = frames.make_transform([0.0, 5.0, 0.0], np.eye(3))
    t = part_tcp_pose(np.eye(4), f, 30.0)
    assert np.allclose(t[:3, 2], [0.0, 0.0, -1.0])
    assert np.allclose(t[:3, 3], [0.0, 5.0, 0.0])


def test_same_input_same_doc(plans, kin):
    assert _doc(plans, kin) == _doc(plans, kin)
    a, b = _doc(plans, kin), _doc(plans, kin)
    b['planner']['params']['collision'] = 'off '
    assert a['planner']['params_sha256'] != build_path_doc(
        plans, _cell(), FLANGE_T_PART, PlannerInfo(kin.name, 'hbox', b['planner']['params']), HEADER,
        CONTEXT)['planner']['params_sha256']


def test_speed_record_is_acceleration_aware_and_params_recorded_are_used(plans, kin):
    """G1.9: the accepted arc carries the spline-based speed record; a sync record that differs from the parameters
    used to time the arcs is refused (what is recorded is what was used)."""
    doc = _doc(plans, kin)
    pl = doc['seams'][0]['planning']
    assert pl['max_travel_speed_mm_s'] > 0.0 and len(pl['max_joint_speed_deg_s']) == 7
    assert pl['speed_within_limits'] is (8.0 <= pl['max_travel_speed_mm_s'])
    with pytest.raises(ValueError, match='differs'):
        build_path_doc(plans, _cell(), FLANGE_T_PART, _info(kin), HEADER, CONTEXT,
                       SyncParams(weld_ramp_accel_mm_s2=50.0))
