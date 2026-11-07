"""Pure G1.5 generator core on the synthetic cell of the G1.3 core tests (no xacro): Mode B (θ fixed), process
assignment passed through, index moves, refusals of the D07 content."""
import copy
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('ur_analytic_ik', reason='ur-analytic-ik venv not available (run inside awm_dev)')

from awm_path.core.d07 import ArcPlan, PlannerInfo, build_path_doc  # noqa: E402
from awm_path.core.kinematics import UrAnalyticKinematics, pose_error  # noqa: E402
from awm_path.core.sync import ArcSpec, SyncParams, plan_arc  # noqa: E402
from awm_program.core import program7  # noqa: E402
from awm_program.core.program7 import ProgramCell, ProgramParams, ProgramRefused, build_program  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'awm_path' / 'test'))
from test_core_sync import _cell, _ring_seam  # noqa: E402

HOME = (0.0, -90.0, 90.0, -90.0, -90.0, 0.0, 0.0)
ACC = (286.0,) * 6 + (50.0,)


@pytest.fixture(scope='module')
def kin():
    return UrAnalyticKinematics()


@pytest.fixture(scope='module')
def mode_b(kin):
    cell = _cell()
    seam = _ring_seam(closed=False)
    res = plan_arc(kin, cell, seam, ArcSpec(1, 1, 0.0, seam['points'][-1]['s_mm'], 35.0), 'B', SyncParams())
    assert res.ok
    header = {'created_by': 't', 'created_at': '2026-11-06T00:00:00Z', 'inputs': [], 'run_id': 'R20261106T000000Z-abcdef'}
    doc = build_path_doc([ArcPlan(res, 2.0, 0.0)], cell, np.eye(4),
                         PlannerInfo('test', 'hbox', {'sync': SyncParams().record()}), header, {})
    pcell = ProgramCell(cell, ACC, HOME, ('a', 'b', 'c', 'd', 'e', 'f'), ('positioner_joint',))
    return doc, pcell


def test_mode_b_program_theta_fixed_events_at_rest(kin, mode_b):
    doc, pcell = mode_b
    body = build_program(doc, {'S01': 1}, np.eye(4), pcell, kin, ProgramParams(), {'S01': ('WPS-T', 3)})
    kinds = [s['kind'] for s in body['segments']]
    assert kinds == ['air', 'approach', 'weld', 'retract', 'air']
    weld = body['segments'][2]
    assert weld['positioner_mode'] == 'B'
    q7 = np.array([p['positions_deg'] for p in weld['points']])
    assert set(q7[:, 6]) == {35.0}
    tcp = program7.tcp_in_part(pcell.sync, np.eye(4), q7)
    for k, p in enumerate(doc['seams'][0]['points']):
        dp, dr = pose_error(tcp[k], program7.pose_matrix(p['pose_mm']))
        assert dp <= 0.01
    by_id = {s['segment_id']: i for i, s in enumerate(body['segments'])}
    for ev in body['events']:
        assert (ev['wps_id'], ev['preset']) == ('WPS-T', 3)
        i = by_id[ev['after_segment']]
        assert body['segments'][i]['points'][-1]['velocities_deg_s'] == [0.0] * 7
        assert body['segments'][i + 1]['points'][0]['velocities_deg_s'] == [0.0] * 7


def test_index_move_when_only_theta_changes():
    b = program7._Builder(np.array(HOME))
    goal = np.array(HOME[:6] + (90.0,))
    pcell = ProgramCell(_cell(), ACC, HOME, tuple('abcdef'), ('p',))
    b.move_to(goal, pcell, ProgramParams())
    seg = b.segments[0]
    assert seg['kind'] == 'index' and seg['collision_checked'] is False
    q = np.array([p['positions_deg'] for p in seg['points']])
    assert np.abs(np.diff(q, axis=0)).max() <= 1.146 + 1e-9                # ≤ 0.02 rad joint step
    assert q[-1][6] == 90.0 and seg['points'][-1]['velocities_deg_s'] == [0.0] * 7
    b.move_to(goal, pcell, ProgramParams())                                   # already there: no segment
    assert len(b.segments) == 1


def test_refusals_of_d07_content(kin, mode_b):
    doc, pcell = mode_b
    bad = copy.deepcopy(doc)
    bad['seams'] = []
    with pytest.raises(ProgramRefused, match='no accepted arc'):
        build_program(bad, {}, np.eye(4), pcell, kin)
    bad = copy.deepcopy(doc)
    bad['seams'][0]['planning']['speed_within_limits'] = False
    bad['seams'][0]['planning']['max_travel_speed_mm_s'] = 1.234
    with pytest.raises(ProgramRefused, match=r'S01 arc 1/1.*max_travel_speed_mm_s = 1\.23'):
        build_program(bad, {}, np.eye(4), pcell, kin)
    bad = copy.deepcopy(doc)
    bad['seams'][0]['points'][5]['pose_mm']['position_mm'][0] += 0.1
    with pytest.raises(ProgramRefused, match='FK of the D07 joint states'):
        build_program(bad, {}, np.eye(4), pcell, kin)
    bad = copy.deepcopy(doc)
    del bad['planner']['params']['sync']
    with pytest.raises(ProgramRefused, match='planner.params.sync'):
        build_program(bad, {}, np.eye(4), pcell, kin)


def test_params_never_raise_limits():
    with pytest.raises(ValueError):
        ProgramParams(motion_velocity_scaling=1.5)
    with pytest.raises(ValueError):
        ProgramParams(approach_mm=60.0)


def test_line_starts_at_the_robot_pose_not_the_d07_target(kin, mode_b):
    """Mode C keeps the robot at rest, so FK(q) differs from the D07 poses by the accepted F8 residual; approach /
    retract must start from FK(q) (regression: golden J1 in Mode C was refused)."""
    doc, pcell = mode_b
    bad = copy.deepcopy(doc)
    for k in (0, -1):
        bad['seams'][0]['points'][k]['pose_mm']['position_mm'][1] += 0.005     # inside the 0.01 mm D07 tolerance
    body = build_program(bad, {'S01': 1}, np.eye(4), pcell, kin)
    for seg, at in ((body['segments'][1], -1), (body['segments'][3], 0)):
        q = np.array([p['positions_deg'] for p in seg['points']])
        assert q[at].tolist() == bad['seams'][0]['points'][0 if at == -1 else -1]['q_deg'] + [35.0]


def test_refuses_d07_weld_timing_params_other_than_the_program(kin, mode_b):
    """G1.12: the D07 maximum travel speed was found with planner.params.sync weld_ramp_accel_mm_s2 /
    speed_search_resolution_mm_s; a program with another ramp or resolution would judge the speed differently."""
    doc, pcell = mode_b
    with pytest.raises(ProgramRefused, match=r'weld_ramp_accel_mm_s2 .*5\.0.*6\.0.*re-plan'):
        build_program(doc, {'S01': 1}, np.eye(4), pcell, kin, ProgramParams(weld_ramp_accel_mm_s2=6.0))
    with pytest.raises(ProgramRefused, match=r'speed_search_resolution_mm_s .*re-plan'):
        build_program(doc, {'S01': 1}, np.eye(4), pcell, kin, ProgramParams(speed_search_resolution_mm_s=0.1))


@pytest.mark.parametrize('key', ['weld_ramp_accel_mm_s2', 'speed_search_resolution_mm_s'])
def test_refuses_d07_without_weld_timing_params(kin, mode_b, key):
    """A D07 planned before G1.9 has no weld ramp / resolution record: its maximum is velocity-only → re-plan."""
    doc, pcell = mode_b
    bad = copy.deepcopy(doc)
    del bad['planner']['params']['sync'][key]
    with pytest.raises(ProgramRefused, match=rf'{key}.*older D07.*re-plan'):
        build_program(bad, {'S01': 1}, np.eye(4), pcell, kin)
