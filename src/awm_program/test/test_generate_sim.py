"""Integration: plan + generate on mock hardware for the generated plate part (needs OCC env)."""

import json
import os

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from awm_bringup import params_file, repo_template  # noqa: E402
from awm_bringup.testing import SimLaunch  # noqa: E402
from awm_cad import import_step  # noqa: E402
from awm_core import jsonio, occ_bridge  # noqa: E402
from awm_core.repository import Repository  # noqa: E402
from awm_path.moveit_client import MoveItClient  # noqa: E402
from awm_path.params import from_yaml  # noqa: E402
from awm_path.planner import plan_path  # noqa: E402
from awm_process import assign  # noqa: E402
from awm_program.generator import ProgramError, generate_program  # noqa: E402
from awm_seam.seams import SeamRequest, define_seams  # noqa: E402

pytestmark = pytest.mark.skipif(not os.environ.get(occ_bridge.ENV_VAR),
                                reason='AWM_OCC_PYTHON not set')


@pytest.fixture(scope='module')
def planned(tmp_path_factory):
    tmp = tmp_path_factory.mktemp('prog')
    repo = Repository(tmp / 'repo')
    repo.init_from_template(repo_template())
    params = from_yaml(params_file())
    step, picks = tmp / 'plate.step', tmp / 'plate.json'
    occ_bridge.run_geometry_cli(['make-test-part', '--kind', 'plate_tjoint', '--out', str(step),
                                 '--picks', str(picks)])
    data = json.loads(picks.read_text())
    job = import_step(repo.root, step, data['part_no'], data['part_rev']).job
    define_seams(job, [SeamRequest.from_dict(s) for s in data['seams']])
    assign(job, allow_sample_wps=True)
    with SimLaunch(log_dir=tmp):
        rclpy.init()
        node = rclpy.create_node('awm_program_test')
        client = MoveItClient(node, params.planning_group, params.tcp_link, params.base_frame,
                              params.joint_names, params.service_timeout_s)
        try:
            client.wait_ready(120.0)
            doc = plan_path(job, params, client)
            assert not doc['rejected'], doc['rejected']
            yield job, params, client
        finally:
            client.close()
            node.destroy_node()
            rclpy.shutdown()


def test_program_structure(planned):
    job, params, client = planned
    rev, doc = generate_program(job, params, client)
    assert rev == 'P001'
    jsonio.read_json(job.program_json(rev), 'program', base_dir=job.dir)
    segs = doc['segments']
    assert [s['kind'] for s in segs] == ['air', 'approach', 'weld', 'retract'] * 2 + ['air']
    home = np.asarray(params.home_joints_rad)
    assert np.allclose(segs[0]['points'][0]['q_rad'], home)
    assert np.allclose(segs[-1]['points'][-1]['q_rad'], home)
    for a, b in zip(segs, segs[1:]):
        assert np.allclose(a['points'][-1]['q_rad'], b['points'][0]['q_rad'], atol=1e-9)
    by_id = {s['segment_id']: s for s in segs}
    for ev in doc['events']:
        kind = by_id[ev['after_segment']]['kind']
        assert kind == ('approach' if ev['type'] == 'WeldStart' else 'weld')
        assert by_id[ev['after_segment']]['seam_id'] == ev['seam_id']
    assert [e['type'] for e in doc['events']] == ['WeldStart', 'WeldEnd'] * 2


def test_weld_runs_at_wps_speed(planned):
    job, params, client = planned
    doc = json.loads(job.program_json('P001').read_text())
    weld = [s for s in doc['segments'] if s['kind'] == 'weld'][0]
    pts = weld['points'][5:-5]
    tcp = [client.compute_fk(p['q_rad'], [params.tcp_link])[params.tcp_link][:3, 3] for p in pts]
    speeds = np.linalg.norm(np.diff(tcp, axis=0), axis=1) / np.diff([p['t_s'] for p in pts])
    assert np.allclose(speeds, weld['tcp_speed_m_s'], rtol=0.02)
    assert weld['tcp_speed_m_s'] == pytest.approx(0.006)
    assert weld['duration_s'] == pytest.approx(0.160 / 0.006, rel=0.01)


def test_rejected_seam_blocks_program(planned):
    job, params, client = planned
    original = job.path_json.read_text()
    doc = json.loads(original)
    doc['rejected'] = [{'seam_id': 'S09', 'reason': 'test'}]
    jsonio.write_json(job.path_json, doc, 'path')
    try:
        with pytest.raises(ProgramError, match='no partial programs'):
            generate_program(job, params, client)
    finally:
        job.path_json.write_text(original)


def test_calibration_change_makes_path_stale(planned):
    job, params, client = planned
    cal_path = job.repo.active_calibration
    original = cal_path.read_text()
    cal = json.loads(original)
    cal['fixture_T_part']['position_m'][0] += 0.001
    jsonio.write_json(cal_path, cal, 'calibration')
    try:
        with pytest.raises(jsonio.StaleInputError, match='active.json'):
            generate_program(job, params, client)
    finally:
        cal_path.write_text(original)
