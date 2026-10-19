"""Integration: plan the generated parts against move_group on mock hardware.

Needs the OCC interpreter (AWM_OCC_PYTHON) for import / sampling; skipped otherwise.
"""

import json
import os

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from awm_bringup import params_file, repo_template  # noqa: E402
from awm_bringup.testing import SimLaunch  # noqa: E402
from awm_cad import import_step  # noqa: E402
from awm_core import frames, jsonio, occ_bridge  # noqa: E402
from awm_core.repository import Repository  # noqa: E402
from awm_path.moveit_client import MoveItClient  # noqa: E402
from awm_path.params import from_yaml  # noqa: E402
from awm_path.planner import PlanningError, plan_path  # noqa: E402
from awm_process import assign  # noqa: E402
from awm_seam.seams import SeamRequest, define_seams  # noqa: E402

pytestmark = pytest.mark.skipif(not os.environ.get(occ_bridge.ENV_VAR),
                                reason='AWM_OCC_PYTHON not set')


def _job(repo, tmp_path, kind):
    step = tmp_path / f'{kind}.step'
    picks = tmp_path / f'{kind}.json'
    occ_bridge.run_geometry_cli(['make-test-part', '--kind', kind, '--out', str(step), '--picks',
                                 str(picks)])
    data = json.loads(picks.read_text())
    job = import_step(repo.root, step, data['part_no'], data['part_rev']).job
    define_seams(job, [SeamRequest.from_dict(s) for s in data['seams']])
    assign(job, allow_sample_wps=True)
    return job


@pytest.fixture(scope='module')
def planning(tmp_path_factory):
    tmp = tmp_path_factory.mktemp('plan')
    repo = Repository(tmp / 'repo')
    repo.init_from_template(repo_template())
    params = from_yaml(params_file())
    with SimLaunch(log_dir=tmp) as sim:
        rclpy.init()
        node = rclpy.create_node('awm_path_test')
        client = MoveItClient(node, params.planning_group, params.tcp_link, params.base_frame,
                              params.joint_names, params.service_timeout_s)
        try:
            client.wait_ready(120.0)
            yield repo, tmp, params, client, sim
        finally:
            client.close()
            node.destroy_node()
            rclpy.shutdown()


def test_plate_both_seams_accepted(planning):
    repo, tmp, params, client, sim = planning
    job = _job(repo, tmp, 'plate_tjoint')
    doc = plan_path(job, params, client)
    assert doc['rejected'] == [], doc['rejected']
    assert [s['seam_id'] for s in doc['seams']] == ['S01', 'S02']
    for seam in doc['seams']:
        assert seam['reach']['fraction'] == 1.0
        # Part frame: weld poses lie on the seam (z = 10 mm plate top), torch points down.
        for pose in seam['weld_poses']:
            assert pose['position_m'][2] == pytest.approx(0.010, abs=1e-6)
            assert frames.pose_to_matrix(pose)[2, 2] < -0.5
    jsonio.read_json(job.path_json, 'path', base_dir=job.dir)


def test_pipe_partial_seams_accepted(planning):
    repo, tmp, params, client, sim = planning
    job = _job(repo, tmp, 'pipe_flange')
    doc = plan_path(job, params, client)
    assert doc['rejected'] == [], doc['rejected']
    lengths = [s['weld_length_m'] for s in doc['seams']]
    assert sum(lengths) == pytest.approx(2 * np.pi * 0.025, abs=1e-4)


def test_unreachable_seam_rejected_with_reason(planning):
    repo, tmp, params, client, sim = planning
    job = repo.job('TEST-PLATE-T', 'A')
    cal = json.loads(repo.active_calibration.read_text())
    original = dict(cal)
    cal['fixture_T_part'] = {'position_m': [1.5, 0.0, 0.0], 'quat_xyzw': [0, 0, 0, 1]}
    jsonio.write_json(repo.active_calibration, cal, 'calibration')
    try:
        doc = plan_path(job, params, client)
    finally:
        jsonio.write_json(repo.active_calibration, original, 'calibration')
    assert doc['seams'] == []
    assert all('no collision-free IK' in r['reason'] for r in doc['rejected'])


def test_tcp_mismatch_stops_planning(planning):
    repo, tmp, params, client, sim = planning
    job = repo.job('TEST-PLATE-T', 'A')
    cal = json.loads(repo.active_calibration.read_text())
    original = json.loads(json.dumps(cal))
    cal['tcp']['tcp_mig']['position_m'][2] += 0.002
    jsonio.write_json(repo.active_calibration, cal, 'calibration')
    try:
        with pytest.raises(PlanningError, match='TCP tcp_mig'):
            plan_path(job, params, client)
    finally:
        jsonio.write_json(repo.active_calibration, original, 'calibration')
