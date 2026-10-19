"""Phase 3 end-to-end: STEP -> part -> seams -> process -> path -> program -> report -> release.

Two generated parts (plate T-joint, pipe-to-flange in three arcs) through ``run_chain`` with its
own headless simulation (mock hardware), plus one negative case: a full 360 deg pipe seam must be
rejected at A4 with a reason and no program must be written.
"""

import json
import os

import pytest

from awm_core import jsonio, occ_bridge
from awm_core.repository import Repository
from awm_verify.release import is_locked
from awm_verify.run_chain import main as run_chain

pytestmark = pytest.mark.skipif(not os.environ.get(occ_bridge.ENV_VAR),
                                reason='AWM_OCC_PYTHON not set')

# Replay at 4x on mock hardware to keep the test short (the report records the factor).
FAST = ['--ros-args', '-p', 'replay_speed_scale:=4.0']


def _part(tmp_path, kind):
    step, picks = tmp_path / f'{kind}.step', tmp_path / f'{kind}.json'
    occ_bridge.run_geometry_cli(['make-test-part', '--kind', kind, '--out', str(step), '--picks',
                                 str(picks)])
    return step, picks, json.loads(picks.read_text())


@pytest.mark.parametrize('kind, n_seams', [('plate_tjoint', 2), ('pipe_flange', 3)])
def test_chain_releases_program(tmp_path, kind, n_seams):
    from awm_bringup import params_file
    step, picks, data = _part(tmp_path, kind)
    repo = Repository(tmp_path / 'repo')
    job = repo.job(data['part_no'], data['part_rev'])
    code = run_chain(['run_chain', '--job', str(job.dir), '--step', str(step), '--part-no',
                      data['part_no'], '--part-rev', data['part_rev'], '--picks', str(picks),
                      '--approved-by', 'pytest e2e', '--start-sim', '--ros-args',
                      '--params-file', str(params_file())] + FAST[1:])
    assert code == 0
    for name, schema in (('part.json', 'part'), ('seams.json', 'seams'),
                         ('process.json', 'process'), ('path.json', 'path')):
        jsonio.read_json(job.dir / name, schema, base_dir=job.dir)
    report = jsonio.read_json(job.report_json('P001'), 'verify_report', base_dir=job.dir)
    assert report['result'] == 'PASS'
    assert report['replay']['segments_ok'] == report['replay']['segments_total']
    assert len(report['seams']) == n_seams
    rel = jsonio.read_json(job.release_json, 'release', base_dir=job.dir)
    assert rel['program_rev'] == 'P001' and rel['scope'] == 'simulation'
    assert is_locked(job, 'P001')


def test_full_orbit_pipe_seam_rejected(tmp_path):
    step, picks, data = _part(tmp_path, 'pipe_flange')
    one = dict(data['seams'][0])
    for key in ('start_pick_m', 'end_pick_m', 'reversed'):
        one.pop(key, None)
    data['seams'] = [one]
    picks.write_text(json.dumps(data))
    repo = Repository(tmp_path / 'repo')
    job = repo.job(data['part_no'], data['part_rev'])
    code = run_chain(['run_chain', '--job', str(job.dir), '--step', str(step), '--part-no',
                      data['part_no'], '--part-rev', data['part_rev'], '--picks', str(picks),
                      '--start-sim'])
    assert code == 2
    path = jsonio.read_json(job.path_json, 'path', base_dir=job.dir)
    assert path['seams'] == [] and path['rejected'][0]['seam_id'] == 'S01'
    assert '100 %' in path['rejected'][0]['reason'] or 'IK' in path['rejected'][0]['reason']
    assert job.program_revs() == []
