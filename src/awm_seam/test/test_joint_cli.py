"""G1.2 end to end: ``joint_seam`` CLI -> conda sampler (awm_core.occ_bridge) -> seams.json + seam_axis.json.

Synthetic part: ``tube_tube`` of awm_geometry (needs AWM_OCC_PYTHON). Golden part: the session fixture ``golden``
(``conftest.py``, G1.22: repository rebuilt from the cell STEP by ``scripts/golden_rebuild.py``, skipped with
``AWM_CELL_STEP not set`` otherwise); its files are copied into a temporary repository, never written.
"""

import json
import math
import os
import shutil
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from awm_core import jsonio, occ_bridge  # noqa: E402
from awm_core.repository import Repository  # noqa: E402
from awm_seam import joint_cli, seam_axis  # noqa: E402
from synthetic import coaxial_pose, write_frames, write_placement  # noqa: E402

needs_occ = pytest.mark.skipif(not os.environ.get(occ_bridge.ENV_VAR),
                               reason=f'{occ_bridge.ENV_VAR} not set (OCC sampler)')
OUTER_R = 13.335  # mm, tube_tube test part (test data)


@pytest.fixture(scope='module')
def tube_job(tmp_path_factory):
    if not os.environ.get(occ_bridge.ENV_VAR):
        pytest.skip(f'{occ_bridge.ENV_VAR} not set (OCC sampler)')
    root = tmp_path_factory.mktemp('repo')
    step = root / 'tube_tube.step'
    occ_bridge.run_geometry_cli(['make-test-part', '--kind', 'tube_tube', '--out', step])
    job = Repository(root).job('TEST-TUBE-TUBE', 'A')
    occ_bridge.run_geometry_cli(['import', '--job', job.dir, '--step', step, '--part-no', 'TEST-TUBE-TUBE',
                                 '--part-rev', 'A'])
    occ_bridge.run_geometry_cli(['export-view', '--step', job.model_step, '--job', job.dir,
                                 '--out', job.dir / 'cache' / 'view'])
    write_placement(job, coaxial_pose())
    write_frames(root / 'cell' / 'model' / 'frames.yaml')
    topo = json.loads((job.dir / 'cache' / 'view' / 'topology.json').read_text())
    outer = next(j for j in topo['joints'] if abs(j['length_mm'] - 2 * math.pi * OUTER_R) < 0.01)
    return job, outer


def _run(argv, capsys):
    rc = joint_cli.main(argv)
    out = capsys.readouterr().out.strip().splitlines()
    return rc, (json.loads(out[-1]) if rc == 0 and out else None)


@needs_occ
def test_build_from_either_edge(tube_job, tmp_path, capsys):
    job, outer = tube_job
    results = []
    for edge in (outer['edge_a'], outer['edge_b']):
        picks = tmp_path / f'picks_{edge}.json'
        picks.write_text(json.dumps({'seams': [{'seam_id': 'S01', 'edges': [edge], 'joint_type': 'butt',
                                                'sequence': 1, 'positioner_mode': 'C', 'arc_overlap_mm': 2.0}]}))
        rc, summary = _run(['build', '--job', str(job.dir), '--picks', str(picks), '--pitch-mm', '1.0'], capsys)
        assert rc == 0
        results.append(json.loads(job.seams_json.read_text())['seams'][0])
        s = summary['seams']['S01']
        assert s['coaxial'] is True and abs(s['r_max_mm'] - s['r_min_mm']) <= 0.01
        assert s['r_min_mm'] == pytest.approx(OUTER_R, abs=0.01)
    assert results[0]['joint_edge_id'] == results[1]['joint_edge_id'] == outer['joint_edge_id']
    assert results[0]['geometry'] == results[1]['geometry']
    jsonio.read_json(job.seams_json, 'v2/seams', base_dir=job.dir)
    seam_axis.read_seam_axis(job)


@needs_occ
def test_axis_rebuild_after_placement_change(tube_job, tmp_path, capsys):
    job, outer = tube_job
    picks = tmp_path / 'picks.json'
    picks.write_text(json.dumps({'seams': [{'seam_id': 'S01', 'edges': [outer['edge_a']], 'joint_type': 'butt',
                                            'sequence': 1}]}))
    assert _run(['build', '--job', str(job.dir), '--picks', str(picks)], capsys)[0] == 0
    write_placement(job, coaxial_pose((0.0, 30.0, 0.0)))
    assert _run(['show', '--job', str(job.dir)], capsys)[0] == 1  # stale -> refused
    rc, summary = _run(['axis', '--job', str(job.dir)], capsys)
    assert rc == 0 and summary['seams']['S01']['coaxial'] is False
    write_placement(job, coaxial_pose())
    assert _run(['axis', '--job', str(job.dir)], capsys)[0] == 0


def test_bad_picks_exit_nonzero(tmp_path, capsys):
    from synthetic import make_job
    job = make_job(tmp_path)
    picks = tmp_path / 'picks.json'
    picks.write_text(json.dumps({'seams': [{'seam_id': 'S01', 'edges': ['E2'], 'joint_type': 'butt',
                                            'sequence': 1}]}))
    assert _run(['build', '--job', str(job.dir), '--picks', str(picks), '--no-axis'], capsys)[0] == 1


def _golden_repo(tmp_path, golden):
    assert os.environ.get(occ_bridge.ENV_VAR), f'{occ_bridge.ENV_VAR} not set (OCC sampler)'
    src_repo = Path(golden['repo'])
    name = golden['job_id']
    src = src_repo / 'jobs' / name
    repo = tmp_path / 'repo'
    dst = repo / 'jobs' / name
    (dst / 'cache' / 'view').mkdir(parents=True)
    for rel in ('model.step', 'part.json', 'placement.json', 'cache/model.brep', 'cache/collision.stl',
                'cache/collision_faces.json', 'cache/view/topology.json'):
        shutil.copyfile(src / rel, dst / rel)  # copies: inputs outside the repository are refused (G1.17)
    (repo / 'cell' / 'model').mkdir(parents=True)
    shutil.copyfile(src_repo / 'cell' / 'model' / 'frames.yaml', repo / 'cell' / 'model' / 'frames.yaml')
    # CP-60 placement.json also records the job / cell roles and axis picks as inputs: copy the missing ones
    for item in json.loads((src / 'placement.json').read_text())['inputs']:
        target = dst / item['name']
        if not os.path.lexists(target):
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src / item['name'], target)
    return dst


def test_golden_part_j1_coaxial_j2_not(tmp_path, capsys, golden):
    """Golden data: J1 = the joint of the outer tube circle (r 13.33 mm), J2 = the solid-elbow joint.

    Which JE ids they are is read from the golden summary (data file golden_564082.yaml)."""
    job_dir = _golden_repo(tmp_path, golden)
    topo = json.loads((job_dir / 'cache' / 'view' / 'topology.json').read_text())
    joints = {j['joint_edge_id']: j for j in topo['joints']}
    j1, j2 = golden['joints']['J1'], golden['joints']['J2']
    picks = tmp_path / 'picks.json'
    picks.write_text(json.dumps({'seams': [
        {'seam_id': 'S01', 'edges': [joints[j1]['edge_b']], 'joint_type': 'butt', 'sequence': 1,
         'positioner_mode': 'C', 'arc_overlap_mm': 2.0},
        {'seam_id': 'S02', 'edges': [joints[j2]['edge_a']], 'joint_type': 'butt', 'sequence': 2,
         'arcs': 2, 'arc_overlap_mm': 2.0}]}))
    rc, summary = _run(['build', '--job', str(job_dir), '--picks', str(picks), '--pitch-mm', '1.0'], capsys)
    assert rc == 0
    s1, s2 = summary['seams']['S01'], summary['seams']['S02']
    assert s1['r_min_mm'] == pytest.approx(13.33, abs=0.01) and s1['r_max_mm'] == pytest.approx(13.33, abs=0.01)
    assert s1['coaxial'] is True
    assert s2['coaxial'] is False
    print('GOLDEN', json.dumps(summary))
