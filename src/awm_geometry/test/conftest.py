"""Shared fixtures: generated test parts imported into a temporary job package."""

import json
import os
from pathlib import Path

import pytest

from awm_core import jsonio
from awm_core.repository import JobPaths, Repository


def seam_from_edges(job, seam_id, edge_ids, sequence=1, reversed_=False, weld_side='bisector'):
    """Build a seams.json entry for test edges (the real builder lives in awm_seam)."""
    edges = {e['id']: e for e in json.loads(job.edges_json.read_text())['edges']}
    first, last = edges[edge_ids[0]], edges[edge_ids[-1]]
    start, end = (last['end_m'], first['start_m']) if reversed_ else (first['start_m'], last['end_m'])
    closed = len(edge_ids) == 1 and first['closed']
    return {'seam_id': seam_id, 'edges': list(edge_ids), 'reversed': reversed_,
            'joint_type': 'fillet', 'weld_side': weld_side, 'leg_mm': 4.0, 'sequence': sequence,
            'process_hint': 'MIG', 'material': 'S235JR', 'thickness_mm': 10.0,
            'start_m': start, 'end_m': end,
            'length_m': sum(edges[e]['length_m'] for e in edge_ids), 'closed': closed,
            'fingerprint': first['fingerprint']}


def write_seams(job, seams):
    doc = jsonio.make_header('test', [job.edges_json], job.dir)
    doc.update({'part_no': 'T', 'part_rev': 'A', 'seams': seams})
    jsonio.write_json(job.seams_json, doc, 'seams')


@pytest.fixture(scope='session')
def occ():
    return pytest.importorskip('OCC.Core.STEPControl')


@pytest.fixture(scope='session')
def imported_parts(tmp_path_factory, occ):
    """Generate both test parts and import them (A11-A14) once per session."""
    from awm_geometry import cli

    root = tmp_path_factory.mktemp('repo')
    repo = Repository(root)
    jobs = {}
    for kind in ('plate_tjoint', 'pipe_flange'):
        step = root / f'{kind}.step'
        assert cli.main(['make-test-part', '--kind', kind, '--out', str(step),
                         '--picks', str(root / f'{kind}_picks.json')]) == 0
        job = repo.job(kind.replace('_', '-'), 'A')
        assert cli.main(['import', '--job', str(job.dir), '--step', str(step),
                         '--part-no', kind, '--part-rev', 'A']) == 0
        jobs[kind] = job
    return jobs


REPO = Path(__file__).resolve().parents[3]


def golden_record():
    """Golden-part facts for real-CAD tests: ``AWM_GOLDEN_RECORD`` or the spike record in the repository."""
    from awm_geometry import miniyaml

    path = Path(os.environ.get('AWM_GOLDEN_RECORD') or
                REPO / 'docs' / 'generated' / 'spike' / 'golden_part_record.yaml')
    if not path.is_file():
        pytest.skip(f'golden-part record not found: {path} (set AWM_GOLDEN_RECORD)')
    return miniyaml.load(path)


def cell_map_path():
    """Role map of the cell CAD for real-CAD tests: ``AWM_CELL_MAP`` or the spike record in the repository."""
    return Path(os.environ.get('AWM_CELL_MAP') or
                REPO / 'docs' / 'generated' / 'spike' / 'cad_map_cell.yaml')


@pytest.fixture(scope='session')
def job_assembly_step(tmp_path_factory, occ):
    """CP-60 synthetic job STEP (testparts.job_assembly_nodes)."""
    from awm_geometry import testparts

    path = tmp_path_factory.mktemp('job') / 'job.step'
    testparts.write_tree_assembly(path, 'job', testparts.job_assembly_nodes())
    return path


@pytest.fixture(scope='session')
def job_assembly(job_assembly_step, tmp_path_factory):
    """(job, topology, inspect-assembly components): `import --components` + `export-view --components`."""
    from awm_geometry import cli

    job = JobPaths(tmp_path_factory.mktemp('repo') / 'JOB_A')
    assert cli.main(['import', '--job', str(job.dir), '--step', str(job_assembly_step), '--part-no', 'JOB',
                     '--part-rev', 'A', '--components']) == 0
    out = job.dir / 'cache' / 'view'
    assert cli.main(['export-view', '--step', str(job.model_step), '--job', str(job.dir), '--components',
                     '--up-axis', '+Z', '--out', str(out)]) == 0
    tree = job.dir / 'tree.json'
    assert cli.main(['inspect-assembly', '--step', str(job.model_step), '--out', str(tree)]) == 0
    return job, json.loads((out / 'topology.json').read_text()), json.loads(tree.read_text())['components']
