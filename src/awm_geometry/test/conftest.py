"""Shared fixtures: generated test parts imported into a temporary job package."""

import json

import pytest

from awm_core import jsonio
from awm_core.repository import Repository


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
