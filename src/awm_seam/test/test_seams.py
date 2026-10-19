"""awm_seam on a synthetic edge catalogue (no OCC): an L-shaped chain of concave edges."""

import json

import numpy as np
import pytest

from awm_core import jsonio
from awm_core.repository import Repository
from awm_seam.cli import main
from awm_seam.seams import (EdgeCatalogue, SeamDefinitionError, SeamRequest, build_seam,
                            define_seams, load_seams, order_chain, rematch, remove_seam)

SHA = 'b' * 64


def _edge(i, a, b, convexity='concave', faces=('F1', 'F2'), salt=''):
    a, b = np.array(a, float), np.array(b, float)
    return {'id': f'E{i}', 'curve_type': 'line', 'length_m': float(np.linalg.norm(b - a)),
            'start_m': a.tolist(), 'end_m': b.tolist(), 'mid_m': ((a + b) / 2).tolist(),
            'closed': False, 'faces': list(faces), 'convexity': convexity, 'dihedral_deg': 90.0,
            'polyline_m': [a.tolist(), b.tolist()],
            'fingerprint': f'{i:064d}' if not salt else f'{salt}{i:063d}'}


@pytest.fixture
def job(tmp_path):
    job = Repository(tmp_path).job('PART', 'A')
    job.cache_dir.mkdir(parents=True)
    step = job.model_step
    step.write_text('fake')
    edges = jsonio.make_header('test', [step], job.dir)
    edges.update({
        'edges': [_edge(1, [0, 0, 0], [0.1, 0, 0]),
                  _edge(2, [0.1, 0.1, 0], [0.1, 0, 0], faces=('F1', 'F3')),  # reversed direction
                  _edge(3, [0, 0, 0], [0, 0, 0.05], convexity='convex'),
                  _edge(4, [0.3, 0, 0], [0.4, 0, 0])],
        'faces': [{'id': f'F{i}', 'surface_type': 'plane', 'area_m2': 0.01} for i in (1, 2, 3)]})
    jsonio.write_json(job.edges_json, edges, 'edges')
    part = jsonio.make_header('test', [step], job.dir)
    part.update({'part_no': 'PART', 'part_rev': 'A', 'step_file': 'p.step', 'step_sha256': SHA,
                 'unit_source': 'mm', 'bbox_m': {'min': [0, 0, 0], 'max': [1, 1, 1]},
                 'counts': {'solids': 1, 'shells': 1, 'faces': 3, 'edges': 4}, 'checks': [],
                 'healed': False,
                 'mesh': {'file': 'cache/mesh.stl', 'sha256': SHA, 'linear_deflection_m': 1e-4,
                          'angular_deflection_rad': 0.1, 'triangles': 1},
                 'brep_cache': {'file': 'cache/model.brep', 'sha256': SHA},
                 'edges_file': {'file': 'cache/edges.json', 'sha256': SHA}})
    jsonio.write_json(job.part_json, part, 'part')
    return job


def _req(seam_id='S01', **kw):
    base = dict(seam_id=seam_id, joint_type='fillet', leg_mm=4.0, sequence=1, material='S235JR',
                thickness_mm=8.0)
    base.update(kw)
    return SeamRequest(**base)


def test_pick_nearest_edge(job):
    cat = EdgeCatalogue.load(job)
    assert cat.pick([0.05, 0.0005, 0.0]) == 'E1'
    with pytest.raises(SeamDefinitionError, match='no edge within'):
        cat.pick([0.05, 0.05, 0.05])
    with pytest.raises(SeamDefinitionError, match='ambiguous'):
        cat.pick([0.0, 0.0, 0.0])  # shared vertex of E1 and E3


def test_chain_order_and_direction(job):
    cat = EdgeCatalogue.load(job)
    ids, start, end, closed = order_chain(cat, ['E2', 'E1'])
    assert not closed
    assert {tuple(start), tuple(end)} == {(0.0, 0.0, 0.0), (0.1, 0.1, 0.0)}
    assert ids[0] == ('E1' if tuple(start) == (0.0, 0.0, 0.0) else 'E2')
    with pytest.raises(SeamDefinitionError, match='not connected'):
        order_chain(cat, ['E1', 'E4'])


def test_build_seam(job):
    cat = EdgeCatalogue.load(job)
    seam = build_seam(cat, _req(edges=['E2'], picks_m=[[0.05, 0.0, 0.0]], process_hint='MIG'))
    assert seam['length_m'] == pytest.approx(0.2)
    rev = build_seam(cat, _req(edges=['E2', 'E1'], reversed=True))
    assert rev['start_m'] == seam['end_m'] and rev['end_m'] == seam['start_m']
    assert rev['edges'] == list(reversed(seam['edges']))
    assert rev['fingerprint'] == seam['fingerprint']


@pytest.mark.parametrize('kw, msg', [
    ({'edges': ['E3']}, 'convex'),
    ({'edges': ['E99']}, 'unknown edge'),
    ({'edges': []}, 'no edges'),
    ({'edges': ['E1'], 'seam_id': 'seam_0'}, 'must look like'),
    ({'edges': ['E1'], 'joint_type': 'spot'}, 'joint type'),
    ({'edges': ['E1'], 'leg_mm': 0.0}, 'must be > 0'),
    ({'edges': ['E1'], 'weld_side': 'F3'}, 'weld_side'),
    ({'edges': ['E1'], 'process_hint': 'TIG'}, 'process_hint'),
])
def test_invalid_requests(job, kw, msg):
    with pytest.raises(SeamDefinitionError, match=msg):
        build_seam(EdgeCatalogue.load(job), _req(**kw))


def test_butt_joint_accepts_non_concave(job):
    seam = build_seam(EdgeCatalogue.load(job), _req(edges=['E3'], joint_type='butt'))
    assert seam['edges'] == ['E3']


def test_define_export_remove(job):
    define_seams(job, [_req('S01', edges=['E1']), _req('S02', edges=['E4'], sequence=2)])
    doc = load_seams(job)
    assert [s['seam_id'] for s in doc['seams']] == ['S01', 'S02']
    assert doc['part_no'] == 'PART'
    with pytest.raises(SeamDefinitionError, match='duplicate sequence'):
        define_seams(job, [_req('S03', edges=['E2'], sequence=2)])
    with pytest.raises(SeamDefinitionError, match='used by'):
        define_seams(job, [_req('S03', edges=['E1'], sequence=3)])
    define_seams(job, [_req('S01', edges=['E2'])])  # replace S01
    assert load_seams(job)['seams'][0]['edges'] == ['E2']
    remove_seam(job, 'S02')
    assert [s['seam_id'] for s in load_seams(job)['seams']] == ['S01']


def test_cli(job, tmp_path):
    picks = tmp_path / 'picks.json'
    picks.write_text(json.dumps({'seams': [
        {'seam_id': 'S01', 'pick_m': [0.35, 0.0, 0.0], 'joint_type': 'fillet', 'leg_mm': 3,
         'sequence': 1, 'material': 'S235JR', 'thickness_mm': 6, 'process_hint': 'MIG'}]}))
    assert main(['from-picks', '--job', str(job.dir), '--picks', str(picks)]) == 0
    assert load_seams(job)['seams'][0]['edges'] == ['E4']
    assert main(['define', '--job', str(job.dir), '--seam-id', 'S02', '--edges', 'E3',
                 '--joint-type', 'fillet', '--leg-mm', '3', '--sequence', '2', '--material', 'X',
                 '--thickness-mm', '6']) == 1
    assert main(['list', '--job', str(job.dir)]) == 0


def test_rematch(job, tmp_path):
    define_seams(job, [_req('S01', edges=['E1'])])
    old_cat = EdgeCatalogue.load(job)
    new_doc = dict(old_cat.doc)
    # New revision: same geometry, edges renumbered (E1 -> E7), E4 changed.
    new_doc['edges'] = [dict(e, id='E7') if e['id'] == 'E1' else e for e in old_cat.doc['edges']]
    new_cat = EdgeCatalogue(new_doc)
    matched, missing = rematch(load_seams(job)['seams'], old_cat, new_cat)
    assert missing == [] and matched[0]['edges'] == ['E7']
    new_doc['edges'] = [e for e in new_doc['edges'] if e['id'] != 'E7']
    matched, missing = rematch(load_seams(job)['seams'], old_cat, EdgeCatalogue(new_doc))
    assert matched == [] and missing == ['S01']


def test_stale_edges_refused(job):
    define_seams(job, [_req('S01', edges=['E1'])])
    job.model_step.chmod(0o644)
    job.model_step.write_text('changed')
    with pytest.raises(jsonio.StaleInputError):
        EdgeCatalogue.load(job)
