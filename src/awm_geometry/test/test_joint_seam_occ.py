"""G1.2: joint seam sampling on the exact B-Rep (``cli sample-joints``), synthetic two-tube part.

Run with ``$AWM_OCC_PYTHON -m pytest``; skipped in an interpreter without OCC (colcon runs it through
test_occ_suite_in_conda.py).
"""

import json
import math

import numpy as np
import pytest

from awm_core.repository import Repository

OUTER_R = 13.335  # mm, tube_tube test part (test data)
INNER_R = 10.465


@pytest.fixture(scope='module')
def tube_job(tmp_path_factory):
    pytest.importorskip('OCC.Core.STEPControl')
    from awm_geometry import cli

    root = tmp_path_factory.mktemp('repo')
    step = root / 'tube_tube.step'
    assert cli.main(['make-test-part', '--kind', 'tube_tube', '--out', str(step)]) == 0
    job = Repository(root).job('TEST-TUBE-TUBE', 'A')
    assert cli.main(['import', '--job', str(job.dir), '--step', str(step), '--part-no', 'TEST-TUBE-TUBE',
                     '--part-rev', 'A']) == 0
    assert cli.main(['export-view', '--step', str(job.model_step), '--job', str(job.dir),
                     '--out', str(job.dir / 'cache' / 'view')]) == 0
    topo = json.loads((job.dir / 'cache' / 'view' / 'topology.json').read_text())
    return job, topo


def _joint(topo, radius):
    for j in topo['joints']:
        if abs(j['length_mm'] - 2 * math.pi * radius) < 0.01:
            return j
    raise AssertionError(f'no joint of radius {radius}')


def _sample(job, tmp_path, seams, pitch=1.0):
    from awm_geometry import cli

    req, out = tmp_path / 'req.json', tmp_path / 'out.json'
    req.write_text(json.dumps({'pitch_mm': pitch, 'seams': seams}))
    rc = cli.main(['sample-joints', '--job', str(job.dir), '--request', str(req), '--out', str(out)])
    return rc, (json.loads(out.read_text()) if rc == 0 else None)


def _seam(jid, **kw):
    base = {'seam_id': 'S01', 'joint_edge_id': jid, 'reversed': False, 'start_offset_mm': 0.0,
            'start_point_mm': None}
    base.update(kw)
    return base


def test_outer_joint_samples(tube_job, tmp_path):
    job, topo = tube_job
    joint = _joint(topo, OUTER_R)
    rc, out = _sample(job, tmp_path, [_seam(joint['joint_edge_id'])], pitch=1.0)
    assert rc == 0
    s = out['seams'][0]
    assert out['unit'] == 'mm' and out['frame'] == 'part'
    for key in ('solid_a', 'solid_b', 'edge_a', 'edge_b'):
        assert s[key] == joint[key]
    assert s['closed'] is True
    assert s['length_mm'] == pytest.approx(2 * math.pi * OUTER_R, abs=1e-6)
    assert s['spacing_mm'] <= 1.0
    pts = s['points']
    assert len(pts) == math.ceil(s['length_mm'] / 1.0) + 1
    assert pts[0]['s_mm'] == 0.0 and pts[-1]['s_mm'] == pytest.approx(s['length_mm'])
    assert np.allclose(pts[0]['p_mm'], pts[-1]['p_mm'], atol=1e-6)  # closed: ends on the start
    for p in pts:
        xy = np.asarray(p['p_mm'][:2])
        assert np.linalg.norm(xy) == pytest.approx(OUTER_R, abs=1e-6)
        radial = np.append(xy / np.linalg.norm(xy), 0.0)
        assert np.allclose(p['n1'], radial, atol=1e-6) and np.allclose(p['n2'], radial, atol=1e-6)
        assert abs(np.dot(p['t'], radial)) < 1e-9 and abs(np.linalg.norm(p['t']) - 1) < 1e-9
    steps = np.linalg.norm(np.diff([p['p_mm'] for p in pts], axis=0), axis=1)
    assert steps.max() <= 1.0
    assert s['faces_n1'] and s['faces_n2'] and set(s['faces_n1']).isdisjoint(s['faces_n2'])


def test_reversed_and_start_offset(tube_job, tmp_path):
    job, topo = tube_job
    jid = _joint(topo, OUTER_R)['joint_edge_id']
    _, fwd = _sample(job, tmp_path, [_seam(jid)])
    _, rev = _sample(job, tmp_path, [_seam(jid, reversed=True)])
    f, r = fwd['seams'][0]['points'], rev['seams'][0]['points']
    assert np.allclose(f[0]['p_mm'], r[0]['p_mm'], atol=1e-6)  # same start vertex
    for a, b in zip(f, reversed(r)):
        assert np.allclose(a['p_mm'], b['p_mm'], atol=1e-6)
        assert np.allclose(a['t'], -np.asarray(b['t']), atol=1e-6)
    quarter = 0.5 * math.pi * OUTER_R
    _, off = _sample(job, tmp_path, [_seam(jid, start_offset_mm=quarter)])
    o = off['seams'][0]
    assert o['start_offset_mm'] == pytest.approx(quarter)
    a0 = math.atan2(f[0]['p_mm'][1], f[0]['p_mm'][0])
    a1 = math.atan2(o['points'][0]['p_mm'][1], o['points'][0]['p_mm'][0])
    assert abs(((a1 - a0) - math.pi / 2 + math.pi) % (2 * math.pi) - math.pi) < 1e-6
    # start point given as a point: same result as the offset
    _, pt = _sample(job, tmp_path, [_seam(jid, start_offset_mm=0.0, start_point_mm=o['points'][0]['p_mm'])])
    assert pt['seams'][0]['start_offset_mm'] == pytest.approx(quarter, abs=1e-6)


def test_inner_joint_and_errors(tube_job, tmp_path):
    job, topo = tube_job
    jid = _joint(topo, INNER_R)['joint_edge_id']
    rc, out = _sample(job, tmp_path, [_seam(jid)], pitch=0.5)
    assert rc == 0 and out['seams'][0]['spacing_mm'] <= 0.5
    assert _sample(job, tmp_path, [_seam('JE99')])[0] == 1
    assert _sample(job, tmp_path, [_seam(jid)], pitch=0.0)[0] == 1
    assert _sample(job, tmp_path, [_seam(jid, start_point_mm=[0.0, 0.0, 50.0])])[0] == 1  # far from the edge
