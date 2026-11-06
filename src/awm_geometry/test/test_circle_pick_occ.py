"""CP-60 circle pick: centre / axis / radius of a circular D20 edge (positioner axis, CP-61 nozzle mouth).

Run with ``$AWM_OCC_PYTHON -m pytest`` (skipped without OCC; colcon runs it through test_occ_suite_in_conda.py).
"""

import json
import math

import numpy as np

TOL = 1e-6


def _pick(step, edge, tmp_path, *extra):
    from awm_geometry import cli

    out = tmp_path / f'pick_{edge}.json'
    rc = cli.main(['pick-circle', '--step', str(step), '--edge', edge, '--up-axis', '+Z', '--out', str(out),
                   *extra])
    return rc, (json.loads(out.read_text()) if out.is_file() else None)


def _edges(topo, path, curve, length_mm=None):
    solids = {s['solid_id'] for s in topo['solids'] if s.get('component_path') == path}
    return [e['edge_id'] for e in topo['edges'] if e['solid_id'] in solids and e['curve_type'] == curve
            and (length_mm is None or abs(e['length_mm'] - length_mm) < 1e-3)]


def test_pick_disc_rim_centre_axis_radius(job_assembly, tmp_path):
    from awm_core import jsonio
    from awm_geometry import testparts

    job, topo, _tree = job_assembly
    rims = _edges(topo, 'Positioner:1 / Disc:1', 'circle', 2 * math.pi * testparts.JOB_DISC_R_MM)
    assert len(rims) == 2
    rc, doc = _pick(job.model_step, rims[0], tmp_path, '--components')
    assert rc == 0
    jsonio.validate(doc, 'v2/axis_pick')
    c = np.array(testparts.JOB_AXIS_POINT_MM)
    assert min(abs(doc['center_file_mm'][0]), abs(doc['center_file_mm'][0] + 20.0)) < TOL
    np.testing.assert_allclose(doc['center_file_mm'][1:], c[1:], atol=TOL)
    assert abs(abs(doc['axis_file'][0]) - 1.0) < TOL
    assert abs(doc['radius_mm'] - testparts.JOB_DISC_R_MM) < TOL
    assert doc['edge_id'] == rims[0] and doc['method'] == 'pick' and doc['reversed'] is False
    assert doc['component_path'] is None and doc['source']['up_axis'] == '+Z'


def test_pick_reverse_flips_axis(job_assembly, tmp_path):
    from awm_geometry import testparts

    job, topo, _tree = job_assembly
    rim = _edges(topo, 'Positioner:1 / Disc:1', 'circle', 2 * math.pi * testparts.JOB_DISC_R_MM)[0]
    _rc, a = _pick(job.model_step, rim, tmp_path, '--components')
    rc, b = _pick(job.model_step, rim, tmp_path / '..', '--components', '--reverse')
    assert rc == 0 and b['reversed'] is True
    np.testing.assert_allclose(b['axis_file'], -np.array(a['axis_file']), atol=TOL)
    np.testing.assert_allclose(b['center_file_mm'], a['center_file_mm'], atol=TOL)


def test_pick_refuses_non_circle(job_assembly, tmp_path):
    job, topo, _tree = job_assembly
    line = _edges(topo, 'Robot:1', 'line')[0]
    rc, doc = _pick(job.model_step, line, tmp_path, '--components')
    assert rc != 0 and doc is None


def test_pick_unknown_edge_refused(job_assembly, tmp_path):
    job, _topo, _tree = job_assembly
    rc, doc = _pick(job.model_step, 'E99999', tmp_path, '--components')
    assert rc != 0 and doc is None


def test_pick_accepts_arc(tmp_path, occ):
    """A half ring (arc edges) gives the centre / axis / radius of its circle."""
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut

    from awm_geometry import cli, testparts

    half = BRepAlgoAPI_Cut(testparts.x_cylinder(25.0, 0.0, 10.0),
                           testparts.box_mm((-1.0, -600.0, 0.0), (11.0, -300.0, 150.0))).Shape()
    step = tmp_path / 'half.step'
    testparts.write_tree_assembly(step, 'half', [{'name': 'Half', 'shape': half}])
    out = tmp_path / 'view'
    assert cli.main(['export-view', '--step', str(step), '--components', '--up-axis', '+Z', '--out', str(out)]) == 0
    topo = json.loads((out / 'topology.json').read_text())
    arcs = [e['edge_id'] for e in topo['edges'] if e['curve_type'] == 'circle' and not e['closed']]
    assert arcs
    rc, doc = _pick(step, arcs[0], tmp_path, '--components')
    assert rc == 0
    np.testing.assert_allclose(doc['center_file_mm'][1:], testparts.JOB_AXIS_POINT_MM[1:], atol=TOL)
    assert abs(doc['radius_mm'] - 25.0) < TOL and abs(abs(doc['axis_file'][0]) - 1.0) < TOL
