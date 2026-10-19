"""OCC tests (run with ``$AWM_OCC_PYTHON -m pytest``; skipped in an interpreter without OCC)."""

import json

import numpy as np
import pytest

from awm_core import jsonio

from conftest import seam_from_edges, write_seams


def _edges(job):
    return json.loads(job.edges_json.read_text())['edges']


def test_part_record(imported_parts):
    job = imported_parts['plate_tjoint']
    part = jsonio.read_json(job.part_json, 'part', base_dir=job.dir)
    assert part['unit_source'] == 'mm'
    assert np.allclose(part['bbox_m']['min'], [-0.100, -0.075, 0.0], atol=1e-5)
    assert np.allclose(part['bbox_m']['max'], [0.100, 0.075, 0.090], atol=1e-5)
    assert part['counts']['solids'] == 1
    assert all(c['ok'] for c in part['checks'])
    assert part['mesh']['triangles'] > 0
    assert job.model_step.is_file() and job.mesh_stl.is_file() and job.brep_cache.is_file()


def test_step_in_metres_reads_same_size(tmp_path, occ):
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.Interface import Interface_Static
    from OCC.Core.STEPControl import STEPControl_AsIs, STEPControl_Writer

    from awm_geometry import occ_io, topology
    box_mm = BRepPrimAPI_MakeBox(120.0, 30.0, 8.0).Shape()
    Interface_Static.SetCVal('write.step.unit', 'M')
    writer = STEPControl_Writer()
    assert writer.Transfer(box_mm, STEPControl_AsIs) == IFSelect_RetDone
    assert writer.Write(str(tmp_path / 'm.step')) == IFSelect_RetDone
    Interface_Static.SetCVal('write.step.unit', 'MM')
    shape, unit = occ_io.read_step(tmp_path / 'm.step')
    lo, hi = topology.bbox(shape)
    assert unit == 'm'
    assert np.allclose(np.subtract(hi, lo), [0.120, 0.030, 0.008], atol=1e-6)


def test_edge_catalogue_plate(imported_parts):
    edges = _edges(imported_parts['plate_tjoint'])
    concave = [e for e in edges if e['convexity'] == 'concave']
    long_ones = [e for e in concave if abs(e['length_m'] - 0.160) < 1e-6]
    assert len(concave) == 4 and len(long_ones) == 2
    for e in long_ones:
        assert e['curve_type'] == 'line'
        assert len(e['faces']) == 2
        assert abs(e['dihedral_deg'] - 90.0) < 1e-6
        assert abs(e['mid_m'][2] - 0.010) < 1e-6
        assert len(e['polyline_m']) >= 2
    assert len({e['fingerprint'] for e in edges}) == len(edges)


def test_edge_catalogue_pipe(imported_parts):
    edges = _edges(imported_parts['pipe_flange'])
    weld = [e for e in edges if e['convexity'] == 'concave' and e['curve_type'] == 'circle'
            and abs(e['length_m'] - 2 * np.pi * 0.025) < 1e-6]
    assert len(weld) == 1 and weld[0]['closed']
    assert not [e for e in edges if e['convexity'] == 'free']


def _long_concave(job):
    return [e for e in _edges(job) if e['convexity'] == 'concave' and e['length_m'] > 0.1]


def test_sample_straight_seam(imported_parts):
    from awm_geometry import cli
    job = imported_parts['plate_tjoint']
    e1, e2 = _long_concave(job)
    write_seams(job, [seam_from_edges(job, 'S01', [e1['id']]),
                      seam_from_edges(job, 'S02', [e2['id']], 2, reversed_=True)])
    assert cli.main(['sample', '--job', str(job.dir), '--spacing-m', '0.005']) == 0
    doc = jsonio.read_json(job.seam_geometry_json, 'seam_geometry', base_dir=job.dir)
    s1, s2 = doc['seams']
    assert len(s1['points']) == 33
    s = [p['s_m'] for p in s1['points']]
    assert np.all(np.diff(s) > 0) and abs(s[-1] - 0.160) < 1e-6
    assert np.allclose(s1['points'][0]['p_m'], e1['start_m'], atol=1e-7)
    assert np.allclose(s2['points'][0]['p_m'], e2['end_m'], atol=1e-7)
    for seam in (s1, s2):
        chord = np.subtract(seam['points'][-1]['p_m'], seam['points'][0]['p_m'])
        for p in seam['points']:
            t, n1, n2 = (np.array(p[k]) for k in ('t', 'n1', 'n2'))
            assert np.isclose(np.linalg.norm(t), 1) and np.isclose(np.linalg.norm(n1), 1)
            assert abs(t @ n1) < 1e-6 and abs(t @ n2) < 1e-6
            assert abs(n1 @ n2) < 1e-6
            assert t @ chord > 0
            # Outward normals of a T-joint root: one is +z (plate top), one is horizontal (web).
            assert np.isclose(max(abs(n1[2]), abs(n2[2])), 1.0)


def test_sample_closed_seam_and_stale_input(imported_parts):
    from awm_geometry import cli
    job = imported_parts['pipe_flange']
    weld = [e for e in _edges(job) if e['convexity'] == 'concave'
            and abs(e['length_m'] - 2 * np.pi * 0.025) < 1e-6][0]
    write_seams(job, [seam_from_edges(job, 'S01', [weld['id']])])
    assert cli.main(['sample', '--job', str(job.dir), '--spacing-m', '0.004']) == 0
    seam = jsonio.read_json(job.seam_geometry_json, 'seam_geometry')['seams'][0]
    pts = seam['points']
    assert seam['closed']
    assert np.allclose(pts[0]['p_m'], pts[-1]['p_m'], atol=1e-7)
    for p in pts:
        r = np.hypot(p['p_m'][0], p['p_m'][1])
        assert abs(r - 0.025) < 1e-6 and abs(p['p_m'][2] - 0.012) < 1e-7
        normals = [np.array(p['n1']), np.array(p['n2'])]
        radial = np.array([p['p_m'][0], p['p_m'][1], 0.0]) / r
        assert any(np.allclose(n, [0, 0, 1], atol=1e-6) for n in normals)
        assert any(np.allclose(n, radial, atol=1e-6) for n in normals)
    # Editing seams.json after sampling makes seam_geometry.json stale.
    seams = json.loads(job.seams_json.read_text())
    seams['seams'][0]['leg_mm'] = 5.0
    jsonio.write_json(job.seams_json, seams, 'seams')
    with pytest.raises(jsonio.StaleInputError):
        jsonio.read_json(job.seam_geometry_json, 'seam_geometry', base_dir=job.dir)


def test_sample_partial_closed_seam(imported_parts):
    """A22 start / end on the pipe circle: half circle crossing the edge origin, reversed."""
    from awm_geometry import cli
    job = imported_parts['pipe_flange']
    weld = [e for e in _edges(job) if e['convexity'] == 'concave'
            and abs(e['length_m'] - 2 * np.pi * 0.025) < 1e-6][0]
    seam = seam_from_edges(job, 'S01', [weld['id']], reversed_=True)
    r = 0.025
    start = [r * np.cos(np.radians(90)), r * np.sin(np.radians(90)), 0.012]
    end = [r * np.cos(np.radians(-90)), r * np.sin(np.radians(-90)), 0.012]
    total = 2 * np.pi * r
    seam.update({'start_m': start, 'end_m': end, 'length_m': np.pi * r, 'closed': False,
                 'range_m': [0.75 * total, 1.25 * total], 'chain_length_m': total})
    write_seams(job, [seam])
    assert cli.main(['sample', '--job', str(job.dir), '--spacing-m', '0.004']) == 0
    out = jsonio.read_json(job.seam_geometry_json, 'seam_geometry')['seams'][0]
    pts = out['points']
    assert out['length_m'] == pytest.approx(np.pi * r, abs=1e-5)  # chord sum of 0.5 mm steps
    assert np.allclose(pts[0]['p_m'], start, atol=1e-6)
    assert np.allclose(pts[-1]['p_m'], end, atol=1e-6)
    # Reversed (clockwise seen from +z): from +y through the seam line at +x down to -y.
    mid = pts[len(pts) // 2]['p_m']
    assert mid[0] > 0.024
    cross = np.cross(np.array(pts[0]['p_m']) - [0, 0, 0.012], pts[0]['t'])
    assert cross[2] < 0


def test_sample_rejects_unknown_edge(imported_parts):
    from awm_geometry import cli
    job = imported_parts['plate_tjoint']
    seam = seam_from_edges(job, 'S01', [_long_concave(job)[0]['id']])
    seam['edges'] = ['E999']
    write_seams(job, [seam])
    assert cli.main(['sample', '--job', str(job.dir), '--spacing-m', '0.005']) == 1


def test_import_missing_step_fails(tmp_path, occ):
    from awm_geometry import cli
    assert cli.main(['import', '--job', str(tmp_path / 'jobs' / 'X_A'), '--step',
                     str(tmp_path / 'none.step'), '--part-no', 'X', '--part-rev', 'A']) == 1
