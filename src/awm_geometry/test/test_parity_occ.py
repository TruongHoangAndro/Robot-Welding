"""D20 contract rule 5: the A21 edge catalogue (``cli import`` -> edges.json) and the D20 view model
(``cli export-view`` -> topology.json) of the same STEP and component have the same ids and fingerprints.

Run with ``$AWM_OCC_PYTHON -m pytest`` (colcon: through test_occ_suite_in_conda.py).
"""

import json

import numpy as np
import pytest

from awm_core import jsonio, units

GRID_MM = 0.05
N_RANDOM = 300
PER_FILE = 15
CELL_MM = 60.0


def _import_and_export(tmp_path, step, name, component=None):
    """Run import (job) and export-view (same job) on ``step``; return (edges.json, topology.json)."""
    from awm_geometry import cli

    job = tmp_path / 'jobs' / f'{name}_A'
    extra = ['--component', component] if component else []
    assert cli.main(['import', '--job', str(job), '--step', str(step), '--part-no', name,
                     '--part-rev', 'A', *extra]) == 0
    out = job / 'cache' / 'view'
    assert cli.main(['export-view', '--step', str(job / 'model.step'), '--job', str(job),
                     '--out', str(out), *extra]) == 0
    catalogue = json.loads((job / 'cache' / 'edges.json').read_text())
    topo = jsonio.read_json(out / 'topology.json', 'view_topology', base_dir=job)
    assert topo['source']['file'] == 'model.step'
    return catalogue, topo


def mismatches(catalogue, topo):
    """List of differences between the catalogue and D20 (ids, faces, closed, length, fingerprint)."""
    out = []
    cat_edges = {e['id']: e for e in catalogue['edges']}
    d20_edges = {e['edge_id']: e for e in topo['edges']}
    if set(cat_edges) != set(d20_edges):
        out.append(f'edge ids differ: {sorted(set(cat_edges) ^ set(d20_edges))}')
    for eid in sorted(set(cat_edges) & set(d20_edges)):
        c, d = cat_edges[eid], d20_edges[eid]
        for key in ('faces', 'closed', 'fingerprint', 'curve_type'):
            if c[key] != d[key]:
                out.append(f'{eid}.{key}: {c[key]} != {d[key]}')
        if abs(units.m_to_mm(c['length_m']) - d['length_mm']) > 1e-9 * max(1.0, d['length_mm']):
            out.append(f'{eid}.length: {c["length_m"]} m vs {d["length_mm"]} mm')
    cat_faces = [(f['id'], f['surface_type']) for f in catalogue['faces']]
    d20_faces = [(f['face_id'], f['surface_type']) for f in topo['faces']]
    if cat_faces != d20_faces:
        out.append('face ids / surface types differ')
    return out


@pytest.mark.parametrize('kind', ['tube_rod', 'tube_tube', 'plate_tjoint', 'pipe_flange'])
def test_parity_test_parts(tmp_path, occ, kind):
    from awm_geometry import occ_io, testparts

    shape, _ = testparts.PARTS[kind]()
    step = tmp_path / f'{kind}.step'
    occ_io.write_step_mm(shape, step)
    catalogue, topo = _import_and_export(tmp_path, step, kind)
    assert mismatches(catalogue, topo) == []


def test_parity_assembly_component(tmp_path, occ):
    from awm_geometry import testparts

    step = tmp_path / 'asm.step'
    testparts.write_assembly(step, testparts.weldment_assembly())
    catalogue, topo = _import_and_export(tmp_path, step, 'weldment', component='weldment')
    assert mismatches(catalogue, topo) == []
    assert len(topo['solids']) == 2 and len(topo['joints']) == 1


def _grid(rng, lo, hi):
    """Random value on the 0.05 mm grid in [lo, hi] mm."""
    return round(float(rng.integers(int(round(lo / GRID_MM)), int(round(hi / GRID_MM)) + 1))
                 * GRID_MM, 2)


def _random_file(rng, path, count):
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt

    from awm_geometry import occ_io, testparts

    shapes = []
    for k in range(count):
        origin = [k * CELL_MM + _grid(rng, 0.0, 5.0), _grid(rng, 0.0, 5.0), _grid(rng, -5.0, 5.0)]
        if rng.random() < 0.5:
            size = [_grid(rng, 1.0, 40.0) for _ in range(3)]
            shapes.append(testparts.box(units.mm_to_m(origin),
                                        units.mm_to_m(np.add(origin, size))))
        else:
            r, h = _grid(rng, 1.0, 20.0), _grid(rng, 1.0, 40.0)
            ax = gp_Ax2(gp_Pnt(*units.mm_to_m(np.add(origin, [r, r, 0.0]))), gp_Dir(0, 0, 1))
            shapes.append(BRepPrimAPI_MakeCylinder(ax, units.mm_to_m(r), units.mm_to_m(h)).Shape())
    occ_io.write_step_mm(testparts.separate(*shapes), path)


def test_parity_random_grid_parts(tmp_path, occ):
    """300 random boxes / cylinders with dimensions and positions on a 0.05 mm grid (values that sit
    on the 0.1 mm rounding boundary of the fingerprint): 0 catalogue / D20 mismatches."""
    rng = np.random.default_rng(20261102)
    problems, edges = [], 0
    for f in range(N_RANDOM // PER_FILE):
        step = tmp_path / f'grid{f:02d}.step'
        _random_file(rng, step, PER_FILE)
        catalogue, topo = _import_and_export(tmp_path, step, f'GRID{f:02d}')
        assert len(topo['solids']) == PER_FILE
        edges += len(topo['edges'])
        problems += [f'grid{f:02d} {m}' for m in mismatches(catalogue, topo)]
    assert problems == []
    assert edges > 2000
