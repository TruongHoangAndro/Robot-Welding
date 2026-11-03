"""Assembly component tree (``inspect-assembly``, input to the A18 cell CAD map, ADR-16).

Run with ``$AWM_OCC_PYTHON -m pytest``; skipped in an interpreter without OCC (colcon runs it through
test_occ_suite_in_conda.py). The cell CAD test runs only when ``AWM_CELL_STEP`` names the cell CAD.
"""

import json
import os

import numpy as np
import pytest

from awm_core import units

TOL_MM = 1e-6

# Rz(90 deg) and a translation of (10, 20, 30) mm: placement of component 'weldment' in the root.
ROT_Z90 = [[0.0, -1.0, 0.0, 10.0], [1.0, 0.0, 0.0, 20.0], [0.0, 0.0, 1.0, 30.0], [0.0, 0.0, 0.0, 1.0]]
SHIFT = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, -5.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


@pytest.fixture(scope='module')
def tree(tmp_path_factory, occ):
    """Synthetic assembly: weldment (tube + rod, rotated + moved), other (box, shifted), two 'dup'."""
    from awm_geometry import cli, testparts

    root = tmp_path_factory.mktemp('asm')
    step = root / 'asm.step'
    testparts.write_assembly(step, testparts.weldment_assembly(), [ROT_Z90, SHIFT, None, None])
    out = root / 'tree.json'
    assert cli.main(['inspect-assembly', '--step', str(step), '--out', str(out)]) == 0
    return json.loads(out.read_text())


def _by_path(tree):
    return {c['path']: c for c in tree['components']}


def test_header_and_unit(tree):
    assert tree['units']['length'] == 'mm' and tree['unit_source'] == 'mm'
    assert tree['source']['file'] == 'asm.step' and len(tree['source']['sha256']) == 64
    assert tree['frame'] == 'file'
    # full header (S.4 A re-review F5): inputs with the STEP hash
    assert tree['schema_version'] == '2.0' and tree['created_by'] and tree['created_at']
    assert tree['inputs'] == [{'name': 'asm.step', 'sha256': tree['source']['sha256']}]


def test_paths_names_depth_and_order(tree):
    comps = tree['components']
    paths = [c['path'] for c in comps]
    assert paths[0] == 'assembly'
    assert 'assembly / weldment / weldment-body1' in paths
    assert 'assembly / weldment / weldment-body2' in paths
    assert 'assembly / other / other-body1' in paths
    assert sum(1 for c in comps if c['product_name'] == 'dup') == 2
    by = _by_path(tree)
    assert by['assembly']['depth'] == 0 and by['assembly']['parent'] is None
    weld = by['assembly / weldment']
    assert weld['depth'] == 1 and weld['is_assembly'] and weld['product_name'] == 'weldment'
    assert weld['parent'] == 0 and weld['children'] == 2
    body = by['assembly / weldment / weldment-body1']
    assert body['depth'] == 2 and not body['is_assembly']
    assert comps[body['parent']]['path'] == 'assembly / weldment'
    # indices are positions in the list (parents come first)
    assert all(c['index'] == i for i, c in enumerate(comps))
    assert all(c['parent'] is None or c['parent'] < c['index'] for c in comps)


def test_placements_in_file_frame(tree):
    by = _by_path(tree)
    np.testing.assert_allclose(by['assembly']['placement_mm'], np.eye(4), atol=TOL_MM)
    np.testing.assert_allclose(by['assembly / weldment']['placement_mm'], ROT_Z90, atol=TOL_MM)
    np.testing.assert_allclose(by['assembly / weldment']['local_placement_mm'], ROT_Z90, atol=TOL_MM)
    # bodies sit at identity in their sub-assembly: global placement = the sub-assembly's
    body = by['assembly / weldment / weldment-body2']
    np.testing.assert_allclose(body['local_placement_mm'], np.eye(4), atol=TOL_MM)
    np.testing.assert_allclose(body['placement_mm'], ROT_Z90, atol=TOL_MM)
    np.testing.assert_allclose(by['assembly / other / other-body1']['placement_mm'], SHIFT,
                               atol=TOL_MM)


def test_bboxes_and_solid_counts(tree):
    by = _by_path(tree)
    # box (100..120, 0..20, 0..20) mm shifted by -5 mm in Y
    other = by['assembly / other / other-body1']
    np.testing.assert_allclose(other['bbox_mm']['min'], [100.0, -5.0, 0.0], atol=1e-3)
    np.testing.assert_allclose(other['bbox_mm']['max'], [120.0, 15.0, 20.0], atol=1e-3)
    assert other['solids'] == 1
    # tube + rod: radius 13.335 mm, z -30..40 mm, then Rz(90) + (10, 20, 30) mm
    r = units.m_to_mm(0.013335)
    weld = by['assembly / weldment']
    np.testing.assert_allclose(weld['bbox_mm']['min'], [10.0 - r, 20.0 - r, 0.0], atol=1e-3)
    np.testing.assert_allclose(weld['bbox_mm']['max'], [10.0 + r, 20.0 + r, 70.0], atol=1e-3)
    assert weld['solids'] == 2
    assert by['assembly']['solids'] == 5
    lo, hi = by['assembly']['bbox_mm']['min'], by['assembly']['bbox_mm']['max']
    assert lo[0] == pytest.approx(10.0 - r, abs=1e-3) and hi[0] == pytest.approx(310.0, abs=1e-3)


def test_main_cylinder_axis(tree):
    """Largest cylindrical face of a leaf: axis in the file frame (Z of the parts, kept by Rz)."""
    rod = _by_path(tree)['assembly / weldment / weldment-body2']
    cyl = rod['main_cylinder']
    assert cyl['radius_mm'] == pytest.approx(units.m_to_mm(0.013335), abs=1e-6)
    assert abs(abs(cyl['axis'][2]) - 1.0) < 1e-9
    np.testing.assert_allclose(cyl['point_mm'][:2], [10.0, 20.0], atol=1e-6)
    assert _by_path(tree)['assembly / other / other-body1']['main_cylinder'] is None


def test_missing_step_fails(tmp_path, occ):
    from awm_geometry import cli

    assert cli.main(['inspect-assembly', '--step', str(tmp_path / 'none.step'),
                     '--out', str(tmp_path / 'o.json')]) == 1
    assert not (tmp_path / 'o.json').exists()


@pytest.mark.skipif(not os.environ.get('AWM_CELL_STEP'), reason='AWM_CELL_STEP not set')
def test_cell_cad_tree(tmp_path, occ):
    """Cell CAD: the golden part component is present once with 3 solids."""
    from awm_geometry import cli

    out = tmp_path / 'cell.json'
    assert cli.main(['inspect-assembly', '--step', os.environ['AWM_CELL_STEP'],
                     '--out', str(out)]) == 0
    comps = json.loads(out.read_text())['components']
    from conftest import golden_record

    rec = golden_record()
    golden = [c for c in comps if c['instance_name'] == rec['component']]
    assert len(golden) == 1 and golden[0]['solids'] == rec['solids']
    assert golden[0]['product_name'] == rec['product']
