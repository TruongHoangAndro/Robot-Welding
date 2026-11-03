"""Cell display model ``cell_view.glb`` (``export-cell-view``, contract S10_lite_cell_model.md v0.1).

Run with ``$AWM_OCC_PYTHON -m pytest``; skipped in an interpreter without OCC (colcon runs it through
test_occ_suite_in_conda.py).
"""

import json

import numpy as np
import pytest

ROT_Z90 = [[0.0, -1.0, 0.0, 10.0], [1.0, 0.0, 0.0, 20.0], [0.0, 0.0, 1.0, 30.0], [0.0, 0.0, 0.0, 1.0]]


@pytest.fixture(scope='module')
def cell(tmp_path_factory, occ):
    from awm_geometry import cli, testparts

    root = tmp_path_factory.mktemp('cellview')
    step = root / 'asm.step'
    testparts.write_assembly(step, testparts.weldment_assembly(), [ROT_Z90, None, None, None],
                             instance_names=True)
    comps = root / 'cell_components.json'
    assert cli.main(['inspect-assembly', '--step', str(step), '--out', str(comps)]) == 0
    out = root / 'view'
    assert cli.main(['export-cell-view', '--step', str(step), '--components', str(comps),
                     '--out', str(out), '--up-axis', '+Y']) == 0
    return step, json.loads(comps.read_text()), (out / 'cell_view.glb').read_bytes(), root


def test_asset_extras(cell):
    from awm_geometry import glb

    gltf, _ = glb.read_glb(cell[2])
    assert gltf['asset']['extras'] == {'unit': 'mm', 'frame': 'file', 'schema': 'cell_view',
                                       'schema_version': '0.1', 'up_axis': '+Y'}


def test_one_node_per_leaf_with_extras(cell):
    from awm_geometry import glb

    _, doc, data, _ = cell
    gltf, _ = glb.read_glb(data)
    leaves = [c for c in doc['components'] if not c['is_assembly']]
    roots = [gltf['nodes'][i] for i in gltf['scenes'][0]['nodes']]
    assert [n['name'] for n in roots] == [f'C{c["index"]}' for c in leaves]
    for node, comp in zip(roots, leaves):
        assert node['extras'] == {'index': comp['index'], 'instance_name': comp['instance_name'],
                                  'path': comp['path']}
        assert 'children' not in node and 'matrix' not in node and 'translation' not in node


def test_positions_in_file_frame_inside_bbox(cell):
    from awm_geometry import glb

    _, doc, data, _ = cell
    gltf, binary = glb.read_glb(data)
    by_index = {c['index']: c for c in doc['components']}
    for i in gltf['scenes'][0]['nodes']:
        node = gltf['nodes'][i]
        comp = by_index[node['extras']['index']]
        prim = gltf['meshes'][node['mesh']]['primitives'][0]
        pos = glb.accessor_array(gltf, binary, prim['attributes']['POSITION'])
        lo, hi = np.array(comp['bbox_mm']['min']), np.array(comp['bbox_mm']['max'])
        assert (pos >= lo - 1e-3).all() and (pos <= hi + 1e-3).all()
        # coarse but complete: the mesh spans the bbox within the 2.0 mm chord
        assert np.abs(pos.min(axis=0) - lo).max() < 2.0 and np.abs(pos.max(axis=0) - hi).max() < 2.0
        nor = glb.accessor_array(gltf, binary, prim['attributes']['NORMAL'])
        np.testing.assert_allclose(np.linalg.norm(nor, axis=1), 1.0, atol=1e-5)


def test_components_json_of_another_file_fails(cell, tmp_path):
    from awm_geometry import cli

    step, doc, _, _ = cell
    doc = dict(doc, source=dict(doc['source'], sha256='0' * 64))
    bad = tmp_path / 'bad.json'
    bad.write_text(json.dumps(doc))
    assert cli.main(['export-cell-view', '--step', str(step), '--components', str(bad),
                     '--out', str(tmp_path / 'o')]) == 1
    assert not (tmp_path / 'o' / 'cell_view.glb').exists()


def test_components_json_in_another_order_fails(cell, tmp_path, caplog):
    from awm_geometry import cli

    step, doc, _, _ = cell
    comps = [dict(c) for c in doc['components']]
    comps[1]['path'], comps[2]['path'] = comps[2]['path'], comps[1]['path']
    bad = tmp_path / 'swapped.json'
    bad.write_text(json.dumps(dict(doc, components=comps)))
    assert cli.main(['export-cell-view', '--step', str(step), '--components', str(bad),
                     '--out', str(tmp_path / 'o')]) == 1
    assert 'same order' in caplog.text
    assert not (tmp_path / 'o' / 'cell_view.glb').exists()


def test_tessellation_within_contract(occ):
    from awm_geometry import cell_view

    assert cell_view.LINEAR_DEFLECTION_MM <= 2.0 and cell_view.ANGULAR_DEFLECTION_DEG <= 30.0
