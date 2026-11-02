"""D20 view model export (A13, contract docs/generated/contracts/D20_view_model.md v2.0).

Run with ``$AWM_OCC_PYTHON -m pytest``; skipped in an interpreter without OCC (colcon runs it through
test_occ_suite_in_conda.py). The golden-part test runs only when ``AWM_CELL_STEP`` names the cell CAD.
"""

import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

from awm_core import jsonio, units

ANGLE_LIMIT_DEG = 0.5
UNIT_TOL = 1e-9
FLOAT32_TOL_MM = 1e-4  # float32 rounding of positions around 100 mm


def _angle_deg(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    c = a @ b / (np.linalg.norm(a) * np.linalg.norm(b))
    return units.rad_to_deg(math.acos(float(np.clip(c, -1.0, 1.0))))


def _export(step, out, *extra):
    from awm_geometry import cli

    assert cli.main(['export-view', '--step', str(step), '--out', str(out), *extra]) == 0
    topo = jsonio.read_json(out / 'topology.json', 'view_topology', base_dir=Path(step).parent)
    return topo, (out / 'view.glb').read_bytes()


def check_glb(topo, data):
    """Structure of view.glb against the contract and topology.json; return {id: positions}."""
    import struct

    from awm_geometry import glb

    magic, version, length = struct.unpack_from('<III', data, 0)
    assert (magic, version, length) == (glb.GLB_MAGIC, 2, len(data))
    gltf, binary = glb.read_glb(data)
    assert gltf['asset']['version'] == '2.0'
    assert gltf['asset']['extras'] == {'unit': 'mm', 'frame': 'part', 'schema': 'D20',
                                       'schema_version': '2.0'}
    assert 'extensionsUsed' not in gltf
    assert gltf['buffers'] == [{'byteLength': len(binary)}]
    for view in gltf['bufferViews']:
        assert view['byteOffset'] % 4 == 0
        assert view['byteOffset'] + view['byteLength'] <= len(binary)
    nodes = gltf['nodes']
    (root,) = [nodes[i] for i in gltf['scenes'][gltf['scene']]['nodes']]
    assert root['name'] == 'part'
    solid_ids = [s['solid_id'] for s in topo['solids']]
    assert [nodes[i]['name'] for i in root['children']] == solid_ids

    by_name = {}
    for i in root['children']:
        for c in nodes[i].get('children', []):
            assert nodes[c]['name'] not in by_name, f'duplicate node {nodes[c]["name"]}'
            by_name[nodes[c]['name']] = (nodes[i]['name'], nodes[c])
    expected = {f['face_id'] for f in topo['faces']} | {e['edge_id'] for e in topo['edges']}
    assert set(by_name) == expected

    positions = {}
    for f in topo['faces']:
        solid, node = by_name[f['face_id']]
        assert solid == f['solid_id']
        assert node['extras'] == {'face_id': f['face_id'], 'solid_id': f['solid_id']}
        (prim,) = gltf['meshes'][node['mesh']]['primitives']
        assert prim['mode'] == glb.MODE_TRIANGLES
        assert set(prim['attributes']) == {'POSITION', 'NORMAL'}
        pos = glb.accessor_array(gltf, binary, prim['attributes']['POSITION'])
        nor = glb.accessor_array(gltf, binary, prim['attributes']['NORMAL'])
        idx_acc = gltf['accessors'][prim['indices']]
        assert idx_acc['componentType'] == glb.UNSIGNED_INT
        idx = glb.accessor_array(gltf, binary, prim['indices'])
        assert len(pos) == len(nor) > 0 and len(idx) % 3 == 0 and len(idx) > 0
        assert idx.max() < len(pos)
        np.testing.assert_allclose(np.linalg.norm(nor, axis=1), 1.0, atol=1e-5)
        _check_bounds(gltf, prim['attributes']['POSITION'], pos)
        positions[f['face_id']] = pos
    for e in topo['edges']:
        solid, node = by_name[e['edge_id']]
        assert solid == e['solid_id']
        assert node['extras'] == {'edge_id': e['edge_id'], 'solid_id': e['solid_id'],
                                  'joint_edge_id': e['joint_edge_id']}
        (prim,) = gltf['meshes'][node['mesh']]['primitives']
        assert prim['mode'] == glb.MODE_LINE_STRIP and set(prim['attributes']) == {'POSITION'}
        pos = glb.accessor_array(gltf, binary, prim['attributes']['POSITION'])
        assert len(pos) >= 2
        _check_bounds(gltf, prim['attributes']['POSITION'], pos)
        positions[e['edge_id']] = pos
    return positions


def _check_bounds(gltf, index, pos):
    acc = gltf['accessors'][index]
    assert acc['type'] == 'VEC3' and acc['componentType'] == 5126 and acc['count'] == len(pos)
    np.testing.assert_array_equal(np.float32(acc['min']), pos.min(axis=0))
    np.testing.assert_array_equal(np.float32(acc['max']), pos.max(axis=0))


def check_joints_consistent(topo):
    edges = {e['edge_id']: e for e in topo['edges']}
    for j in topo['joints']:
        assert edges[j['edge_a']]['joint_edge_id'] == j['joint_edge_id']
        assert edges[j['edge_b']]['joint_edge_id'] == j['joint_edge_id']
        assert edges[j['edge_a']]['solid_id'] == j['solid_a'] != j['solid_b']
        assert edges[j['edge_b']]['solid_id'] == j['solid_b']
        assert len(j['preview_frames']) == 4
        for f in j['preview_frames']:
            assert abs(np.linalg.norm(f['x']) - 1.0) < UNIT_TOL
            assert abs(np.linalg.norm(f['z']) - 1.0) < UNIT_TOL
            assert abs(float(np.dot(f['x'], f['z']))) < UNIT_TOL
    tagged = {e['edge_id'] for e in topo['edges'] if e['joint_edge_id']}
    assert tagged == {j['edge_a'] for j in topo['joints']} | {j['edge_b'] for j in topo['joints']}


def _radial_check(frame, outward):
    """Z radial (outward or inward) and X tangential for a circle about the part Z axis."""
    p = np.array(frame['point_mm'])
    radial = np.array([p[0], p[1], 0.0])
    radial /= np.linalg.norm(radial)
    if not outward:
        radial = -radial
    tangent = np.cross([0.0, 0.0, 1.0], radial)
    assert _angle_deg(frame['z'], radial) < ANGLE_LIMIT_DEG
    assert min(_angle_deg(frame['x'], tangent), _angle_deg(frame['x'], -tangent)) < ANGLE_LIMIT_DEG


def _write_part(tmp_path, kind):
    from awm_geometry import occ_io, testparts

    shape, _ = testparts.PARTS[kind]()
    step = tmp_path / f'{kind}.step'
    occ_io.write_step_mm(shape, step)
    return step


@pytest.fixture(scope='module')
def tube_rod(tmp_path_factory, occ):
    tmp = tmp_path_factory.mktemp('tube_rod')
    step = _write_part(tmp, 'tube_rod')
    first = _export(step, tmp / 'out1')
    second = _export(step, tmp / 'out2')
    return step, first, second


def test_tube_rod_topology_and_glb(tube_rod):
    _, (topo, data), _ = tube_rod
    assert topo['schema_version'] == '2.0' and topo['unit'] == 'mm'
    assert topo['source'] == {'file': 'tube_rod.step', 'component': None, 'up_axis': '+Z'}
    assert len(topo['solids']) == 2
    check_glb(topo, data)
    check_joints_consistent(topo)


def test_tube_rod_single_closed_joint_with_radial_frames(tube_rod):
    _, (topo, _), _ = tube_rod
    (joint,) = topo['joints']
    assert joint['joint_edge_id'] == 'JE1' and joint['closed']
    assert joint['length_mm'] == pytest.approx(math.pi * 26.67, abs=0.01)
    assert sum(1 for e in topo['edges'] if e['joint_edge_id'] == 'JE1') == 2
    for frame in joint['preview_frames']:
        _radial_check(frame, outward=True)


def test_tube_rod_edge_chord(tube_rod):
    """Every exact curve point lies within edge_chord_mm of its GLB polyline."""
    from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
    from OCC.Core.TopoDS import topods

    from awm_geometry import assembly, view

    step, (topo, data), _ = tube_rod
    chord = topo['tessellation']['edge_chord_mm']
    assert chord <= 0.05
    polylines = check_glb(topo, data)
    model = view.build_model(assembly.read_part(step, None))
    worst = 0.0
    for e in topo['edges']:
        curve = BRepAdaptor_Curve(topods.Edge(model.edges.FindKey(int(e['edge_id'][1:]))))
        u0, u1 = curve.FirstParameter(), curve.LastParameter()
        exact = np.array([[q.X(), q.Y(), q.Z()] for q in
                          (curve.Value(u0 + (u1 - u0) * k / 999) for k in range(1000))])
        poly = polylines[e['edge_id']].astype(float)
        worst = max(worst, float(_dist_to_polyline(exact, poly).max()))
    assert worst <= chord + FLOAT32_TOL_MM, worst


def _dist_to_polyline(points, poly):
    a, b = poly[:-1], poly[1:]
    ab = b - a
    denom = np.maximum((ab * ab).sum(axis=1), 1e-30)
    t = np.clip(((points[:, None, :] - a[None]) * ab[None]).sum(axis=2) / denom, 0.0, 1.0)
    closest = a[None] + t[..., None] * ab[None]
    return np.linalg.norm(points[:, None, :] - closest, axis=2).min(axis=1)


def test_tube_rod_ids_stable_across_exports(tube_rod):
    _, (topo1, glb1), (topo2, glb2) = tube_rod
    topo1, topo2 = dict(topo1), dict(topo2)
    topo1.pop('created_at'), topo2.pop('created_at')
    assert json.dumps(topo1, indent=2) == json.dumps(topo2, indent=2)
    assert glb1 == glb2


def test_tube_tube_bore_joint_points_into_bore(tmp_path, occ):
    """Two tubes: joints at the outer circle (Z radial out) and the bore circle (Z radial in)."""
    step = _write_part(tmp_path, 'tube_tube')
    topo, data = _export(step, tmp_path / 'out')
    check_glb(topo, data)
    check_joints_consistent(topo)
    assert len(topo['joints']) == 2 and all(j['closed'] for j in topo['joints'])
    for joint in topo['joints']:
        outer = joint['length_mm'] == pytest.approx(math.pi * 26.67, abs=0.01)
        for frame in joint['preview_frames']:
            _radial_check(frame, outward=outer)


def test_unknown_component_fails(tmp_path, occ):
    from awm_geometry import cli

    step = _write_part(tmp_path, 'tube_rod')
    assert cli.main(['export-view', '--step', str(step), '--component', 'no such part',
                     '--out', str(tmp_path / 'out')]) == 1
    assert not (tmp_path / 'out' / 'topology.json').exists()


@pytest.mark.skipif(not os.environ.get('AWM_CELL_STEP'), reason='AWM_CELL_STEP not set')
def test_golden_part_view(tmp_path, occ):
    """Golden part (cell CAD component): 3 solids, the two weld joints J1 / J2 closed, schema valid."""
    name = os.environ.get('AWM_GOLDEN_COMPONENT', '564082-Assembly-3 parts')
    topo, data = _export(os.environ['AWM_CELL_STEP'], tmp_path / 'golden',
                         '--component', name, '--up-axis', '+Y')
    assert topo['source']['component'] == name and topo['source']['up_axis'] == '+Y'
    assert len(topo['solids']) == 3
    check_glb(topo, data)
    check_joints_consistent(topo)
    assert all(j['closed'] for j in topo['joints'])
    outer = [j for j in topo['joints'] if abs(j['length_mm'] - math.pi * 26.67) <= 1.0]
    assert len(outer) == 2  # J1 (nipple - solid) and J2 (solid - elbow), outer diameter
    assert {(j['solid_a'], j['solid_b']) for j in outer} != {(outer[0]['solid_a'],
                                                             outer[0]['solid_b'])}
    # The bores of nipple, solid and elbow also meet edge to edge: find_joint_edges returns
    # those circles as well (contract rule 3 numbers every result). Pinned so a change shows up.
    assert len(topo['joints']) == 4
