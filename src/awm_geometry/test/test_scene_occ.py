"""Cell collision scene export (``export-scene``, WP-S S.4 A3; DES-001 §5.3 / §5.4, ADR-16).

Run with ``$AWM_OCC_PYTHON -m pytest``; skipped in an interpreter without OCC (colcon runs it through
test_occ_suite_in_conda.py). The cell CAD test runs only when ``AWM_CELL_STEP`` names the cell CAD; its role map
is ``AWM_CELL_MAP`` or ``docs/generated/spike/cad_map_cell.yaml`` of the repository.
"""

import json
import os
from pathlib import Path

import numpy as np
import pytest


from awm_core import frames, hashing, jsonio, stl, units
from awm_geometry import miniyaml

TOL_MM = 1e-3  # float32 STL coordinates around 100 mm
RX90 = frames.axis_angle_to_matrix([1, 0, 0], units.deg_to_rad(90.0))  # file +Y up -> world +Z up

# Synthetic cell, file frame Y up (mm). The weldment's local Z (tube / rod axis) lies along file +X.
PART_T = [[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 100.0], [0.0, 1.0, 0.0, 50.0], [0.0, 0.0, 0.0, 1.0]]
ROBOT_T = [[1.0, 0.0, 0.0, -200.0], [0.0, 1.0, 0.0, 20.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
AXIS_POINT, AXIS_DIR = [0.0, 100.0, 50.0], [1.0, 0.0, 0.0]
HOLDER = ((-0.060, 0.090, 0.040), (-0.040, 0.110, 0.060))   # metres (testparts unit), on the axis
TABLE = ((-0.300, -0.010, -0.100), (0.100, 0.000, 0.100))   # plate under everything


def _cell(tmp_path):
    from awm_geometry import testparts

    comps = testparts.weldment_assembly()[:1] + [
        ('robot', [testparts.box((0.0, 0.0, 0.0), (0.020, 0.030, 0.020))]),
        ('holder', [testparts.box(*HOLDER)]),
        ('table', [testparts.box(*TABLE)]),
        ('tool', [testparts.box((0.0, 0.200, 0.0), (0.010, 0.210, 0.010))]),
    ]
    step = tmp_path / 'cell.step'
    testparts.write_assembly(step, comps, [PART_T, ROBOT_T, None, None, None], instance_names=True)
    return step


def _map(step, **override):
    cmap = {
        'schema': 'cad_map/0.1',
        'source': {'file': step.name, 'sha256': hashing.sha256_file(step), 'unit': 'mm', 'up_axis': '+Y'},
        'match': 'instance_name', 'default': 'deny',
        'frames': {'world': {'origin_from': 'ur_base'},
                   'ur_base': {'component': 'robot:1', 'use': 'placement'},
                   'positioner_axis': {'point_file_mm': AXIS_POINT, 'direction_file': AXIS_DIR}},
        'roles': {'part': ['weldment:1'], 'positioner_rotating': ['holder:1'],
                  'cell_static': ['table:1'], 'head_cad': [{'path': 'tool:1', 'depth': 1}],
                  'ignore': ['robot:1']},
    }
    cmap['roles'].update(override)
    return cmap


def _run(tmp_path, cmap, step=None):
    from awm_geometry import cli

    step = step or _cell(tmp_path)
    map_path = tmp_path / 'map.yaml'
    map_path.write_text(miniyaml.dumps(cmap(step) if callable(cmap) else cmap))
    repo = tmp_path / 'repo'
    code = cli.main(['export-scene', '--step', str(step), '--map', str(map_path), '--repo', str(repo),
                     '--job', 'TEST_A', '--chord-near-mm', '0.1'])
    return code, repo


@pytest.fixture(scope='module')
def scene(tmp_path_factory, occ):
    tmp = tmp_path_factory.mktemp('scene')
    code, repo = _run(tmp, _map)
    assert code == 0
    return repo


def _pose(p):
    return frames.make_transform(p['position_mm'], frames.quat_to_matrix(p['quat_xyzw']))


def test_frames_world_base_and_axis(scene):
    doc = miniyaml.load((scene / 'cell' / 'model' / 'frames.yaml'))
    base = _pose(doc['world_T_base'])
    np.testing.assert_allclose(base, np.eye(4), atol=1e-9)
    axis = doc['world_T_positioner_axis']
    # (axis point - robot origin) = (200, 80, 50) in the file -> world (x, -z, y)
    np.testing.assert_allclose(axis['point_mm'], [200.0, -50.0, 80.0], atol=1e-9)
    np.testing.assert_allclose(axis['direction'], [1.0, 0.0, 0.0], atol=1e-12)
    t = _pose(axis)
    np.testing.assert_allclose(t[:3, 0], [1.0, 0.0, 0.0], atol=1e-12)  # x = axis
    assert t[2, 2] == pytest.approx(1.0)                                # z = world up
    np.testing.assert_allclose(_pose(doc['world_T_file'])[:3, :3], RX90, atol=1e-12)
    assert 'Assumption' in doc['world_T_base']['note'] and 'Assumption' in axis['note']


def _manifest(scene):
    return json.loads((scene / 'cell' / 'model' / 'scene_manifest.json').read_text())


def test_manifest_objects_roles_parents(scene):
    man = _manifest(scene)
    objs = {o['object_id']: o for o in man['objects']}
    assert set(objs) == {'holder_1', 'table_1'}
    assert objs['holder_1']['role'] == 'positioner_rotating'
    assert objs['holder_1']['parent_frame'] == 'positioner_flange'
    assert objs['table_1']['role'] == 'cell_static' and objs['table_1']['parent_frame'] == 'world'
    for o in objs.values():
        assert o['padding_mm'] == 3.0
        mesh = scene / 'cell' / 'model' / o['mesh']['file']
        assert hashing.sha256_file(mesh) == o['mesh']['sha256']
    canon = json.dumps(man['objects'], sort_keys=True, separators=(',', ':')).encode()
    assert man['scene_sha256'] == hashing.sha256_bytes(canon)


def _corners(lo, hi):
    lo, hi = units.m_to_mm(np.array(lo)), units.m_to_mm(np.array(hi))
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def _in_parent(scene, obj):
    verts, tris = stl.read_stl(scene / 'cell' / 'model' / obj['mesh']['file'])
    t = _pose(obj['pose_mm'])
    return verts @ t[:3, :3].T + t[:3, 3], tris


def test_hulls_contain_component_vertices(scene):
    """The hull of a box is the box: its 8 corners, mapped into the parent frame, are the hull vertices."""
    from awm_geometry.hull import contains

    doc = miniyaml.load((scene / 'cell' / 'model' / 'frames.yaml'))
    world_T_file = _pose(doc['world_T_file'])
    flange_T_file = frames.invert(_pose(doc['world_T_positioner_axis'])) @ world_T_file
    objs = {o['object_id']: o for o in _manifest(scene)['objects']}
    for oid, box, parent_T_file in (('table_1', TABLE, world_T_file), ('holder_1', HOLDER, flange_T_file)):
        verts, tris = _in_parent(scene, objs[oid])
        corners = _corners(*box) @ parent_T_file[:3, :3].T + parent_T_file[:3, 3]
        assert len(verts) == 8 and len(tris) == 12
        assert contains(verts, tris, corners, TOL_MM).all()
        assert np.abs(np.sort(verts, axis=0) - np.sort(corners, axis=0)).max() < TOL_MM


def test_holder_on_flange_frame(scene):
    """Holder centre (-50, 100, 50) mm in the file: on the axis, 50 mm before the axis point along x."""
    holder = {o['object_id']: o for o in _manifest(scene)['objects']}['holder_1']
    verts, _ = _in_parent(scene, holder)
    np.testing.assert_allclose(verts.mean(axis=0), [-50.0, 0.0, 0.0], atol=TOL_MM)


def test_placement_valid_and_part_on_axis(scene):
    job = scene / 'jobs' / 'TEST_A'
    doc = jsonio.read_json(job / 'placement.json', 'v2/placement', base_dir=job)
    assert doc['source'] == 'fixture_cad' and doc['verified'] is False
    (part,) = doc['objects']
    assert part['role'] == 'part' and part['parent_frame'] == 'positioner_flange'
    assert part['mesh'] == {'file': 'cache/collision.stl', 'face_ids': True,
                            'sha256': hashing.sha256_file(job / 'cache' / 'collision.stl')}
    flange_T_part = _pose(doc['flange_T_part'])
    np.testing.assert_allclose(_pose(part['pose_mm']), flange_T_part, atol=1e-12)
    # part frame = file frame: the axis point maps to the flange origin, the axis to flange x
    np.testing.assert_allclose(flange_T_part @ [*AXIS_POINT, 1.0], [0, 0, 0, 1], atol=1e-9)
    np.testing.assert_allclose(flange_T_part[:3, :3] @ AXIS_DIR, [1, 0, 0], atol=1e-12)
    # tube + rod radius 13.335 mm around the axis: every part vertex within that radius in the flange frame
    verts, _ = stl.read_stl(job / 'cache' / 'collision.stl')
    flange = verts @ flange_T_part[:3, :3].T + flange_T_part[:3, 3]
    assert np.hypot(flange[:, 1], flange[:, 2]).max() <= 13.335 + TOL_MM


def test_collision_faces_cover_all_triangles_with_d20_ids(scene):
    job = scene / 'jobs' / 'TEST_A'
    faces = json.loads((job / 'cache' / 'collision_faces.json').read_text())
    _, tris = stl.read_stl(job / 'cache' / 'collision.stl')
    first = 0
    for f in faces['faces']:
        assert f['first_triangle'] == first
        first += f['triangle_count']
    assert first == len(tris) == faces['triangles']
    topo = json.loads((job / 'cache' / 'view' / 'topology.json').read_text())
    edges = json.loads((job / 'cache' / 'edges.json').read_text())
    assert [f['face_id'] for f in faces['faces']] == [f['face_id'] for f in topo['faces']]
    # D20 lists the non-degenerate edges; their ids and fingerprints equal the edge catalogue's
    catalogue = {e['id']: e['fingerprint'] for e in edges['edges']}
    assert all(catalogue[e['edge_id']] == e['fingerprint'] for e in topo['edges'])
    assert all(f['triangle_count'] > 0 for f in faces['faces'])


def test_chord_near_joint_and_coarser_elsewhere(scene):
    job = scene / 'jobs' / 'TEST_A'
    faces = json.loads((job / 'cache' / 'collision_faces.json').read_text())
    topo = json.loads((job / 'cache' / 'view' / 'topology.json').read_text())
    joint_faces = {fid for e in topo['edges'] if e['joint_edge_id'] for fid in e['faces']}
    near = {f['face_id'] for f in faces['faces'] if f['near_joint']}
    assert near == joint_faces and near
    assert faces['chord_near_mm'] == 0.1 and faces['chord_far_mm'] > 0.1
    for f in faces['faces']:
        limit = faces['chord_near_mm'] if f['near_joint'] else faces['chord_far_mm']
        assert f['deflection_mm'] <= limit + 1e-9


def test_unmapped_component_is_an_error(tmp_path, occ, caplog):
    code, repo = _run(tmp_path, lambda s: _map(s, cell_static=[]))
    assert code == 1
    text = ' '.join(r.getMessage() for r in caplog.records)
    assert 'not mapped' in text and 'table:1' in text
    assert not (repo / 'cell' / 'model' / 'scene_manifest.json').exists()


def test_ambiguous_and_stale_map_entries_fail(tmp_path, occ, caplog):
    code, _ = _run(tmp_path, lambda s: _map(s, head_cad=['nothing:1']))
    assert code == 1 and 'matches no component' in caplog.text
    step = _cell(tmp_path)
    bad = _map(step)
    bad['source']['sha256'] = '0' * 64
    code, _ = _run(tmp_path, bad, step)
    assert code == 1 and 'SHA-256' in caplog.text


def test_object_ids_are_sanitised():
    from awm_geometry.scene import object_id

    assert object_id('HTPK20S5M150-A-C15:1') == 'htpk20s5m150_a_c15_1'
    assert object_id('2500001:1') == 'c_2500001_1'
    assert object_id('Rotary Shaft:1') == 'rotary_shaft_1'


def _cell_map_path():
    env = os.environ.get('AWM_CELL_MAP')
    if env:
        return Path(env)
    return Path(__file__).resolve().parents[3] / 'docs' / 'generated' / 'spike' / 'cad_map_cell.yaml'


@pytest.mark.skipif(not os.environ.get('AWM_CELL_STEP'), reason='AWM_CELL_STEP not set')
def test_cell_cad_scene(tmp_path, occ):
    """Real run: every component mapped, golden part on the positioner axis, placement valid."""
    from awm_geometry import cli

    map_path = _cell_map_path()
    assert map_path.is_file(), f'role map not found: {map_path} (set AWM_CELL_MAP)'
    repo = tmp_path / 'repo'
    assert cli.main(['export-scene', '--step', os.environ['AWM_CELL_STEP'], '--map', str(map_path),
                     '--repo', str(repo), '--job', '564082_A']) == 0
    man = _manifest(repo)
    roles = {o['role'] for o in man['objects']}
    assert roles == {'cell_static', 'positioner_rotating'}
    job = repo / 'jobs' / '564082_A'
    doc = jsonio.read_json(job / 'placement.json', 'v2/placement', base_dir=job)
    verts, _ = stl.read_stl(job / 'cache' / 'collision.stl')
    t = _pose(doc['flange_T_part'])
    flange = verts @ t[:3, :3].T + t[:3, 3]
    # nipple and solid are coaxial with the axis (outer radius 13.335 mm), the elbow reaches further out
    assert np.hypot(flange[:, 1], flange[:, 2]).min() < 13.335
    topo = json.loads((job / 'cache' / 'view' / 'topology.json').read_text())
    assert len(topo['solids']) == 3 and len(topo['joints']) == 4
