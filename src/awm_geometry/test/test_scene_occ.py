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
# U-shaped jig around the tube (part: x -30..40, y 86.665..113.335, z 36.665..63.335 mm), open towards +Y:
# its convex hull swallows the part, its mesh keeps >= 6 mm clearance.
JIG = [((-0.010, 0.070, 0.025), (0.010, 0.080, 0.075)),
       ((-0.010, 0.080, 0.025), (0.010, 0.120, 0.030)),
       ((-0.010, 0.080, 0.070), (0.010, 0.120, 0.075))]
TUBE_BOTTOM_MM = 100.0 - 13.335


def _clamp(overlap_mm):
    """Box under the tube whose top face is ``overlap_mm`` above the tube bottom line (metres)."""
    return ((-0.020, 0.075, 0.045), (-0.010, units.mm_to_m(TUBE_BOTTOM_MM + overlap_mm), 0.055))


def _cell(tmp_path, clamp_overlap_mm=None, name='cell.step'):
    from OCC.Core.TopAbs import TopAbs_FACE
    from OCC.Core.TopExp import TopExp_Explorer

    from awm_geometry import testparts

    sheet = TopExp_Explorer(testparts.box((0.3, 0.3, 0.3), (0.31, 0.31, 0.31)), TopAbs_FACE).Current()
    comps = testparts.weldment_assembly()[:1] + [
        ('robot', [testparts.box((0.0, 0.0, 0.0), (0.020, 0.030, 0.020))]),
        ('holder', [testparts.box(*HOLDER)]),
        ('table', [testparts.box(*TABLE)]),
        ('tool', [testparts.box((0.0, 0.200, 0.0), (0.010, 0.210, 0.010))]),
        ('jig', [testparts.box(*b) for b in JIG]),
        ('dup', [testparts.box((0.5, 0.5, 0.5), (0.51, 0.51, 0.51))]),
        ('dup', [testparts.box((0.6, 0.5, 0.5), (0.61, 0.51, 0.51))]),
        ('sheet', [sheet]),
    ]
    if clamp_overlap_mm is not None:
        comps.append(('clamp', [testparts.box(*_clamp(clamp_overlap_mm))]))
    step = tmp_path / name
    testparts.write_assembly(step, comps, [PART_T, ROBOT_T] + [None] * (len(comps) - 2), instance_names=True)
    return step


def _map(step, collision=None, tolerance=None, axis_dir=None, **override):
    cmap = {
        'schema': 'cad_map/0.1',
        'source': {'file': step.name, 'sha256': hashing.sha256_file(step), 'unit': 'mm', 'up_axis': '+Y'},
        'match': 'instance_name', 'default': 'deny',
        'frames': {'world': {'origin_from': 'ur_base'},
                   'ur_base': {'component': 'robot:1', 'use': 'placement'},
                   'positioner_axis': {'point_file_mm': AXIS_POINT,
                                       'direction_file': axis_dir or AXIS_DIR}},
        'roles': {'part': ['weldment:1'], 'positioner_rotating': ['holder:1'],
                  'cell_static': ['table:1', 'jig:1'], 'head_cad': [{'path': 'tool:1', 'depth': 1}],
                  'ignore': ['robot:1', 'dup:1', 'dup:2', 'sheet:1']},
        'collision': {'jig:1': 'mesh'} if collision is None else collision,
        'contact_tolerance_mm': 1.0 if tolerance is None else tolerance,
    }
    cmap['roles'].update(override)
    return cmap


def _run(tmp_path, cmap, step=None, job='TEST_A'):
    from awm_geometry import cli

    step = step or _cell(tmp_path)
    map_path = tmp_path / 'map.yaml'
    map_path.write_text(miniyaml.dumps(cmap(step) if callable(cmap) else cmap))
    repo = tmp_path / 'repo'
    code = cli.main(['export-scene', '--step', str(step), '--map', str(map_path), '--repo', str(repo),
                     '--job', job, '--chord-near-mm', '0.1'])
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
    assert set(objs) == {'holder_1', 'table_1', 'jig_1'}
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


def test_unknown_entry_and_wrong_sha_fail(tmp_path, occ, caplog):
    code, _ = _run(tmp_path, lambda s: _map(s, cell_static=['table:1', 'jig:1', 'nothing:1']))
    assert code == 1 and 'matches no component' in caplog.text
    step = _cell(tmp_path)
    bad = _map(step)
    bad['source']['sha256'] = '0' * 64
    code, _ = _run(tmp_path, bad, step)
    assert code == 1 and 'SHA-256' in caplog.text


def _fails(tmp_path, caplog, cmap, *needles, **kw):
    code, repo = _run(tmp_path, cmap, **kw)
    assert code == 1
    for needle in needles:
        assert needle in caplog.text, (needle, caplog.text[-2000:])
    assert not (repo / 'cell').exists() and not (repo / 'jobs').exists()  # nothing written


def test_ambiguous_entry_fails(tmp_path, occ, caplog):
    _fails(tmp_path, caplog, lambda s: _map(s, ignore=['robot:1', 'dup-body1:1', 'sheet:1']),
           "'dup-body1:1' matches 2 components")


def test_component_under_two_roles_fails(tmp_path, occ, caplog):
    _fails(tmp_path, caplog, lambda s: _map(s, ignore=['robot:1', 'dup:1', 'dup:2', 'sheet:1', 'table:1']),
           "'table:1' is already mapped as cell_static")


def test_child_inside_mapped_ancestor_fails(tmp_path, occ, caplog):
    _fails(tmp_path, caplog, lambda s: _map(s, ignore=['robot:1', 'dup:1', 'dup:2', 'sheet:1', 'table-body1:1']),
           'inside a component mapped as cell_static')


def test_surface_only_component_is_denied(tmp_path, occ, caplog):
    _fails(tmp_path, caplog, lambda s: _map(s, ignore=['robot:1', 'dup:1', 'dup:2']), 'not mapped', 'sheet:1')


def test_bad_axis_fails(tmp_path, occ, caplog):
    _fails(tmp_path, caplog, lambda s: _map(s, axis_dir=[2.0, 0.0, 0.0]), 'unit length')


def test_non_finite_axis_raises(occ):
    """NaN cannot be written in the YAML subset (the schema refuses it); the frame code refuses it as well."""
    from awm_geometry import scene

    tree = [{'index': 0, 'parent': None, 'instance_name': '', 'placement_mm': np.eye(4).tolist()},
            {'index': 1, 'parent': 0, 'instance_name': 'robot:1', 'placement_mm': np.eye(4).tolist()}]
    cmap = {'source': {'up_axis': '+Y'},
            'frames': {'world': {'origin_from': 'ur_base'}, 'ur_base': {'component': 'robot:1'},
                       'positioner_axis': {'point_file_mm': [0.0, 0.0, 0.0],
                                           'direction_file': [float('nan'), 0.0, 0.0]}}}
    with pytest.raises(scene.MapError, match='finite'):
        scene.cell_frames(tree, cmap)
    cmap['frames']['positioner_axis'] = {'point_file_mm': [0.0, float('inf'), 0.0], 'direction_file': AXIS_DIR}
    with pytest.raises(scene.MapError, match='finite'):
        scene.cell_frames(tree, cmap)


def test_map_schema_is_checked(tmp_path, occ, caplog):
    def bad(step):
        cmap = _map(step)
        cmap['frames']['ur_base']['extra'] = 1
        return cmap
    _fails(tmp_path, caplog, bad, 'v2/cad_map')


def test_bad_job_fails_before_writing(tmp_path, occ, caplog):
    _fails(tmp_path, caplog, _map, 'job', job='bad job')


def test_hull_around_part_is_refused(tmp_path, occ, caplog):
    """The U-jig as a convex hull swallows the tube: export refused, object, part faces and depth listed."""
    _fails(tmp_path, caplog, lambda s: _map(s, collision={'jig:1': 'hull'}), 'jig_1', 'deeper than', 'F')


def test_contact_within_tolerance_passes_beyond_fails(tmp_path, occ, caplog):
    """Clamp under the middle of the tube (no tessellation vertex there): found by the surface samples."""
    step = _cell(tmp_path, clamp_overlap_mm=0.5, name='c05.step')
    code, repo = _run(tmp_path, lambda s: _map(s, cell_static=['table:1', 'jig:1', 'clamp:1']), step=step)
    assert code == 0
    objs = {o['object_id']: o for o in _manifest(repo)['objects']}
    assert 0.3 < objs['clamp_1']['part_max_depth_mm'] <= 0.5 + TOL_MM
    assert objs['jig_1']['part_max_depth_mm'] == 0.0
    other = tmp_path / 'x'
    other.mkdir()
    step = _cell(other, clamp_overlap_mm=2.0, name='c20.step')
    _fails(other, caplog, lambda s: _map(s, cell_static=['table:1', 'jig:1', 'clamp:1']), 'clamp_1', step=step)


def test_collision_modes_in_manifest(scene):
    from awm_geometry import meshops

    objs = {o['object_id']: o for o in _manifest(scene)['objects']}
    assert objs['jig_1']['collision'] == 'mesh' and objs['table_1']['collision'] == 'hull'
    # three touching boxes = three closed shells (checked at export); the STL reader merges their shared corners,
    # so check the enclosed volume of the whole mesh here
    verts, tris = stl.read_stl(scene / 'cell' / 'model' / objs['jig_1']['mesh']['file'])
    assert meshops.volume(verts, tris) == pytest.approx(20 * 10 * 50 + 2 * 20 * 40 * 5, rel=1e-6)
    assert objs['table_1']['hull_volume_ratio'] == pytest.approx(1.0, rel=1e-6)
    assert 'hull_volume_ratio' not in objs['jig_1']


def test_stale_files_are_removed(tmp_path, occ):
    step = _cell(tmp_path)
    stale = tmp_path / 'repo' / 'cell' / 'model' / 'old_object.stl'
    stale.parent.mkdir(parents=True)
    stale.write_bytes(b'x')
    code, repo = _run(tmp_path, _map, step=step)
    assert code == 0 and not stale.exists()
    files = {p.name for p in (repo / 'cell' / 'model').iterdir()}
    assert files == {'frames.yaml', 'scene_manifest.json'} | {o['mesh']['file'] for o in _manifest(repo)['objects']}


def test_object_ids_are_sanitised(occ):
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
