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


def test_export_cell_view_needs_up_axis(tmp_path, occ):
    from awm_geometry import cli

    with pytest.raises(SystemExit):
        cli.build_parser().parse_args(['export-cell-view', '--step', 'a', '--components', 'b', '--out', 'c'])


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


def _tree_snapshot(root):
    return {str(p.relative_to(root)): hashing.sha256_file(p) for p in sorted(root.rglob('*')) if p.is_file()}


def test_failing_model_check_writes_nothing(tmp_path, occ, caplog):
    """A part without a solid fails the model check: the previous export stays as it was (S.4 A re-review F2)."""
    step = _cell(tmp_path)
    code, repo = _run(tmp_path, _map, step=step)
    assert code == 0
    before = _tree_snapshot(repo)
    bad = lambda s: _map(s, part=['sheet:1'], ignore=['robot:1', 'dup:1', 'dup:2', 'weldment:1'])  # noqa: E731
    code, _ = _run(tmp_path, bad, step=step)
    assert code == 1 and 'has_solid' in caplog.text
    assert _tree_snapshot(repo) == before


def test_failing_view_export_writes_nothing(tmp_path, occ, caplog, monkeypatch):
    from awm_geometry import view
    from awm_geometry.occ_io import GeometryError

    step = _cell(tmp_path)
    code, repo = _run(tmp_path, _map, step=step)
    assert code == 0
    before = _tree_snapshot(repo)

    def broken(*_a, **_k):
        raise GeometryError('edge E1 belongs to two joints (test)')
    monkeypatch.setattr(view, 'export', broken)
    code, _ = _run(tmp_path, _map, step=step)
    assert code == 1 and 'two joints' in caplog.text
    assert _tree_snapshot(repo) == before
    assert not any(p.name.startswith('.staging') for p in repo.iterdir())


def test_inside_out_shell_is_refused(occ):
    from awm_geometry import scene
    from awm_geometry.hull import convex_hull
    from awm_geometry.occ_io import GeometryError

    verts, tris = convex_hull(np.array([[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)], float))
    v, t = scene.closed_shell(verts, tris, 'box:1', 1)
    assert len(t) == 12
    with pytest.raises(GeometryError, match=r"inside-out.*'box:1'|'box:1'.*inside-out"):
        scene.closed_shell(verts, tris[:, [0, 2, 1]], 'box:1', 1)


def _drop_first_face(monkeypatch, times=None):
    """Make the first face seen return no triangulation (always, or only ``times`` times)."""
    from awm_geometry import scene

    original, state = scene._face_triangles, {'face': None, 'dropped': 0}

    def patched(face):
        if state['face'] is None:
            state['face'] = face
        if face.IsSame(state['face']) and (times is None or state['dropped'] < times):
            state['dropped'] += 1
            return None, None, 0.0
        return original(face)
    monkeypatch.setattr(scene, '_face_triangles', patched)
    return state


def test_cell_face_retessellated_once(occ, monkeypatch, caplog):
    """A face BRepMesh leaves empty is meshed again alone (coarser angle, same chord) and logged."""
    from awm_geometry import scene, testparts
    from awm_geometry.occ_io import scale_shape

    shape = scale_shape(testparts.box((0, 0, 0), (0.01, 0.01, 0.01)), units.m_to_mm(1.0))
    state = _drop_first_face(monkeypatch, times=1)
    pieces = scene._tessellate_mm(shape, 0.5, 15.0, 'table:1')
    assert state['dropped'] == 1 and sum(len(t) for _, t in pieces) == 12
    assert "'table:1': face 1 meshed again" in caplog.text


def test_retry_rejected_when_chord_does_not_hold(occ, monkeypatch):
    """Every retry meshes the face but with a chord above the requested one: still an error (N4)."""
    from awm_geometry import scene, testparts
    from awm_geometry.occ_io import GeometryError, scale_shape

    original, state = scene._face_triangles, {'face': None, 'calls': 0}

    def patched(face):
        if state['face'] is None:
            state['face'] = face
        if face.IsSame(state['face']):
            state['calls'] += 1
            if state['calls'] == 1:
                return None, None, 0.0
            pos, idx, _ = original(face)
            return pos, idx, 99.0  # meshed, but far too coarse
        return original(face)
    monkeypatch.setattr(scene, '_face_triangles', patched)
    shape = scale_shape(testparts.box((0, 0, 0), (0.01, 0.01, 0.01)), units.m_to_mm(1.0))
    with pytest.raises(GeometryError, match=r"'table:1': face 1 has no triangulation"):
        scene._tessellate_mm(shape, 0.5, 15.0, 'table:1')
    assert state['calls'] == 1 + len(scene.RETRY_ANGULAR_DEG)


def _degenerate_retry(monkeypatch):
    """First face: unmeshed, then every retry gives nodes with one zero-area triangle (real case: pulley face 230)."""
    from awm_geometry import scene

    original, state = scene._face_triangles, {'face': None}

    def patched(face):
        if state['face'] is None:
            state['face'] = face
            return None, None, 0.0
        if face.IsSame(state['face']):
            pos, _, _ = original(face)
            return pos, np.array([[0, 0, 1]], dtype=np.int64), 0.0
        return original(face)
    monkeypatch.setattr(scene, '_face_triangles', patched)


def test_degenerate_retry_is_nodes_only_in_hull_mode(occ, monkeypatch):
    """M1: a zero-area retry mesh never passes as 'chord holds'; hull mode keeps the face nodes, recorded nodes_only."""
    from awm_geometry import scene, testparts
    from awm_geometry.occ_io import scale_shape

    _degenerate_retry(monkeypatch)
    shape = scale_shape(testparts.box((0, 0, 0), (0.01, 0.01, 0.01)), units.m_to_mm(1.0))
    retried = []
    pieces = scene._tessellate_mm(shape, 0.5, 15.0, 'table:1', retried, hull_mode=True)
    assert retried == [{'face': 1, 'angular_deflection_deg': scene.RETRY_ANGULAR_DEG[-1],
                        'achieved_chord_mm': None, 'nodes_only': True}]
    assert sum(len(t) for _, t in pieces) == 10  # the face adds nodes, no triangles


def test_degenerate_retry_is_an_error_in_mesh_mode(occ, monkeypatch):
    """M1: mesh mode never accepts a face without a covering triangulation."""
    from awm_geometry import scene, testparts
    from awm_geometry.occ_io import GeometryError, scale_shape

    _degenerate_retry(monkeypatch)
    shape = scale_shape(testparts.box((0, 0, 0), (0.01, 0.01, 0.01)), units.m_to_mm(1.0))
    with pytest.raises(GeometryError, match=r"'table:1': face 1 has no triangulation"):
        scene._tessellate_mm(shape, 0.5, 15.0, 'table:1')


def test_retried_face_is_recorded_in_manifest(tmp_path, occ, monkeypatch):
    """N1: a retried face appears in the object's retried_faces and in the CLI summary."""
    import io
    import contextlib

    _drop_first_face(monkeypatch, times=1)
    step = _cell(tmp_path)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code, repo = _run(tmp_path, _map, step=step)
    assert code == 0
    man = jsonio.read_json(repo / 'cell' / 'model' / 'scene_manifest.json', 'v2/cell_scene',
                           base_dir=repo / 'cell' / 'model')
    retried = [(o['object_id'], f) for o in man['objects'] for f in o.get('retried_faces', [])]
    assert len(retried) == 1
    _, entry = retried[0]
    assert entry['face'] == 1 and entry['angular_deflection_deg'] == scene_retry_first()
    assert entry['achieved_chord_mm'] <= man['hull_tessellation']['linear_deflection_mm']


def scene_retry_first():
    from awm_geometry import scene
    return scene.RETRY_ANGULAR_DEG[0]


def test_orphaned_staging_folders_are_removed(tmp_path, occ):
    """N2: staging folders of dead runs are removed at the start; one of a live process is kept."""
    step = _cell(tmp_path)
    repo = tmp_path / 'repo'
    dead = repo / '.staging-export-scene-999999999'
    alive = repo / f'.staging-export-scene-{os.getppid()}'
    for d in (dead, alive):
        (d / 'cell').mkdir(parents=True)
    code, _ = _run(tmp_path, _map, step=step)
    assert code == 0
    assert not dead.exists() and alive.exists()


def test_placement_does_not_hash_the_cell_manifest(scene):
    """M2 / D10: the cell manifest file is not a placement input (another job's export must not make this job
    stale); the content-based cell <-> job link comes with T0.2."""
    job = scene / 'jobs' / 'TEST_A'
    doc = jsonio.read_json(job / 'placement.json', 'v2/placement', base_dir=job)
    assert not any('scene_manifest' in i['name'] for i in doc['inputs'])


def test_failed_swap_restores_previous_outputs(tmp_path, occ, monkeypatch, caplog):
    """N3b: if moving the staged job into place fails, cell/model and the job are as before; no leftovers."""
    from awm_geometry import scene

    step = _cell(tmp_path)
    code, repo = _run(tmp_path, _map, step=step)
    assert code == 0
    (repo / 'jobs' / 'TEST_A' / 'seams.json').write_text('{}')  # a file of another module stays
    before = _tree_snapshot(repo)
    real_replace = os.replace

    def failing(src, dst, *a, **k):
        if Path(dst) == repo / 'jobs' / 'TEST_A' and '.staging' in str(src):
            raise OSError('disk full (test)')
        return real_replace(src, dst, *a, **k)
    monkeypatch.setattr(scene.os, 'replace', failing)
    code, _ = _run(tmp_path, _map, step=step)
    assert code == 1 and 'disk full' in caplog.text
    assert _tree_snapshot(repo) == before
    assert sorted(p.name for p in repo.iterdir()) == ['cell', 'jobs']
    assert sorted(p.name for p in (repo / 'cell').iterdir()) == ['model']
    assert sorted(p.name for p in (repo / 'jobs').iterdir()) == ['TEST_A']


def test_foreign_job_files_survive_a_new_export(tmp_path, occ):
    step = _cell(tmp_path)
    code, repo = _run(tmp_path, _map, step=step)
    seams = repo / 'jobs' / 'TEST_A' / 'seams.json'
    seams.write_text('{"keep": 1}')
    code, repo = _run(tmp_path, _map, step=step)
    assert code == 0 and seams.read_text() == '{"keep": 1}'


def test_untessellated_cell_face_is_an_error(occ, monkeypatch):
    from awm_geometry import scene, testparts
    from awm_geometry.occ_io import GeometryError, scale_shape

    shape = scale_shape(testparts.box((0, 0, 0), (0.01, 0.01, 0.01)), units.m_to_mm(1.0))
    _drop_first_face(monkeypatch)
    with pytest.raises(GeometryError, match=r"'table:1'.*face 1"):
        scene._tessellate_mm(shape, 0.5, 15.0, 'table:1')


def test_empty_part_face_is_an_error(tmp_path, occ, monkeypatch):
    from awm_geometry import part, scene
    from awm_geometry.occ_io import GeometryError

    loaded = part.load_part(_cell(tmp_path), 'weldment:1')
    _drop_first_face(monkeypatch)
    with pytest.raises(GeometryError, match='part face F1 has no triangulation'):
        scene.part_collision_mesh(loaded.shape, 0.1, 0.5)


def test_headers_and_guard_record(scene):
    """Full headers on the cell outputs, manifest valid against v2/cell_scene, guard method recorded (F3, F5)."""
    model = scene / 'cell' / 'model'
    man = jsonio.read_json(model / 'scene_manifest.json', 'v2/cell_scene', base_dir=model)
    assert [Path(i['name']).name for i in man['inputs']] == ['cell.step', 'map.yaml']
    guard = man['export_guard']
    assert guard['theta_deg'] == 0.0 and guard['sample_spacing_mm'] == 1.0
    assert guard['part_chord_near_mm'] == 0.1 and guard['object_chord_mesh_mm'] == 0.5
    assert 'exceed' in guard['note']
    frames_doc = miniyaml.load(model / 'frames.yaml')
    assert frames_doc['schema_version'] == '2.0' and frames_doc['created_at']
    jsonio.check_inputs(frames_doc, model)


def test_object_ids_are_sanitised(occ):
    from awm_geometry.scene import object_id

    assert object_id('HTPK20S5M150-A-C15:1') == 'htpk20s5m150_a_c15_1'
    assert object_id('2500001:1') == 'c_2500001_1'
    assert object_id('Rotary Shaft:1') == 'rotary_shaft_1'


@pytest.mark.skipif(not os.environ.get('AWM_CELL_STEP'), reason='AWM_CELL_STEP not set')
def test_cell_cad_scene(tmp_path, occ):
    """Real run: every component mapped, collision modes as the map says, no part deeper than the contact
    tolerance, golden part on the positioner axis, placement valid."""
    from conftest import cell_map_path, golden_record

    from awm_geometry import cli

    rec = golden_record()
    map_path = cell_map_path()
    assert map_path.is_file(), f'role map not found: {map_path} (set AWM_CELL_MAP)'
    cmap = miniyaml.load(map_path)
    repo = tmp_path / 'repo'
    assert cli.main(['export-scene', '--step', os.environ['AWM_CELL_STEP'], '--map', str(map_path),
                     '--repo', str(repo), '--job', str(rec['job'])]) == 0
    man = jsonio.read_json(repo / 'cell' / 'model' / 'scene_manifest.json', 'v2/cell_scene',
                           base_dir=repo / 'cell' / 'model')
    assert {o['role'] for o in man['objects']} == {'cell_static', 'positioner_rotating'}
    by_component = {o['component']: o for o in man['objects']}
    for name, mode in (cmap.get('collision') or {}).items():
        assert by_component[name]['collision'] == mode, name          # e.g. Jig:1 -> mesh (jig_1)
    assert any(o['collision'] == 'mesh' for o in man['objects'])
    tolerance = cmap.get('contact_tolerance_mm', 1.0)
    assert all(o['part_max_depth_mm'] <= tolerance for o in man['objects'])
    assert man['export_guard']['contact_tolerance_mm'] == tolerance
    job = repo / 'jobs' / str(rec['job'])
    doc = jsonio.read_json(job / 'placement.json', 'v2/placement', base_dir=job)
    verts, _ = stl.read_stl(job / 'cache' / 'collision.stl')
    t = _pose(doc['flange_T_part'])
    flange = verts @ t[:3, :3].T + t[:3, 3]
    # nipple and solid are coaxial with the axis, the elbow reaches further out
    assert np.hypot(flange[:, 1], flange[:, 2]).min() < rec['outer_radius_mm']
    topo = json.loads((job / 'cache' / 'view' / 'topology.json').read_text())
    assert len(topo['solids']) == rec['solids'] and len(topo['joints']) == rec['joints']
