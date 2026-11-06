"""CP-60 job placement and cross-check (``job-placement``) on synthetic cell / job assemblies.

Cell = the synthetic job assembly imported as a cell (Robot robot, Disc + Fixture rotary_fixture, Part excluded).
Jobs = the same file, turned / moved / re-oriented copies, or a cell with an extra static block.
Run with ``$AWM_OCC_PYTHON -m pytest`` (skipped without OCC; colcon runs it through test_occ_suite_in_conda.py).
The golden test needs ``AWM_CELL_STEP``.
"""

import json
import math
import os
import shutil
from pathlib import Path

import numpy as np
import pytest

from awm_core import frames, jsonio
from awm_core.hashing import sha256_file

REPO = Path(__file__).resolve().parents[3]
TOL = 1e-6
CELL_ROLES = [{'path': 'Robot:1', 'role': 'robot'}, {'path': 'Positioner:1 / Disc:1', 'role': 'rotary_fixture'},
              {'path': 'Fixture:1', 'role': 'rotary_fixture'}, {'path': 'Part:1', 'role': 'exclude'}]
JOB_ROLES = [{'path': 'Robot:1', 'role': 'robot'}, {'path': 'Positioner:1 / Disc:1', 'role': 'rotary_fixture'},
             {'path': 'Fixture:1', 'role': 'rotary_fixture'}, {'path': 'Part:1', 'role': 'part'}]
TOP = ['Positioner:1', 'Fixture:1', 'Part:1', 'Clamps:1', 'Robot:1']


def _roles(path, step, up_axis, scope, assignments):
    doc = jsonio.make_header('test', [step], path.parent)
    doc.update({'schema_version': '2.0', 'source': {'file': step.name, 'sha256': sha256_file(step), 'up_axis': up_axis},
                'scope': scope, 'assignments': assignments})
    jsonio.write_json(path, doc, 'v2/cad_roles')


def _rim(topo, which='max_x'):
    """Edge id of a disc rim circle (r 60) of a components-mode topology."""
    from awm_geometry import testparts

    disc = {s['solid_id'] for s in topo['solids'] if s['component_path'] == 'Positioner:1 / Disc:1'}
    rims = [e['edge_id'] for e in topo['edges'] if e['solid_id'] in disc and e['curve_type'] == 'circle'
            and abs(e['length_mm'] - 2 * math.pi * testparts.JOB_DISC_R_MM) < 1e-3]
    assert len(rims) == 2
    return rims


def _pick_rim(step, rims, up_axis, out, want_x_file=None, transform=np.eye(4)):
    """Pick the rim whose centre is the transformed cell point x = want_x_file (default the first rim)."""
    from awm_geometry import cli

    for rim in rims:
        assert cli.main(['pick-circle', '--step', str(step), '--components', '--edge', rim, '--up-axis', up_axis,
                         '--out', str(out)]) == 0
        c = np.array(json.loads(out.read_text())['center_file_mm'])
        if want_x_file is None or np.linalg.norm(c - (transform @ np.append(want_x_file, 1.0))[:3]) < 1e-3:
            return
    raise AssertionError('no rim at the wanted centre')


def build_cell(repo: Path, step: Path):
    """Cell import as S10 does it: inspect, roles, axis pick (rim at x = 0), apply-roles, export-scene."""
    from awm_geometry import cli, testparts

    cad = repo / 'cell' / 'cad'
    cad.mkdir(parents=True)
    cell_step = cad / 'cell.step'
    shutil.copyfile(step, cell_step)
    assert cli.main(['inspect-assembly', '--step', str(cell_step), '--out', str(cad / 'cell_components.json')]) == 0
    view = repo / 'tmp_view'
    assert cli.main(['export-view', '--step', str(cell_step), '--components', '--up-axis', '+Z', '--out',
                     str(view)]) == 0
    rims = _rim(json.loads((view / 'topology.json').read_text()))
    centre = np.array([0.0, *testparts.JOB_AXIS_POINT_MM[1:]])
    _pick_rim(cell_step, rims, '+Z', cad / 'axis_pick.json', centre)
    _roles(cad / 'roles.json', cell_step, '+Z', 'cell', CELL_ROLES)
    assert cli.main(['apply-roles', '--step', str(cell_step), '--tree', str(cad / 'cell_components.json'),
                     '--roles', str(cad / 'roles.json'), '--axis', str(cad / 'axis_pick.json'),
                     '--out', str(cad / 'cad_map.yaml')]) == 0
    assert cli.main(['export-scene', '--step', str(cell_step), '--map', str(cad / 'cad_map.yaml'),
                     '--repo', str(repo)]) == 0
    shutil.rmtree(view)


def import_job(repo: Path, job_name: str, step: Path, up_axis='+Z', transform=np.eye(4), rim_x=0.0,
               roles=JOB_ROLES):
    """Job import as S1 / S2 do it; returns (job dir, cad_check, placement or None)."""
    from awm_geometry import cli, testparts

    job = repo / 'jobs' / job_name
    part_no, part_rev = job_name.rsplit('_', 1)
    assert cli.main(['import', '--job', str(job), '--step', str(step), '--part-no', part_no, '--part-rev', part_rev,
                     '--components']) == 0
    view = job / 'cache' / 'view'
    assert cli.main(['export-view', '--step', str(job / 'model.step'), '--job', str(job), '--components',
                     '--up-axis', up_axis, '--out', str(view)]) == 0
    rims = _rim(json.loads((view / 'topology.json').read_text()))
    _pick_rim(job / 'model.step', rims, up_axis, job / 'axis_pick.json',
              np.array([rim_x, *testparts.JOB_AXIS_POINT_MM[1:]]), transform)
    _roles(job / 'roles.json', job / 'model.step', up_axis, 'job', roles)
    assert cli.main(['job-placement', '--repo', str(repo), '--job', job_name]) == 0
    check = json.loads((job / 'cad_check.json').read_text())
    jsonio.validate(check, 'v2/cad_check')
    placement = job / 'placement.json'
    return job, check, (json.loads(placement.read_text()) if placement.is_file() else None)


def _pose_mm(p):
    return frames.make_transform(p['position_mm'], frames.quat_to_matrix(p['quat_xyzw']))


def world_point(repo: Path, placement, p_file):
    """World position of a job-file point through the cell flange (theta 0) and flange_T_part."""
    from awm_geometry import miniyaml

    fr = miniyaml.load(repo / 'cell' / 'model' / 'frames.yaml')
    w_T_f = _pose_mm(fr['world_T_positioner_axis'])
    f_T_p = _pose_mm(placement['flange_T_part'])
    return (w_T_f @ f_T_p @ np.append(np.asarray(p_file, dtype=float), 1.0))[:3]


def rot_about_axis(deg):
    from awm_geometry import testparts

    c = np.array(testparts.JOB_AXIS_POINT_MM)
    r = frames.axis_angle_to_matrix([1, 0, 0], math.radians(deg))
    return frames.make_transform(c - r @ c, r)


def _status(check, cid):
    return next(c for c in check['checks'] if c['id'] == cid)['status']


@pytest.fixture(scope='module')
def cell_repo(tmp_path_factory, job_assembly_step):
    repo = tmp_path_factory.mktemp('cellrepo')
    build_cell(repo, job_assembly_step)
    return repo


@pytest.fixture(scope='module')
def plain(cell_repo, job_assembly_step):
    return import_job(cell_repo, 'PLAIN_A', job_assembly_step)


def _p_pipe():
    from awm_geometry import testparts

    c = testparts.JOB_AXIS_POINT_MM
    return np.array([45.0, c[1] + testparts.JOB_PIPE_R_MM, c[2]])


def test_same_file_is_green(cell_repo, plain):
    job, check, placement = plain
    assert check['status'] == 'green', check
    assert abs(check['phi_deg']) < 1e-9
    assert placement['source'] == 'job_cad' and placement['axis_pick'] == 'axis_pick.json'
    jsonio.validate(placement, 'v2/placement')
    # job file = cell file: the part lands where the cell file has it (world_T_file)
    from awm_geometry import miniyaml
    fr = miniyaml.load(cell_repo / 'cell' / 'model' / 'frames.yaml')
    expect = (_pose_mm(fr['world_T_file']) @ np.append(_p_pipe(), 1.0))[:3]
    np.testing.assert_allclose(world_point(cell_repo, placement, _p_pipe()), expect, atol=TOL)


def test_job_positioner_rotated_30deg(cell_repo, plain, tmp_path, occ):
    from awm_geometry import testparts

    t = rot_about_axis(30.0)
    step = tmp_path / 'rot.step'
    testparts.write_tree_assembly(step, 'job', testparts.job_assembly_nodes(t.tolist()))
    _job, check, placement = import_job(cell_repo, 'ROT_A', step)
    assert check['status'] == 'green', check
    assert abs(abs(check['phi_deg']) - 30.0) < 1e-6
    p_rot = (t @ np.append(_p_pipe(), 1.0))[:3]
    np.testing.assert_allclose(world_point(cell_repo, placement, p_rot),
                               world_point(cell_repo, plain[2], _p_pipe()), atol=TOL)


def test_job_file_moved_and_y_up(cell_repo, plain, job_assembly_step, tmp_path):
    from awm_geometry import testparts

    t = frames.xyz_rpy_to_matrix([100.0, -50.0, 20.0], [math.radians(-90.0), 0.0, 0.0])  # file +Y = up
    step = tmp_path / 'moved.step'
    testparts.make_job_step(job_assembly_step, TOP, step, t.tolist())
    _job, check, placement = import_job(cell_repo, 'MOVED_A', step, up_axis='+Y', transform=t)
    assert check['status'] == 'green', check
    p_moved = (t @ np.append(_p_pipe(), 1.0))[:3]
    np.testing.assert_allclose(world_point(cell_repo, placement, p_moved),
                               world_point(cell_repo, plain[2], _p_pipe()), atol=TOL)


def test_other_rim_is_red_and_no_placement(cell_repo):
    """The rim at the other end of the disc (20 mm along the axis): check 1 red, placement not written."""
    from awm_geometry import testparts

    step = cell_repo / 'cell' / 'cad' / 'cell.step'
    job, check, placement = import_job(cell_repo, 'WRONG_A', step, rim_x=-20.0)
    assert _status(check, 1) == 'red' and check['status'] == 'red'
    assert placement is None and not (job / 'placement.json').exists()


def test_static_block_in_swept_volume_is_red(tmp_path_factory, job_assembly_step, occ):
    from awm_geometry import testparts

    c = testparts.JOB_AXIS_POINT_MM
    nodes = testparts.job_assembly_nodes()
    # static block inside the swept ring of the fixture holder (r 20..30 mm, x 0..30) at r 24..26
    nodes.append({'name': 'Block', 'shape': testparts.box_mm((10.0, c[1] - 1.0, c[2] + 24.0),
                                                             (15.0, c[1] + 1.0, c[2] + 26.0))})
    repo = tmp_path_factory.mktemp('blockrepo')
    step = repo / 'with_block.step'
    testparts.write_tree_assembly(step, 'job', nodes)
    build_cell(repo, step)
    _job, check, placement = import_job(repo, 'BLOCK_A', job_assembly_step)
    four = next(c for c in check['checks'] if c['id'] == 4)
    assert four['status'] == 'red' and check['status'] == 'red'
    assert any('Block:1' in h['static'] for h in four['values']['hits'])


def test_collision_mesh_keeps_d20_face_ids(plain, occ):
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
    from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
    from OCC.Core.TopAbs import TopAbs_FACE
    from OCC.Core.gp import gp_Pnt

    from awm_core import stl, units
    from awm_geometry import part, topology

    job, _check, placement = plain
    topo = json.loads((job / 'cache' / 'view' / 'topology.json').read_text())
    part_solids = {s['solid_id'] for s in topo['solids'] if s['component_path'].startswith('Part:1')}
    want = {f['face_id'] for f in topo['faces'] if f['solid_id'] in part_solids}
    faces_doc = json.loads((job / 'cache' / 'collision_faces.json').read_text())
    assert {r['face_id'] for r in faces_doc['faces']} == want
    verts, tris = stl.read_stl(job / 'cache' / 'collision.stl')
    loaded = part.load_part(job / 'model.step', components=True)
    fmap = topology.index_map(loaded.shape, TopAbs_FACE)
    for rec in faces_doc['faces']:
        tri = verts[tris[rec['first_triangle']]]
        c = units.mm_to_m(tri.mean(axis=0))
        d = BRepExtrema_DistShapeShape(BRepBuilderAPI_MakeVertex(gp_Pnt(*map(float, c))).Vertex(),
                                       fmap.FindKey(int(rec['face_id'][1:])))
        assert units.m_to_mm(d.Value()) < 0.01, rec['face_id']


def test_unassigned_appears_static_and_excluded_listed(cell_repo, job_assembly_step):
    roles = JOB_ROLES + [{'path': 'Clamps:1', 'role': 'exclude'}]
    _job, check, _placement = import_job(cell_repo, 'EXCL_A', job_assembly_step, roles=roles)
    assert 'Clamps:1' in check['excluded']
    assert 'Positioner:1 / Base:1' in check['unassigned_static']


GOLDEN_JOB = ['Part1:1', 'Part2:1', 'Part3:1', 'HTPK20S5M150-A-C15:1', 'HTPK20S5M150-A-C15:4',
              '564082-Assembly-3 parts:2']  # rotating set of the spike map (OI-27) + the golden part


def _circle_like(step, components, center, radius, tol=1e-3):
    """Edge id of the circle with this centre / radius in a view loaded like export-view (test-side search)."""
    from OCC.Core.TopAbs import TopAbs_EDGE

    from awm_geometry import circle_pick, part, topology
    from awm_geometry.occ_io import GeometryError

    loaded = part.load_part(step, components=components)
    edges = topology.index_map(loaded.shape, TopAbs_EDGE)
    for k in range(1, edges.Size() + 1):
        try:
            fr = circle_pick.circle_frame(circle_pick.edge_by_id(loaded.shape, f'E{k}'))
        except GeometryError:
            continue
        if np.linalg.norm(np.array(fr['center_mm']) - center) < tol and abs(fr['radius_mm'] - radius) < tol:
            return f'E{k}'
    raise AssertionError(f'no circle r {radius} at {center}')


@pytest.mark.skipif(not os.environ.get('AWM_CELL_STEP'), reason='AWM_CELL_STEP not set')
def test_golden_job_from_cell_cad(tmp_path, occ):
    """Cell CAD imported with roles (spike map, part excluded) + a Part1 circle; the job STEP cut from the cell CAD
    (rotating set + 564082) imported with roles and the same circle: check 1 green, J1 coaxial (r 13.33 mm)."""
    from awm_core import cad_roles
    from awm_geometry import cli, miniyaml, testparts

    cell_step = Path(os.environ['AWM_CELL_STEP'])
    repo = tmp_path / 'repo'
    cad = repo / 'cell' / 'cad'
    cad.mkdir(parents=True)
    old_map = miniyaml.load(REPO / 'docs' / 'generated' / 'spike' / 'cad_map_cell.yaml')
    assert cli.main(['inspect-assembly', '--step', str(cell_step), '--out', str(cad / 'cell_components.json'),
                     '--no-cylinders']) == 0
    tree = json.loads((cad / 'cell_components.json').read_text())['components']
    assignments = [dict(a, role='exclude') if a['role'] == 'part' else a
                   for a in cad_roles.from_cad_map(tree, old_map)]
    doc = jsonio.make_header('test', [cell_step], cad)
    doc.update({'schema_version': '2.0', 'scope': 'cell', 'assignments': assignments,
                'source': {'file': cell_step.name, 'sha256': sha256_file(cell_step), 'up_axis': '+Y'},
                'collision': old_map.get('collision', {}), 'contact_tolerance_mm': old_map['contact_tolerance_mm']})
    jsonio.write_json(cad / 'roles.json', doc, 'v2/cad_roles')
    p0 = np.array(old_map['frames']['positioner_axis']['point_file_mm'], dtype=float)
    d0 = np.array(old_map['frames']['positioner_axis']['direction_file'], dtype=float)
    # the largest circle of Part1:1 on the spike axis line (what the Admin clicks)
    from OCC.Core.TopAbs import TopAbs_EDGE
    from awm_geometry import circle_pick, part, topology
    from awm_geometry.occ_io import GeometryError
    loaded = part.load_part(cell_step, 'Part1:1')
    edges = topology.index_map(loaded.shape, TopAbs_EDGE)
    best = None
    for k in range(1, edges.Size() + 1):
        try:
            fr = circle_pick.circle_frame(circle_pick.edge_by_id(loaded.shape, f'E{k}'))
        except GeometryError:
            continue
        c, a = np.array(fr['center_mm']), np.array(fr['axis'])
        if abs(abs(a @ d0) - 1) < 1e-6 and np.linalg.norm(np.cross(c - p0, d0)) < 0.05 \
                and (best is None or fr['radius_mm'] > best[1]['radius_mm']):
            best = (f'E{k}', fr)
    assert best is not None
    del loaded
    assert cli.main(['pick-circle', '--step', str(cell_step), '--component', 'Part1:1', '--edge', best[0],
                     '--up-axis', '+Y', '--out', str(cad / 'axis_pick.json')]) == 0
    assert cli.main(['apply-roles', '--step', str(cell_step), '--tree', str(cad / 'cell_components.json'),
                     '--roles', str(cad / 'roles.json'), '--axis', str(cad / 'axis_pick.json'),
                     '--out', str(cad / 'cad_map.yaml')]) == 0
    assert cli.main(['export-scene', '--step', str(cell_step), '--map', str(cad / 'cad_map.yaml'),
                     '--repo', str(repo)]) == 0

    job_step = tmp_path / '564082_job.step'
    assert cli.main(['make-job-step', '--step', str(cell_step), '--components', *GOLDEN_JOB,
                     '--out', str(job_step)]) == 0
    job = repo / 'jobs' / '564082_A'
    assert cli.main(['import', '--job', str(job), '--step', str(job_step), '--part-no', '564082', '--part-rev', 'A',
                     '--components']) == 0
    assert cli.main(['export-view', '--step', str(job / 'model.step'), '--job', str(job), '--components',
                     '--up-axis', '+Y', '--out', str(job / 'cache' / 'view')]) == 0
    edge = _circle_like(job / 'model.step', True, np.array(best[1]['center_mm']), best[1]['radius_mm'])
    assert cli.main(['pick-circle', '--step', str(job / 'model.step'), '--components', '--edge', edge,
                     '--up-axis', '+Y', '--out', str(job / 'axis_pick.json')]) == 0
    roles = [{'path': n, 'role': 'rotary_fixture'} for n in GOLDEN_JOB[:-1]] + \
        [{'path': GOLDEN_JOB[-1], 'role': 'part'}]
    _roles(job / 'roles.json', job / 'model.step', '+Y', 'job', roles)
    assert cli.main(['job-placement', '--repo', str(repo), '--job', '564082_A']) == 0
    check = json.loads((job / 'cad_check.json').read_text())
    print(json.dumps({c['id']: (c['status'], c.get('note')) for c in check['checks']}, indent=1))
    assert _status(check, 1) == 'green', check['checks'][0]
    placement = json.loads((job / 'placement.json').read_text())
    # J1: the closed part joint of r 13.33 mm must be coaxial with the cell axis
    topo = json.loads((job / 'cache' / 'view' / 'topology.json').read_text())
    j1 = [j for j in topo['joints'] if j['closed'] and abs(j['length_mm'] - 2 * math.pi * 13.335) < 0.5]
    assert j1, 'J1 not found'
    out = tmp_path / 'j1.json'
    assert cli.main(['pick-circle', '--step', str(job / 'model.step'), '--components', '--edge', j1[0]['edge_a'],
                     '--up-axis', '+Y', '--out', str(out)]) == 0
    ring = json.loads(out.read_text())
    fr = miniyaml.load(repo / 'cell' / 'model' / 'frames.yaml')
    w_T_f = _pose_mm(fr['world_T_positioner_axis'])
    w_T_p = w_T_f @ _pose_mm(placement['flange_T_part'])
    c_w = (w_T_p @ np.append(ring['center_file_mm'], 1.0))[:3]
    a_w = w_T_p[:3, :3] @ np.array(ring['axis_file'])
    axis_p, axis_d = np.array(fr['world_T_positioner_axis']['point_mm']), np.array(fr['world_T_positioner_axis']['direction'])
    assert abs(ring['radius_mm'] - 13.335) < 0.01
    assert math.degrees(math.acos(min(1.0, abs(float(a_w @ axis_d))))) <= 0.1
    assert np.linalg.norm(np.cross(c_w - axis_p, axis_d)) <= 0.05
    if _status(check, 4) == 'red':
        # Found 2026-11-06: designed contacts of the real cell (jig bearing seats around holder Part1 / pulley :1,
        # timing-belt hulls around the pulleys, 4.5-5.1 mm deep). How check 4 treats them is Truong's decision;
        # kept visible as xfail until then (plan Task 5 step 6: no tolerance change without Truong).
        pytest.xfail('check 4 red on the golden cell (designed contacts: '
                     + next(c['note'] for c in check['checks'] if c['id'] == 4) + ') - decision pending')
