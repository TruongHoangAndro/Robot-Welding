"""Head export ``export-head`` (WP-S S.5c Task 1; DES-001 §5.3 / §5.4, ADR-16, CP-56).

The Fusion head CAD contains the posed UR arm, the head mount and the LightWELD head (a surface model). The export
writes the head body and mount in the ``tool0`` frame (mm) and estimates ``tool0`` (flange contact) and
``tcp_laser`` (beam axis) from the geometry. Synthetic assemblies below have known frames; the real-CAD test runs only
when ``AWM_HEAD_STEP`` names the head CAD, or when ``AWM_CELL_STEP`` is set and the head CAD lies next to it (map: ``AWM_HEAD_MAP`` or ``docs/generated/spike/cad_map_head.yaml``,
output repository: ``AWM_HEAD_REPO`` or a temporary folder).

Run with ``$AWM_OCC_PYTHON -m pytest``; skipped in an interpreter without OCC.
"""

import os
from pathlib import Path

import numpy as np
import pytest

from awm_core import frames, hashing, stl, units
from awm_geometry import miniyaml

STL_TOL_MM = 1e-3  # float32 STL coordinates
HEAD_STEP_NAME = 'Dau_han_robot.step'  # test data: the Fusion head CAD next to the cell CAD (not in git)
# Synthetic head in tool0 coordinates (mm). Flange disc r 31.5 behind tool0 (z -10..0), pin hole r 3 at tool0 -Y
# (ur_description wrist3 mesh: pin hole and tool connector on tool0 -Y), mount box on the flange, body box, nozzle
# cylinder r 10 x 40 on the beam axis (0, -60) along +Z, a wire-like rod parallel to the beam (decoy), a thin screw
# reaching slightly farther than the nozzle (decoy, as on the real head) and a cross rod.
NOZZLE_END = np.array([0.0, -60.0, 240.0])
MOUNT_BOX = ((-20.0, -50.0, 0.0), (20.0, 20.0, 30.0))
BODY_BOX = ((-15.0, -80.0, 30.0), (15.0, -40.0, 200.0))
POSE_A = frames.make_transform([100.0, -50.0, 300.0],
                               frames.axis_angle_to_matrix([1.0, 2.0, 3.0], units.deg_to_rad(37.0)))
POSE_B = frames.make_transform([-400.0, 20.0, 15.0],
                               frames.axis_angle_to_matrix([-2.0, 0.5, 1.0], units.deg_to_rad(-121.0)))
CABLE = (8.0, (0.0, -60.0, -350.0), 300.0)  # r, base, length along +Z: behind the flange, reach about 355 mm
INNER = frames.make_transform([5.0, 7.0, -3.0], frames.axis_angle_to_matrix([0.0, 1.0, 0.0], units.deg_to_rad(20.0)))


def _cyl(r, base, h, axis=(0.0, 0.0, 1.0)):
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt

    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*base), gp_Dir(*axis)), r, h).Shape()


def _box(lo, hi):
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCC.Core.gp import gp_Pnt

    return BRepPrimAPI_MakeBox(gp_Pnt(*lo), gp_Pnt(*hi)).Shape()


def _open_box(lo, hi):
    """Compound of the 5 faces of a box without its top face (an open surface model, no solid)."""
    from OCC.Core.TopAbs import TopAbs_FACE
    from OCC.Core.TopExp import TopExp_Explorer

    from awm_geometry import testparts

    faces, exp = [], TopExp_Explorer(_box(lo, hi), TopAbs_FACE)
    while exp.More():
        faces.append(exp.Current())
        exp.Next()
    return testparts.separate(*faces[:-1]), faces  # MakeBox face order: last face = z max


def _flange():
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut

    return BRepAlgoAPI_Cut(_cyl(31.5, (0.0, 0.0, -10.0), 10.0), _cyl(3.0, (0.0, -25.0, -6.0), 6.0)).Shape()


def _head_step(path, pose, surface_body=False, cable=False):
    """Write the synthetic head CAD with the flange (and the head with it) at ``pose`` in the file.

    ``cable``: add a long cable cylinder hanging behind the flange (tool0 -Z), reaching farther from tool0 than the
    nozzle (the real head CAD: fiber / cable connector end, bug of 2026-11-06)."""
    from awm_geometry import testparts

    body = (_open_box(*BODY_BOX)[0] if surface_body else _box(*BODY_BOX))
    extra = [{'name': 'cable', 'shape': _cyl(*CABLE)}] if cable else []
    head_children = [
        {'name': 'body', 'placement_mm': frames.invert(INNER), 'children': extra + [
            {'name': 'skin', 'shape': body},
            {'name': 'nozzle', 'shape': _cyl(10.0, (0.0, -60.0, 200.0), 40.0)},
            {'name': 'wire', 'shape': _cyl(1.0, (0.0, -90.0, 40.0), 140.0)},
            {'name': 'screw', 'shape': _cyl(1.0, (8.0, -52.0, 225.0), 20.0)},  # decoy: 5 mm past the nozzle end
            {'name': 'crossrod', 'shape': _cyl(2.0, (-15.0, -60.0, 100.0), 30.0, axis=(1.0, 0.0, 0.0))},
        ]},
        {'name': 'mount', 'placement_mm': frames.invert(INNER), 'shape': _box(*MOUNT_BOX)},
    ]
    testparts.write_tree_assembly(path, 'robot', [
        {'name': 'wrist', 'placement_mm': pose, 'shape': _flange()},
        {'name': 'tool', 'placement_mm': pose @ INNER, 'children': head_children},
    ])
    return path


def _map(step, **frames_override):
    tool0 = {'robot_link': 'wrist:1', 'mount': 'mount:1', 'pin_radius_mm': 3.0, 'pin_axis': '-Y'}
    tcp = {'beam_link': 'head_body', 'nozzle_len_mm': 0.0}
    tool0.update(frames_override.pop('tool0', {}))
    tcp.update(frames_override.pop('tcp', {}))
    return {
        'schema': 'cad_map/0.1',
        'source': {'file': step.name, 'sha256': hashing.sha256_file(step), 'unit': 'mm', 'up_axis': '+Z'},
        'match': 'instance_name', 'default': 'deny',
        'roles': {'head_cad': [{'path': 'tool:1', 'depth': 1}], 'ignore': ['wrist:1']},
        'head_links': {'head_body': ['body:1'], 'head_mount': ['mount:1']},
        'frames': {'tool0': tool0, 'tcp': tcp},
    }


def _export(tmp_path, step, cmap):
    from awm_geometry import cli

    map_path = tmp_path / 'cad_map_head.yaml'
    map_path.write_text(miniyaml.dumps(cmap))
    repo = tmp_path / 'repo'
    code = cli.main(['export-head', '--step', str(step), '--map', str(map_path), '--repo', str(repo)])
    return code, repo / 'cell' / 'model' / 'head'


def _pose(p):
    return frames.make_transform(p['xyz'], frames.quat_to_matrix(p['quat_xyzw']))


@pytest.fixture(scope='module')
def head_a(tmp_path_factory, occ):
    tmp = tmp_path_factory.mktemp('head_a')
    step = _head_step(tmp / 'head.step', POSE_A)
    code, out = _export(tmp, step, _map(step))
    assert code == 0
    return out, step


def test_synthetic_head_frames(head_a):
    out, _step = head_a
    doc = miniyaml.load(out / 'head.yaml')
    file_T_tool0 = _pose(doc['file_T_tool0_mm'])
    assert np.linalg.norm(file_T_tool0[:3, 3] - POSE_A[:3, 3]) <= 0.01
    z_err = np.degrees(np.arccos(np.clip(file_T_tool0[:3, 2] @ POSE_A[:3, 2], -1, 1)))
    assert z_err <= 0.01
    assert units.rad_to_deg(frames.rotation_angle_between(file_T_tool0[:3, :3], POSE_A[:3, :3])) <= 0.01  # yaw (pin)
    tcp = _pose(doc['tool0_T_tcp_mm'])
    assert np.linalg.norm(tcp[:3, 3] - NOZZLE_END) <= 0.01
    np.testing.assert_allclose(doc['beam_axis_tool0'], [0.0, 0.0, 1.0], atol=units.deg_to_rad(0.01))
    np.testing.assert_allclose(tcp[:3, 2], doc['beam_axis_tool0'], atol=1e-9)  # tcp Z = beam direction
    est = doc['estimates']
    assert est['tool0']['method'] == 'flange_contact' and est['tcp']['method'] == 'beam_axis'
    assert est['tool0']['residuals']['contact_plane_gap_mm'] <= 0.01
    assert est['tcp']['residuals']['tip_ray_vs_faces_mm'] <= 0.01
    assert any('Assumption' in a for a in doc['assumptions'])
    # meshes are in tool0 (mm): the mount box comes back where it was modelled
    v, _t = stl.read_stl(out / 'head_mount.stl')
    np.testing.assert_allclose(v.min(axis=0), MOUNT_BOX[0], atol=STL_TOL_MM)
    np.testing.assert_allclose(v.max(axis=0), MOUNT_BOX[1], atol=STL_TOL_MM)
    for link in ('head_body', 'head_mount'):
        rec = doc['links'][link]
        for key in ('mesh', 'hull'):
            assert hashing.sha256_file(out / rec[key]['file']) == rec[key]['sha256']
        assert rec['faces_meshed'] == rec['faces_in']
    assert doc['source']['sha256'] == hashing.sha256_file(_step)


def test_meshes_independent_of_robot_pose(head_a, tmp_path):
    out_a, _ = head_a
    step = _head_step(tmp_path / 'head_b.step', POSE_B)
    code, out_b = _export(tmp_path, step, _map(step))
    assert code == 0
    from awm_geometry import hull, meshops

    for name in ('head_body.stl', 'head_mount.stl', 'head_body_hull.stl'):
        va, ta = stl.read_stl(out_a / name)
        vb, tb = stl.read_stl(out_b / name)
        if name.endswith('_hull.stl'):  # same convex body; the triangulation of coplanar facets is not unique
            assert meshops.volume(va, ta) == pytest.approx(meshops.volume(vb, tb), rel=1e-6)
            assert hull.contains(va, ta, vb, STL_TOL_MM).all() and hull.contains(vb, tb, va, STL_TOL_MM).all()
        else:
            assert len(ta) == len(tb)
        np.testing.assert_allclose(va.min(axis=0), vb.min(axis=0), atol=STL_TOL_MM)
        np.testing.assert_allclose(va.max(axis=0), vb.max(axis=0), atol=STL_TOL_MM)
    tcp_a = _pose(miniyaml.load(out_a / 'head.yaml')['tool0_T_tcp_mm'])
    tcp_b = _pose(miniyaml.load(out_b / 'head.yaml')['tool0_T_tcp_mm'])
    np.testing.assert_allclose(tcp_a, tcp_b, atol=1e-6)


def test_map_override_wins(tmp_path, occ):
    """frames.tcp override: point_file_mm = TCP (nozzle tip + stick-out), axis_file = beam out of the nozzle.

    head.yaml keeps the xacro rule tcp = beam_exit + nozzle_len_mm * beam_axis: beam_exit = tcp - len * axis, so the
    parametric collision nozzle ends at the TCP (CP-56, CP-61 draft)."""
    step = _head_step(tmp_path / 'head.step', POSE_A)
    point_tool0 = np.array([3.0, -55.0, 250.0])
    axis_tool0 = np.array([np.sin(units.deg_to_rad(60.0)), 0.0, np.cos(units.deg_to_rad(60.0))])
    point_file = (POSE_A @ np.append(point_tool0, 1.0))[:3]
    cmap = _map(step, tcp={'point_file_mm': point_file.tolist(), 'nozzle_len_mm': 40.0,
                           'axis_file': (POSE_A[:3, :3] @ axis_tool0).tolist()})
    code, out = _export(tmp_path, step, cmap)
    assert code == 0
    doc = miniyaml.load(out / 'head.yaml')
    assert doc['estimates']['tcp']['method'] == 'map'
    tcp = _pose(doc['tool0_T_tcp_mm'])
    np.testing.assert_allclose(tcp[:3, 3], point_tool0, atol=1e-6)
    np.testing.assert_allclose(tcp[:3, 2], axis_tool0, atol=1e-9)
    np.testing.assert_allclose(doc['beam_axis_tool0'], axis_tool0, atol=1e-9)
    # xacro-compatible keys: none null, and the tcp rule holds at the file nozzle length
    for key in ('beam_exit_tool0_mm', 'beam_axis_tool0', 'nozzle_len_mm', 'tool0_T_tcp_mm'):
        assert doc[key] is not None
    assert doc['nozzle_len_mm'] == 40.0
    np.testing.assert_allclose(doc['beam_exit_tool0_mm'], point_tool0 - 40.0 * axis_tool0, atol=1e-6)
    np.testing.assert_allclose(np.asarray(doc['beam_exit_tool0_mm']) + doc['nozzle_len_mm'] * axis_tool0,
                               tcp[:3, 3], atol=1e-6)
    # the geometric estimate is still recorded for comparison (nozzle end + 40 mm along tool0 +Z)
    geo = doc['estimates']['tcp']['geometry_estimate']
    geo_tcp = NOZZLE_END + [0.0, 0.0, 40.0]
    assert geo['tcp_offset_mm'] == pytest.approx(float(np.linalg.norm(point_tool0 - geo_tcp)), abs=0.01)
    np.testing.assert_allclose(geo['tcp_tool0_mm'], geo_tcp, atol=0.01)
    assert geo['axis_angle_deg'] == pytest.approx(60.0, abs=0.01)
    assert geo['behind_flange'] is False
    assert doc['estimates']['tcp']['behind_flange'] is False
    assert any('point_file_mm' in a and 'nozzle tip' in a for a in doc['assumptions'])


def test_estimate_behind_flange_refused(tmp_path, occ):
    """An automatic beam estimate whose TCP lies behind the flange plane (tool0 z < 0) is the cable end, not the
    nozzle: export-head refuses it and tells the user to set frames.tcp; nothing is written."""
    from awm_geometry import head

    step = _head_step(tmp_path / 'head.step', POSE_A, cable=True)
    map_path = tmp_path / 'cad_map_head.yaml'
    map_path.write_text(miniyaml.dumps(_map(step)))
    with pytest.raises(head.HeadError, match='behind the flange') as err:
        head.export_head(step, map_path, tmp_path / 'repo', 'test')
    assert 'frames.tcp' in str(err.value)
    assert not (tmp_path / 'repo' / 'cell' / 'model' / 'head').exists()
    code, out = _export(tmp_path, step, _map(step))
    assert code != 0 and not out.exists()


def test_override_accepted_with_cable_behind_flange(tmp_path, occ):
    """With frames.tcp set, the wrong automatic estimate (cable end) is kept as a comparison record, flagged."""
    step = _head_step(tmp_path / 'head.step', POSE_A, cable=True)
    point_tool0, axis_tool0 = NOZZLE_END, np.array([0.0, 0.0, 1.0])
    cmap = _map(step, tcp={'point_file_mm': (POSE_A @ np.append(point_tool0, 1.0))[:3].tolist(),
                           'axis_file': (POSE_A[:3, :3] @ axis_tool0).tolist(), 'nozzle_len_mm': 40.0})
    code, out = _export(tmp_path, step, cmap)
    assert code == 0
    doc = miniyaml.load(out / 'head.yaml')
    np.testing.assert_allclose(_pose(doc['tool0_T_tcp_mm'])[:3, 3], point_tool0, atol=1e-6)
    geo = doc['estimates']['tcp']['geometry_estimate']
    assert geo['behind_flange'] is True and geo['tcp_tool0_mm'][2] < 0.0
    assert geo['tcp_offset_mm'] > 500.0


def test_override_behind_flange_accepted_and_recorded(tmp_path, occ):
    """An explicit override is accepted wherever it is (the user said so), but recorded."""
    step = _head_step(tmp_path / 'head.step', POSE_A)
    point_tool0, axis_tool0 = np.array([0.0, 0.0, -50.0]), np.array([0.0, 0.0, -1.0])
    cmap = _map(step, tcp={'point_file_mm': (POSE_A @ np.append(point_tool0, 1.0))[:3].tolist(),
                           'axis_file': (POSE_A[:3, :3] @ axis_tool0).tolist()})
    code, out = _export(tmp_path, step, cmap)
    assert code == 0
    doc = miniyaml.load(out / 'head.yaml')
    np.testing.assert_allclose(_pose(doc['tool0_T_tcp_mm'])[:3, 3], point_tool0, atol=1e-6)
    assert doc['estimates']['tcp']['behind_flange'] is True
    assert any('behind the flange' in a for a in doc['assumptions'])


@pytest.mark.parametrize('point_tool0, axis_tool0', [
    ((0.0, -60.0, 240.0), (0.0, 0.0, -1.0)),   # tip in front of the flange, beam back into the head
    ((0.0, 0.0, -50.0), (0.0, 0.0, 1.0)),      # behind the flange, beam towards the flange
    ((100.0, 0.0, 10.0), (-1.0, 0.0, 0.0)),    # beside the flange, beam towards the tool0 axis
])
def test_override_beam_towards_tool0_refused(tmp_path, occ, point_tool0, axis_tool0):
    """Review I2: a map TCP whose beam points back towards tool0 (dot(beam, tcp) <= 0 in tool0) is refused with a
    clear message, nothing written; the explicit-override rule ("accepted as set") does not cover a reversed beam."""
    from awm_geometry import head

    step = _head_step(tmp_path / 'head.step', POSE_A)
    p, a = np.array(point_tool0), np.array(axis_tool0)
    cmap = _map(step, tcp={'point_file_mm': (POSE_A @ np.append(p, 1.0))[:3].tolist(),
                           'axis_file': (POSE_A[:3, :3] @ a).tolist(), 'nozzle_len_mm': 40.0})
    map_path = tmp_path / 'cad_map_head.yaml'
    map_path.write_text(miniyaml.dumps(cmap))
    with pytest.raises(head.HeadError, match='points back towards tool0') as err:
        head.export_head(step, map_path, tmp_path / 'repo', 'test')
    assert 'frames.tcp' in str(err.value)
    assert not (tmp_path / 'repo' / 'cell' / 'model' / 'head').exists()


def test_override_real_head_direction_accepted(tmp_path, occ):
    """The real LightWELD beam in tool0 (0.866, 0, 0.5) with the TCP at (43.8, 0, 279.6) mm passes the beam check."""
    step = _head_step(tmp_path / 'head.step', POSE_A)
    p, a = np.array([43.8285, 0.0, 279.606]), np.array([0.866019, 0.0, 0.500011])
    a = a / np.linalg.norm(a)
    cmap = _map(step, tcp={'point_file_mm': (POSE_A @ np.append(p, 1.0))[:3].tolist(),
                           'axis_file': (POSE_A[:3, :3] @ a).tolist(), 'nozzle_len_mm': 40.0})
    code, out = _export(tmp_path, step, cmap)
    assert code == 0
    np.testing.assert_allclose(miniyaml.load(out / 'head.yaml')['beam_axis_tool0'], a, atol=1e-9)


def test_tool0_override_and_nozzle_length(tmp_path, occ):
    step = _head_step(tmp_path / 'head.step', POSE_A)
    cmap = _map(step, tool0={'origin_file_mm': POSE_A[:3, 3].tolist(), 'z_file': POSE_A[:3, 2].tolist(),
                             'x_file': POSE_A[:3, 0].tolist()}, tcp={'nozzle_len_mm': 40.0})
    code, out = _export(tmp_path, step, cmap)
    assert code == 0
    doc = miniyaml.load(out / 'head.yaml')
    assert doc['estimates']['tool0']['method'] == 'map'
    # parametric nozzle beyond the CAD beam exit (CP-56: nozzle is cell data)
    np.testing.assert_allclose(_pose(doc['tool0_T_tcp_mm'])[:3, 3], NOZZLE_END + [0.0, 0.0, 40.0], atol=0.01)
    np.testing.assert_allclose(doc['beam_exit_tool0_mm'], NOZZLE_END, atol=0.01)
    assert doc['nozzle_len_mm'] == 40.0


def test_surface_model_all_faces(tmp_path, occ):
    from awm_geometry import head, hull

    step = _head_step(tmp_path / 'head.step', POSE_A, surface_body=True)
    code, out = _export(tmp_path, step, _map(step))
    assert code == 0
    doc = miniyaml.load(out / 'head.yaml')
    rec = doc['links']['head_body']
    # 5 open faces + nozzle, wire, screw, cross rod (3 each)
    assert rec['faces_in'] == 17 and rec['faces_meshed'] == 17
    v, t = stl.read_stl(out / 'head_body.stl')
    area = 0.5 * np.linalg.norm(np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]]), axis=1).sum()
    assert area == pytest.approx(rec['brep_area_mm2'], rel=0.01)
    hv, ht = stl.read_stl(out / 'head_body_hull.stl')
    assert hull.contains(hv, ht, v, 1e-3).all()
    # the open box's faces are in the mesh: its side x = -15 is covered over its full height
    side = v[np.abs(v[:, 0] + 15.0) < STL_TOL_MM]
    assert side[:, 2].min() == pytest.approx(30.0, abs=STL_TOL_MM)
    assert side[:, 2].max() == pytest.approx(200.0, abs=STL_TOL_MM)
    assert head.MESH_LINEAR_DEFLECTION_MM <= 0.5


def test_face_short_of_area_meshed_again(occ, monkeypatch):
    """A face whose first mesh covers less than the area fraction is meshed again alone, finer (real CAD: small
    B-spline faces at 0.96-0.99). Here the fraction is raised so that the rod's disc caps (0.997) miss it."""
    from awm_geometry import head, scene

    monkeypatch.setattr(scene, 'RETRY_MIN_AREA_FRACTION', 0.9995)
    out = head.mesh_faces(_cyl(0.5, (0.0, 0.0, 0.0), 10.0), head.MESH_LINEAR_DEFLECTION_MM,
                          head.MESH_ANGULAR_DEFLECTION_DEG, 'rod')
    assert out['faces_meshed'] == out['faces_in'] == 3
    assert len(out['retried']) >= 2 and all(r['coverage'] >= 0.9995 for r in out['retried'])
    assert all(r['linear_deflection_mm'] <= head.MESH_LINEAR_DEFLECTION_MM for r in out['retried'])


def test_default_deny_and_unknown_link(tmp_path, occ):
    step = _head_step(tmp_path / 'head.step', POSE_A)
    cmap = _map(step)
    cmap['roles']['ignore'] = []  # the robot link has geometry and is not mapped
    code, _ = _export(tmp_path, step, cmap)
    assert code != 0
    cmap = _map(step)
    cmap['head_links'] = {'head_body': ['body:1']}  # mount below the head component but in no link
    code, _ = _export(tmp_path, step, cmap)
    assert code != 0


def test_failed_estimate_writes_nothing(tmp_path, occ):
    step = _head_step(tmp_path / 'head.step', POSE_A)
    code, out = _export(tmp_path, step, _map(step, tool0={'pin_radius_mm': 4.0}))  # no such pin hole
    assert code != 0
    assert not out.exists()


def test_export_scene_keeps_head_folder(tmp_path, occ):
    """export-scene rewrites cell/model; the head folder written by export-head must survive it."""
    from test_scene_occ import _map as scene_map, _run

    repo = tmp_path / 'repo'
    head_dir = repo / 'cell' / 'model' / 'head'
    head_dir.mkdir(parents=True)
    (head_dir / 'head.yaml').write_text('marker: 1\n')
    code, repo_out = _run(tmp_path, scene_map)
    assert code == 0 and repo_out == repo
    assert (head_dir / 'head.yaml').read_text() == 'marker: 1\n'


def test_dau_han_robot(tmp_path):
    """Real head CAD (76 MB, about 1 min): frames and meshes; prints the result for the report."""
    step = os.environ.get('AWM_HEAD_STEP')
    if not step and os.environ.get('AWM_CELL_STEP'):
        step = str(Path(os.environ['AWM_CELL_STEP']).parent / HEAD_STEP_NAME)  # next to the cell CAD (docs/3D)
    if not step:
        pytest.skip('AWM_CELL_STEP not set')
    if not Path(step).is_file():
        pytest.skip(f'head CAD not found: {step} (set AWM_HEAD_STEP)')
    pytest.importorskip('OCC.Core.STEPControl')
    from awm_geometry import cli

    repo_root = Path(__file__).resolve().parents[3]
    map_path = Path(os.environ.get('AWM_HEAD_MAP') or repo_root / 'docs' / 'generated' / 'spike' / 'cad_map_head.yaml')
    repo = Path(os.environ.get('AWM_HEAD_REPO') or tmp_path / 'repo')
    assert cli.main(['export-head', '--step', step, '--map', str(map_path), '--repo', str(repo)]) == 0
    out = repo / 'cell' / 'model' / 'head'
    doc = miniyaml.load(out / 'head.yaml')
    tcp = _pose(doc['tool0_T_tcp_mm'])
    print('\ntool0_T_tcp xyz mm', np.round(tcp[:3, 3], 3).tolist(),
          'rpy deg', np.round(units.rad_to_deg(frames.matrix_to_rpy(tcp[:3, :3])), 3).tolist())
    print('beam axis tool0', doc['beam_axis_tool0'], 'beam exit', doc['beam_exit_tool0_mm'])
    print('estimates', doc['estimates'])
    print('extent', doc['extent_behind_body_mm'])
    for link, rec in doc['links'].items():
        print(link, 'mesh triangles', rec['mesh']['triangles'], 'hull triangles', rec['hull']['triangles'],
              'faces', rec['faces_in'], rec['faces_meshed'])
        assert rec['faces_meshed'] == rec['faces_in']
    assert doc['estimates']['tool0']['residuals']['contact_plane_gap_mm'] <= 0.1
    assert abs(np.linalg.norm(doc['beam_axis_tool0']) - 1.0) < 1e-9
    assert tcp[2, 3] > 0.0  # TCP in front of the flange (bug 2026-11-06: the cable end was taken as the nozzle)
    exit_mm = np.asarray(doc['beam_exit_tool0_mm'])
    np.testing.assert_allclose(exit_mm + doc['nozzle_len_mm'] * np.asarray(doc['beam_axis_tool0']), tcp[:3, 3],
                               atol=1e-6)
