"""Head model selection (Goal 1 G1.1, CP-58): ``head_model:=hbox`` (default) | ``hcad`` (meshes + TCP from head.yaml).

The hcad cases always run on a synthetic head folder (head_fixture.py, tilted beam) and, when the environment variable
AWM_HEAD_DIR names the real export (``<repo_root>/cell/model/head``), also on the real CAD data. Expected values are
computed from the head.yaml in use, never from constants.
"""

import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml
from urdf_parser_py.urdf import URDF

from awm_core import units

sys.path.insert(0, str(Path(__file__).resolve().parent))
from head_fixture import real_head_dir, write_map_override_head_dir, write_synthetic_head_dir  # noqa: E402
from urdf_fk import fk, origin_matrix  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
URDF_XACRO = PKG / 'urdf' / 'awm_cell_v2.urdf.xacro'
HEAD_LINKS = ('head_mount', 'head_body', 'nozzle_body', 'nozzle_tip', 'conduit', 'tcp_laser')
TOL_MM = 1e-6


def _run_xacro(**args):
    cmd = ['xacro', str(URDF_XACRO)] + [f'{k}:={v}' for k, v in args.items()]
    return subprocess.run(cmd, capture_output=True, text=True, check=False)


def _urdf(**args) -> URDF:
    proc = _run_xacro(**args)
    assert proc.returncode == 0, proc.stderr
    return URDF.from_xml_string(proc.stdout)


@pytest.fixture(scope='module', params=['synthetic', 'map_override'] + (['real'] if real_head_dir() else []))
def head_dir(request, tmp_path_factory):
    if request.param == 'real':
        return real_head_dir()
    if request.param == 'map_override':
        return write_map_override_head_dir(tmp_path_factory.mktemp('head') / 'map_override')
    return write_synthetic_head_dir(tmp_path_factory.mktemp('head') / 'synthetic')


@pytest.fixture(scope='module')
def head(head_dir):
    return yaml.safe_load((Path(head_dir) / 'head.yaml').read_text())


@pytest.fixture(scope='module')
def hcad(head_dir):
    return _urdf(head_model='hcad', head_dir=str(head_dir))


def _quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _expected_tcp_mm(head, nozzle_len_mm):
    axis = np.asarray(head['beam_axis_tool0'], dtype=float)
    axis = axis / np.linalg.norm(axis)
    return np.asarray(head['beam_exit_tool0_mm'], dtype=float) + float(nozzle_len_mm) * axis


def test_default_is_hbox():
    """head_model defaults to hbox: the default URDF equals an explicit hbox URDF (old records reproducible)."""
    assert _run_xacro().stdout == _run_xacro(head_model='hbox').stdout


def test_both_models_render_same_names(hcad):
    hbox = _urdf(head_model='hbox')
    assert {link.name for link in hbox.links} == {link.name for link in hcad.links}
    assert {(j.name, j.type, j.parent, j.child) for j in hbox.joints} == \
        {(j.name, j.type, j.parent, j.child) for j in hcad.joints}
    for name in HEAD_LINKS:
        joint = hcad.joint_map[f'tool0_to_{name}']
        assert joint.parent == 'tool0' and joint.type == 'fixed'


def test_hcad_tcp_equals_head_yaml(hcad, head):
    """tcp_laser in tool0 = head.yaml tool0_T_tcp_mm (position <= 1e-6 mm, rotation 1e-9) at the file nozzle length."""
    t = fk(hcad, 'tcp_laser', root='tool0')
    xyz_mm = np.asarray(units.m_to_mm(list(t[:3, 3])))
    assert np.max(np.abs(xyz_mm - np.asarray(head['tool0_T_tcp_mm']['xyz']))) <= TOL_MM
    assert np.max(np.abs(xyz_mm - _expected_tcp_mm(head, head['nozzle_len_mm']))) <= TOL_MM
    assert np.allclose(t[:3, :3], _quat_matrix(head['tool0_T_tcp_mm']['quat_xyzw']), atol=1e-9)
    # tool Z = beam direction
    axis = np.asarray(head['beam_axis_tool0'], dtype=float)
    assert np.allclose(t[:3, 2], axis / np.linalg.norm(axis), atol=1e-9)


@pytest.mark.parametrize('delta_mm', [-15.0, 12.5, 1e-3])
def test_hcad_nozzle_len_differing_from_head_yaml_refused(head_dir, head, delta_mm):
    """Review C1: the nozzle length is part of the TCP record of head.yaml (map / CP-61 teach); an explicit
    nozzle_len_mm arg that differs from it (> 1e-6 mm) stops xacro instead of moving tcp_laser silently."""
    proc = _run_xacro(head_model='hcad', head_dir=str(head_dir), nozzle_len_mm=float(head['nozzle_len_mm']) + delta_mm)
    assert proc.returncode != 0 and 'nozzle_len_mm' in proc.stderr and 'head.yaml' in proc.stderr


def test_hcad_nozzle_len_equal_to_head_yaml_accepted(head_dir, head):
    r = _urdf(head_model='hcad', head_dir=str(head_dir), nozzle_len_mm=head['nozzle_len_mm'])
    xyz_mm = np.asarray(units.m_to_mm(list(fk(r, 'tcp_laser', root='tool0')[:3, 3])))
    assert np.max(np.abs(xyz_mm - np.asarray(head['tool0_T_tcp_mm']['xyz']))) <= TOL_MM


@pytest.mark.parametrize('method', ['map', 'teach_ui'])
def test_hcad_taught_tcp_from_head_yaml(tmp_path, method):
    """Review C1: for a map / teach_ui TCP, tcp_laser = tool0_T_tcp_mm.xyz of head.yaml even when beam_exit does
    not satisfy the rule exactly, and the nozzle cylinders are built backwards from that TCP (nozzle_len_mm of
    head.yaml, here 30 mm, no xacro arg needed)."""
    folder = write_map_override_head_dir(tmp_path / 'h', nozzle_len_mm=30.0)
    doc = yaml.safe_load((folder / 'head.yaml').read_text())
    doc['estimates']['tcp']['method'] = method
    doc['beam_exit_tool0_mm'] = [v + 2.0 for v in doc['beam_exit_tool0_mm']]   # stale / rounded exit
    (folder / 'head.yaml').write_text(yaml.safe_dump(doc))
    r = _urdf(head_model='hcad', head_dir=str(folder))
    tcp = np.asarray(doc['tool0_T_tcp_mm']['xyz'], float)
    xyz_mm = np.asarray(units.m_to_mm(list(fk(r, 'tcp_laser', root='tool0')[:3, 3])))
    assert np.max(np.abs(xyz_mm - tcp)) <= TOL_MM
    axis = np.asarray(doc['beam_axis_tool0'], float)
    axis = axis / np.linalg.norm(axis)
    b0, _b1, _r = _segment(r, 'nozzle_body')
    _t0, t1, _r = _segment(r, 'nozzle_tip')
    assert np.max(np.abs(b0 - (tcp - 30.0 * axis))) <= TOL_MM
    assert np.linalg.norm(tcp - t1) == pytest.approx(0.3, abs=TOL_MM)


def test_hbox_tcp_from_args():
    r = _urdf(head_model='hbox', mount_h_mm=55, head_h_mm=320, nozzle_len_mm=33)
    xyz_mm = np.asarray(units.m_to_mm(list(fk(r, 'tcp_laser', root='tool0')[:3, 3])))
    assert np.max(np.abs(xyz_mm - np.array([0.0, 0.0, 55.0 + 320.0 + 33.0]))) <= TOL_MM


def test_hcad_meshes_from_head_dir(hcad, head_dir, head):
    """Visual = full mesh, collision = hull, both file:// URIs into head_dir, scale mm -> m; link frame = tool0."""
    scale = [units.mm_to_m(1.0)] * 3
    for name in ('head_body', 'head_mount'):
        link = hcad.link_map[name]
        assert len(link.visuals) == 1 and len(link.collisions) == 1
        vis, col = link.visuals[0].geometry, link.collisions[0].geometry
        assert vis.filename == f"file://{head_dir}/{head['links'][name]['mesh']['file']}"
        assert col.filename == f"file://{head_dir}/{head['links'][name]['hull']['file']}"
        assert list(vis.scale) == pytest.approx(scale) and list(col.scale) == pytest.approx(scale)
        assert np.allclose(fk(hcad, name, root='tool0'), np.eye(4))
        assert np.allclose(origin_matrix(link.visuals[0].origin), np.eye(4))
        assert np.allclose(origin_matrix(link.collisions[0].origin), np.eye(4))


def _segment(robot, link):
    """Cylinder of a nozzle link in tool0 (mm): start point, end point, radius."""
    c = robot.link_map[link].collisions
    assert len(c) == 1
    t = fk(robot, link, root='tool0') @ origin_matrix(c[0].origin)
    centre = np.asarray(units.m_to_mm(list(t[:3, 3])))
    half = units.m_to_mm(c[0].geometry.length) / 2.0
    return centre - half * t[:3, 2], centre + half * t[:3, 2], units.m_to_mm(c[0].geometry.radius)


def test_hcad_nozzle_on_beam_axis(hcad, head):
    """nozzle_body from the beam exit, nozzle_tip up to s_c = 0.3 mm before tcp_laser, both on the beam axis."""
    exit_mm = np.asarray(head['beam_exit_tool0_mm'], dtype=float)
    tcp = _expected_tcp_mm(head, head['nozzle_len_mm'])
    b0, b1, rb = _segment(hcad, 'nozzle_body')
    t0, t1, rt = _segment(hcad, 'nozzle_tip')
    assert rb == pytest.approx(10.0) and rt == pytest.approx(10.0)
    assert np.max(np.abs(b0 - exit_mm)) <= TOL_MM
    assert np.max(np.abs(b1 - t0)) <= TOL_MM
    assert np.linalg.norm(tcp - t0) == pytest.approx(5.0, abs=TOL_MM)
    assert np.linalg.norm(tcp - t1) == pytest.approx(0.3, abs=TOL_MM)


def _cable_exit(head):
    """Expected cable exit (point mm, unit outward axis) in tool0 from head.yaml, computed independently of xacro:
    explicit cable_exit_tool0_mm / cable_axis_tool0, else the refused rear estimate of export-head (connector end =
    estimate TCP - nozzle_len_mm * estimate axis)."""
    if head.get('cable_exit_tool0_mm') is not None:
        p, a = np.asarray(head['cable_exit_tool0_mm'], float), np.asarray(head['cable_axis_tool0'], float)
    else:
        g = head['estimates']['tcp']['geometry_estimate']
        a = np.asarray(g['beam_axis_tool0'], float)
        a = a / np.linalg.norm(a)
        p = np.asarray(g['tcp_tool0_mm'], float) - float(head['nozzle_len_mm']) * a
    return p, a / np.linalg.norm(a)


def _seg_dist(p0, p1, q0, q1):
    """Smallest distance between segments p0-p1 and q0-q1 (dense sampling, test only; 0.05 mm steps)."""
    n = 2000
    s = np.linspace(0.0, 1.0, n)[:, None]
    a = p0 + s * (p1 - p0)
    b = q0 + s * (q1 - q0)
    return float(np.min(np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)))


def test_hcad_conduit_at_cable_exit(hcad, head):
    """G1.8: the conduit capsule starts at the head cable exit (head.yaml data) and runs outward along the cable
    axis; its dimensions stay the parametric Assumption (Ø30 x 300 mm, TST-002 §3.1)."""
    exit_mm, axis = _cable_exit(head)
    col = hcad.link_map['conduit'].collisions
    assert [type(c.geometry).__name__ for c in col] == ['Cylinder', 'Sphere', 'Sphere']
    t = fk(hcad, 'conduit', root='tool0')
    assert np.max(np.abs(np.asarray(units.m_to_mm(list(t[:3, 3]))) - exit_mm)) <= TOL_MM
    assert np.allclose(t[:3, 2], axis, atol=1e-9)
    radius = units.m_to_mm(col[1].geometry.radius)
    assert radius == pytest.approx(15.0) and units.m_to_mm(col[0].geometry.length) == pytest.approx(270.0)


def test_hcad_conduit_clear_of_nozzle(hcad, head):
    """The capsule (axis segment between its sphere centres, radius r) does not overlap the nozzle cylinders."""
    t = fk(hcad, 'conduit', root='tool0')
    col = hcad.link_map['conduit'].collisions
    centres = [np.asarray(units.m_to_mm(list((t @ origin_matrix(c.origin))[:3, 3]))) for c in col[1:]]
    r_c = units.m_to_mm(col[1].geometry.radius)
    for link in ('nozzle_body', 'nozzle_tip'):
        n0, n1, r_n = _segment(hcad, link)
        assert _seg_dist(centres[0], centres[1], n0, n1) > r_c + r_n, link


def test_hcad_without_cable_exit_fails(tmp_path):
    """No cable exit in head.yaml (no explicit keys, no rear estimate): xacro stops with a message, no default."""
    folder = write_synthetic_head_dir(tmp_path / 'h')
    doc = yaml.safe_load((folder / 'head.yaml').read_text())
    doc.pop('cable_exit_tool0_mm')
    doc.pop('cable_axis_tool0')
    (folder / 'head.yaml').write_text(yaml.safe_dump(doc))
    proc = _run_xacro(head_model='hcad', head_dir=str(folder))
    assert proc.returncode != 0 and 'cable exit' in proc.stderr and 'export-head' in proc.stderr


def test_hcad_without_head_dir_fails():
    proc = _run_xacro(head_model='hcad')
    assert proc.returncode != 0 and 'head_dir' in proc.stderr


def test_hcad_missing_head_yaml_fails(tmp_path):
    proc = _run_xacro(head_model='hcad', head_dir=str(tmp_path))
    assert proc.returncode != 0 and 'head.yaml' in proc.stderr


def test_hcad_relative_head_dir_fails():
    proc = _run_xacro(head_model='hcad', head_dir='cell/model/head')
    assert proc.returncode != 0 and 'absolute' in proc.stderr


def test_unknown_head_model_fails():
    proc = _run_xacro(head_model='hcube')
    assert proc.returncode != 0 and 'head_model' in proc.stderr


def test_map_override_tcp_in_front_of_flange(tmp_path):
    """Map TCP override (export-head, bug of 2026-11-06): tool0 -> tcp_laser equals head.yaml and lies in front of
    the flange; the nozzle cylinders end at the TCP (beam exit = tcp - nozzle_len_mm * beam)."""
    folder = write_map_override_head_dir(tmp_path / 'h')
    head = yaml.safe_load((folder / 'head.yaml').read_text())
    r = _urdf(head_model='hcad', head_dir=str(folder))
    t = fk(r, 'tcp_laser', root='tool0')
    xyz_mm = np.asarray(units.m_to_mm(list(t[:3, 3])))
    assert np.max(np.abs(xyz_mm - np.asarray(head['tool0_T_tcp_mm']['xyz']))) <= TOL_MM
    assert xyz_mm[2] > 0.0
    assert np.allclose(t[:3, 2], head['beam_axis_tool0'], atol=1e-9)
    _b0, _b1, _r = _segment(r, 'nozzle_body')
    assert np.max(np.abs(_b0 - np.asarray(head['beam_exit_tool0_mm']))) <= TOL_MM


def test_hcad_null_beam_exit_fails(tmp_path):
    """A head.yaml without a beam exit (old export-head map override) stops xacro with a message."""
    folder = write_map_override_head_dir(tmp_path / 'h')
    doc = yaml.safe_load((folder / 'head.yaml').read_text())
    doc['beam_exit_tool0_mm'] = None
    (folder / 'head.yaml').write_text(yaml.safe_dump(doc))
    proc = _run_xacro(head_model='hcad', head_dir=str(folder))
    assert proc.returncode != 0 and 'beam_exit_tool0_mm' in proc.stderr and 'export-head' in proc.stderr


def test_hcad_wrong_unit_fails(tmp_path):
    folder = write_synthetic_head_dir(tmp_path / 'h')
    doc = yaml.safe_load((folder / 'head.yaml').read_text())
    doc['unit'] = 'm'
    (folder / 'head.yaml').write_text(yaml.safe_dump(doc))
    proc = _run_xacro(head_model='hcad', head_dir=str(folder))
    assert proc.returncode != 0 and 'unit' in proc.stderr
