"""Cell model v2 (WP-S S.4 Part B): xacro -> URDF, H-BOX head, tcp_laser, positioner, SRDF rules, units."""

import math
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from urdf_parser_py.urdf import URDF

sys.path.insert(0, str(Path(__file__).resolve().parent))
from urdf_fk import axis_angle, fk, origin_matrix  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
URDF_XACRO = PKG / 'urdf' / 'awm_cell_v2.urdf.xacro'
SRDF_XACRO = PKG / 'srdf' / 'awm_cell_v2.srdf.xacro'
HEAD_LINKS = ('head_mount', 'head_body', 'nozzle_body', 'nozzle_tip', 'conduit')
ARM_JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
              'wrist_2_joint', 'wrist_3_joint')
# Documented H-BOX offset tool0 -> tcp_laser (mm): mount 60 + head body 381 + nozzle 40 on tool0 +Z.
TOOL0_TCP_MM = (0.0, 0.0, 481.0)
MM = 1e-3  # test-side expectation only (the xacro factor itself is checked against awm_core.units)


def _xacro(path: Path, **args) -> str:
    cmd = ['xacro', str(path)] + [f'{k}:={v}' for k, v in args.items()]
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    assert proc.returncode == 0, proc.stderr
    return proc.stdout


def _urdf(**args) -> URDF:
    return URDF.from_xml_string(_xacro(URDF_XACRO, **args))


@pytest.fixture(scope='module')
def robot():
    return _urdf()


def test_units_match_awm_core():
    """units.xacro holds the only scale factors and they equal the single Python adapter (ADR-14)."""
    units = pytest.importorskip('awm_core.units')
    text = (PKG / 'urdf' / 'units.xacro').read_text()
    m_per_mm = float(re.search(r'name="m_per_mm" value="([^"]+)"', text).group(1))
    assert m_per_mm == pytest.approx(1.0 / units.MM_PER_M, rel=1e-15)
    assert 'pi / 180.0' in text and units.DEG_PER_RAD == pytest.approx(180.0 / math.pi)
    for f in ('awm_cell_v2.urdf.xacro', 'lightweld_hbox.xacro', 'positioner.xacro'):
        body = re.sub(r'<!--.*?-->', '', (PKG / 'urdf' / f).read_text(), flags=re.S)
        assert not re.search(r'\b(0\.001|1000(\.0)?|57\.29|0\.01745)\b', body), f


def test_parses_with_links_and_root(robot):
    names = {link.name for link in robot.links}
    for name in ('world', 'base_link', 'tool0', 'tcp_laser', 'positioner_axis', 'positioner_flange') + HEAD_LINKS:
        assert name in names, name
    assert robot.get_root() == 'world'
    assert 'tcp_mig' not in names and 'torch_mig' not in names


def test_seven_actuated_joints(robot):
    moving = [j.name for j in robot.joints if j.type != 'fixed']
    assert sorted(moving) == sorted(ARM_JOINTS + ('positioner_joint',))
    pj = robot.joint_map['positioner_joint']
    assert pj.type == 'revolute' and pj.parent == 'positioner_axis' and pj.child == 'positioner_flange'
    assert np.allclose(pj.axis, [1, 0, 0])
    assert pj.limit.lower == pytest.approx(-math.radians(400)) and pj.limit.upper == pytest.approx(math.radians(400))
    assert pj.limit.velocity == pytest.approx(math.radians(30))


def test_tcp_laser_offset(robot):
    """FK tool0 -> tcp_laser = documented offset (0, 0, 481) mm, tool Z = beam direction = tool0 Z."""
    t = fk(robot, 'tcp_laser', root='tool0')
    assert np.allclose(t[:3, 3], np.array(TOOL0_TCP_MM) * MM, atol=1e-12)
    assert np.allclose(t[:3, :3], np.eye(3), atol=1e-12)


def test_tcp_follows_head_args():
    r = _urdf(mount_h_mm=50, head_h_mm=300, nozzle_len_mm=30)
    assert np.allclose(fk(r, 'tcp_laser', root='tool0')[:3, 3], [0, 0, 0.380], atol=1e-12)


def _cyl(robot, link):
    col = robot.link_map[link].collisions
    assert len(col) == 1
    c = col[0]
    centre_in_tool0 = (fk(robot, link, root='tool0') @ origin_matrix(c.origin))[:3, 3]
    return c.geometry, centre_in_tool0


def test_nozzle_split_and_contact_standoff(robot):
    """nozzle_body + nozzle_tip on the beam axis; the tip collision ends s_c = 0.3 mm before tcp_laser (F9)."""
    tcp_z = fk(robot, 'tcp_laser', root='tool0')[2, 3]
    body, cb = _cyl(robot, 'nozzle_body')
    tip, ct = _cyl(robot, 'nozzle_tip')
    for g, c in ((body, cb), (tip, ct)):
        assert g.radius == pytest.approx(10 * MM)
        assert np.allclose(c[:2], 0.0, atol=1e-12)
    tip_end = ct[2] + tip.length / 2
    tip_start = ct[2] - tip.length / 2
    body_end = cb[2] + body.length / 2
    assert tcp_z - tip_end == pytest.approx(0.3 * MM, abs=1e-12)
    assert tcp_z - tip_start == pytest.approx(5.0 * MM, abs=1e-12)
    assert body_end == pytest.approx(tip_start, abs=1e-12)
    assert body.length == pytest.approx(35 * MM)
    assert tcp_z - (cb[2] - body.length / 2) == pytest.approx(40 * MM, abs=1e-12)


def test_hbox_defaults(robot):
    head = robot.link_map['head_body'].collisions[0]
    assert np.allclose(head.geometry.size, np.array([194, 35, 381]) * MM)
    mount = robot.link_map['head_mount'].collisions[0]
    assert np.allclose(mount.geometry.size, np.array([100, 100, 60]) * MM)
    conduit = robot.link_map['conduit'].collisions
    assert [type(c.geometry).__name__ for c in conduit] == ['Cylinder', 'Sphere', 'Sphere']
    assert conduit[0].geometry.radius == pytest.approx(15 * MM)
    assert conduit[0].geometry.length + 2 * conduit[1].geometry.radius == pytest.approx(300 * MM)
    # Capsule axis along tool0 +X (default), starting at the head body side face.
    t = fk(robot, 'conduit', root='tool0')
    assert np.allclose(t[:3, 2], [1, 0, 0], atol=1e-9)
    assert np.allclose(t[:3, 3], np.array([97, 0, 110]) * MM, atol=1e-12)


def test_head_box_args():
    r = _urdf(head_w_mm=150, head_d_mm=40, head_h_mm=300, nozzle_dia_mm=16)
    assert np.allclose(r.link_map['head_body'].collisions[0].geometry.size, np.array([150, 40, 300]) * MM)
    assert r.link_map['nozzle_tip'].collisions[0].geometry.radius == pytest.approx(8 * MM)


def test_base_and_positioner_pose_args():
    q = (0.0, 0.0, math.sin(math.radians(45)), math.cos(math.radians(45)))  # Rz(90 deg)
    r = _urdf(base_xyz_mm='100 -20 50', base_quat_xyzw=' '.join(map(str, q)),
              positioner_xyz_mm='700 0 300', positioner_quat_xyzw='0 0 0 1', positioner_axis_xyz='0 1 0')
    base = fk(r, 'base_link')
    assert np.allclose(base[:3, 3], [0.1, -0.02, 0.05])
    assert np.allclose(base[:3, :3], axis_angle([0, 0, 1], math.pi / 2), atol=1e-12)
    flange = fk(r, 'positioner_flange', {'positioner_joint': math.pi / 2})
    assert np.allclose(flange[:3, 3], [0.7, 0.0, 0.3])
    assert np.allclose(flange[:3, :3], axis_angle([0, 1, 0], math.pi / 2), atol=1e-12)


def test_default_poses_identity(robot):
    assert np.allclose(fk(robot, 'base_link'), np.eye(4))
    assert np.allclose(fk(robot, 'positioner_axis'), np.eye(4))


def test_mock_hardware_one_manager_two_components():
    root = ET.fromstring(_xacro(URDF_XACRO))
    systems = root.findall('ros2_control')
    assert {s.get('name') for s in systems} == {'awm_ur', 'awm_positioner'}
    assert [p.text for p in root.iter('plugin')] == ['mock_components/GenericSystem'] * 2
    joints = {j.get('name') for s in systems for j in s.findall('joint')}
    assert joints == set(ARM_JOINTS) | {'positioner_joint'}


def _srdf():
    return ET.fromstring(_xacro(SRDF_XACRO))


def test_srdf_groups_and_home():
    import yaml
    root = _srdf()
    groups = {g.get('name'): g for g in root.iter('group')}
    chain = groups['laser_arm'].find('chain')
    assert chain.get('base_link') == 'base_link' and chain.get('tip_link') == 'tcp_laser'
    assert [j.get('name') for j in groups['positioner'].findall('joint')] == ['positioner_joint']
    home = [s for s in root.iter('group_state') if s.get('name') == 'home' and s.get('group') == 'laser_arm'][0]
    srdf_home = {j.get('name'): float(j.get('value')) for j in home.iter('joint')}
    init = yaml.safe_load((PKG / 'config' / 'initial_positions_v2.yaml').read_text())
    assert srdf_home == pytest.approx({k: v for k, v in init.items() if k != 'positioner_joint'})


def test_srdf_no_head_allowance_outside_rigid_tool():
    """B-04 / F9: a head link may be disabled only against links rigidly fixed to it (wrist_3_link + head)."""
    rigid = {'wrist_3_link', 'flange', 'tool0', 'tcp_laser'} | set(HEAD_LINKS)
    robot_links = {link.name for link in _urdf().links}
    pairs = [(d.get('link1'), d.get('link2')) for d in _srdf().iter('disable_collisions')]
    for a, b in pairs:
        assert a in robot_links and b in robot_links, (a, b)  # nothing but URDF links (no part / holder / jig)
        if a in HEAD_LINKS or b in HEAD_LINKS:
            assert a in rigid and b in rigid, (a, b)
    # And none for the positioner frames (the rotating set lives in the planning scene).
    assert not [p for p in pairs if {'positioner_axis', 'positioner_flange'} & set(p)]
