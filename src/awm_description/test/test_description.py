"""The cell xacro expands, parses, and matches cell_nominal.yaml (D14)."""

import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml
from urdf_parser_py.urdf import URDF

PKG = Path(__file__).resolve().parents[1]
CFG = yaml.safe_load((PKG / 'config' / 'cell_nominal.yaml').read_text())


def _xacro(path: Path) -> str:
    proc = subprocess.run(['xacro', str(path)], capture_output=True, text=True, check=False)
    assert proc.returncode == 0, proc.stderr
    return proc.stdout


@pytest.fixture(scope='module')
def robot():
    return URDF.from_xml_string(_xacro(PKG / 'urdf' / 'awm_cell.urdf.xacro'))


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), \
        math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def test_links_present(robot):
    names = {link.name for link in robot.links}
    for name in ('world', 'base_link', 'tool0', 'torch_mig', 'tcp_mig', 'fixture',
                 'fixture_block', 'pismo_frame'):
        assert name in names, name
    assert robot.get_root() == 'world'


def test_tcp_from_config(robot):
    joint = robot.joint_map['torch_mig_to_tcp']
    assert joint.parent == 'tool0'
    assert np.allclose(joint.origin.xyz, CFG['torch_mig']['tcp_xyz_m'])
    assert np.allclose(joint.origin.rpy, CFG['torch_mig']['tcp_rpy_rad'])


def test_nozzle_ends_stickout_before_tcp(robot):
    """The nozzle cylinder lies on the wire axis and ends stickout_m before the TCP."""
    t = CFG['torch_mig']
    tcp = np.array(t['tcp_xyz_m'])
    wire = _rpy(*t['tcp_rpy_rad'])[:, 2]
    nozzle = robot.link_map['torch_mig'].collisions[2]
    centre = np.array(nozzle.origin.xyz)
    axis = _rpy(*nozzle.origin.rpy)[:, 2]
    assert abs(abs(axis @ wire) - 1.0) < 1e-9
    tip = centre + wire * nozzle.geometry.length / 2
    assert np.allclose(tip + wire * t['stickout_m'], tcp, atol=1e-9)
    assert nozzle.geometry.radius == pytest.approx(t['nozzle_radius_m'])


def test_fixture_pose(robot):
    joint = robot.joint_map['world_to_fixture']
    assert np.allclose(joint.origin.xyz, CFG['fixture']['xyz_m'])


def test_mock_hardware_only():
    root = ET.fromstring(_xacro(PKG / 'urdf' / 'awm_cell.urdf.xacro'))
    plugins = [p.text for p in root.iter('plugin')]
    assert plugins == ['mock_components/GenericSystem']


def test_srdf_groups():
    root = ET.fromstring(_xacro(PKG / 'srdf' / 'awm_cell.srdf.xacro'))
    group = [g for g in root.iter('group') if g.get('name') == 'weld_arm'][0]
    chain = group.find('chain')
    assert chain.get('base_link') == 'base_link' and chain.get('tip_link') == 'tcp_mig'
    home_state = [s for s in root.iter('group_state')
                  if s.get('name') == 'home' and s.get('group') == 'weld_arm'][0]
    srdf_home = {j.get('name'): float(j.get('value')) for j in home_state.iter('joint')}
    home = yaml.safe_load((PKG / 'config' / 'initial_positions.yaml').read_text())
    assert srdf_home == pytest.approx(home)
