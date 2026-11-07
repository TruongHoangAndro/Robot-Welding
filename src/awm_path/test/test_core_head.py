"""CP-56 gap / S.5c Task 0: head geometry (tool0 -> tcp_laser, nozzle radius, s_c) read from the robot description.

Synthetic URDF strings only (no cell data); lengths in the URDF in m, results in mm.
"""

import math

import numpy as np
import pytest

from awm_core import units
from awm_path.core.contact import ContactParams
from awm_path.core.head import HeadGeometry, check_against_fk, contact_params_for, head_from_urdf

R_OUT = 'rpy="0 0 0"'


def _urdf(tcp_z_mm=481.0, nozzle_r_mm=10.0, tip_len_mm=5.0, s_c_mm=0.3, mid_rpy='0 0 0', tip_joint='fixed',
          nozzle_rpy='0 0 0', nozzle_off_mm=0.0, with_nozzle=True):
    """tool0 -> mid (rotated by mid_rpy, undone by the child joints) -> tcp_laser; nozzle_tip on tool0."""
    m = units.mm_to_m
    roll, pitch, yaw = (float(v) for v in mid_rpy.split())
    # mid frame rotated by R(mid); tcp_laser = mid · R(mid)^-1 at tcp_z along tool0 Z: express via a 2nd joint.
    r = _rpy(roll, pitch, yaw)
    tcp_in_mid = r.T @ np.array([0.0, 0.0, tcp_z_mm])
    back = _matrix_to_rpy(r.T)
    tip_cz = tcp_z_mm - s_c_mm - (tip_len_mm - s_c_mm) / 2.0
    nozzle = f'''
  <link name="nozzle_tip"><collision><origin xyz="{m(nozzle_off_mm)} 0 {m(tip_cz)}" rpy="{nozzle_rpy}"/>
    <geometry><cylinder radius="{m(nozzle_r_mm)}" length="{m(tip_len_mm - s_c_mm)}"/></geometry></collision></link>
  <joint name="tool0_to_nozzle_tip" type="fixed"><parent link="tool0"/><child link="nozzle_tip"/>
    <origin xyz="0 0 0" rpy="0 0 0"/></joint>''' if with_nozzle else ''
    return f'''<?xml version="1.0"?>
<robot name="t">
  <link name="wrist_3_link"/><link name="tool0"/><link name="mid"/><link name="tcp_laser"/>
  <joint name="flange" type="fixed"><parent link="wrist_3_link"/><child link="tool0"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/></joint>
  <joint name="tool0_to_mid" type="fixed"><parent link="tool0"/><child link="mid"/>
    <origin xyz="0 0 0" rpy="{roll} {pitch} {yaw}"/></joint>
  <joint name="mid_to_tcp" type="{tip_joint}"><parent link="mid"/><child link="tcp_laser"/>
    <origin xyz="{m(tcp_in_mid[0])} {m(tcp_in_mid[1])} {m(tcp_in_mid[2])}" rpy="{back[0]} {back[1]} {back[2]}"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>{nozzle}
</robot>'''


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _matrix_to_rpy(r):
    pitch = math.asin(-max(-1.0, min(1.0, r[2, 0])))
    return math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])


def test_tool0_T_tcp_is_the_composed_fixed_chain() -> None:
    head = head_from_urdf(_urdf(tcp_z_mm=437.5, mid_rpy='0.3 -0.2 1.1'))
    assert isinstance(head, HeadGeometry)
    assert np.allclose(head.tool0_T_tcp_mm[:3, 3], [0.0, 0.0, 437.5], atol=1e-9)
    assert np.allclose(head.tool0_T_tcp_mm[:3, :3], np.eye(3), atol=1e-12)


def test_nozzle_radius_and_standoff_come_from_the_nozzle_tip_cylinder() -> None:
    head = head_from_urdf(_urdf(nozzle_r_mm=6.0, s_c_mm=0.25))
    assert head.nozzle_radius_mm == pytest.approx(6.0, abs=1e-9)
    assert head.contact_standoff_mm == pytest.approx(0.25, abs=1e-9)


def test_contact_params_for_takes_the_nozzle_from_the_description() -> None:
    head = head_from_urdf(_urdf(nozzle_r_mm=4.0, s_c_mm=0.3))
    p = contact_params_for(head, ContactParams(point_tolerance_mm=0.15))
    assert p.nozzle_radius_mm == pytest.approx(4.0) and p.standoff_mm == pytest.approx(0.3)
    assert p.point_tolerance_mm == 0.15 and p.rule_b == 'rising'


def test_movable_joint_in_the_tcp_chain_is_an_error() -> None:
    with pytest.raises(ValueError, match='fixed'):
        head_from_urdf(_urdf(tip_joint='revolute'))


def test_missing_links_are_errors() -> None:
    with pytest.raises(ValueError, match='tcp_laser'):
        head_from_urdf(_urdf().replace('tcp_laser', 'tcp_other'))
    with pytest.raises(ValueError, match='nozzle_tip'):
        head_from_urdf(_urdf(with_nozzle=False))


def test_nozzle_off_the_beam_axis_or_tilted_is_an_error() -> None:
    with pytest.raises(ValueError, match='beam axis'):
        head_from_urdf(_urdf(nozzle_off_mm=0.5))
    with pytest.raises(ValueError, match='beam axis'):
        head_from_urdf(_urdf(nozzle_rpy='0.1 0 0'))


def test_check_against_fk() -> None:
    head = head_from_urdf(_urdf())
    fk = head.tool0_T_tcp_mm.copy()
    check_against_fk(head, fk)
    fk[2, 3] += 0.01
    with pytest.raises(ValueError, match='tool0'):
        check_against_fk(head, fk)


def test_record_has_hash_and_values() -> None:
    xml = _urdf(nozzle_r_mm=6.0)
    rec = head_from_urdf(xml).as_record()
    from awm_core.hashing import sha256_text
    assert rec['robot_description_sha256'] == sha256_text(xml)
    assert rec['nozzle_radius_mm'] == pytest.approx(6.0)
    assert rec['tool0_T_tcp_mm']['position_mm'] == pytest.approx([0.0, 0.0, 481.0])


def test_contact_params_for_derives_rim_samples_from_radius_and_standoff() -> None:
    """Review M1: rim spacing 2πr / rim_samples ≤ 2·s_c for the configured nozzle (a face crossing the rim is never
    missed), rim_samples a multiple of ring_samples (ψ invariance of the sample set kept)."""
    head = head_from_urdf(_urdf(nozzle_r_mm=10.0, s_c_mm=0.05))
    p = contact_params_for(head)
    assert 2.0 * math.pi * p.nozzle_radius_mm / p.rim_samples <= 2.0 * p.standoff_mm + 1e-12
    assert p.rim_samples % p.ring_samples == 0 and p.rim_samples > ContactParams().rim_samples
    small = contact_params_for(head_from_urdf(_urdf(nozzle_r_mm=4.0, s_c_mm=0.3)))
    assert small.rim_samples == ContactParams().rim_samples          # default already dense enough


def _with_head_body(urdf: str, geometry: str) -> str:
    link = (f'<link name="head_body"><collision><geometry>{geometry}</geometry></collision></link>'
            '<joint name="tool0_to_head_body" type="fixed"><parent link="tool0"/><child link="head_body"/></joint>')
    return urdf.replace('</robot>', link + '</robot>')


def test_head_model_of_the_description_and_label_check() -> None:
    """Review I1: the loaded description must match the head label of the run (H-CAD = mesh head_body collision,
    H-BOX = box)."""
    from awm_path.core.head import check_head_model, head_model_of_urdf, normalise_head_model
    box = _with_head_body(_urdf(), '<box size="0.194 0.035 0.381"/>')
    mesh = _with_head_body(_urdf(), '<mesh filename="file:///x/head_body_hull.stl" scale="0.001 0.001 0.001"/>')
    assert head_model_of_urdf(box) == 'hbox' and head_model_of_urdf(mesh) == 'hcad'
    assert normalise_head_model('H-CAD') == 'hcad' and normalise_head_model('hbox') == 'hbox'
    with pytest.raises(ValueError, match='head model'):
        normalise_head_model('H-FOO')
    check_head_model(mesh, 'H-CAD')
    with pytest.raises(ValueError, match='H-CAD'):
        check_head_model(box, 'H-CAD')
    with pytest.raises(ValueError, match='head_body'):
        head_model_of_urdf(_urdf())
