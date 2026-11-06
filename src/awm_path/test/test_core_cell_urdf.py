"""Cell frames and joint limits from the robot description (G1.3; one source with frames.yaml)."""
import math

import numpy as np
import pytest

from awm_core import frames
from awm_path.core.cell_urdf import CellFrameError, cell_from_urdf, check_against_frames

URDF = """<robot name="t">
  <link name="world"/><link name="base_link"/><link name="positioner_axis"/><link name="positioner_flange"/>
  <link name="l1"/><link name="l2"/><link name="l3"/><link name="l4"/><link name="l5"/><link name="l6"/>
  <joint name="base_joint" type="fixed"><parent link="world"/><child link="base_link"/>
    <origin xyz="0.1 0.2 0.3" rpy="0 0 1.5707963267948966"/></joint>
  <joint name="shoulder_pan_joint" type="revolute"><parent link="base_link"/><child link="l1"/>
    <limit lower="-6.283185307179586" upper="6.283185307179586" velocity="3.14159" effort="1"/></joint>
  <joint name="shoulder_lift_joint" type="revolute"><parent link="l1"/><child link="l2"/>
    <limit lower="-6.283185307179586" upper="6.283185307179586" velocity="3.14159" effort="1"/></joint>
  <joint name="elbow_joint" type="revolute"><parent link="l2"/><child link="l3"/>
    <limit lower="-3.141592653589793" upper="3.141592653589793" velocity="3.14159" effort="1"/></joint>
  <joint name="wrist_1_joint" type="revolute"><parent link="l3"/><child link="l4"/>
    <limit lower="-6.283185307179586" upper="6.283185307179586" velocity="3.14159" effort="1"/></joint>
  <joint name="wrist_2_joint" type="revolute"><parent link="l4"/><child link="l5"/>
    <limit lower="-6.283185307179586" upper="6.283185307179586" velocity="3.14159" effort="1"/></joint>
  <joint name="wrist_3_joint" type="revolute"><parent link="l5"/><child link="l6"/>
    <limit lower="-6.283185307179586" upper="6.283185307179586" velocity="3.14159" effort="1"/></joint>
  <joint name="world_to_positioner_axis" type="fixed"><parent link="world"/><child link="positioner_axis"/>
    <origin xyz="-0.002 -0.475 0.075" rpy="0 0 0"/></joint>
  <joint name="positioner_joint" type="revolute"><parent link="positioner_axis"/><child link="positioner_flange"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-6.981317007977318" upper="6.981317007977318" velocity="0.5235987755982988" effort="100"/></joint>
</robot>"""

FRAMES = {'unit': 'mm',
          'world_T_base': {'position_mm': [100.0, 200.0, 300.0],
                           'quat_xyzw': [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)]},
          'world_T_positioner_axis': {'position_mm': [-2.0, -475.0, 75.0], 'quat_xyzw': [0.0, 0.0, 0.0, 1.0],
                                      'direction': [1.0, 0.0, 0.0]}}


def test_cell_from_urdf_frames_and_limits():
    cell = cell_from_urdf(URDF)
    assert np.allclose(cell.world_T_base_link[:3, 3], [100.0, 200.0, 300.0])
    assert np.allclose(cell.world_T_base_link[:3, 0], [0.0, 1.0, 0.0], atol=1e-12)
    assert np.allclose(cell.world_T_positioner_axis[:3, 3], [-2.0, -475.0, 75.0])
    assert np.allclose(cell.positioner_axis_local, [1.0, 0.0, 0.0])
    assert cell.positioner_limits_deg == pytest.approx((-400.0, 400.0))
    assert cell.positioner_velocity_deg_s == pytest.approx(30.0)
    assert cell.joint_limits_deg[2] == pytest.approx((-180.0, 180.0))
    assert cell.joint_limits_deg[0] == pytest.approx((-360.0, 360.0))
    assert cell.joint_velocity_deg_s[0] == pytest.approx(180.0, abs=1e-3)
    base_T_axis = cell.base_link_T_positioner_axis
    assert np.allclose(base_T_axis, frames.invert(cell.world_T_base_link) @ cell.world_T_positioner_axis)


def test_check_against_frames_agrees_and_refuses():
    cell = cell_from_urdf(URDF)
    rep = check_against_frames(cell, FRAMES)
    assert rep['base_mm'] < 1e-6 and rep['positioner_mm'] < 1e-6
    bad = {**FRAMES, 'world_T_positioner_axis': {**FRAMES['world_T_positioner_axis'], 'position_mm': [-2.0, -475.0, 76.0]}}
    with pytest.raises(CellFrameError, match='positioner'):
        check_against_frames(cell, bad)
    tilted = URDF.replace('<axis xyz="1 0 0"/>', '<axis xyz="0 1 0"/>')
    with pytest.raises(CellFrameError, match='axis'):
        check_against_frames(cell_from_urdf(tilted), FRAMES)


def test_positioner_limits_must_match_config():
    cell = cell_from_urdf(URDF)
    cell.check_positioner_limits({'lower_deg': -400, 'upper_deg': 400, 'max_velocity_deg_s': 30})
    with pytest.raises(CellFrameError, match='positioner limits'):
        cell.check_positioner_limits({'lower_deg': -360, 'upper_deg': 400, 'max_velocity_deg_s': 30})
