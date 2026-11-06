"""Cell values for the 7-joint program generator from the installed robot description (G1.5).

One source for every value: the planning description of ``awm_path.description`` (rendered xacro, the same one the
planner used: tool0_T_tcp, cell frames, joint and positioner position / velocity limits) plus

* the UR joint acceleration limits of ``awm_description/config/joint_limits_v2.yaml`` (rad/s², REP 103; converted
  only through ``awm_core.units``),
* the positioner acceleration limit of ``positioner_limits.yaml`` (``awm_bringup.cell_config``),
* home = ``awm_description/config/initial_positions_v2.yaml`` (the mock-hardware start state = SRDF ``home``), so
  a program starts where the simulation starts (G1.7 start-state check).

Needs the ament index (ROS overlay); no rclpy, no running nodes. Limits are read, never changed.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict

import yaml

from awm_core import units
from awm_path.core.cell_urdf import POSITIONER_JOINT, UR_JOINTS
from awm_program.core.program7 import ProgramCell


def _share() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('awm_description')) / 'config'


def joint_limits_file() -> Path:
    return _share() / 'joint_limits_v2.yaml'


def home_file() -> Path:
    return _share() / 'initial_positions_v2.yaml'


def load_program_cell(desc) -> ProgramCell:
    """:class:`ProgramCell` for a ``PlanningDescription``."""
    from awm_bringup.cell_config import load_positioner_limits
    jl = yaml.safe_load(joint_limits_file().read_text(encoding='utf-8'))['joint_limits']
    acc = []
    for j in UR_JOINTS:
        entry = jl[j]
        if not entry.get('has_acceleration_limits'):
            raise ValueError(f'{joint_limits_file()}: {j} has no acceleration limit')
        acc.append(float(units.rad_to_deg(float(entry['max_acceleration']))))
    acc.append(float(load_positioner_limits(desc.positioner_limits_file)['max_acceleration_deg_s2']))
    home: Dict = yaml.safe_load(home_file().read_text(encoding='utf-8'))
    home_deg = tuple(float(units.rad_to_deg(float(home[j]))) for j in UR_JOINTS + (POSITIONER_JOINT,))
    return ProgramCell(sync=desc.sync_cell, acceleration_deg_s2=tuple(acc), home_deg=home_deg,
                       robot_joints=tuple(UR_JOINTS), positioner_joints=(POSITIONER_JOINT,))


__all__ = ['joint_limits_file', 'home_file', 'load_program_cell']
