"""Cell v2 configuration helpers shared by the launch, the scene tools and tests (no ROS graph needed)."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional, Union

import yaml

from awm_core import units

PathLike = Union[str, Path]


def positioner_limits_path() -> Path:
    """Installed ``awm_description/config/positioner_limits.yaml`` (the only source of the positioner limits)."""
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('awm_description')) / 'config' / 'positioner_limits.yaml'


def load_positioner_limits(path: Optional[PathLike] = None) -> Dict[str, float]:
    """positioner_joint limits in deg, deg/s, deg/s^2."""
    data = yaml.safe_load(Path(path or positioner_limits_path()).read_text())
    lim = data['positioner_joint']
    for key in ('lower_deg', 'upper_deg', 'max_velocity_deg_s', 'max_acceleration_deg_s2'):
        lim[key] = float(lim[key])
    if not (lim['lower_deg'] < lim['upper_deg'] and lim['max_velocity_deg_s'] > 0
            and lim['max_acceleration_deg_s2'] > 0):
        raise ValueError(f'invalid positioner limits: {lim}')
    return lim


def positioner_moveit_limits(lim: Dict[str, float]) -> Dict[str, object]:
    """MoveIt ``robot_description_planning.joint_limits.positioner_joint`` entry (rad, rad/s, rad/s^2)."""
    return {'has_velocity_limits': True, 'max_velocity': units.deg_to_rad(lim['max_velocity_deg_s']),
            'has_acceleration_limits': True, 'max_acceleration': units.deg_to_rad(lim['max_acceleration_deg_s2'])}
