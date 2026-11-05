"""Cell v2 configuration helpers shared by the launch, the scene tools and tests (no ROS graph needed)."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Dict, List, Optional, Union

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


HEAD_MESH_LINKS = ('head_body', 'head_mount')


class HeadDirError(ValueError):
    """The H-CAD head folder (head_model:=hcad) is unusable: path, head.yaml or a mesh file is missing or changed."""


def check_head_dir(head_dir: str) -> List[Path]:
    """Check an H-CAD head folder before xacro runs; return head.yaml and the mesh files it names.

    head_dir must be absolute (it becomes ``file://`` mesh URIs). head.yaml (awm_geometry export-head, A18) must name a
    mesh and a hull file for head_body and head_mount; each file must exist and match its SHA-256 in head.yaml.
    """
    value = (head_dir or '').strip()
    if not value or not Path(value).is_absolute():
        raise HeadDirError(f'head_model:=hcad needs head_dir:=<absolute folder with head.yaml>, got {head_dir!r}')
    folder = Path(value)
    head_yaml = folder / 'head.yaml'
    if not head_yaml.is_file():
        raise HeadDirError(f'{head_yaml} not found')
    doc = yaml.safe_load(head_yaml.read_text()) or {}
    files = [head_yaml]
    for link in HEAD_MESH_LINKS:
        entry = (doc.get('links') or {}).get(link)
        if not entry:
            raise HeadDirError(f'{head_yaml}: links.{link} missing')
        for kind in ('mesh', 'hull'):
            spec = entry.get(kind) or {}
            path = folder / str(spec.get('file', ''))
            if not spec.get('file') or not path.is_file():
                raise HeadDirError(f'{head_yaml}: links.{link}.{kind} file {path} not found')
            sha = hashlib.sha256(path.read_bytes()).hexdigest()
            if spec.get('sha256') and sha != spec['sha256']:
                raise HeadDirError(f'SHA-256 of {path} does not match {head_yaml} (file changed after export)')
            files.append(path)
    return files
