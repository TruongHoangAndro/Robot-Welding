"""Robot description for offline planning, rendered from xacro without a running simulation (G1.3).

``awm_cell_v2.urdf.xacro`` is rendered with the same pose arguments as ``sim_mock.launch.py`` (taken from the cell
``frames.yaml``) and the head model arguments, so the planner and the simulation use one description. From it:
``tool0_T_tcp`` and the nozzle (``head_from_urdf``), the cell frames, the UR joint limits and the positioner limits
(``cell_urdf``), each cross-checked against ``frames.yaml`` and ``positioner_limits.yaml``.

Needs the ROS overlay (xacro, ament index): not part of the pure core (B-06). No rclpy, no running nodes.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping, Optional

import numpy as np
import yaml

from awm_core.hashing import sha256_file, sha256_text
from awm_path.core.cell_urdf import CellKinematics, cell_from_urdf, check_against_frames
from awm_path.core.head import HeadGeometry, head_from_urdf
from awm_path.core.sync import SyncCell

HEAD_MODELS = ('hcad', 'hbox')


@dataclass(frozen=True)
class PlanningDescription:
    """Everything the kinematic planner takes from the description, plus its provenance."""

    urdf: str
    head: HeadGeometry
    cell: CellKinematics
    sync_cell: SyncCell
    head_model: str
    mappings: Dict[str, str]
    frame_check: Dict[str, float]
    positioner_limits_file: Path
    kinematics_file: Path

    def as_record(self) -> Dict:
        return {'robot_description_sha256': sha256_text(self.urdf), 'head_model': self.head_model,
                'xacro_args': dict(sorted(self.mappings.items())), 'head': self.head.as_record(),
                'cell': self.cell.as_record(), 'frames_yaml_check': self.frame_check,
                'positioner_limits_sha256': sha256_file(self.positioner_limits_file),
                'ur_kinematics_sha256': sha256_file(self.kinematics_file)}


def _share(pkg: str) -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(pkg))


def render_cell_urdf(mappings: Mapping[str, str]) -> str:
    """Render ``awm_description/urdf/awm_cell_v2.urdf.xacro`` with xacro ``mappings`` (strings)."""
    import xacro
    doc = xacro.process_file(str(_share('awm_description') / 'urdf' / 'awm_cell_v2.urdf.xacro'),
                             mappings=dict(mappings))
    return doc.toxml()


def load_planning_description(frames_yaml: Path, head_model: str, head_dir: Optional[str] = None,
                              head_args: Optional[Mapping[str, str]] = None) -> PlanningDescription:
    """Render the description for ``head_model`` (hcad: ``head_dir`` with head.yaml + meshes) and read the planning
    values from it. Raises ``ValueError`` (incl. ``CellFrameError`` / ``HeadDirError``) on any disagreement."""
    from awm_bringup.cell_config import check_head_dir, load_positioner_limits, positioner_limits_path
    from awm_path.gate_g2 import launch_args_from_frames

    if head_model not in HEAD_MODELS:
        raise ValueError(f'head model must be one of {HEAD_MODELS}, got {head_model!r}')
    frames_doc = yaml.safe_load(Path(frames_yaml).read_text(encoding='utf-8'))
    mappings = dict(launch_args_from_frames(frames_yaml))
    mappings.update({k: str(v) for k, v in (head_args or {}).items()})
    mappings['head_model'] = head_model
    if head_model == 'hcad':
        check_head_dir(str(head_dir or ''))
        mappings['head_dir'] = str(head_dir)
    urdf = render_cell_urdf(mappings)
    head = head_from_urdf(urdf)
    cell = cell_from_urdf(urdf)
    frame_check = check_against_frames(cell, frames_doc)
    lim_file = positioner_limits_path()
    cell.check_positioner_limits(load_positioner_limits(lim_file))
    world_z_base = cell.base_link_T_world[:3, 2].copy()
    sync_cell = SyncCell(base_link_T_axis=cell.base_link_T_positioner_axis, world_z_base=world_z_base,
                         tool0_T_tcp=np.asarray(head.tool0_T_tcp_mm, float), joint_limits_deg=cell.joint_limits_deg,
                         theta_limits_deg=cell.positioner_limits_deg, joint_velocity_deg_s=cell.joint_velocity_deg_s,
                         theta_velocity_deg_s=cell.positioner_velocity_deg_s)
    kin_file = _share('ur_description') / 'config' / 'ur5e' / 'default_kinematics.yaml'
    return PlanningDescription(urdf, head, cell, sync_cell, head_model, mappings, frame_check, Path(lim_file),
                               kin_file)


__all__ = ['HEAD_MODELS', 'PlanningDescription', 'render_cell_urdf', 'load_planning_description']
