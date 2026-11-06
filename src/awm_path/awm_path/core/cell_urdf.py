"""Cell frames and joint limits from the robot description in use (G1.3; CP-58 "one source"; DES-001 §5.3).

The planner needs ``world_T_base_link``, ``world_T_positioner_axis`` (= ``positioner_flange`` at θ = 0), the
positioner axis, the six UR joint limits and the positioner limits. They are read from the URDF that the simulation
uses (``awm_cell_v2.urdf.xacro`` rendered with the ``frames.yaml`` arguments, as ``sim_mock`` does) and checked
against ``frames.yaml`` (:func:`check_against_frames`) and ``positioner_limits.yaml``
(:meth:`CellKinematics.check_positioner_limits`). Disagreement → :class:`CellFrameError` (the planner refuses).

Pure Python (xml.etree + numpy), mm / degrees; URDF m / rad only through ``awm_core.units`` (ADR-14). No rclpy (B-06).
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, Mapping, Tuple

import numpy as np

from awm_core import frames, units
from awm_path.core.head import _chain_mm, _floats
from awm_path.core.kinematics import pose_error

#: UR joint names in the KinematicsBackend order.
UR_JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
             'wrist_3_joint')
POSITIONER_JOINT = 'positioner_joint'
#: Agreement tolerances description vs frames.yaml / positioner_limits.yaml (numerical: both come from one file).
FRAME_TOL_MM = 1e-6
FRAME_TOL_DEG = 1e-6
LIMIT_TOL_DEG = 1e-6


class CellFrameError(ValueError):
    """Robot description and cell data disagree, or the description lacks a frame / joint."""


@dataclass(frozen=True)
class CellKinematics:
    """Cell values of one robot description (mm, degrees)."""

    world_T_base_link: np.ndarray
    world_T_positioner_axis: np.ndarray
    positioner_axis_local: np.ndarray
    positioner_limits_deg: Tuple[float, float]
    positioner_velocity_deg_s: float
    joint_limits_deg: Tuple[Tuple[float, float], ...]
    joint_velocity_deg_s: Tuple[float, ...]

    @property
    def base_link_T_positioner_axis(self) -> np.ndarray:
        return frames.invert(self.world_T_base_link) @ self.world_T_positioner_axis

    @property
    def base_link_T_world(self) -> np.ndarray:
        return frames.invert(self.world_T_base_link)

    def check_positioner_limits(self, config: Mapping[str, float]) -> None:
        """Raise unless the URDF limits equal ``positioner_limits.yaml`` (the single source, OI-14)."""
        want = (float(config['lower_deg']), float(config['upper_deg']), float(config['max_velocity_deg_s']))
        have = (*self.positioner_limits_deg, self.positioner_velocity_deg_s)
        if any(abs(a - b) > LIMIT_TOL_DEG for a, b in zip(want, have)):
            raise CellFrameError(f'positioner limits of the description {have} differ from positioner_limits.yaml '
                                 f'{want}')

    def as_record(self) -> Dict:
        return {'world_T_base_link': pose_mm(self.world_T_base_link),
                'world_T_positioner_axis': pose_mm(self.world_T_positioner_axis),
                'positioner_axis_local': [float(v) for v in self.positioner_axis_local],
                'positioner_limits_deg': list(self.positioner_limits_deg),
                'positioner_velocity_deg_s': self.positioner_velocity_deg_s,
                'joint_limits_deg': [list(v) for v in self.joint_limits_deg],
                'joint_velocity_deg_s': list(self.joint_velocity_deg_s)}


def pose_mm(t: np.ndarray) -> Dict:
    """4x4 transform (mm) -> ``{position_mm, quat_xyzw}`` (v2 layout)."""
    t = np.asarray(t, dtype=float)
    return {'position_mm': [float(v) for v in t[:3, 3]],
            'quat_xyzw': [float(v) for v in frames.matrix_to_quat(t[:3, :3])]}


def _limit(joint: ET.Element) -> Tuple[float, float, float]:
    lim = joint.find('limit')
    if lim is None or joint.get('type') != 'revolute':
        raise CellFrameError(f'joint {joint.get("name")!r} must be revolute with a <limit>')
    return (float(units.rad_to_deg(float(lim.get('lower')))), float(units.rad_to_deg(float(lim.get('upper')))),
            float(units.rad_to_deg(float(lim.get('velocity')))))


def cell_from_urdf(urdf_xml: str, world: str = 'world', base_link: str = 'base_link',
                   axis_link: str = 'positioner_axis') -> CellKinematics:
    """Read :class:`CellKinematics` from a URDF string."""
    root = ET.fromstring(urdf_xml)
    by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
    by_name = {j.get('name'): j for j in root.findall('joint')}
    try:
        w_b = _chain_mm(by_child, world, base_link)
        w_a = _chain_mm(by_child, world, axis_link)
    except ValueError as exc:
        raise CellFrameError(str(exc)) from exc
    missing = [n for n in UR_JOINTS + (POSITIONER_JOINT,) if n not in by_name]
    if missing:
        raise CellFrameError(f'joints {missing} not in the robot description')
    pj = by_name[POSITIONER_JOINT]
    if pj.find('parent').get('link') != axis_link:
        raise CellFrameError(f'{POSITIONER_JOINT} must be a child of {axis_link!r}')
    origin = pj.find('origin')
    if origin is not None and (np.any(np.abs(_floats(origin.get('xyz'))) > 0.0)
                               or np.any(np.abs(_floats(origin.get('rpy'))) > 0.0)):
        raise CellFrameError(f'{POSITIONER_JOINT} origin must be zero (flange = axis frame at theta = 0)')
    axis = _floats(pj.find('axis').get('xyz') if pj.find('axis') is not None else '1 0 0')
    axis = axis / np.linalg.norm(axis)
    plo, phi, pv = _limit(pj)
    lims = [_limit(by_name[n]) for n in UR_JOINTS]
    return CellKinematics(w_b, w_a, axis, (plo, phi), pv, tuple((lo, hi) for lo, hi, _ in lims),
                          tuple(v for _, _, v in lims))


def _pose(entry: Mapping) -> np.ndarray:
    return frames.make_transform(entry['position_mm'], frames.quat_to_matrix(entry['quat_xyzw']))


def check_against_frames(cell: CellKinematics, frames_doc: Mapping) -> Dict[str, float]:
    """Compare the description with ``frames.yaml`` (``world_T_base`` = ``base_link``, ``world_T_positioner_axis``,
    axis = +X of the positioner frame, the seam_axis.json convention). Returns the differences; raises
    :class:`CellFrameError` beyond 1e-6 mm / 1e-6 deg."""
    if frames_doc.get('unit') != 'mm':
        raise CellFrameError('frames.yaml must have unit "mm"')
    out = {}
    for key, have, name in (('world_T_base', cell.world_T_base_link, 'base'),
                            ('world_T_positioner_axis', cell.world_T_positioner_axis, 'positioner')):
        want = _pose(frames_doc[key])
        dp, dr_rad = pose_error(want, have)
        dr = float(units.rad_to_deg(dr_rad))
        out[f'{name}_mm'], out[f'{name}_deg'] = dp, dr
        if dp > FRAME_TOL_MM or dr > FRAME_TOL_DEG:
            raise CellFrameError(f'{name} frame of the robot description differs from frames.yaml {key} by '
                                 f'{dp:.6g} mm / {dr:.6g} deg (one source: re-render the description)')
    if np.linalg.norm(cell.positioner_axis_local - np.array([1.0, 0.0, 0.0])) > 1e-9:
        raise CellFrameError(f'positioner axis {list(cell.positioner_axis_local)} is not +X of positioner_axis '
                             '(seam_axis.json convention)')
    return out


__all__ = ['pose_mm', 'UR_JOINTS', 'POSITIONER_JOINT', 'CellFrameError', 'CellKinematics', 'cell_from_urdf',
           'check_against_frames']
