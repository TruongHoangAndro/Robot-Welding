"""A42 torch orientation and A43 approach / retract, in the part frame (pure numpy).

Tool frame convention (D14 ``tcp_mig``): z = torch / wire axis pointing INTO the work, y = z × x.
The roll about z does not change the weld (round nozzle / beam); it is chosen by ``tool_roll``:
- ``travel``: x = travel direction (perpendicular to z);
- ``up``: x = part +z projected perpendicular to z, so the torch body stays above the joint and the
  wrist does not have to turn with the travel direction (orbital seams around a pipe).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

from awm_core import frames


class PathError(ValueError):
    """A seam cannot be turned into tool poses (reason in the message)."""


@dataclass
class SeamPoses:
    seam_id: str
    approach: np.ndarray
    weld: List[np.ndarray]
    retract: np.ndarray
    length_m: float


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise PathError('degenerate direction while building the torch frame')
    return v / n


def torch_axis(t: Sequence[float], n1: Sequence[float], n2: Optional[Sequence[float]],
               weld_side: str, work_angle_deg: float, travel_angle_deg: float) -> np.ndarray:
    """Unit torch axis z (into the work) at one seam point.

    - ``weld_side == 'bisector'``: the torch lies on the bisector of the two faces (work angle 45° of
      a 90° fillet); the WPS work angle is not used.
    - ``weld_side`` = reference face (n1): the torch makes ``work_angle_deg`` with that face, in the
      plane perpendicular to the seam.
    Travel angle > 0 = push: the torch tip leans forward along the travel direction.
    """
    t = _unit(np.asarray(t, float))
    n1 = _unit(np.asarray(n1, float))
    if n2 is None:
        raise PathError('seam edge has a single adjacent face: torch orientation needs two faces '
                        '(lap / edge joints planned for R3)')
    n2 = _unit(np.asarray(n2, float))
    if weld_side == 'bisector':
        away = n1 + n2
        if np.linalg.norm(away) < 1e-6:
            raise PathError('faces are opposite (180 deg): bisector undefined')
        away = _unit(away - (away @ t) * t)
    else:
        # In-face direction of the reference face pointing away from the root (towards n2 for a
        # concave root), then rotate up by the work angle.
        along = n2 - (n2 @ n1) * n1
        along = _unit(along - (along @ t) * t)
        wa = np.radians(work_angle_deg)
        away = _unit(np.cos(wa) * along + np.sin(wa) * n1)
    z0 = -away
    ta = np.radians(travel_angle_deg)
    t_perp = _unit(t - (t @ z0) * z0)
    return _unit(np.cos(ta) * z0 + np.sin(ta) * t_perp)


TOOL_ROLLS = ('travel', 'up')
UP = np.array([0.0, 0.0, 1.0])


def tool_pose(point: Sequence[float], t: Sequence[float], z: np.ndarray,
              focus_offset_m: float = 0.0, tool_roll: str = 'travel') -> np.ndarray:
    """4x4 part_T_tool. ``focus_offset_m`` moves the TCP along z (laser focus below the surface)."""
    if tool_roll == 'travel':
        x_hint = np.asarray(t, float)
    elif tool_roll == 'up':
        if abs(z @ UP) > 0.98:
            raise PathError('tool_roll "up" undefined: torch axis is (nearly) vertical')
        x_hint = UP
    else:
        raise PathError(f'tool_roll must be one of {TOOL_ROLLS}')
    rot = frames.frame_from_z_x(z, x_hint)
    return frames.make_transform(np.asarray(point, float) + focus_offset_m * z, rot)


def offset_along_tool_z(pose: np.ndarray, distance_m: float) -> np.ndarray:
    """Move a pose back along its own z axis by ``distance_m`` (approach / retract, A43)."""
    out = pose.copy()
    out[:3, 3] = pose[:3, 3] - distance_m * pose[:3, 2]
    return out


def seam_poses(geometry: Dict, seam: Dict, process_row: Dict, approach_distance_m: float,
               tool_roll: str = 'travel') -> SeamPoses:
    """Tool poses of one seam in the part frame (A41 samples -> A42 -> A43)."""
    motion = process_row['motion']
    if motion['weave_width_mm'] > 0.0:
        raise PathError('weave requested by the WPS: weave is planned for R3')
    if seam['joint_type'] not in ('fillet',):
        raise PathError(f'joint type {seam["joint_type"]} not supported in R1 (fillet only)')
    focus = motion['focus_offset_mm'] / 1000.0 if process_row['process'] == 'LASER' else 0.0
    points = geometry['points']
    if len(points) < 2:
        raise PathError('seam has fewer than 2 samples')
    axes = [torch_axis(p['t'], p['n1'], p['n2'], seam['weld_side'], motion['work_angle_deg'],
                       motion['travel_angle_deg']) for p in points]
    weld = [tool_pose(p['p_m'], p['t'], z, focus, tool_roll) for p, z in zip(points, axes)]
    for a, b in zip(weld, weld[1:]):
        if frames.rotation_angle_between(a[:3, :3], b[:3, :3]) > np.radians(30.0):
            raise PathError('torch orientation jumps by more than 30 deg between samples '
                            '(sharp corner in the seam): split the seam at the corner')
    return SeamPoses(seam['seam_id'], offset_along_tool_z(weld[0], approach_distance_m), weld,
                     offset_along_tool_z(weld[-1], approach_distance_m),
                     float(geometry['length_m']))
