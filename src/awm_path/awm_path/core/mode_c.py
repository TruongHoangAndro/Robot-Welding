"""Mode C core (WP-S S.6; TST-002 Rev 0.3 draft §5.1 F7, F8, F10, §5.3 C9; DES-001 Rev 0.6 draft §4.3 steps 3 / 6,
§5.4, §6.3; B-03).

Mode C: the robot is at rest at one pose, the positioner turns the part (and the holders) through 360° plus the
closing overlap. This module holds the pure parts of the check:

* :func:`angular_step_max_deg` — B-03 / F10 positioner step Δθ ≤ 2·(c − margin) / r_max. Reading used by S.6
  (Assumption, recorded): c = clearance verified at every checked angle, margin = clearance guaranteed between two
  checked angles (no rotating point moves more than r_max·Δθ / 2 from its nearest checked position);
* :func:`theta_grid`, :func:`closing_overlap_deg`, :func:`travel_sign` — the angles to check and the direction that
  advances the contact point along the seam travel direction (+X of the seam frame);
* :func:`coaxiality` — F8 seam axis vs positioner axis (0.05 mm / 0.1°, D16 Assumption);
* :func:`world_T_flange_at`, :func:`part_T_tcp_at` — the TCP is fixed in ``world`` while the part turns; the contact
  check (F9) is evaluated on the real relative pose part_T_tcp(θ) at every angle;
* :func:`select_pose` — ranking of single poses by the S.3 node cost with the F7 tie-break (+ point index);
* :func:`interval_bounds`, :func:`failing_ranges` — report helpers.

Pure Python + numpy, mm / degrees (B-06); angle conversions only through ``awm_core.units``.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from awm_core import units
from awm_path.core.kinematics import check_rigid
from awm_path.core.ladder import NodeScore, node_cost
from awm_path.core.states import LABEL_ORDER
from awm_path.core.sweep import Candidate, PlannerParams

#: F8 Mode C coaxiality tolerance (TST-002 Rev 0.3 §5.1; D16, Assumption).
F8_TOL_MM = 0.05
F8_TOL_DEG = 0.1


def angular_step_max_deg(clearance_mm: float, margin_mm: float, r_max_mm: float) -> float:
    """B-03 positioner step bound Δθ_max = 2·(c − margin) / r_max, in degrees."""
    if not 0.0 <= margin_mm < clearance_mm:
        raise ValueError('0 <= margin < clearance is required')
    if r_max_mm <= 0.0:
        raise ValueError('r_max must be > 0')
    return float(units.rad_to_deg(2.0 * (clearance_mm - margin_mm) / r_max_mm))


def theta_grid(span_deg: float, step_max_deg: float, direction: int = 1) -> np.ndarray:
    """Uniform angles 0 … direction·span (both included) with a step ≤ ``step_max_deg``."""
    if direction not in (1, -1):
        raise ValueError('direction must be +1 or -1')
    if span_deg <= 0.0 or step_max_deg <= 0.0:
        raise ValueError('span and step must be > 0')
    n = max(1, int(math.ceil(span_deg / step_max_deg - 1e-9)))
    return direction * np.linspace(0.0, float(span_deg), n + 1)


def closing_overlap_deg(overlap_mm: float, radius_mm: float) -> float:
    """Closing overlap of a Mode C seam as a positioner angle (deg)."""
    if overlap_mm < 0.0 or radius_mm <= 0.0:
        raise ValueError('overlap >= 0 and radius > 0 are required')
    return float(units.rad_to_deg(overlap_mm / radius_mm))


def _unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError('zero direction vector')
    return v / n


def _line_distance(points: np.ndarray, axis_point: np.ndarray, axis_dir: np.ndarray) -> np.ndarray:
    d = _unit(axis_dir)
    r = np.asarray(points, dtype=float).reshape(-1, 3) - np.asarray(axis_point, dtype=float)
    return np.linalg.norm(r - np.outer(r @ d, d), axis=1)


def radial_extent_mm(points_mm: np.ndarray, axis_point: np.ndarray, axis_dir: np.ndarray) -> float:
    """Largest distance of ``points_mm`` (N, 3) from the axis line (r_max of a rotating set)."""
    return float(_line_distance(points_mm, axis_point, axis_dir).max())


def coaxiality(centre_mm: np.ndarray, seam_axis: np.ndarray, axis_point: np.ndarray, axis_dir: np.ndarray,
               tol_mm: float = F8_TOL_MM, tol_deg: float = F8_TOL_DEG) -> Dict[str, Any]:
    """F8: distance of the seam circle centre from the positioner axis line and the angle between the axes
    (direction-free). All values in the same frame."""
    off = float(_line_distance(np.asarray(centre_mm, float), axis_point, axis_dir)[0])
    c = abs(float(_unit(seam_axis) @ _unit(axis_dir)))
    ang = float(units.rad_to_deg(math.acos(min(1.0, c))))
    return {'offset_mm': off, 'angle_deg': ang, 'tol_mm': tol_mm, 'tol_deg': tol_deg,
            'pass': bool(off <= tol_mm and ang <= tol_deg)}


def rot_x_deg(theta_deg: float) -> np.ndarray:
    """4x4 rotation about X by ``theta_deg``."""
    a = units.deg_to_rad(float(theta_deg))
    c, s = math.cos(a), math.sin(a)
    t = np.eye(4)
    t[1:3, 1:3] = [[c, -s], [s, c]]
    return t


def world_T_flange_at(world_T_flange0: np.ndarray, theta_deg: float) -> np.ndarray:
    """Flange pose at angle θ: the flange turns about its own X axis (= positioner axis at θ = 0)."""
    return check_rigid(world_T_flange0) @ rot_x_deg(theta_deg)


def part_T_tcp_at(world_T_flange0: np.ndarray, flange_T_part: np.ndarray, world_T_tcp: np.ndarray,
                  theta_deg: float) -> np.ndarray:
    """TCP pose in the part frame at angle θ, with the TCP fixed in ``world`` (robot at rest)."""
    world_T_part = world_T_flange_at(world_T_flange0, theta_deg) @ check_rigid(flange_T_part)
    return np.linalg.inv(world_T_part) @ check_rigid(world_T_tcp)


def travel_sign(world_T_seam: np.ndarray, axis_point: np.ndarray, axis_dir: np.ndarray) -> int:
    """Sign of θ that moves the (world-fixed) contact point along +X of the seam frame on the part.

    A positive turn moves material at p with velocity d × (p − a); the contact point moves over the part the
    opposite way, so +θ advances along t when t · (d × (p − a)) < 0.
    """
    f = check_rigid(world_T_seam)
    t, p = f[:3, 0], f[:3, 3]
    v = float(t @ np.cross(_unit(axis_dir), p - np.asarray(axis_point, float)))
    if abs(v) < 1e-9:
        raise ValueError('seam point on the axis or tangent parallel to it: no Mode C travel direction')
    return 1 if v < 0.0 else -1


def select_pose(entries: Sequence[Tuple[int, Candidate, NodeScore]], params: PlannerParams) -> List[Dict[str, Any]]:
    """Rank valid single poses: lowest S.3 node cost (rounded 1e-9), then ψ index, then configuration-label order
    (F7), then point index. Kinematic rejects and scene-invalid nodes are not ranked."""
    rows = []
    for point, c, s in entries:
        if c.reason is not None or not s.valid:
            continue
        cost = node_cost(c, s, params)
        rows.append((round(cost, 9), c.psi_index, LABEL_ORDER[c.label], int(point), cost, c, s))
    rows.sort(key=lambda r: r[:4])
    return [{'rank': k + 1, 'point': r[3], 'psi_index': r[1], 'psi_deg': r[5].psi_deg, 'label': r[5].label,
             'q_deg': list(r[5].q_deg), 'cost': r[4], 'clearance_lb_mm': r[6].clearance_mm, 'candidate': r[5]}
            for k, r in enumerate(rows)]


def facing_points(world_T_seams: Sequence[np.ndarray], target_mm: np.ndarray, min_cos: float
                  ) -> Tuple[List[int], List[float]]:
    """Indices of the seam frames whose Z (out of the material) points towards ``target_mm`` with
    cos ≥ ``min_cos``, and the cosine of every frame (Engineer choice of the Mode C contact location, data)."""
    cos = []
    for f in world_T_seams:
        f = check_rigid(f)
        cos.append(float(f[:3, 2] @ _unit(np.asarray(target_mm, float) - f[:3, 3])))
    return [i for i, c in enumerate(cos) if c >= min_cos], cos


def interval_bounds(lb_mm: Sequence[Optional[float]], step_deg: float, r_max_mm: float) -> List[Optional[float]]:
    """Clearance guaranteed between consecutive checked angles: min(lb_k, lb_k+1) − r_max·Δθ / 2 (None if either
    end has no clearance level passed)."""
    half = r_max_mm * units.deg_to_rad(abs(float(step_deg))) / 2.0
    return [None if a is None or b is None else float(min(a, b) - half) for a, b in zip(lb_mm[:-1], lb_mm[1:])]


def failing_ranges(thetas: Sequence[float], ok: Sequence[bool]) -> List[Dict[str, Any]]:
    """Runs of consecutive failing angles: first / last failing θ and the number of states."""
    out: List[Dict[str, Any]] = []
    run: Optional[List[int]] = None
    for i, good in enumerate(ok):
        if not good:
            run = [i, i] if run is None else [run[0], i]
        elif run is not None:
            out.append(run)
            run = None
    if run is not None:
        out.append(run)
    return [{'theta_from_deg': float(thetas[a]), 'theta_to_deg': float(thetas[b]), 'states': b - a + 1}
            for a, b in out]


__all__ = ['F8_TOL_MM', 'F8_TOL_DEG', 'angular_step_max_deg', 'theta_grid', 'closing_overlap_deg',
           'radial_extent_mm', 'coaxiality', 'rot_x_deg', 'world_T_flange_at', 'part_T_tcp_at', 'travel_sign',
           'select_pose', 'facing_points', 'interval_bounds', 'failing_ranges']
