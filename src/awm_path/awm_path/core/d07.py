"""Pure D07 v2 builder (MINOR 2.1) for the kinematic 7-axis planner (G1.3; CP-58, CP-59; DES-001 Rev 0.5 §5.1 D07).

Input: the arc results of :mod:`awm_path.core.sync`, the cell values, ``flange_T_part`` (from ``seam_axis.json``),
planner info and the header / context records made by the caller. Output: a ``v2/path`` document (validated by the
caller with ``awm_core.jsonio``):

* accepted arcs in ``seams``: per point the ``tcp_laser`` pose in the **part** frame
  (``part_T_tcp = flange_T_part⁻¹ · F · Rx(180°) · Rz(ψ)``, F = seam frame in ``positioner_flange`` at θ = 0),
  θ, unwrapped q (6), ``status: ok`` and ``min_clearance_mm: null``; one timed segment at the travel speed relative
  to the part (``weld``; Mode C ``positioner_weld``) and the per-arc planning record;
* rejected arcs in ``rejected`` with failing points and failing s ranges (never a partial path);
* ``collision_checked: false`` and ``scene_backend: none …`` — collision is not evaluated in Goal 1 (CP-58).

No clock, no randomness, no file access: same inputs give the same document (F7). Pure Python + numpy (B-06).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Dict, List, Mapping, Sequence

import numpy as np

from awm_core import frames, units
from awm_core.hashing import sha256_text
from awm_path.core.sync import ArcResult, SyncCell, timing

SCHEMA_VERSION = '2.1'
SCENE_BACKEND_OFF = 'none (collision not evaluated, Goal 1 / CP-58)'
#: CLI head model → D07 ``planner.head_model`` (contract T0.5: cad | box)
HEAD_MODEL_RECORD = {'hcad': 'cad', 'hbox': 'box'}
_RX180 = np.diag([1.0, -1.0, -1.0, 1.0])


@dataclass(frozen=True)
class ArcPlan:
    """One planned arc with the process values used to time it."""

    result: ArcResult
    travel_speed_mm_s: float
    overlap_mm: float = 0.0


@dataclass(frozen=True)
class PlannerInfo:
    kinematics_backend: str
    head_model: str            # hcad | hbox
    params: Mapping
    psi_case: str = 'free'


def params_sha256(params: Mapping) -> str:
    """SHA-256 of the canonical JSON of the planner parameters."""
    return sha256_text(json.dumps(params, sort_keys=True, separators=(',', ':'), allow_nan=False))


def part_tcp_pose(flange_t_part: np.ndarray, frame_flange: np.ndarray, psi_deg: float) -> np.ndarray:
    """``part_T_tcp``: seam frame (flange, θ = 0) brought to the part frame, beam = −seam Z, yaw ψ about the beam."""
    return frames.invert(flange_t_part) @ frame_flange @ _RX180 @ _rot_z(psi_deg)


def _rot_z(psi_deg: float) -> np.ndarray:
    a = units.deg_to_rad(float(psi_deg))
    t = np.eye(4)
    t[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    return t


def _f(v: float) -> float:
    return float(round(float(v), 9))


def _pose(t: np.ndarray) -> Dict:
    q = frames.matrix_to_quat(t[:3, :3])
    return {'position_mm': [_f(v) for v in t[:3, 3]], 'quat_xyzw': [_f(v) for v in q]}


def _arc(res: ArcResult) -> Dict:
    return {'index': int(res.arc.index), 'of': int(res.arc.of), 'start_s_mm': _f(res.arc.s_start_mm),
            'end_s_mm': _f(res.arc.s_end_mm)}


_PLANNING_KEYS = ('pitch_mm', 'points', 'interpolated_points', 'theta_min_deg', 'theta_max_deg', 'theta_travel_deg',
                  'wrist3_travel_deg', 'max_step_deg', 'fk_max_error_mm', 'fk_max_error_deg', 'seam_z_up_max_deg',
                  'closure', 'cost', 'mode_c_residual_mm', 'mode_c_residual_deg')


def _jsonable(v):
    if isinstance(v, dict):
        return {k: _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, (bool, np.bool_)):
        return bool(v)
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        return _f(v)
    return v


def _accepted(plan: ArcPlan, cell: SyncCell, flange_t_part: np.ndarray) -> Dict:
    res = plan.result
    pts = []
    for p in res.points:
        pts.append({'s_mm': _f(p.s_mm), 'pose_mm': _pose(part_tcp_pose(flange_t_part, p.frame_flange, p.psi_deg)),
                    'psi_deg': _f(p.psi_deg), 'theta_deg': _f(p.theta_deg), 'q_deg': [_f(v) for v in p.q_deg],
                    'min_clearance_mm': None, 'status': p.status})
    t, speed = timing(res, plan.travel_speed_mm_s, cell)
    kind = 'positioner_weld' if res.mode == 'C' else 'weld'
    timed = [{'kind': kind, 'points': [{'t_s': _f(ti), 'q_deg': [_f(v) for v in p.q_deg],
                                        'theta_deg': _f(p.theta_deg)} for ti, p in zip(t, res.points)]}]
    planning = {k: _jsonable(res.planning[k]) for k in _PLANNING_KEYS if k in res.planning}
    planning.update(_jsonable(speed))
    planning['attempts'] = _jsonable(res.attempts)
    return {'seam_id': res.seam_id, 'arc': _arc(res), 'mode': res.mode, 'config_label': res.points[0].label,
            'travel_speed_mm_s': _f(plan.travel_speed_mm_s), 'overlap_mm': _f(plan.overlap_mm), 'points': pts,
            'timed': timed, 'planning': planning}


def _rejected(plan: ArcPlan) -> Dict:
    res = plan.result
    return {'seam_id': res.seam_id, 'arc': _arc(res), 'mode': res.mode, 'reason': res.reason,
            'coverage_fraction': _f(min(1.0, max(0.0, res.coverage_fraction))),
            'failing_points': _jsonable(res.failing_points), 'pitch_mm': _f(res.pitch_mm),
            'failing_ranges': _jsonable(res.failing_ranges), 'attempts': _jsonable(res.attempts)}


def build_path_doc(plans: Sequence[ArcPlan], cell: SyncCell, flange_t_part: np.ndarray, info: PlannerInfo,
                   header: Mapping, context: Mapping) -> Dict:
    """D07 v2 (MINOR 2.1) document. ``header``: created_by, created_at, inputs, run_id; ``context``: context_record."""
    if info.head_model not in HEAD_MODEL_RECORD:
        raise ValueError(f'unknown head model {info.head_model!r}')
    params = _jsonable(dict(info.params))
    seams: List[Dict] = []
    rejected: List[Dict] = []
    for plan in plans:
        (seams if plan.result.ok else rejected).append(
            _accepted(plan, cell, np.asarray(flange_t_part, float)) if plan.result.ok else _rejected(plan))
    return {
        'schema_version': SCHEMA_VERSION, 'created_by': header['created_by'], 'created_at': header['created_at'],
        'inputs': [dict(i) for i in header['inputs']], 'run_id': header['run_id'], 'context': dict(context),
        'frame': 'part', 'tcp': 'tcp_laser',
        'planner': {'kinematics_backend': info.kinematics_backend, 'scene_backend': SCENE_BACKEND_OFF,
                    'params_sha256': params_sha256(params), 'psi_case': info.psi_case,
                    'head_model': HEAD_MODEL_RECORD[info.head_model], 'params': params},
        'collision_checked': False,
        'seams': seams, 'rejected': rejected,
    }


__all__ = ['SCHEMA_VERSION', 'SCENE_BACKEND_OFF', 'HEAD_MODEL_RECORD', 'ArcPlan', 'PlannerInfo', 'params_sha256',
           'part_tcp_pose', 'build_path_doc']
