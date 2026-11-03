"""Tool pose per seam point and tool yaw ψ, and the kinematic candidate sweep (WP-S S.3; TST-002 Rev 0.3 §3.3,
§4, §5.1 F1, F2, F6, F11; DES-001 Rev 0.6 draft §4.3 steps 1 and 3).

Public values mm / degrees. Frames are 4x4 transforms ``base_link_T_seam`` (mm) sampled by the caller (A41). The scene
(validity, clearance, contact) is not evaluated here; S.4 adds it as extra columns / node scores.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from awm_core import units
from awm_path.core.kinematics import KinematicsBackend, check_rigid, ik_tcp, pose_error
from awm_path.core.states import (LABEL_ORDER, SingularityMetrics, SingularityThresholds,
                                  ik_existence, label_and_metrics, singular_reason, unwrapped_variants)

#: Candidate rejection reasons of the kinematic sweep (``None`` = kinematically valid).
REASONS = ('no_ik_unreachable', 'no_ik_near_singular', 'joint_limit',
           'singular_wrist', 'singular_elbow', 'singular_shoulder')

#: UR5e joint limits (deg) from ``ur_description`` ``config/ur5e/joint_limits.yaml`` (elbow limited to ±180°).
UR5E_JOINT_LIMITS_DEG: Tuple[Tuple[float, float], ...] = (
    (-360.0, 360.0), (-360.0, 360.0), (-180.0, 180.0), (-360.0, 360.0), (-360.0, 360.0), (-360.0, 360.0))


def _psi_range(start: float, stop: float, step: float) -> Tuple[float, ...]:
    n = int(round((stop - start) / step))
    return tuple(float(start + i * step) for i in range(n + 1))


@dataclass(frozen=True)
class PlannerParams:
    """Planner parameters (TST-002 Rev 0.3 §5.1 defaults; D16 ``cell.yaml`` supplies them later, T0.1).

    Assumptions (marked in TST-002 or chosen here, recorded with every result):
    * F6 elbow 5° and shoulder 50 mm (TST-002: Assumption);
    * Ψ-WIRE centre 0° (wire-leading direction unknown, I5) ± 15° in 5° steps;
    * cost weights and ``clearance_target_mm`` (not defined by TST-002; they rank feasible paths only and never make
      an infeasible path feasible).
    """

    pitch_max_mm: float = 3.0
    dq_max_deg: float = 5.0
    joint_limits_deg: Tuple[Tuple[float, float], ...] = UR5E_JOINT_LIMITS_DEG
    singular_wrist_deg: float = 8.6
    singular_elbow_deg: float = 5.0
    singular_shoulder_mm: float = 50.0
    wrist3_abs_deg: float = 270.0
    wrist3_travel_deg: float = 180.0
    psi_free: Tuple[float, ...] = _psi_range(0.0, 350.0, 10.0)
    psi_wire: Tuple[float, ...] = _psi_range(-15.0, 15.0, 5.0)
    psi_nominal_deg: Optional[float] = None
    w_dq: float = 1.0
    w_dev: float = 0.1
    w_clear: float = 1.0
    w_sing: float = 1.0
    w_wrist3: float = 0.5
    clearance_target_mm: float = 10.0

    @property
    def singularity_thresholds(self) -> SingularityThresholds:
        return SingularityThresholds(self.singular_wrist_deg, self.singular_elbow_deg, self.singular_shoulder_mm)

    def effective_limits_deg(self) -> Tuple[Tuple[float, float], ...]:
        """Joint limits with the wrist-3 absolute soft limit (F11) applied to joint 6."""
        lims = list(self.joint_limits_deg)
        lo, hi = lims[5]
        lims[5] = (max(lo, -self.wrist3_abs_deg), min(hi, self.wrist3_abs_deg))
        return tuple(lims)


def _rot_x_deg(deg: float) -> np.ndarray:
    c, s = math.cos(units.deg_to_rad(deg)), math.sin(units.deg_to_rad(deg))
    t = np.eye(4)
    t[1:3, 1:3] = [[c, -s], [s, c]]
    return t


def _rot_z_deg(deg: float) -> np.ndarray:
    c, s = math.cos(units.deg_to_rad(deg)), math.sin(units.deg_to_rad(deg))
    t = np.eye(4)
    t[0:2, 0:2] = [[c, -s], [s, c]]
    return t


_RX180 = np.diag([1.0, -1.0, -1.0, 1.0])


def tcp_pose(seam_frame_mm: np.ndarray, psi_deg: float) -> np.ndarray:
    """TCP pose ``seam_frame · Rx(180°) · Rz(ψ)``: tool Z = beam axis = −seam Z; ψ = rotation about the beam axis,
    ψ = 0 ↔ tool X = seam X. Work and travel angle 0 (TST-002 §2: nominal); focus offset 0 (§3.1)."""
    return check_rigid(seam_frame_mm) @ _RX180 @ _rot_z_deg(psi_deg)


@dataclass(frozen=True)
class PitchCheck:
    """F1 check of sampled frames: per step chord length, frame rotation, and the pitch limit."""

    step_mm: Tuple[float, ...]
    rotation_deg: Tuple[float, ...]
    limit_mm: Tuple[float, ...]
    ok: bool
    failing_steps: Tuple[int, ...]


def pitch_points(frames_mm: Sequence[np.ndarray], params: PlannerParams, closed: bool) -> PitchCheck:
    """F1: Δs ≤ min(pitch_max, Δq_max / rate) for every step, rate = frame rotation / Δs (deg / mm).

    ``closed`` adds the step from the last frame back to the first.
    """
    fr = [check_rigid(f) for f in frames_mm]
    pairs = list(zip(fr[:-1], fr[1:])) + ([(fr[-1], fr[0])] if closed and len(fr) > 1 else [])
    steps, rots, lims, bad = [], [], [], []
    for i, (a, b) in enumerate(pairs):
        ds, dr_rad = pose_error(a, b)
        dr = units.rad_to_deg(dr_rad)
        lim = params.pitch_max_mm if dr <= 0.0 else min(params.pitch_max_mm, params.dq_max_deg * ds / dr)
        steps.append(ds)
        rots.append(dr)
        lims.append(lim)
        if ds > lim + 1e-9:
            bad.append(i)
    return PitchCheck(tuple(steps), tuple(rots), tuple(lims), not bad, tuple(bad))


def required_points(length_mm: float, rate_deg_per_mm: float, params: PlannerParams, closed: bool) -> int:
    """Smallest number of points so that the pitch obeys F1 on a seam of constant rotation rate."""
    pitch = params.pitch_max_mm if rate_deg_per_mm <= 0.0 else min(params.pitch_max_mm,
                                                                    params.dq_max_deg / rate_deg_per_mm)
    n_steps = max(1, math.ceil(length_mm / pitch - 1e-9))
    return n_steps if closed else n_steps + 1


@dataclass(frozen=True)
class Candidate:
    """One kinematic candidate: (point, ψ, IK solution). ``q_deg`` in (-180, 180] (unwrapping happens in the DP).

    ``reason`` None = kinematically valid. For ``no_ik_*`` there is one candidate per (point, ψ) with no joint values.
    """

    point: int
    psi_index: int
    psi_deg: float
    label: Optional[str]
    q_deg: Optional[Tuple[float, ...]]
    metrics: Optional[SingularityMetrics]
    reason: Optional[str]


def sweep(frames_mm: Sequence[np.ndarray], tool0_T_tcp: np.ndarray, kin: KinematicsBackend,
          params: PlannerParams, psi_values: Sequence[float]) -> List[List[Candidate]]:
    """Return one layer of candidates per seam point: every ψ × every IK solution with its label and reason.

    Order inside a layer: ψ index, then label order, then joint values (deterministic).
    """
    limits = params.effective_limits_deg()
    thresholds = params.singularity_thresholds
    tool = check_rigid(tool0_T_tcp)
    tool_inv = np.linalg.inv(tool)
    layers: List[List[Candidate]] = []
    for i, frame in enumerate(frames_mm):
        layer: List[Candidate] = []
        for j, psi in enumerate(psi_values):
            target = tcp_pose(frame, float(psi))
            sols = ik_tcp(kin, target, tool)
            if not sols:
                cls = ik_existence(target @ tool_inv)
                reason = 'no_ik_unreachable' if cls == 'unreachable' else 'no_ik_near_singular'
                layer.append(Candidate(i, j, float(psi), None, None, None, reason))
                continue
            entries = []
            for q in sols:
                label, metrics = label_and_metrics(q)
                entries.append((LABEL_ORDER[label], tuple(float(x) for x in q), label, metrics))
            entries.sort(key=lambda e: e[:2])
            for _, q, label, metrics in entries:
                if not unwrapped_variants(q, limits):
                    reason: Optional[str] = 'joint_limit'
                else:
                    reason = singular_reason(metrics, thresholds)
                layer.append(Candidate(i, j, float(psi), label, q, metrics, reason))
        layers.append(layer)
    return layers


CSV_COLUMNS = ('point', 'psi_index', 'psi_deg', 'label', 'q1_deg', 'q2_deg', 'q3_deg', 'q4_deg', 'q5_deg', 'q6_deg',
               'sin_q5', 'sin_q3', 'wrist_centre_mm', 'reason')


def _fmt(x: object) -> str:
    if x is None:
        return ''
    if isinstance(x, bool):
        return 'true' if x else 'false'
    if isinstance(x, float):
        return f'{x:.6f}'
    return str(x)


def write_candidates_csv(path: str, layers: Sequence[Sequence[Candidate]],
                         extra_columns: Optional[Mapping[str, Sequence[Sequence[object]]]] = None) -> None:
    """Write ``candidates.csv`` (TST-002 §4): one row per candidate, kinematic columns plus ``extra_columns``.

    ``extra_columns`` maps a column name to values with the same nested shape as ``layers`` (S.4 fills the scene
    columns: validity, clearance, closest pair, contact). Floats with 6 decimals; byte-identical for equal inputs.
    """
    extra: Dict[str, Sequence[Sequence[object]]] = dict(extra_columns or {})
    for name, vals in extra.items():
        if len(vals) != len(layers) or any(len(v) != len(l) for v, l in zip(vals, layers)):
            raise ValueError(f'extra column {name!r} does not match the candidate layers')
    with open(path, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh, lineterminator='\n')
        w.writerow(list(CSV_COLUMNS) + list(extra))
        for li, layer in enumerate(layers):
            for ci, c in enumerate(layer):
                q = list(c.q_deg) if c.q_deg is not None else [None] * 6
                m = c.metrics
                row = [c.point, c.psi_index, c.psi_deg, c.label] + q + [
                    m.sin_q5 if m else None, m.sin_q3 if m else None, m.wrist_centre_mm if m else None, c.reason]
                row += [extra[name][li][ci] for name in extra]
                w.writerow([_fmt(x) for x in row])


__all__ = ['REASONS', 'UR5E_JOINT_LIMITS_DEG', 'PlannerParams', 'tcp_pose', 'PitchCheck', 'pitch_points',
           'required_points', 'Candidate', 'sweep', 'CSV_COLUMNS', 'write_candidates_csv']
