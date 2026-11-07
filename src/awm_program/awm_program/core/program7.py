"""Synchronised 7-joint program D09 (v2 MINOR 2.1) from the kinematic 7-axis path D07 (G1.5; A61–A64;
DES-001 Rev 0.5 §4.3 steps 5–6, §5.3, §6.2; CP-58, CP-59 incl. the follow-up decisions of 2026-11-06).

Per accepted D07 arc (seam order, then arc index), every segment one ``robot_positioner`` trajectory
(6 robot joints then the positioner joint, unwrapped degrees):

1. ``air`` (or ``index`` when only θ moves) from home / the previous retract end: joint-interpolated, points every
   ≤ ``dense_step_rad`` — **not planned around obstacles and not collision-checked** (``collision_checked: false``);
2. ``approach``: TCP on a straight line from ``approach_mm`` along the seam +Z (= against the beam, tool −Z) to the
   weld start, θ held at the weld start, same IK configuration label as the weld (``ur-analytic-ik`` through
   ``awm_path.core.batch_kin``), |Δq| ≤ Δq_max of the D07 run per step, joint limits, wrist-3 ±270°, F6 margins;
3. ``weld`` (Mode C: ``positioner_weld``) with ``positioner_mode``: the D07 joint states (robot q and θ) unchanged,
   timed along the D07 arc length s at the travel speed **relative to the part** (A47) with a speed ramp at both
   ends (:func:`awm_path.core.weld_timing.weld_times`, the planner's model); WeldStart before it and WeldEnd
   after it, both while robot and positioner are at rest (segment boundaries, velocity 0);
4. ``retract`` like the approach, away from the weld end; finally ``air`` back to home.

A61 frame chain (part frame → base_link with θ): ``base_link_T_tcp = base_link_T_axis · Rx(θ) · flange_T_part ·
part_T_tcp`` (the positioner axis is +X of ``positioner_axis``; DES-001 §5.3 writes it as Rz with the axis as Z).
The generator checks that FK(q) · tool0_T_tcp reproduces every D07 weld pose through this chain (sync / B: the D07
tolerance ``fk_tol``; Mode C: the F8 tolerance ``mode_c_tol``, decision 2026-11-06).

One weld timing model (G1.12, decision Truong 2026-11-07): the weld spline, its limit check and the largest feasible
travel speed are those of the planner (``awm_path.core.weld_timing``), with the weld ramp and the speed resolution
recorded in D07 ``planner.params.sync``. The program refuses a D07 whose values differ from :class:`ProgramParams`
(or that has none: planned before G1.9, re-plan) instead of adopting them silently — the parameters are recorded in
every D09 as the ones the program used, and a different ramp would make the D07 speed verdict invalid.

Refusals (:class:`ProgramRefused`, nothing written): a rejected arc (no partial programs, DES-001 rule 4), an arc
whose D07 travel speed is above its feasible maximum (``speed_within_limits: false``), a D07 weld ramp / speed
resolution other than the program's or missing, a weld segment whose spline
breaks a joint / positioner velocity or acceleration limit (the largest feasible travel speed is reported; the
weld is **never slowed down silently** and **no limit is raised**), an approach / retract that is not reachable on
the weld IK branch, a D07 pose that FK does not reproduce.

Assumptions (parameters, recorded in the program ``generator`` record): approach / retract 40 mm, weld ramp
acceleration 5 mm/s² (the ramp lies on the seam after WeldStart / before WeldEnd — events stay at rest), air /
approach / retract at 25 % of the joint velocity and acceleration limits.

Pure Python + numpy (B-06): no rclpy, no file access. mm / degrees / seconds; radians only through
``awm_core.units``.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from typing import Callable, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from awm_core import frames, units
from awm_core.hashing import sha256_text
from awm_path.core.batch_kin import batch_fk, ik_by_label
from awm_path.core.kinematics import pose_error
from awm_path.core.states import (LABELS, SingularityThresholds, nearest_representative, singular_reason,
                                  singularity_metrics)
from awm_path.core import weld_timing
from awm_path.core.sync import SyncCell, rot_x
from awm_path.core.weld_timing import TimingError
from awm_program.core.spline import rest_to_rest

SCHEMA_VERSION = '2.1'
WELD_KINDS = ('weld', 'positioner_weld')
ProgressFn = Optional[Callable[[float, str], None]]


class ProgramRefused(Exception):
    """The program cannot be generated; the message names the arc and the reason. Nothing is written."""


@dataclass(frozen=True)
class ProgramParams:
    """Generator parameters (D16 later). All values are Assumptions, recorded with every program."""

    approach_mm: float = 40.0
    retract_mm: float = 40.0
    line_step_mm: float = 1.0
    line_step_floor_mm: float = 0.05
    weld_ramp_accel_mm_s2: float = 5.0
    motion_velocity_scaling: float = 0.25
    motion_acceleration_scaling: float = 0.25
    dense_step_rad: float = 0.02
    speed_search_resolution_mm_s: float = 0.01

    def __post_init__(self) -> None:
        if not 30.0 <= self.approach_mm <= 50.0 or not 30.0 <= self.retract_mm <= 50.0:
            raise ValueError('approach / retract must be 30–50 mm along the beam axis (DES-001 §4.3 step 6)')
        for k in ('motion_velocity_scaling', 'motion_acceleration_scaling'):
            if not 0.0 < getattr(self, k) <= 1.0:
                raise ValueError(f'{k} must be in (0, 1]: limits are never raised')
        if self.weld_ramp_accel_mm_s2 <= 0.0 or self.line_step_mm <= 0.0 or self.dense_step_rad <= 0.0:
            raise ValueError('ramp acceleration, line step and dense step must be > 0')

    def record(self) -> Dict:
        return asdict(self)


@dataclass(frozen=True)
class ProgramCell:
    """Cell values from the robot description (the same ones the planner used) plus the acceleration limits and
    home. ``velocity_deg_s`` / ``acceleration_deg_s2``: 7 values, robot joints then the positioner."""

    sync: SyncCell
    acceleration_deg_s2: Tuple[float, ...]
    home_deg: Tuple[float, ...]
    robot_joints: Tuple[str, ...]
    positioner_joints: Tuple[str, ...]

    def __post_init__(self) -> None:
        # one source for the limits (G1.12): the acceleration limits are the planner cell's (SyncCell, G1.9)
        if not np.array_equal(np.asarray(self.acceleration_deg_s2, float), self.sync.acceleration_limits_deg_s2):
            raise ValueError(f'program acceleration limits {list(self.acceleration_deg_s2)} differ from the planner '
                             f'cell acceleration limits {self.sync.acceleration_limits_deg_s2.tolist()} deg/s²')

    @property
    def velocity_deg_s(self) -> np.ndarray:
        return np.array(list(self.sync.joint_velocity_deg_s) + [self.sync.theta_velocity_deg_s], float)

    @property
    def accel_deg_s2(self) -> np.ndarray:
        return np.asarray(self.acceleration_deg_s2, float)


@dataclass(frozen=True)
class D07Tolerances:
    """Values of the D07 planning run (``planner.params.sync``) the program must keep."""

    dq_max_deg: float
    fk_tol_mm: float
    fk_tol_deg: float
    mode_c_tol_mm: float
    mode_c_tol_deg: float
    wrist3_abs_deg: float
    singular: SingularityThresholds

    @classmethod
    def from_path(cls, path_doc: Mapping) -> 'D07Tolerances':
        try:
            p = path_doc['planner']['params']['sync']
            return cls(float(p['dq_max_deg']), float(p['fk_tol_mm']), float(p['fk_tol_deg']), float(p['mode_c_tol_mm']),
                       float(p['mode_c_tol_deg']), float(p['wrist3_abs_deg']),
                       SingularityThresholds(float(p['singular_wrist_deg']), float(p['singular_elbow_deg']),
                                             float(p['singular_shoulder_mm'])))
        except (KeyError, TypeError) as exc:
            raise ProgramRefused(f'D07 has no planner.params.sync record ({exc}): not a G1.3 kinematic 7-axis path; '
                                 're-plan with awm_path.cli plan') from exc


WELD_TIMING_KEYS = ('weld_ramp_accel_mm_s2', 'speed_search_resolution_mm_s')


def check_weld_timing_params(path_doc: Mapping, params: 'ProgramParams') -> None:
    """G1.12: the weld ramp and speed resolution of D07 ``planner.params.sync`` (with which the planner found
    ``speed_within_limits`` / ``max_travel_speed_mm_s``) must equal the program's; refuses a D07 without them."""
    sync = path_doc['planner']['params']['sync']
    for key in WELD_TIMING_KEYS:
        if key not in sync:
            raise ProgramRefused(f'D07 planner.params.sync has no {key}: older D07 (planned before G1.9, maximum travel '
                                 'speed from the velocity limits only); re-plan with awm_path.cli plan')
        d07, prog = float(sync[key]), float(getattr(params, key))
        if d07 != prog:
            raise ProgramRefused(f'D07 was planned with {key} = {d07} but the program uses {prog}: the D07 travel speed '
                                 'check and maximum do not hold for this program; re-plan with the program value')


# ---------------------------------------------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------------------------------------------

def _f(v: float) -> float:
    return float(round(float(v), 9))


def _trans_z(d_mm: float) -> np.ndarray:
    t = np.eye(4)
    t[2, 3] = d_mm
    return t


def pose_matrix(pose: Mapping) -> np.ndarray:
    """``{position_mm, quat_xyzw}`` → 4x4 (mm)."""
    return frames.make_transform(pose['position_mm'], frames.quat_to_matrix(pose['quat_xyzw']))


def part_to_base_link(cell: SyncCell, flange_t_part: np.ndarray, theta_deg: float) -> np.ndarray:
    """A61: ``base_link_T_part`` with the positioner at θ."""
    return cell.base_link_T_axis @ rot_x(theta_deg) @ flange_t_part


def tcp_in_part(cell: SyncCell, flange_t_part: np.ndarray, q7_deg: np.ndarray) -> np.ndarray:
    """``part_T_tcp`` (n, 4, 4) of 7-joint states (robot q then θ): FK of the robot through the cell chain."""
    q7 = np.asarray(q7_deg, float).reshape(-1, 7)
    fk = batch_fk(q7[:, :6]) @ cell.tool0_T_tcp
    return np.array([np.linalg.inv(part_to_base_link(cell, flange_t_part, th)) @ f for th, f in zip(q7[:, 6], fk)])


def params_sha256(params: Mapping) -> str:
    return sha256_text(json.dumps(params, sort_keys=True, separators=(',', ':'), allow_nan=False))


def _arc_name(arc: Mapping) -> str:
    return f'{arc["seam_id"]} arc {arc["arc"]["index"]}/{arc["arc"]["of"]}'


# ---------------------------------------------------------------------------------------------------------------
# approach / retract on the weld IK branch
# ---------------------------------------------------------------------------------------------------------------

def line_states(kin, cell: SyncCell, tcp_end: np.ndarray, q_end: np.ndarray, label: str, dist_mm: float,
                tol: D07Tolerances, params: ProgramParams, what: str) -> np.ndarray:
    """Robot states (k, 6) for the TCP on the line ``tcp_end · Tz(−d)``, d = ``dist_mm`` … 0 (tool −Z = seam +Z,
    away from the material), on configuration ``label``, unwrapped continuously into ``q_end`` (last row = q_end
    exactly). Refuses when a state is missing on that label, breaks a limit / F6 margin or a step > Δq_max."""
    lab = LABELS.index(label)
    tool_inv = np.linalg.inv(cell.tool0_T_tcp)
    lims = [list(v) for v in cell.joint_limits_deg]
    lims[5] = [max(lims[5][0], -tol.wrist3_abs_deg), min(lims[5][1], tol.wrist3_abs_deg)]
    step = params.line_step_mm
    reason = ''
    while step >= params.line_step_floor_mm - 1e-12:
        n = int(math.ceil(dist_mm / step - 1e-9))
        ds = np.linspace(0.0, dist_mm, n + 1)
        targets = np.array([tcp_end @ _trans_z(-d) @ tool_inv for d in ds])
        q, ok = ik_by_label(kin, targets)
        if not ok[:, lab].all():
            k = int(np.nonzero(~ok[:, lab])[0][0])
            raise ProgramRefused(f'{what}: no IK solution on the weld configuration {label} at {ds[k]:.2f} mm from the '
                                 'seam (the approach / retract line leaves the reachable space of this branch)')
        out = [np.asarray(q_end, float)]
        if np.abs(nearest_representative(q[0, lab], q_end) - q_end).max() > 1e-6:
            raise ProgramRefused(f'{what}: IK at the seam point does not reproduce the D07 joint state')
        for k in range(1, len(ds)):
            out.append(nearest_representative(q[k, lab], out[-1]))
        u = np.array(out)
        steps = np.abs(np.diff(u, axis=0)).max(axis=1)
        if steps.max() > tol.dq_max_deg + 1e-9:
            reason = f'joint step {steps.max():.2f}° > {tol.dq_max_deg}° at line step {step:g} mm'
            step /= 2.0
            continue
        for k, qk in enumerate(u):
            for j, (lo, hi) in enumerate(lims):
                if not lo - 1e-9 <= qk[j] <= hi + 1e-9:
                    raise ProgramRefused(f'{what}: joint {j + 1} = {qk[j]:.2f}° outside [{lo}, {hi}]° at {ds[k]:.2f} mm '
                                         'from the seam')
            why = singular_reason(singularity_metrics(qk), tol.singular)
            if why:
                raise ProgramRefused(f'{what}: {why} at {ds[k]:.2f} mm from the seam (F6 margins of the D07 run)')
        u[0] = q_end
        return u[::-1].copy()
    raise ProgramRefused(f'{what}: no continuous joint path on {label} ({reason})')


def joint_line(a: np.ndarray, b: np.ndarray, step_deg: float) -> np.ndarray:
    """Joint-interpolated states from a to b (7 joints), every joint step ≤ ``step_deg``."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(1, int(math.ceil(np.abs(b - a).max() / step_deg - 1e-12)))
    return a + (b - a) * np.linspace(0.0, 1.0, n + 1)[:, None]


# ---------------------------------------------------------------------------------------------------------------
# weld segment
# ---------------------------------------------------------------------------------------------------------------

def weld_spline(s: np.ndarray, q7: np.ndarray, speed: float, params: ProgramParams):
    """``(t, v, a, vmax, amax, ramp_mm)`` of the weld spline at ``speed`` — the planner's model
    (:mod:`awm_path.core.weld_timing`): trapezoidal arc-length timing with the weld ramp, clamped C2 cubic, exact
    maxima."""
    t, ramp = weld_timing.weld_times(s, speed, params.weld_ramp_accel_mm_s2)
    v, a = weld_timing.clamped_cubic(t, q7)
    vmax, amax = weld_timing.max_abs(t, v, a)
    return t, v, a, vmax, amax, ramp


def speed_within_limits(s: np.ndarray, q7: np.ndarray, speed: float, cell: ProgramCell, params: ProgramParams) -> bool:
    """The planner's verdict (:func:`awm_path.core.weld_timing.within_limits`) for the weld at ``speed``."""
    return weld_timing.within_limits(s, q7, speed, cell.velocity_deg_s, cell.accel_deg_s2,
                                     params.weld_ramp_accel_mm_s2)


def max_feasible_speed(s: np.ndarray, q7: np.ndarray, speed: float, cell: ProgramCell, params: ProgramParams
                       ) -> float:
    """Largest feasible travel speed (velocity **and** acceleration limits, resolution
    ``speed_search_resolution_mm_s``, rounded down) — the planner's :func:`awm_path.core.weld_timing.max_feasible_speed`
    called exactly as the D07 speed record calls it (``speed`` passed as the request only when it is feasible), so the
    value equals D07 ``max_travel_speed_mm_s`` for the same points."""
    ok = speed_within_limits(s, q7, speed, cell, params)
    return weld_timing.max_feasible_speed(s, q7, cell.velocity_deg_s, cell.accel_deg_s2, params.weld_ramp_accel_mm_s2,
                                          params.speed_search_resolution_mm_s, float(speed) if ok else None)


#: Advice of every travel-speed refusal. The maximum can lie above a refused request (feasibility is not monotonic in
#: the speed, :mod:`awm_path.core.weld_timing`), so "a lower speed" would be wrong advice (review 1.a, G1.12).
SPEED_ADVICE = (' — the travel speed is an Engineer input: re-plan with a travel speed ≤ the maximum (feasibility is not '
                'monotonic in the travel speed, so not every lower speed is feasible; the software never slows a weld '
                'down and never raises a limit)')


def weld_speed_refusal(name: str, s: np.ndarray, q7: np.ndarray, speed: float, cell: ProgramCell,
                       params: ProgramParams, d07_max: float) -> str:
    """Refusal text of one weld whose spline at ``speed`` breaks a limit: the limits broken, the largest feasible
    travel speed (same model as D07) and the D07 record (``d07_max``). Join with :data:`SPEED_ADVICE`."""
    _, _, _, vmax, amax, _ = weld_spline(s, q7, speed, params)
    vmx = max_feasible_speed(s, q7, speed, cell, params)
    return (f'{name}: at travel speed {speed} mm/s the weld motion breaks {_limit_text(vmax, amax, cell)}; '
            f'max feasible travel speed {vmx:.2f} mm/s with velocity and acceleration limits '
            f'(D07 max_travel_speed_mm_s {d07_max:.2f}, same timing model awm_path.core.weld_timing; D07 said within '
            'limits, so its speed record does not match this path or these limits)')


def _limit_text(vmax, amax, cell: ProgramCell) -> str:
    names = list(cell.robot_joints) + list(cell.positioner_joints)
    out = []
    for j, n in enumerate(names):
        if vmax[j] > cell.velocity_deg_s[j] * (1 + 1e-9):
            out.append(f'{n} velocity {vmax[j]:.1f} > {cell.velocity_deg_s[j]:.1f} deg/s')
        if amax[j] > cell.accel_deg_s2[j] * (1 + 1e-9):
            out.append(f'{n} acceleration {amax[j]:.1f} > {cell.accel_deg_s2[j]:.1f} deg/s²')
    return ', '.join(out)


# ---------------------------------------------------------------------------------------------------------------
# program
# ---------------------------------------------------------------------------------------------------------------

class _Builder:
    def __init__(self, home: np.ndarray) -> None:
        self.state = np.asarray(home, float)
        self.segments: List[Dict] = []
        self.events: List[Dict] = []

    def add(self, kind: str, seam_id: Optional[str], q7: np.ndarray, t: np.ndarray, v: np.ndarray, a: np.ndarray,
            extra: Optional[Dict] = None) -> str:
        if np.abs(q7[0] - self.state).max() > 1e-9:
            raise RuntimeError(f'{kind} segment does not start where the previous one ends')
        sid = f'G{len(self.segments) + 1:03d}'
        t = np.asarray(t, float) - float(t[0])
        seg = {'segment_id': sid, 'controller': 'robot_positioner', 'kind': kind, 'seam_id': seam_id, 'hold_deg': [],
               'tcp_speed_mm_s': None, 'collision_checked': False}
        seg.update(extra or {})
        seg['points'] = [{'time_from_start_s': _f(ti), 'positions_deg': [_f(x) for x in p],
                          'velocities_deg_s': [_f(x) for x in vi], 'accelerations_deg_s2': [_f(x) for x in ai]}
                         for ti, p, vi, ai in zip(t, q7, v, a)]
        self.segments.append(seg)
        self.state = np.asarray(q7[-1], float)
        return sid

    def timed(self, kind: str, seam_id: Optional[str], q7: np.ndarray, cell: ProgramCell, params: ProgramParams,
              extra: Optional[Dict] = None) -> str:
        t, v, a = rest_to_rest(q7, cell.velocity_deg_s, cell.accel_deg_s2, min(params.motion_velocity_scaling,
                                                                                 params.motion_acceleration_scaling))
        return self.add(kind, seam_id, q7, t, v, a, extra)

    def move_to(self, goal: np.ndarray, cell: ProgramCell, params: ProgramParams) -> None:
        goal = np.asarray(goal, float)
        if np.abs(goal - self.state).max() <= 1e-9:
            return
        kind = 'index' if np.abs(goal[:6] - self.state[:6]).max() <= 1e-9 else 'air'
        path = joint_line(self.state, goal, float(units.rad_to_deg(params.dense_step_rad)))
        path[-1] = goal
        self.timed(kind, None, path, cell, params)


def arc_order(path_doc: Mapping, sequence: Mapping[str, int]) -> List[Dict]:
    """Accepted arcs in weld order: seam sequence (D04), then arc index."""
    return sorted(path_doc['seams'], key=lambda a: (sequence.get(a['seam_id'], 10 ** 6), a['seam_id'],
                                                    int(a['arc']['index'])))


def check_path(path_doc: Mapping) -> None:
    """Program-level refusals of D07 content: rejected arcs, no arc, travel speed above the feasible maximum."""
    if path_doc['rejected']:
        rows = [f'{_arc_name(r)} ({r["reason"]}; failing s ranges '
                f'{[(x["s_from_mm"], x["s_to_mm"], x["status"]) for x in r.get("failing_ranges", [])]})'
                for r in path_doc['rejected']]
        raise ProgramRefused('D07 has rejected arcs: ' + '; '.join(rows) + ' — no partial programs (DES-001 rule 4); '
                             'change the seam / arcs / angles and re-plan')
    if not path_doc['seams']:
        raise ProgramRefused('D07 has no accepted arc: nothing to program')
    slow = []
    for arc in path_doc['seams']:
        pl = arc.get('planning', {})
        if pl.get('speed_within_limits') is False:
            slow.append(f'{_arc_name(arc)}: travel speed {arc["travel_speed_mm_s"]} mm/s needs joint speeds above the '
                        f'limits; max_travel_speed_mm_s = {float(pl.get("max_travel_speed_mm_s", float("nan"))):.2f}')
    if slow:
        raise ProgramRefused('; '.join(slow) + SPEED_ADVICE)


def build_program(path_doc: Mapping, sequence: Mapping[str, int], flange_t_part: np.ndarray, cell: ProgramCell,
                  kin, params: ProgramParams = ProgramParams(), process: Optional[Mapping[str, Tuple]] = None,
                  progress: ProgressFn = None) -> Dict:
    """Segments, events, cycle time and the generator record of a D09 2.1 program for ``path_doc`` (D07 2.1).

    ``sequence``: seam id → D04 sequence; ``process``: seam id → (wps_id, preset) or None (no D06: events carry
    null and every release gate refuses the program). Raises :class:`ProgramRefused`."""
    check_path(path_doc)
    tol = D07Tolerances.from_path(path_doc)
    check_weld_timing_params(path_doc, params)
    sc = cell.sync
    ftp = np.asarray(flange_t_part, float)
    home = np.asarray(cell.home_deg, float)
    b = _Builder(home)
    arcs = arc_order(path_doc, sequence)
    welds: List[Dict] = []
    refusals: List[str] = []
    prepared = []
    for i, arc in enumerate(arcs):
        name = _arc_name(arc)
        if progress:
            progress(i / max(1, len(arcs)), f'{name}: weld timing and approach / retract')
        pts = arc['points']
        q = np.array([p['q_deg'] for p in pts], float)
        th = np.array([p['theta_deg'] for p in pts], float)
        s = np.array([p['s_mm'] for p in pts], float)
        q7 = np.column_stack([q, th])
        mode = arc['mode']
        if mode not in ('sync', 'C', 'B'):
            raise ProgramRefused(f'{name}: positioner mode {mode!r} is not a 7-axis mode (sync | C | B)')
        tol_mm, tol_deg = (tol.mode_c_tol_mm, tol.mode_c_tol_deg) if mode == 'C' else (tol.fk_tol_mm, tol.fk_tol_deg)
        targets = [part_to_base_link(sc, ftp, th[k]) @ pose_matrix(pts[k]['pose_mm']) for k in range(len(pts))]
        fk = batch_fk(q) @ sc.tool0_T_tcp
        errs = [pose_error(fk[k], targets[k]) for k in range(len(pts))]
        e_mm = max(e[0] for e in errs)
        e_deg = max(float(units.rad_to_deg(e[1])) for e in errs)
        if e_mm > tol_mm or e_deg > tol_deg:
            raise ProgramRefused(f'{name}: FK of the D07 joint states is {e_mm:.4f} mm / {e_deg:.4f}° off the D07 TCP '
                                 f'poses (tolerance {tol_mm} mm / {tol_deg}°): description or calibration changed — '
                                 're-plan')
        if np.abs(np.diff(q7, axis=0)).max() > tol.dq_max_deg + 1e-9:
            raise ProgramRefused(f'{name}: D07 joint step above Δq_max {tol.dq_max_deg}°')
        speed = float(arc['travel_speed_mm_s'])
        try:
            t, v, a, vmax, amax, ramp = weld_spline(s, q7, speed, params)
        except TimingError as exc:
            raise ProgramRefused(f'{name}: {exc}') from exc
        if not speed_within_limits(s, q7, speed, cell, params):
            refusals.append(weld_speed_refusal(name, s, q7, speed, cell, params,
                                               float(arc['planning'].get('max_travel_speed_mm_s', float('nan')))))
            continue
        label = arc['config_label']
        # the line starts at the robot's own TCP pose FK(q) (= the D07 pose within the FK tolerance; Mode C: F8)
        app = line_states(kin, sc, fk[0], q[0], label, params.approach_mm, tol, params, f'{name} approach')
        ret = line_states(kin, sc, fk[-1], q[-1], label, params.retract_mm, tol, params, f'{name} retract')[::-1]
        prepared.append((arc, q7, s, t, v, a, vmax, amax, ramp, app, ret, (e_mm, e_deg, tol_mm, tol_deg)))
    if refusals:
        raise ProgramRefused('; '.join(refusals) + SPEED_ADVICE)
    for arc, q7, s, t, v, a, vmax, amax, ramp, app, ret, fkr in prepared:
        sid = arc['seam_id']
        idx = int(arc['arc']['index'])
        app7 = np.column_stack([app, np.full(len(app), q7[0, 6])])
        ret7 = np.column_stack([ret, np.full(len(ret), q7[-1, 6])])
        b.move_to(app7[0], cell, params)
        g_app = b.timed('approach', sid, app7, cell, params, {'arc_index': idx})
        kind = 'positioner_weld' if arc['mode'] == 'C' else 'weld'
        g_weld = b.add(kind, sid, q7, t, v, a, {'arc_index': idx, 'positioner_mode': arc['mode'],
                                               'tcp_speed_mm_s': _f(arc['travel_speed_mm_s'])})
        b.timed('retract', sid, ret7, cell, params, {'arc_index': idx})
        wps, preset = (process or {}).get(sid, (None, None))
        for etype, after in (('WeldStart', g_app), ('WeldEnd', g_weld)):
            b.events.append({'event_id': f'V{len(b.events) + 1:03d}', 'type': etype, 'seam_id': sid,
                             'after_segment': after, 'wps_id': wps, 'preset': preset})
        welds.append({'segment_id': g_weld, 'seam_id': sid, 'arc_index': idx, 'mode': arc['mode'],
                      'travel_speed_mm_s': _f(arc['travel_speed_mm_s']), 'ramp_mm': _f(ramp),
                      'constant_speed_s_mm': [_f(s[0] + ramp), _f(s[-1] - ramp)], 'duration_s': _f(t[-1] - t[0]),
                      'max_velocity_deg_s': [_f(x) for x in vmax], 'max_acceleration_deg_s2': [_f(x) for x in amax],
                      'fk_max_error_mm': _f(fkr[0]), 'fk_max_error_deg': _f(fkr[1]),
                      'fk_tolerance': [fkr[2], fkr[3]], 'approach_points': len(app), 'retract_points': len(ret)})
    b.move_to(home, cell, params)
    if progress:
        progress(1.0, f'{len(b.segments)} segments, {len(b.events)} events')
    cycle = sum(seg['points'][-1]['time_from_start_s'] for seg in b.segments)
    return {'segments': b.segments, 'events': b.events, 'cycle_time_s': _f(cycle), 'weld_segments': welds,
            'limits': {'velocity_deg_s': [_f(x) for x in cell.velocity_deg_s],
                       'acceleration_deg_s2': [_f(x) for x in cell.accel_deg_s2]}}


__all__ = ['SCHEMA_VERSION', 'WELD_KINDS', 'ProgramRefused', 'ProgramParams', 'ProgramCell', 'D07Tolerances',
           'pose_matrix', 'part_to_base_link', 'tcp_in_part', 'params_sha256', 'line_states', 'joint_line',
           'weld_spline', 'speed_within_limits', 'max_feasible_speed', 'SPEED_ADVICE', 'weld_speed_refusal', 'WELD_TIMING_KEYS', 'check_weld_timing_params',
           'arc_order', 'check_path', 'build_program']
