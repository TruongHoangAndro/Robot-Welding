"""Kinematic 7-axis weld-path planner: robot + 1-axis positioner (G1.3; CP-58, CP-59, CP-53; DES-001 Rev 0.5 §4.3
steps 1, 3–5, A41, A44, A47–A49; TST-002 Rev 0.3 draft §5.1 F1, F2, F3, F6, F7, F11).

**Collision is not evaluated here (Goal 1, CP-58).** Every other rule of a weld path holds: 100 % of the points of an
arc or the arc is rejected with its failing s ranges (no partial path), one IK configuration label per arc,
|Δq| ≤ Δq_max on all 7 joints per step, joint and positioner limits, F6 singularity margins, F11 wrist 3.

Model (CP-58 decision 2). ``seam_axis.json`` gives every seam point and seam frame ``F`` in ``positioner_flange`` at
θ = 0 (= ``positioner_axis`` at θ = 0, axis = +X). At positioner angle θ the point is
``base_link_T_point(θ) = base_link_T_axis · Rx(θ) · F`` and the TCP target is ``… · Rx(180°) · Rz(ψ)`` (tool Z = beam
= −seam Z, ψ = yaw about the beam, work / travel angle 0, focus offset 0).

State per point = (θ, IK label, ψ) with the unwrapped 7-joint vector carried along the ladder DP:

* ``sync`` (CP-59): θ on a lattice of the *world angle* α = φ + θ about the axis, α_k = k·δ (δ = ``theta_step_deg``
  ≤ Δq_max), i.e. per point θ_k = k·δ − φ_i inside the positioner limits (unwrapped, more than 360° if the limits
  allow). Equal k on consecutive points keeps the point at the same angle about the axis.
* ``B`` (B-12): θ = the Engineer-entered angle of the arc only.
* ``C``: robot at rest (one state for every point), θ_i = α₀ − φ_i; coaxial seams only (``coaxial.is_coaxial``),
  every point within the F8 tolerance of the fixed TCP pose.

Ladder DP (F2, F3, F7): edges only inside one label; exact edge set — θ offsets from |Δθ| ≤ Δq_max, ψ offsets
bounded by |Δψ| ≤ Σ₆|Δq| + |Δθ| + seam-frame rotation (rotation triangle inequality). Cost (CP-59; weights =
parameters, Assumption defaults, recorded): w_dq·(Σ₆|Δq| + |Δθ|) + w_theta·|Δθ| + w_wrist3·|Δq6| per edge +
w_up·∠(seam Z, world +Z) per node. Tie-break: cost (1e-9), then ψ index, label order, θ index of the predecessor
(F7 with θ last). Limits and F11 are checked on the unwrapped result; if the optimum breaks F11 the DP is re-run in
wrist-3 windows (start within c ± h, path within c ± (travel − h): sufficient, never accepts a violation).

Pitch (CP-53 F1): the arc is first planned at the seam_axis spacing; a step break (no edge with |Δq| ≤ Δq_max)
halves the pitch down to ``pitch_floor_mm``; inserted points lie on the circle through the neighbouring samples with
a slerped frame (Assumption: exact for circular seams, chord sag otherwise), recorded per arc.

Pure Python + numpy (B-06); mm / degrees, radians only through ``awm_core.units``.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import asdict, dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from awm_core import frames, units
from awm_path.core.batch_kin import LABEL_COUNT, batch_fk, batch_label_metrics, ik_by_label
from awm_path.core.kinematics import pose_error, wrap_deg
from awm_path.core.states import LABELS

ProgressFn = Optional[Callable[[float, str], None]]

_RX180 = np.diag([1.0, -1.0, -1.0, 1.0])
_EPS = 1e-9
_S_EPS = 1e-6
MODES = ('sync', 'C', 'B')


# ---------------------------------------------------------------------------------------------------------------
# parameters and cell
# ---------------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class SyncParams:
    """Planner parameters (D16 later). Assumptions (recorded with every run): cost weights, θ lattice step, wrist-3
    window half width, pitch floor 0.05 mm (CP-53), F8 tolerances for Mode C (TST-002: D16 Assumption), F6 elbow /
    shoulder margins (TST-002: Assumption)."""

    theta_step_deg: float = 5.0
    psi_step_deg: float = 10.0
    dq_max_deg: float = 5.0
    singular_wrist_deg: float = 8.6
    singular_elbow_deg: float = 5.0
    singular_shoulder_mm: float = 50.0
    wrist3_abs_deg: float = 270.0
    wrist3_travel_deg: float = 180.0
    w_dq: float = 1.0
    w_theta: float = 0.5
    w_wrist3: float = 0.5
    w_up: float = 0.05
    pitch_floor_mm: float = 0.05
    max_refinements: int = 4
    mode_c_tol_mm: float = 0.05
    mode_c_tol_deg: float = 0.1
    fk_tol_mm: float = 0.01
    fk_tol_deg: float = 0.01
    wrist3_window_half_deg: float = 45.0

    def __post_init__(self) -> None:
        if not 0.0 < self.theta_step_deg <= self.dq_max_deg:
            raise ValueError('theta_step_deg must be in (0, dq_max_deg] (θ grid ≤ Δq_max)')
        for step in (self.theta_step_deg, self.psi_step_deg):
            if abs(360.0 / step - round(360.0 / step)) > 1e-9:
                raise ValueError('theta_step_deg and psi_step_deg must divide 360')
        if not 0.0 < self.wrist3_window_half_deg < self.wrist3_travel_deg:
            raise ValueError('0 < wrist3_window_half_deg < wrist3_travel_deg is required')

    @property
    def psi_grid(self) -> np.ndarray:
        return np.arange(int(round(360.0 / self.psi_step_deg))) * self.psi_step_deg

    def record(self) -> Dict:
        return asdict(self)


@dataclass(frozen=True)
class SyncCell:
    """Cell values used by the planner (mm, degrees), from the robot description (``cell_urdf``) and the head."""

    base_link_T_axis: np.ndarray
    world_z_base: np.ndarray
    tool0_T_tcp: np.ndarray
    joint_limits_deg: Tuple[Tuple[float, float], ...]
    theta_limits_deg: Tuple[float, float]
    joint_velocity_deg_s: Tuple[float, ...]
    theta_velocity_deg_s: float


@dataclass(frozen=True)
class ArcSpec:
    """One arc of a seam (CP-53; from ``seam_axis.json``)."""

    index: int
    of: int
    s_start_mm: float
    s_end_mm: float
    positioner_angle_deg: Optional[float]


# ---------------------------------------------------------------------------------------------------------------
# rotation model
# ---------------------------------------------------------------------------------------------------------------

def rot_x(theta_deg: float) -> np.ndarray:
    """4x4 rotation about +X by θ (positioner axis convention of ``seam_axis.json``)."""
    a = units.deg_to_rad(float(theta_deg))
    c, s = math.cos(a), math.sin(a)
    t = np.eye(4)
    t[1:3, 1:3] = [[c, -s], [s, c]]
    return t


def _rot_x_stack(theta_deg: np.ndarray) -> np.ndarray:
    a = units.deg_to_rad(np.asarray(theta_deg, dtype=float))
    t = np.broadcast_to(np.eye(4), (len(a), 4, 4)).copy()
    t[:, 1, 1], t[:, 1, 2], t[:, 2, 1], t[:, 2, 2] = np.cos(a), -np.sin(a), np.sin(a), np.cos(a)
    return t


def _rot_z_stack(psi_deg: np.ndarray) -> np.ndarray:
    a = units.deg_to_rad(np.asarray(psi_deg, dtype=float))
    t = np.broadcast_to(np.eye(4), (len(a), 4, 4)).copy()
    t[:, 0, 0], t[:, 0, 1], t[:, 1, 0], t[:, 1, 1] = np.cos(a), -np.sin(a), np.sin(a), np.cos(a)
    return t


def flange_point_at(cell: SyncCell, frame_flange: np.ndarray, theta_deg: float) -> np.ndarray:
    """``base_link_T_point`` (mm) of a seam frame given in ``positioner_flange`` at θ = 0, with the positioner at θ."""
    return cell.base_link_T_axis @ rot_x(theta_deg) @ frame_flange


def tcp_target(cell: SyncCell, frame_flange: np.ndarray, theta_deg: float, psi_deg: float) -> np.ndarray:
    """``base_link_T_tcp`` target: seam frame at θ, beam = −seam Z, yaw ψ about the beam."""
    return flange_point_at(cell, frame_flange, theta_deg) @ _RX180 @ _rot_z_stack([psi_deg])[0]


def seam_z_up_deg(cell: SyncCell, frame_flange: np.ndarray, thetas: np.ndarray) -> np.ndarray:
    """Angle (deg) between the seam Z at each θ and world +Z (weld position term of the CP-59 cost)."""
    z = (cell.base_link_T_axis[:3, :3] @ _rot_x_stack(thetas)[:, :3, :3] @ frame_flange[:3, :3])[:, :, 2]
    c = np.clip(z @ (cell.world_z_base / np.linalg.norm(cell.world_z_base)), -1.0, 1.0)
    return units.rad_to_deg(np.arccos(c))


# ---------------------------------------------------------------------------------------------------------------
# arc geometry (seam_axis.json -> points of one arc, pitch refinement)
# ---------------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class ArcGeometry:
    """Points of one arc in ``positioner_flange`` at θ = 0: s (mm, seam arc length, may exceed L on a closed seam),
    frames (n, 4, 4), φ about the axis (deg, unwrapped), interpolated flag (not a B-Rep sample)."""

    s: np.ndarray
    frames: np.ndarray
    phi: np.ndarray
    interpolated: np.ndarray

    @property
    def pitch_mm(self) -> float:
        return float(np.diff(self.s).max()) if len(self.s) > 1 else 0.0


def _seam_samples(seam: Dict) -> Tuple[np.ndarray, np.ndarray]:
    s = np.array([p['s_mm'] for p in seam['points']], dtype=float)
    f = np.array([frames.make_transform(p['p_mm'], frames.quat_to_matrix(p['quat_xyzw'])) for p in seam['points']])
    if seam['closed'] and abs(s[-1] - float(seam['length_mm'])) < _S_EPS:
        s, f = s[:-1], f[:-1]                     # closing duplicate of the first sample
    return s, f


def _extended(seam: Dict, s_end: float) -> Tuple[np.ndarray, np.ndarray]:
    s, f = _seam_samples(seam)
    if not seam['closed']:
        return s, f
    L = float(seam['length_mm'])
    reps = int(math.ceil((s_end + _S_EPS) / L)) + 1
    return (np.concatenate([s + k * L for k in range(reps)]), np.concatenate([f] * reps))


def _slerp(r0: np.ndarray, r1: np.ndarray, u: float) -> np.ndarray:
    rel = r0.T @ r1
    v = np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]])
    s, c = 0.5 * np.linalg.norm(v), 0.5 * (np.trace(rel) - 1.0)
    ang = math.atan2(s, c)
    if ang < 1e-12:
        return r0.copy()
    if math.pi - ang < 1e-6:
        raise ValueError('seam frames of consecutive samples differ by 180 deg: cannot interpolate')
    return r0 @ frames.axis_angle_to_matrix(v / np.linalg.norm(v), u * ang)


def _circle_point(a: np.ndarray, b: np.ndarray, c: np.ndarray, p0: np.ndarray, p1: np.ndarray, u: float
                  ) -> np.ndarray:
    """Point at fraction u (of the arc angle) from p0 to p1 on the circle through a, b, c; linear if collinear."""
    ab, ac = b - a, c - a
    n = np.cross(ab, ac)
    nn = float(n @ n)
    if nn < 1e-18 * max(1.0, float(ab @ ab) * float(ac @ ac)):
        return p0 + u * (p1 - p0)
    centre = a + (np.cross(n, ab) * float(ac @ ac) + np.cross(ac, n) * float(ab @ ab)) / (2.0 * nn)
    v0, v1 = p0 - centre, p1 - centre
    r = 0.5 * (np.linalg.norm(v0) + np.linalg.norm(v1))
    e1 = v0 / np.linalg.norm(v0)
    e2 = np.cross(n / math.sqrt(nn), e1)
    ang = math.atan2(float(v1 @ e2), float(v1 @ e1))
    return centre + r * (math.cos(u * ang) * e1 + math.sin(u * ang) * e2)


def _interp(ext_s: np.ndarray, ext_f: np.ndarray, s: float) -> np.ndarray:
    i = int(np.searchsorted(ext_s, s, side='right') - 1)
    i = min(max(i, 0), len(ext_s) - 2)
    u = (s - ext_s[i]) / (ext_s[i + 1] - ext_s[i])
    if abs(u) < 1e-12:
        return ext_f[i].copy()
    tri = (i - 1, i, i + 1) if i >= 1 else ((i, i + 1, i + 2) if i + 2 < len(ext_s) else None)
    p0, p1 = ext_f[i][:3, 3], ext_f[i + 1][:3, 3]
    if tri is None:
        p = p0 + u * (p1 - p0)
    else:
        p = _circle_point(*(ext_f[k][:3, 3] for k in tri), p0, p1, u)
    return frames.make_transform(p, _slerp(ext_f[i][:3, :3], ext_f[i + 1][:3, :3], u))


def arc_geometry(seam: Dict, arc: ArcSpec, max_pitch_mm: float) -> ArcGeometry:
    """Points of ``arc`` of a ``seam_axis.json`` seam: the B-Rep samples inside [s_start, s_end] plus both ends,
    every gap subdivided to ≤ ``max_pitch_mm`` (inserted points interpolated, see module docstring)."""
    s0, s1 = float(arc.s_start_mm), float(arc.s_end_mm)
    if not s1 > s0:
        raise ValueError(f'arc {arc.index}: s_end must be > s_start')
    ext_s, ext_f = _extended(seam, s1)
    if s0 < ext_s[0] - _S_EPS or s1 > ext_s[-1] + _S_EPS:
        raise ValueError(f'arc {arc.index}: s range [{s0}, {s1}] outside the seam samples')
    inside = np.nonzero((ext_s > s0 + _S_EPS) & (ext_s < s1 - _S_EPS))[0]
    knots = [(s0, None)] + [(float(ext_s[k]), int(k)) for k in inside] + [(s1, None)]
    for end in (0, -1):                               # an end that coincides with a sample is that sample
        hit = np.nonzero(np.abs(ext_s - knots[end][0]) <= _S_EPS)[0]
        if len(hit):
            knots[end] = (float(ext_s[hit[0]]), int(hit[0]))
    s_out, f_out, interp = [], [], []
    for (sa, ka), (sb, kb) in zip(knots[:-1], knots[1:]):
        if not s_out:
            s_out.append(sa)
            f_out.append(ext_f[ka].copy() if ka is not None else _interp(ext_s, ext_f, sa))
            interp.append(ka is None)
        m = max(1, int(math.ceil((sb - sa) / max_pitch_mm - 1e-9)))
        for j in range(1, m + 1):
            s = sa + (sb - sa) * j / m
            last = j == m
            s_out.append(sb if last else s)
            f_out.append(ext_f[kb].copy() if last and kb is not None else _interp(ext_s, ext_f, s))
            interp.append(not (last and kb is not None))
    f_arr = np.array(f_out)
    p = f_arr[:, :3, 3]
    phi = np.unwrap(units.rad_to_deg(np.arctan2(p[:, 2], p[:, 1])), period=360.0)
    return ArcGeometry(np.array(s_out), f_arr, phi, np.array(interp, dtype=bool))


# ---------------------------------------------------------------------------------------------------------------
# states of one point
# ---------------------------------------------------------------------------------------------------------------

@dataclass
class _Layer:
    """IK states of one point. θ values share IK per class mod 360° (``cls`` maps θ index → class); joint values are
    stored compactly as float32 (DP only; the planned path is recomputed in float64)."""

    theta: np.ndarray          # (J,)
    cls: np.ndarray            # (J,) class index
    qc: np.ndarray             # (C, 8, P, 6) float32 wrapped degrees, NaN where invalid
    vc: np.ndarray             # (C, 8, P) bool
    node_cost: np.ndarray      # (J,)
    reasons: Counter

    @property
    def shape(self) -> Tuple[int, int, int]:
        return (len(self.theta),) + self.vc.shape[1:]

    @property
    def valid(self) -> np.ndarray:
        return self.vc[self.cls]

    @property
    def any_valid(self) -> bool:
        return bool(len(self.theta)) and bool(self.vc[np.unique(self.cls)].any())


def _limit_ok(q: np.ndarray, lims: np.ndarray) -> np.ndarray:
    lo, hi = lims[:, 0], lims[:, 1]
    return np.all(np.ceil((lo - q - _EPS) / 360.0) <= np.floor((hi - q + _EPS) / 360.0), axis=-1)


def _layer(kin, cell: SyncCell, frame: np.ndarray, thetas: np.ndarray, params: SyncParams) -> _Layer:
    """IK states of one point for the θ values (IK once per θ class mod 360°, reused for θ ± 360°)."""
    thetas = np.asarray(thetas, dtype=float)
    psi = params.psi_grid
    P, J = len(psi), len(thetas)
    cls_val = np.round(np.mod(thetas, 360.0), 9)
    cls_val[cls_val >= 360.0] -= 360.0
    uniq, inv = np.unique(cls_val, return_inverse=True)
    rep = np.array([thetas[np.nonzero(inv == c)[0][0]] for c in range(len(uniq))]) if J else np.zeros(0)
    C = len(rep)
    if C:
        tool_inv = np.linalg.inv(cell.tool0_T_tcp)
        pts = cell.base_link_T_axis @ _rot_x_stack(rep) @ frame @ _RX180                  # (C, 4, 4)
        tg = pts[:, None] @ _rot_z_stack(psi)[None] @ tool_inv                            # (C, P, 4, 4)
        q, ok, s5, s3, u = ik_by_label(kin, tg.reshape(-1, 4, 4), with_metrics=True)
    else:
        q = np.zeros((0, LABEL_COUNT, 6))
        ok = np.zeros((0, LABEL_COUNT), dtype=bool)
        s5 = s3 = u = np.zeros((0, LABEL_COUNT))
    shp = (C, P, LABEL_COUNT)
    q = q.reshape(C, P, LABEL_COUNT, 6).transpose(0, 2, 1, 3)                             # (C, 8, P, 6)
    ok, s5, s3, u = (a.reshape(shp).transpose(0, 2, 1) for a in (ok, s5, s3, u))
    with np.errstate(invalid='ignore'):
        sing = ok & ~((s5 >= math.sin(units.deg_to_rad(params.singular_wrist_deg)))
                      & (s3 >= math.sin(units.deg_to_rad(params.singular_elbow_deg)))
                      & (u >= params.singular_shoulder_mm))
    lim = np.zeros_like(ok)
    if ok.any():
        lim[ok] = ~_limit_ok(q[ok], np.asarray(cell.joint_limits_deg, float))
    valid = ok & ~sing & ~lim
    mult = np.bincount(inv, minlength=C)[:, None, None] if C else np.zeros((0, 1, 1), dtype=np.int64)
    reasons: Counter = Counter()
    reasons['ik'] += int(((~ok) * mult).sum())
    reasons['singularity'] += int((sing * mult).sum())
    reasons['limit'] += int(((ok & ~sing & lim) * mult).sum())
    qc = np.where(valid[..., None], q, np.nan).astype(np.float32)
    node = params.w_up * seam_z_up_deg(cell, frame, thetas) if J else np.zeros(0)
    return _Layer(thetas, inv.astype(np.int64), qc, valid, node, reasons)


def _point_status(layer: _Layer) -> str:
    """Status of a point without any valid state (dominant reason, precedence ik < limit < singularity)."""
    r = layer.reasons
    if r['singularity'] > 0 and r['singularity'] >= r['limit']:
        return 'singularity'
    if r['limit'] > 0:
        return 'limit'
    return 'ik'


# ---------------------------------------------------------------------------------------------------------------
# ladder DP
# ---------------------------------------------------------------------------------------------------------------

#: The DP edge test uses Δq_max minus this margin (deg): joint values are stored as float32 (≤ 3e-5° error); the
#: planned path is recomputed in float64 and checked against Δq_max exactly.
_DQ_MARGIN_DEG = 1e-4
_TIE_J = 1 << 20            # F7 tie key = (psi · 8 + label) · _TIE_J + θ index


@dataclass
class _DpOut:
    ok: bool
    path: List[Tuple[int, int, int]]          # (j, label, psi index) per layer
    cost: Optional[float]
    status: List[str]                         # per layer
    reasons: List[Counter]
    breaks: int
    longest: int                              # longest unbroken run of layers


def _psi_window(params: SyncParams, frame_rot_deg: float, P: int) -> int:
    w = int(math.ceil((6.0 * params.dq_max_deg + params.dq_max_deg + frame_rot_deg) / params.psi_step_deg + 1e-9))
    return min(w, P // 2)


def _start(cur: _Layer, window: Optional[Tuple[float, float, float]]) -> Tuple[np.ndarray, np.ndarray]:
    valid = cur.valid
    cost = np.where(valid, cur.node_cost[:, None, None], np.inf)
    u6 = cur.qc[cur.cls][..., 5].astype(float)
    if window is not None:
        c, h = 0.5 * (window[0] + window[1]), window[2]
        with np.errstate(invalid='ignore'):
            u6 = u6 + 360.0 * np.round((c - u6) / 360.0)
            cost = np.where(np.abs(u6 - c) <= h + _EPS, cost, np.inf)
    return cost, u6


def _ladder(layers_fn: Callable[[int], _Layer], n: int, frame_rot: Sequence[float], params: SyncParams,
            window: Optional[Tuple[float, float, float]], progress: ProgressFn, label: str) -> _DpOut:
    """DP over n layers (built lazily by ``layers_fn``). ``window`` = (lo, hi, start half width) on the unwrapped q6
    (F11 fallback) or None. After a break the DP restarts (to locate every failing point); ``ok`` only without
    breaks."""
    dq = params.dq_max_deg - _DQ_MARGIN_DEG
    status: List[str] = []
    reasons: List[Counter] = []
    backs: List[np.ndarray] = []
    breaks, run, longest = 0, 0, 0
    prev: Optional[_Layer] = None
    cost = u6 = None
    for i in range(n):
        if progress and (i % 5 == 0 or i == n - 1):
            progress(i / max(1, n), f'{label}: point {i + 1}/{n}')
        cur = layers_fn(i)
        reasons.append(cur.reasons)
        shape = cur.shape
        if not cur.any_valid:
            status.append(_point_status(cur))
            backs.append(np.full(shape, -1, dtype=np.int64))
            breaks += 1
            longest, run = max(longest, run), 0
            prev, cost, u6 = cur, np.full(shape, np.inf), np.full(shape, np.nan)
            continue
        status.append('ok')
        if prev is None or not np.isfinite(cost).any():
            back = np.full(shape, -1, dtype=np.int64)
            new_cost, new_u6 = _start(cur, window)
        else:
            new_cost = np.full(shape, np.inf)
            new_tie = np.full(shape, np.iinfo(np.int64).max, dtype=np.int64)
            back = np.full(shape, -1, dtype=np.int64)
            new_u6 = np.full(shape, np.nan)
            P = shape[2]
            w = _psi_window(params, frame_rot[i - 1], P)
            Jp, Jc = len(prev.theta), len(cur.theta)
            pshape = prev.shape
            for d in range(-(Jp - 1), Jc):
                jp = np.arange(max(0, -d), min(Jp, Jc - d))
                if not len(jp):
                    continue
                jc = jp + d
                dth = cur.theta[jc] - prev.theta[jp]
                keep = np.abs(dth) <= dq
                if not keep.any():
                    continue
                jp, jc, dth = jp[keep], jc[keep], dth[keep]
                act = np.isfinite(cost[jp])
                if not act.any():
                    continue
                qp6 = prev.qc[prev.cls[jp]][..., 5]
                cur6 = cur.qc[cur.cls[jc]][..., 5]
                curv = cur.vc[cur.cls[jc]]
                for dpsi in range(-w, w + 1):
                    tp = (np.arange(P) + dpsi) % P
                    with np.errstate(invalid='ignore'):
                        m = act & curv[:, :, tp] & (np.abs(wrap_deg(cur6[:, :, tp] - qp6)) <= dq)
                    if not m.any():
                        continue
                    a_j, a_b, a_p = np.nonzero(m)
                    sj, cj, ctp = jp[a_j], jc[a_j], tp[a_p]
                    dlt = wrap_deg(cur.qc[cur.cls[cj], a_b, ctp].astype(float)
                                   - prev.qc[prev.cls[sj], a_b, a_p].astype(float))
                    ok = np.all(np.abs(dlt) <= dq, axis=1)
                    nu6 = u6[sj, a_b, a_p] + dlt[:, 5]
                    if window is not None:
                        ok &= (nu6 >= window[0] - _EPS) & (nu6 <= window[1] + _EPS)
                    if not ok.any():
                        continue
                    a_j, a_b, a_p, sj, cj, ctp, dlt, nu6 = (x[ok] for x in (a_j, a_b, a_p, sj, cj, ctp, dlt, nu6))
                    adt = np.abs(dth[a_j])
                    cn = (cost[sj, a_b, a_p] + params.w_dq * (np.abs(dlt).sum(axis=1) + adt) + params.w_theta * adt
                          + params.w_wrist3 * np.abs(dlt[:, 5]) + cur.node_cost[cj])
                    st = (a_p * LABEL_COUNT + a_b) * _TIE_J + sj
                    cur_c, cur_t = new_cost[cj, a_b, ctp], new_tie[cj, a_b, ctp]
                    better = (cn < cur_c - _EPS) | ((np.abs(cn - cur_c) <= _EPS) & (st < cur_t))
                    if not better.any():
                        continue
                    cj, a_b, ctp, cn, st, sj, a_p, nu6 = (x[better] for x in (cj, a_b, ctp, cn, st, sj, a_p, nu6))
                    order = np.lexsort((st, np.round(cn, 9), ctp, a_b, cj))
                    key = (cj[order] * LABEL_COUNT + a_b[order]) * P + ctp[order]
                    first = np.ones(len(order), dtype=bool)
                    first[1:] = key[1:] != key[:-1]
                    o = order[first]
                    new_cost[cj[o], a_b[o], ctp[o]] = cn[o]
                    new_tie[cj[o], a_b[o], ctp[o]] = st[o]
                    back[cj[o], a_b[o], ctp[o]] = np.ravel_multi_index((sj[o], a_b[o], a_p[o]), pshape)
                    new_u6[cj[o], a_b[o], ctp[o]] = nu6[o]
            if not np.isfinite(new_cost).any():
                status[i] = 'wrist3' if window is not None else 'step'
                breaks += 1
                longest, run = max(longest, run), 0
                back = np.full(shape, -1, dtype=np.int64)
                new_cost, new_u6 = _start(cur, window)
        backs.append(back)
        run += 1
        longest = max(longest, run)
        prev, cost, u6 = cur, new_cost, new_u6
    if progress:
        progress(1.0, f'{label}: DP done')
    if breaks or cost is None or not np.isfinite(cost).any():
        return _DpOut(False, [], None, status, reasons, max(1, breaks), longest)
    J, B, P = prev.shape
    jj, bb, pp = np.meshgrid(np.arange(J), np.arange(B), np.arange(P), indexing='ij')
    tie = ((pp * LABEL_COUNT + bb) * _TIE_J + jj).ravel()
    flat_cost = cost.ravel()
    fin = np.nonzero(np.isfinite(flat_cost))[0]
    best = int(fin[np.lexsort((tie[fin], np.round(flat_cost[fin], 9)))[0]])
    path = []
    k = best
    for i in range(n - 1, -1, -1):
        j, b, p = np.unravel_index(k, backs[i].shape)
        path.append((int(j), int(b), int(p)))
        k = int(backs[i].ravel()[k])
        if k < 0 and i > 0:
            raise RuntimeError('broken back pointer chain')
    path.reverse()
    return _DpOut(True, path, float(flat_cost[best]), status, reasons, 0, longest)


# ---------------------------------------------------------------------------------------------------------------
# results
# ---------------------------------------------------------------------------------------------------------------

@dataclass
class PlannedPoint:
    s_mm: float
    frame_flange: np.ndarray
    theta_deg: float
    psi_deg: float
    q_deg: Tuple[float, ...]
    label: str
    status: str = 'ok'
    interpolated: bool = False


@dataclass
class ArcResult:
    seam_id: str
    arc: ArcSpec
    mode: str
    ok: bool
    reason: str
    points: List[PlannedPoint] = field(default_factory=list)
    planning: Dict = field(default_factory=dict)
    failing_ranges: List[Dict] = field(default_factory=list)
    failing_points: List[Dict] = field(default_factory=list)
    coverage_fraction: float = 0.0
    pitch_mm: float = 0.0
    attempts: List[Dict] = field(default_factory=list)


def _ranges(s: np.ndarray, status: Sequence[str]) -> List[Dict]:
    out: List[Dict] = []
    for i, st in enumerate(status):
        if st == 'ok':
            continue
        if out and out[-1]['_last'] == i - 1 and out[-1]['status'] == st:
            out[-1]['_last'] = i
            out[-1]['s_to_mm'] = float(s[i])
            out[-1]['points'] += 1
        else:
            out.append({'s_from_mm': float(s[i]), 's_to_mm': float(s[i]), 'status': st, 'points': 1, '_last': i})
    for r in out:
        del r['_last']
    return out


def _failing_points(s: np.ndarray, status: Sequence[str], reasons: Sequence[Counter]) -> List[Dict]:
    return [{'index': i, 's_mm': float(s[i]), 'status': st,
             'reasons': {k: int(v) for k, v in sorted(reasons[i].items()) if v}}
            for i, st in enumerate(status) if st != 'ok']


def _unwrap_path(q: np.ndarray) -> np.ndarray:
    u = q.copy()
    for i in range(1, len(u)):
        u[i] = u[i - 1] + wrap_deg(q[i] - u[i - 1])
    return u


def _fit_limits(u: np.ndarray, cell: SyncCell, params: SyncParams) -> Tuple[Optional[np.ndarray], str]:
    """Shift each joint of the unwrapped path by 360°·k into its limits (start nearest 0); F11 on q6."""
    out = u.copy()
    lims = list(cell.joint_limits_deg)
    lo6, hi6 = lims[5]
    lims[5] = (max(lo6, -params.wrist3_abs_deg), min(hi6, params.wrist3_abs_deg))
    if np.max(np.abs(u[:, 5] - u[0, 5])) > params.wrist3_travel_deg + _EPS:
        return None, 'wrist3'
    for j, (lo, hi) in enumerate(lims):
        k_lo = math.ceil((lo - u[:, j].min() - _EPS) / 360.0)
        k_hi = math.floor((hi - u[:, j].max() + _EPS) / 360.0)
        if k_lo > k_hi:
            return None, 'wrist3' if j == 5 else 'limit'
        ks = range(k_lo, k_hi + 1)
        k = min(ks, key=lambda kk: (abs(u[0, j] + 360.0 * kk), kk))
        out[:, j] = u[:, j] + 360.0 * k
    return out, 'ok'


def _path_q(kin, cell: SyncCell, geo: ArcGeometry, thetas: Sequence[float], psis: Sequence[float],
            labels: Sequence[int]) -> np.ndarray:
    tool_inv = np.linalg.inv(cell.tool0_T_tcp)
    tg = np.array([tcp_target(cell, geo.frames[i], thetas[i], psis[i]) @ tool_inv for i in range(len(thetas))])
    q, ok = ik_by_label(kin, tg)
    rows = np.arange(len(thetas))
    if not ok[rows, labels].all():
        raise RuntimeError('IK of a planned state not reproducible')
    return q[rows, labels]


def _finish(kin, cell: SyncCell, geo: ArcGeometry, thetas: np.ndarray, psis: np.ndarray, labels: np.ndarray,
            q_unwrapped: np.ndarray, params: SyncParams, mode: str, cost: float, closed_len: Optional[float],
            arc: ArcSpec) -> Tuple[List[PlannedPoint], Dict]:
    fk = batch_fk(q_unwrapped) @ cell.tool0_T_tcp
    errs = [pose_error(fk[i], tcp_target(cell, geo.frames[i], thetas[i], psis[i])) for i in range(len(thetas))]
    e_mm = max(e[0] for e in errs)
    e_deg = max(float(units.rad_to_deg(e[1])) for e in errs)
    q7 = np.column_stack([q_unwrapped, thetas])
    steps = np.abs(np.diff(q7, axis=0)).max(axis=0) if len(q7) > 1 else np.zeros(7)
    if steps.max() > params.dq_max_deg + _EPS:
        raise RuntimeError(f'planned path has a joint step of {steps.max()} deg > dq_max (float64 re-check)')
    ups = np.array([seam_z_up_deg(cell, geo.frames[i], np.array([thetas[i]]))[0] for i in range(len(thetas))])
    closure = None
    if closed_len is not None:
        hit = np.nonzero(np.abs(geo.s - (geo.s[0] + closed_len)) <= _S_EPS)[0]
        if len(hit):
            k = int(hit[0])
            closure = {'s_mm': float(geo.s[k]),
                       'dq_robot_deg': [float(v) for v in q_unwrapped[k] - q_unwrapped[0]],
                       'dq_robot_max_deg': float(np.abs(q_unwrapped[k] - q_unwrapped[0]).max()),
                       'dtheta_deg': float(thetas[k] - thetas[0]),
                       'same_robot_state': bool(np.abs(q_unwrapped[k] - q_unwrapped[0]).max() <= params.dq_max_deg)}
    lab = LABELS[int(labels[0])]
    pts = [PlannedPoint(float(geo.s[i]), geo.frames[i], float(thetas[i]), float(psis[i]),
                        tuple(float(v) for v in q_unwrapped[i]), lab, 'ok', bool(geo.interpolated[i]))
           for i in range(len(thetas))]
    planning = {'pitch_mm': geo.pitch_mm, 'points': len(pts), 'interpolated_points': int(geo.interpolated.sum()),
                'theta_min_deg': float(thetas.min()), 'theta_max_deg': float(thetas.max()),
                'theta_travel_deg': float(np.abs(np.diff(thetas)).sum()) if len(thetas) > 1 else 0.0,
                'wrist3_travel_deg': float(np.abs(q_unwrapped[:, 5] - q_unwrapped[0, 5]).max()),
                'max_step_deg': [float(v) for v in steps], 'fk_max_error_mm': float(e_mm),
                'fk_max_error_deg': float(e_deg), 'seam_z_up_max_deg': float(ups.max()), 'closure': closure,
                'cost': float(cost)}
    return pts, planning


def _reject(seam_id, arc, mode, geo, reason, status, reasons, longest, attempts) -> ArcResult:
    n = len(geo.s)
    return ArcResult(seam_id, arc, mode, False, reason, [], {}, _ranges(geo.s, status),
                     _failing_points(geo.s, status, reasons),
                     float(max(0, longest - 1)) / max(1, n - 1), geo.pitch_mm, attempts)


# ---------------------------------------------------------------------------------------------------------------
# modes
# ---------------------------------------------------------------------------------------------------------------

def _lattice(cell: SyncCell, phi: float, params: SyncParams) -> np.ndarray:
    lo, hi = cell.theta_limits_deg
    d = params.theta_step_deg
    k0 = math.ceil((lo + phi) / d - 1e-9)
    k1 = math.floor((hi + phi) / d + 1e-9)
    return np.array([k * d - phi for k in range(k0, k1 + 1)], dtype=float)


def _frame_rot_deg(geo: ArcGeometry) -> List[float]:
    out = []
    for a, b in zip(geo.frames[:-1], geo.frames[1:]):
        out.append(float(units.rad_to_deg(pose_error(a, b)[1])))
    return out


def _plan_ladder(kin, cell, seam, arc, mode, params, geo, progress, label, cache: Dict
                 ) -> Tuple[Optional[ArcResult], str, Dict]:
    """One DP attempt at the pitch of ``geo``. Returns (result or None, failure kind, attempt record). ``cache``
    keeps the IK layers of points already computed (same s and φ) across pitch refinements."""
    n = len(geo.s)
    if mode == 'B':
        th = float(arc.positioner_angle_deg)
        lo, hi = cell.theta_limits_deg
        if not lo - _EPS <= th <= hi + _EPS:
            status = ['limit'] * n
            return (_reject(seam['seam_id'], arc, mode, geo, f'mode B angle {th} deg outside the positioner limits '
                            f'[{lo}, {hi}]', status, [Counter({'limit': 1})] * n, 0, []), 'final', {})
        theta_fn = lambda i: np.array([th])  # noqa: E731
    else:
        theta_fn = lambda i: _lattice(cell, float(geo.phi[i]), params)  # noqa: E731
    def layers_fn(i: int) -> _Layer:
        key = (round(float(geo.s[i]), 9), round(float(geo.phi[i]), 6))
        if key not in cache:
            cache[key] = _layer(kin, cell, geo.frames[i], theta_fn(i), params)
        return cache[key]

    rot = _frame_rot_deg(geo)
    out = _ladder(layers_fn, n, rot, params, None, progress, label)
    attempt = {'pitch_mm': geo.pitch_mm, 'points': n, 'result': 'ok' if out.ok else 'rejected',
               'status_counts': dict(Counter(out.status))}
    closed_len = float(seam['length_mm']) if seam['closed'] and arc.of == 1 else None
    if not out.ok:
        kind = 'step' if 'step' in out.status else 'final'
        res = _reject(seam['seam_id'], arc, mode, geo, _reason_text(out.status), out.status, out.reasons,
                      out.longest, [])
        return res, kind, attempt
    thetas = np.array([layers_fn(i).theta[j] for i, (j, _, _) in enumerate(out.path)])
    psis = np.array([params.psi_grid[p] for _, _, p in out.path])
    labels = np.array([b for _, b, _ in out.path])
    q = _path_q(kin, cell, geo, thetas, psis, labels)
    u, why = _fit_limits(_unwrap_path(q), cell, params)
    cost = out.cost
    if u is None and why == 'wrist3':
        best = None
        for c in _window_centres(params):
            half = params.wrist3_travel_deg - params.wrist3_window_half_deg
            win = (c - half, c + half, params.wrist3_window_half_deg)
            o2 = _ladder(layers_fn, n, rot, params, win, None, label)
            if o2.ok and (best is None or round(o2.cost, 9) < round(best[0].cost, 9)):
                best = (o2, c)
        attempt['wrist3_windows'] = [float(c) for c in _window_centres(params)]
        if best is not None:
            out = best[0]
            attempt['wrist3_window_centre_deg'] = float(best[1])
            thetas = np.array([layers_fn(i).theta[j] for i, (j, _, _) in enumerate(out.path)])
            psis = np.array([params.psi_grid[p] for _, _, p in out.path])
            labels = np.array([b for _, b, _ in out.path])
            q = _path_q(kin, cell, geo, thetas, psis, labels)
            u, why = _fit_limits(_unwrap_path(q), cell, params)
            cost = out.cost
    if u is None:
        uu = _unwrap_path(q)
        if why == 'wrist3':
            status = ['wrist3' if abs(uu[i, 5] - uu[0, 5]) > params.wrist3_travel_deg or
                      abs(uu[i, 5]) > params.wrist3_abs_deg else 'ok' for i in range(n)]
            if all(s == 'ok' for s in status):
                status = ['wrist3'] * n
        else:
            status = ['limit'] * n
        res = _reject(seam['seam_id'], arc, mode, geo, _reason_text(status), status,
                      [Counter({why: 1})] * n, 0, [])
        attempt['result'] = 'rejected'
        return res, 'final', attempt
    pts, planning = _finish(kin, cell, geo, thetas, psis, labels, u, params, mode, cost, closed_len, arc)
    if planning['fk_max_error_mm'] > params.fk_tol_mm or planning['fk_max_error_deg'] > params.fk_tol_deg:
        raise RuntimeError(f'FK of the planned joints misses the seam pose by {planning["fk_max_error_mm"]} mm / '
                           f'{planning["fk_max_error_deg"]} deg')
    return ArcResult(seam['seam_id'], arc, mode, True, 'ok', pts, planning, [], [], 1.0, geo.pitch_mm), 'ok', attempt


def _window_centres(params: SyncParams) -> List[float]:
    h = params.wrist3_window_half_deg
    k = int(math.ceil(180.0 / (2.0 * h) - 1e-9))
    return [float(-180.0 + h + 2.0 * h * i) for i in range(k)]


def _reason_text(status: Sequence[str]) -> str:
    c = Counter(s for s in status if s != 'ok')
    parts = ', '.join(f'{k} at {v} point(s)' for k, v in sorted(c.items()))
    return f'no continuous 100 % path ({parts})'


def _plan_c(kin, cell, seam, arc, params, geo, progress, label) -> Tuple[ArcResult, str, Dict]:
    n = len(geo.s)
    attempt = {'pitch_mm': geo.pitch_mm, 'points': n}
    if not seam.get('coaxial', {}).get('is_coaxial', False):
        status = ['limit'] * n
        res = _reject(seam['seam_id'], arc, 'C', geo, 'mode C refused: seam is not coaxial with the positioner axis '
                      '(seam_axis.json coaxial test)', status, [Counter()] * n, 0, [])
        res.failing_ranges, res.failing_points = [], []
        return res, 'final', attempt
    # residual of the rigid rotation: point i brought to the angle of point 0 vs point 0 (F8)
    f0 = rot_x(-geo.phi[0]) @ geo.frames[0]
    res_mm, res_deg = [], []
    for i in range(n):
        dp, dr = pose_error(f0, rot_x(-geo.phi[i]) @ geo.frames[i])
        res_mm.append(dp)
        res_deg.append(float(units.rad_to_deg(dr)))
    bad = [i for i in range(n) if res_mm[i] > params.mode_c_tol_mm or res_deg[i] > params.mode_c_tol_deg]
    if bad:
        status = ['ok' if i not in bad else 'step' for i in range(n)]
        return _reject(seam['seam_id'], arc, 'C', geo, f'mode C: {len(bad)} point(s) beyond the F8 tolerance '
                       f'({params.mode_c_tol_mm} mm / {params.mode_c_tol_deg} deg) of a rigid rotation', status,
                       [Counter()] * n, 0, []), 'final', attempt
    dphi = np.abs(np.diff(geo.phi))
    if len(dphi) and dphi.max() > params.dq_max_deg + _EPS:
        status = ['ok'] + ['step' if d > params.dq_max_deg + _EPS else 'ok' for d in dphi]
        return _reject(seam['seam_id'], arc, 'C', geo, _reason_text(status), status, [Counter()] * n, 0, []), \
            'step', attempt
    # α0 lattice: every θ_i = α0 − φ_i inside the limits
    lo, hi = cell.theta_limits_deg
    d = params.theta_step_deg
    k0 = math.ceil((lo + geo.phi.max()) / d - 1e-9)
    k1 = math.floor((hi + geo.phi.min()) / d + 1e-9)
    alphas = np.array([k * d for k in range(k0, k1 + 1)], dtype=float)
    if progress:
        progress(0.1, f'{label}: mode C, {len(alphas)} positioner start angles')
    if not len(alphas):
        status = ['limit'] * n
        return _reject(seam['seam_id'], arc, 'C', geo, 'mode C: the positioner limits do not allow the turn',
                       status, [Counter({'limit': 1})] * n, 0, []), 'final', attempt
    lay = _layer(kin, cell, geo.frames[0], alphas - geo.phi[0], params)
    if not lay.valid.any():
        status = [_point_status(lay)] + ['ok'] * (n - 1)
        return _reject(seam['seam_id'], arc, 'C', geo, _reason_text(status), status, [lay.reasons] + [Counter()] *
                       (n - 1), 0, []), 'final', attempt
    ups = np.array([seam_z_up_deg(cell, geo.frames[i], alphas - geo.phi[i]) for i in range(n)])  # (n, A)
    theta_cost = (params.w_dq + params.w_theta) * float(np.abs(np.diff(geo.phi)).sum())   # same for every state
    node = params.w_up * ups.sum(axis=0)                                  # (A,)
    cost = np.where(lay.valid, (node + theta_cost)[:, None, None], np.inf)
    # F11 absolute: some 360 deg representative of q6 within the soft limit
    q6 = lay.qc[lay.cls][..., 5].astype(float)
    ok6 = np.ceil((-params.wrist3_abs_deg - q6) / 360.0 - 1e-12) <= np.floor((params.wrist3_abs_deg - q6) / 360.0
                                                                           + 1e-12)
    with np.errstate(invalid='ignore'):
        cost = np.where(ok6, cost, np.inf)
    flat = cost.ravel()
    fin = np.nonzero(np.isfinite(flat))[0]
    if not len(fin):
        status = ['wrist3'] * n
        return _reject(seam['seam_id'], arc, 'C', geo, _reason_text(status), status, [Counter()] * n, 0, []), \
            'final', attempt
    jj, bb, pp = np.meshgrid(*(np.arange(k) for k in cost.shape), indexing='ij')
    tie = ((pp * LABEL_COUNT + bb) * _TIE_J + jj).ravel()
    best = int(fin[np.lexsort((tie[fin], np.round(flat[fin], 9)))[0]])
    a, b, p = np.unravel_index(best, cost.shape)
    thetas = alphas[a] - geo.phi
    psis = np.full(n, params.psi_grid[p])
    q0 = _path_q(kin, cell, geo, thetas[:1], psis[:1], np.array([b]))[0]
    u, why = _fit_limits(np.tile(q0, (n, 1)), cell, params)
    if u is None:
        status = [why] * n
        return _reject(seam['seam_id'], arc, 'C', geo, _reason_text(status), status, [Counter()] * n, 0, []), \
            'final', attempt
    closed_len = float(seam['length_mm']) if seam['closed'] and arc.of == 1 else None
    pts, planning = _finish(kin, cell, geo, thetas, psis, np.full(n, b), u, params, 'C', float(flat[best]),
                            closed_len, arc)
    planning['mode_c_residual_mm'] = float(max(res_mm))
    planning['mode_c_residual_deg'] = float(max(res_deg))
    if planning['fk_max_error_mm'] > params.mode_c_tol_mm or planning['fk_max_error_deg'] > params.mode_c_tol_deg:
        raise RuntimeError('mode C: FK of the robot state misses a seam pose beyond the F8 tolerance')
    return ArcResult(seam['seam_id'], arc, 'C', True, 'ok', pts, planning, [], [], 1.0, geo.pitch_mm), 'ok', attempt


def plan_arc(kin, cell: SyncCell, seam: Dict, arc: ArcSpec, mode: str, params: SyncParams = SyncParams(),
             progress: ProgressFn = None) -> ArcResult:
    """Plan one arc of a ``seam_axis.json`` seam in ``mode`` (sync | C | B). 100 % or rejected (no partial path).
    The pitch starts at the seam spacing and is halved on a step break down to ``pitch_floor_mm`` (CP-53 F1)."""
    if mode not in MODES:
        raise ValueError(f'unknown positioner mode {mode!r}')
    if mode == 'B' and arc.positioner_angle_deg is None:
        raise ValueError(f'{seam["seam_id"]} arc {arc.index}: mode B needs positioner_angle_deg (Engineer data)')
    pitch = float(seam['spacing_mm'])
    attempts: List[Dict] = []
    cache: Dict = {}
    label = f'{seam["seam_id"]} arc {arc.index}/{arc.of} {mode}'
    for r in range(params.max_refinements + 1):
        geo = arc_geometry(seam, arc, pitch)
        if mode == 'C':
            res, kind, att = _plan_c(kin, cell, seam, arc, params, geo, progress, label)
        else:
            res, kind, att = _plan_ladder(kin, cell, seam, arc, mode, params, geo, progress, label, cache)
        att['result'] = 'ok' if res.ok else 'rejected'
        attempts.append(att)
        res.attempts = attempts
        if kind != 'step' or pitch / 2.0 < params.pitch_floor_mm - 1e-12 or r == params.max_refinements:
            return res
        pitch /= 2.0
        if progress:
            progress(0.0, f'{label}: step break, pitch refined to {pitch:.4g} mm (CP-53 F1)')
    return res


def timing(res: ArcResult, travel_speed_mm_s: float, cell: SyncCell) -> Tuple[List[float], Dict]:
    """A47: times at constant travel speed relative to the part (t = Δs / v) and the joint speeds they imply
    (reported against the velocity limits; the travel speed is a process value and is never changed here)."""
    if travel_speed_mm_s <= 0.0:
        raise ValueError('travel speed must be > 0')
    s = np.array([p.s_mm for p in res.points])
    t = (s - s[0]) / travel_speed_mm_s
    q7 = np.array([list(p.q_deg) + [p.theta_deg] for p in res.points])
    dt = np.diff(t)
    speed = (np.abs(np.diff(q7, axis=0)) / dt[:, None]).max(axis=0) if len(dt) else np.zeros(7)
    lim = np.array(list(cell.joint_velocity_deg_s) + [cell.theta_velocity_deg_s])
    ratio = float((speed / lim).max())
    return [float(v) for v in t], {'max_joint_speed_deg_s': [float(v) for v in speed],
                                   'speed_within_limits': bool(ratio <= 1.0 + 1e-12),
                                   'max_travel_speed_mm_s': float(travel_speed_mm_s / ratio) if ratio > 0 else 0.0}


__all__ = ['MODES', 'SyncParams', 'SyncCell', 'ArcSpec', 'ArcGeometry', 'ArcResult', 'PlannedPoint', 'rot_x',
           'flange_point_at', 'tcp_target', 'seam_z_up_deg', 'arc_geometry', 'plan_arc', 'timing']
