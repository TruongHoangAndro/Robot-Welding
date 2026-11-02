"""Joint-state helpers of the M4 state space (WP-S S.3; TST-002 Rev 0.3 §5.1 F2, F6; S.2 follow-ups R1, R2).

* ``config_label`` — the UR configuration label ``S±E±W±`` computed from the joint values only (F2).
* ``unwrapped_variants`` / ``nearest_representative`` — 2π representatives of a joint vector (F2, F3).
* ``singularity_metrics`` / ``singular_reason`` — the three F6 metrics and the rejection reason.
* ``ik_existence`` — analytic existence test used to tell ``no_ik_unreachable`` from ``no_ik_near_singular`` when
  the IK backend returns ``[]`` (R2).

All public values are mm and degrees; radians only through ``awm_core.units``. Pure Python (no rclpy, B-06).

Robot geometry: ``UR5E_GEOMETRY`` holds the nominal UR5e DH parameters in mm (Fact: UR DH table, identical to
``ur_description`` ``config/ur5e/default_kinematics.yaml``). They are used only for the configuration label and the
singularity metrics, never for planning poses (that is the ``KinematicsBackend``). Assumption (S.2 R8): nominal DH =
nominal URDF; calibrated kinematics may change the metrics slightly, not the labels of non-singular states.
The DH chain is expressed in the UR controller ``base`` frame (library base), which is ``base_link`` rotated 180° about
Z (DES-001 §5.3); ``ik_existence`` takes ``base_link_T_tool0`` like ``KinematicsBackend.ik``.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from awm_core import units
from awm_path.core.kinematics import LIB_BASE_T_BASE_LINK, check_rigid, wrap_deg


@dataclass(frozen=True)
class UrGeometry:
    """UR DH parameters in mm (standard DH, alpha = [90, 0, 0, 90, -90, 0] deg)."""

    d1: float
    a2: float
    a3: float
    d4: float
    d5: float
    d6: float


#: Nominal UR5e DH parameters (mm).
UR5E_GEOMETRY = UrGeometry(d1=162.5, a2=-425.0, a3=-392.2, d4=133.3, d5=99.7, d6=99.6)

_ALPHA_DEG = (90.0, 0.0, 0.0, 90.0, -90.0, 0.0)

#: The 8 configuration labels in their fixed order (F7 tie-break "configuration-label order").
LABELS: Tuple[str, ...] = tuple(f'S{s}E{e}W{w}' for s, e, w in itertools.product('+-', repeat=3))
LABEL_ORDER = {label: i for i, label in enumerate(LABELS)}

#: Values whose magnitude is below these are treated as zero and get the '+' flag (deterministic boundary).
_SIN_ZERO = 1e-12
_MM_ZERO = 1e-9
#: Tolerance of the analytic existence test (cosines) and of the shoulder cylinder (mm).
_COS_TOL = 1e-9
_CYL_TOL_MM = 1e-6
#: Tolerance of joint-limit membership (deg).
_LIMIT_TOL_DEG = 1e-9


def _sin(deg: float) -> float:
    return math.sin(units.deg_to_rad(deg))


def _cos(deg: float) -> float:
    return math.cos(units.deg_to_rad(deg))


def _dh(theta_deg: float, d: float, a: float, alpha_deg: float) -> np.ndarray:
    ct, st, ca, sa = _cos(theta_deg), _sin(theta_deg), _cos(alpha_deg), _sin(alpha_deg)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def wrist_centre_lib_mm(q_deg: Sequence[float], geo: UrGeometry = UR5E_GEOMETRY) -> np.ndarray:
    """Wrist centre = origin of DH frame 5 (intersection of the wrist-2 and wrist-3 axes), UR ``base`` frame, mm."""
    q = np.asarray(q_deg, dtype=float)
    d = (geo.d1, 0.0, 0.0, geo.d4, geo.d5)
    a = (0.0, geo.a2, geo.a3, 0.0, 0.0)
    t = np.eye(4)
    for i in range(5):
        t = t @ _dh(float(q[i]), d[i], a[i], _ALPHA_DEG[i])
    return t[:3, 3].copy()


def _shoulder_u_mm(q_deg: Sequence[float], geo: UrGeometry) -> float:
    """Signed distance of the wrist centre from the plane of the joint-1 and joint-2 axes (mm).

    In the frame turned by q1 the wrist centre has a constant component d4 along the joint-2 axis; the remaining
    horizontal component ``u`` is the planar reach. The two shoulder IK branches have ``u = ±sqrt(r² - d4²)``.
    """
    c = wrist_centre_lib_mm(q_deg, geo)
    q1 = float(np.asarray(q_deg, dtype=float)[0])
    return _cos(q1) * c[0] + _sin(q1) * c[1]


def _flag(x: float, zero: float) -> str:
    return '-' if x < -zero else '+'


def config_label(q_deg: Sequence[float], geo: UrGeometry = UR5E_GEOMETRY) -> str:
    """Configuration label ``S±E±W±`` from the joint values only (TST-002 F2, S.2 R1).

    * shoulder S = sign of ``u``, the planar reach of the wrist centre in the vertical plane turned by q1
      (``u = cos q1 · x_wc + sin q1 · y_wc`` in the UR ``base`` frame) — the two q1 IK branches have opposite signs;
    * elbow E = sign(sin q3); wrist W = sign(sin q5).

    Values within 1e-12 (sines) / 1e-9 mm of zero map to '+' (deterministic at the singular boundaries). The label
    is invariant under 360° offsets of any joint.
    """
    q = np.asarray(q_deg, dtype=float)
    return ('S' + _flag(_shoulder_u_mm(q, geo), _MM_ZERO)
            + 'E' + _flag(_sin(float(q[2])), _SIN_ZERO)
            + 'W' + _flag(_sin(float(q[4])), _SIN_ZERO))


def unwrapped_variants(q_deg: Sequence[float], limits_deg: Sequence[Tuple[float, float]]) -> List[np.ndarray]:
    """All representatives ``q + 360·k`` (integer k per joint) inside the joint limits, in lexicographic order.

    Returns ``[]`` if some joint has no representative inside its limits.
    """
    q = np.asarray(q_deg, dtype=float)
    per_joint: List[List[float]] = []
    for x, (lo, hi) in zip(q, limits_deg):
        k_lo = math.ceil((lo - _LIMIT_TOL_DEG - x) / 360.0)
        k_hi = math.floor((hi + _LIMIT_TOL_DEG - x) / 360.0)
        per_joint.append([float(x + 360.0 * k) for k in range(k_lo, k_hi + 1)])
    return [np.array(v) for v in itertools.product(*per_joint)]


def nearest_representative(q_deg: Sequence[float], prev_unwrapped_deg: Sequence[float]) -> np.ndarray:
    """Per joint, the 360° representative of ``q`` nearest to the predecessor (|difference| ≤ 180°)."""
    prev = np.asarray(prev_unwrapped_deg, dtype=float)
    return prev + wrap_deg(np.asarray(q_deg, dtype=float) - prev)


@dataclass(frozen=True)
class SingularityMetrics:
    """F6 metrics: |sin q5| (wrist), |sin q3| (elbow), wrist-centre distance (mm, shoulder; see ``singularity_metrics``)."""

    sin_q5: float
    sin_q3: float
    wrist_centre_mm: float


@dataclass(frozen=True)
class SingularityThresholds:
    """F6 thresholds (D16): wrist ε_w (deg), elbow ε_e (deg), shoulder d_s (mm)."""

    wrist_deg: float
    elbow_deg: float
    shoulder_mm: float


def singularity_metrics(q_deg: Sequence[float], geo: UrGeometry = UR5E_GEOMETRY) -> SingularityMetrics:
    """F6 metrics of a joint state.

    ``wrist_centre_mm`` = |u|, the distance of the wrist centre from the plane spanned by the joint-1 and joint-2
    axes, i.e. the wrist-centre distance from the base Z axis measured in the arm plane (shoulder offset d4 removed).
    Assumption (F6 wording "wrist centre ≥ 50 mm from the base Z axis"): the radial distance itself is always
    ≥ d4 = 133.3 mm on a UR5e and could never trigger the 50 mm threshold; |u| is 0 exactly at the shoulder
    singularity (both q1 branches coincide).
    """
    q = np.asarray(q_deg, dtype=float)
    return SingularityMetrics(sin_q5=abs(_sin(float(q[4]))), sin_q3=abs(_sin(float(q[2]))),
                              wrist_centre_mm=abs(_shoulder_u_mm(q, geo)))


def singular_reason(metrics: SingularityMetrics, thresholds: SingularityThresholds) -> Optional[str]:
    """``None`` if the state is clear of all three F6 singularities, else the first failing one.

    Valid iff |sin q5| ≥ sin ε_w, |sin q3| ≥ sin ε_e and wrist centre ≥ d_s (boundary values are valid).
    Precedence: ``singular_wrist``, ``singular_elbow``, ``singular_shoulder``.
    """
    if metrics.sin_q5 < _sin(thresholds.wrist_deg):
        return 'singular_wrist'
    if metrics.sin_q3 < _sin(thresholds.elbow_deg):
        return 'singular_elbow'
    if metrics.wrist_centre_mm < thresholds.shoulder_mm:
        return 'singular_shoulder'
    return None


def ik_existence(base_link_T_tool0: np.ndarray, geo: UrGeometry = UR5E_GEOMETRY) -> str:
    """Analytic IK existence test for a ``tool0`` pose: ``'reachable'``, ``'singular'`` or ``'unreachable'``.

    Follows the closed-form UR solution: the shoulder needs r ≥ d4 (r = horizontal distance of the wrist centre),
    the wrist needs |cos q5| ≤ 1, the elbow |cos q3| ≤ 1, per branch. ``'singular'`` if any branch sits on one of
    these boundaries (within 1e-9 / 1e-6 mm) — the pose is inside the reach envelope but on a singularity, where the
    backend may return ``[]`` (R2). ``'reachable'`` if some branch is strictly feasible, else ``'unreachable'``.
    """
    t = LIB_BASE_T_BASE_LINK @ check_rigid(base_link_T_tool0)
    p6, r6 = t[:3, 3], t[:3, :3]
    p5 = p6 - geo.d6 * r6[:, 2]
    r = math.hypot(p5[0], p5[1])
    if r < geo.d4 - _CYL_TOL_MM:
        return 'unreachable'
    if r <= geo.d4 + _CYL_TOL_MM:
        return 'singular'
    phi = math.atan2(p5[1], p5[0])
    half = math.acos(geo.d4 / r)
    singular = feasible = False
    for q1_rad in (phi + half + math.pi / 2.0, phi - half + math.pi / 2.0):
        s1, c1 = math.sin(q1_rad), math.cos(q1_rad)
        c5 = (p6[0] * s1 - p6[1] * c1 - geo.d4) / geo.d6
        if abs(c5) > 1.0 + _COS_TOL:
            continue
        if abs(c5) >= 1.0 - _COS_TOL:
            singular = True
            continue
        for q5_rad in (math.acos(c5), -math.acos(c5)):
            s5 = math.sin(q5_rad)
            q6_rad = math.atan2((-r6[0, 1] * s1 + r6[1, 1] * c1) / s5, (r6[0, 0] * s1 - r6[1, 0] * c1) / s5)
            r56 = _dh(units.rad_to_deg(q6_rad), 0.0, 0.0, 0.0)[:3, :3]
            r45 = _dh(units.rad_to_deg(q5_rad), 0.0, 0.0, _ALPHA_DEG[4])[:3, :3]
            r04 = r6 @ r56.T @ r45.T
            o4 = p5 - geo.d5 * r04[:, 2]
            w = o4 - np.array([0.0, 0.0, geo.d1])
            z1 = np.array([s1, -c1, 0.0])
            planar2 = float(w @ w - (w @ z1) ** 2)
            c3 = (planar2 - geo.a2 ** 2 - geo.a3 ** 2) / (2.0 * geo.a2 * geo.a3)
            if abs(c3) > 1.0 + _COS_TOL:
                continue
            if abs(c3) >= 1.0 - _COS_TOL:
                singular = True
            else:
                feasible = True
    if singular:
        return 'singular'
    return 'reachable' if feasible else 'unreachable'
