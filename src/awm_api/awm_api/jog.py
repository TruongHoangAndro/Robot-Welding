"""CP-64 jog page kinematics (S11): FK / IK in mm and degrees, in-process, no ROS graph.

Poses are ``{"xyz_mm": [x, y, z], "quat_xyzw": [x, y, z, w]}`` (same shape as ``head.yaml`` ``tool0_T_tcp_mm``). All
frames are ``base_link``; the unit conversion to metres / radians lives inside ``UrAnalyticKinematics`` (ADR-14).
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from awm_api import cad_api
from awm_api.settings import Settings
from awm_core import frames
from awm_path.core.kinematics import wrap_deg
from awm_path.core.states import SingularityThresholds, singular_reason, singularity_metrics
from awm_path.core.sweep import UR5E_JOINT_LIMITS_DEG, PlannerParams

#: Same F6 thresholds as the planner (``PlannerParams`` defaults), so S11 and the planner agree on "singular".
THRESHOLDS: SingularityThresholds = PlannerParams().singularity_thresholds


def pose_to_matrix(pose: Dict[str, Any]) -> np.ndarray:
    """``{xyz_mm, quat_xyzw}`` -> 4x4 transform (mm)."""
    return frames.make_transform(pose['xyz_mm'], frames.quat_to_matrix(pose['quat_xyzw']))


def matrix_to_pose(t: np.ndarray) -> Dict[str, List[float]]:
    """4x4 transform (mm) -> ``{xyz_mm, quat_xyzw}``."""
    t = np.asarray(t, dtype=float)
    return {'xyz_mm': [float(v) for v in t[:3, 3]],
            'quat_xyzw': [float(v) for v in frames.matrix_to_quat(t[:3, :3])]}


def tool0_T_tcp(settings: Settings) -> Optional[np.ndarray]:
    """Taught / mapped ``tool0_T_tcp`` of ``head.yaml`` (mm) or None when there is no TCP yet."""
    rec = cad_api.head_tcp_record(settings.head_dir / 'head.yaml')
    if rec is None or rec.get('xyz_mm') is None or rec.get('quat_xyzw') is None:
        return None
    try:
        return pose_to_matrix({'xyz_mm': rec['xyz_mm'], 'quat_xyzw': rec['quat_xyzw']})
    except (KeyError, TypeError, ValueError):
        return None


def wrist_singular(q_deg: Sequence[float]) -> Optional[str]:
    """Planner F6 reason (``singular_wrist`` / ``singular_elbow`` / ``singular_shoulder``) or None when clear."""
    return singular_reason(singularity_metrics(q_deg), THRESHOLDS)


def nearest_solution(solutions: Sequence[Sequence[float]], current_deg: Sequence[float]) -> List[float]:
    """Solution closest to ``current_deg`` by wrapped joint distance, unwrapped toward the current pose.

    Distance = sum |wrap_deg(q - current)|. Each joint is returned as ``current + wrap_deg(q - current)`` (no full-turn
    spin) unless that leaves the UR5e joint limit, then the in-range solution value is kept.
    """
    cur = np.asarray(current_deg, dtype=float)
    best = min((np.asarray(q, dtype=float) for q in solutions),
               key=lambda q: float(np.abs(wrap_deg(q - cur)).sum()))
    out = cur + wrap_deg(best - cur)
    for i, (lo, hi) in enumerate(UR5E_JOINT_LIMITS_DEG):
        if not lo <= out[i] <= hi:
            out[i] = best[i]
    return [float(v) for v in out]
