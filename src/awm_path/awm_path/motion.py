"""A44 IK / reach and A45 collision check of one seam (approach, weld, retract) via move_group.

A seam is accepted only if the approach, weld and retract Cartesian segments are each 100 %
complete and collision-free from one IK solution of the approach pose (no partial paths, no
unchecked fallback moves: demo findings F01, F07).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

from awm_path.moveit_client import MoveItClient, TrajectoryPoints
from awm_path.params import ChainParams

FULL = 1.0 - 1e-9
WRIST_2 = 4


@dataclass
class SeamMotion:
    seam_id: str
    ok: bool
    reason: str
    ik_start: Optional[np.ndarray] = None
    approach: Optional[TrajectoryPoints] = None
    weld: Optional[TrajectoryPoints] = None
    retract: Optional[TrajectoryPoints] = None
    attempts: List[str] = field(default_factory=list)


def wrist_margin(q: np.ndarray) -> float:
    """Distance (rad) of wrist_2 from its singular values 0 and ±pi."""
    return float(np.min(np.abs(np.sin(np.atleast_2d(q)[:, WRIST_2]))))


def ik_candidates(client: MoveItClient, pose: np.ndarray, params: ChainParams,
                  extra_seeds: Sequence[Sequence[float]] = ()) -> List[np.ndarray]:
    """Distinct collision-free IK solutions of ``pose``, nearest to home first.

    Seeds: given seeds, home, then a deterministic pseudo-random set (same plan every run).
    """
    home = np.asarray(params.home_joints_rad)
    rng = np.random.default_rng(20261019)
    seeds = [np.asarray(s, float) for s in extra_seeds] + [home]
    seeds += [home + rng.uniform(-np.pi, np.pi, 6) * np.array([1, 0.5, 0.8, 1, 1, 1])
              for _ in range(params.ik_seed_count)]
    margin = np.sin(params.min_wrist_singularity_margin_rad)
    solutions: List[np.ndarray] = []
    for seed in seeds:
        q = client.compute_ik(pose, seed, timeout_s=params.ik_timeout_s)
        if q is None or wrist_margin(q) < margin:
            continue
        if any(np.max(np.abs(q - s)) < 1e-3 for s in solutions):
            continue
        solutions.append(q)
    solutions.sort(key=lambda q: float(np.sum(np.abs(q - home))))
    return solutions


def plan_seam_motion(client: MoveItClient, seam_id: str, approach: np.ndarray,
                     weld: Sequence[np.ndarray], retract: np.ndarray, params: ChainParams,
                     seeds: Sequence[Sequence[float]] = (), max_candidates: int = 8) -> SeamMotion:
    """Find one IK branch for which approach, weld and retract are all 100 % (base-frame poses)."""
    candidates = ik_candidates(client, approach, params, seeds)
    if not candidates:
        return SeamMotion(seam_id, False, 'approach pose: no collision-free IK solution '
                          '(out of reach, wrist singular or colliding)')
    attempts: List[str] = []
    best = 0.0
    scale = (params.velocity_scaling, params.acceleration_scaling)
    margin = np.sin(params.min_wrist_singularity_margin_rad)
    for k, q0 in enumerate(candidates[:max_candidates]):
        f_app, t_app = client.cartesian_path(q0, [weld[0]], params.cartesian_max_step_m,
                                             params.revolute_jump_threshold_rad, *scale)
        if f_app < FULL or t_app is None:
            attempts.append(f'IK#{k}: approach {f_app:.0%}')
            continue
        f_weld, t_weld = client.cartesian_path(t_app.positions[-1], list(weld[1:]),
                                               params.cartesian_max_step_m,
                                               params.revolute_jump_threshold_rad, *scale)
        best = max(best, f_weld)
        if f_weld < FULL or t_weld is None:
            attempts.append(f'IK#{k}: weld {f_weld:.1%}')
            continue
        if wrist_margin(t_weld.positions) < margin:
            attempts.append(f'IK#{k}: weld passes near the wrist singularity')
            continue
        f_ret, t_ret = client.cartesian_path(t_weld.positions[-1], [retract],
                                             params.cartesian_max_step_m,
                                             params.revolute_jump_threshold_rad, *scale)
        if f_ret < FULL or t_ret is None:
            attempts.append(f'IK#{k}: retract {f_ret:.0%}')
            continue
        attempts.append(f'IK#{k}: ok')
        return SeamMotion(seam_id, True, f'100 % reachable and collision-free (IK branch {k} of '
                          f'{len(candidates)})', q0, t_app, t_weld, t_ret, attempts)
    return SeamMotion(seam_id, False,
                      f'no IK branch gives a 100 % collision-free path (best weld fraction '
                      f'{best:.1%}; tried {"; ".join(attempts)})', attempts=attempts)
