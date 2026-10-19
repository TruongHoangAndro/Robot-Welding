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
COLLISION_STEP_RAD = 0.02  # joint-space resolution of the dense collision check (M6 and A71)


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


def densify(q: np.ndarray, step_rad: float = COLLISION_STEP_RAD) -> np.ndarray:
    """Insert linearly interpolated states so no joint moves more than ``step_rad`` per step."""
    q = np.asarray(q, float)
    out = [q[0]]
    for a, b in zip(q, q[1:]):
        n = max(1, int(np.ceil(np.max(np.abs(b - a)) / step_rad)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * k / n)
    return np.array(out)


def first_collision(client: MoveItClient, q: np.ndarray,
                    step_rad: float = COLLISION_STEP_RAD) -> Optional[str]:
    """Contacts of the first invalid state along a densified joint path, None if all valid."""
    for qi in densify(q, step_rad):
        valid, contacts = client.state_valid(qi)
        if not valid:
            return ', '.join(contacts) or 'invalid state'
    return None


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
        q = client.compute_ik(pose, seed, timeout_s=params.ik_timeout_s,
                              group=params.ik_search_group)
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

    def cartesian(start, poses):
        """Best of ``cartesian_retries`` tries (pick_ik is randomised); stops at 100 %."""
        best_f, best_t = 0.0, None
        for _ in range(max(1, params.cartesian_retries)):
            f, t = client.cartesian_path(start, poses, params.cartesian_max_step_m,
                                         params.revolute_jump_threshold_rad, *scale)
            if f >= best_f:
                best_f, best_t = f, t
            if f >= FULL and t is not None:
                break
        return best_f, best_t

    for k, q0 in enumerate(candidates[:max_candidates]):
        f_app, t_app = cartesian(q0, [weld[0]])
        if f_app < FULL or t_app is None:
            attempts.append(f'IK#{k}: approach {f_app:.0%}')
            continue
        f_weld, t_weld = cartesian(t_app.positions[-1], list(weld[1:]))
        best = max(best, f_weld)
        if f_weld < FULL or t_weld is None:
            attempts.append(f'IK#{k}: weld {f_weld:.1%}')
            continue
        if wrist_margin(t_weld.positions) < margin:
            attempts.append(f'IK#{k}: weld passes near the wrist singularity')
            continue
        f_ret, t_ret = cartesian(t_weld.positions[-1], [retract])
        if f_ret < FULL or t_ret is None:
            attempts.append(f'IK#{k}: retract {f_ret:.0%}')
            continue
        attempts.append(f'IK#{k}: ok')
        return SeamMotion(seam_id, True, f'100 % reachable and collision-free (IK branch {k} of '
                          f'{len(candidates)})', q0, t_app, t_weld, t_ret, attempts)
    return SeamMotion(seam_id, False,
                      f'no IK branch gives a 100 % collision-free path (best weld fraction '
                      f'{best:.1%}; tried {"; ".join(attempts)})', attempts=attempts)
