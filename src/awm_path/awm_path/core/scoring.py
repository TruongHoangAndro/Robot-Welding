"""Scene scores of the kinematic candidates (WP-S S.4 C5; TST-002 Rev 0.3 draft §5.1 F4, F5, F9, §5.3 C2, C5).

For every candidate of the S.3 sweep that is kinematically valid:

* F4 validity (3 mm padding on every link except ``nozzle_tip``) — one batched ``check`` call;
* F5 clearance on unpadded geometry at increasing levels (default 5 mm = weld clearance, 10 mm) — one
  ``check_clearance`` call per level on the states that passed the previous level; the clearance of a node is the
  highest level passed (a **lower bound**, method recorded by the backend);
* F9 contact — once per seam point (the verdict is independent of ψ and of the IK branch, ``awm_path.core.contact``).

``NodeScore.valid`` = F4 ∧ clearance ≥ weld clearance ∧ contact (TST-002 C2); ``NodeScore.clearance_mm`` = the
lower bound (feeds the low-clearance cost of the ladder, never makes a node valid). Kinematic rejects are not sent
to the scene. Pure Python (B-06); the scene is any object with ``check(states_deg, theta_deg)`` and
``check_clearance(states_deg, theta_deg, clearance_mm)``.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from awm_path.core.contact import ContactParams, ContactResult, PartMesh, contact_check
from awm_path.core.ladder import NodeScore
from awm_path.core.sweep import Candidate, tcp_pose

#: Scene columns of ``candidates.csv`` before / after the clearance-level columns ``clear_<level>mm``.
_COLUMNS_HEAD = ('scene_valid', 'scene_contacts')
_COLUMNS_TAIL = ('clearance_lb_mm', 'clearance_blocking', 'contact_ok', 'contact_reason', 'node_valid')


def _level_column(level_mm: float) -> str:
    return f'clear_{level_mm:g}mm'


@dataclass(frozen=True)
class SceneScoreParams:
    """Clearance levels (mm, ascending) and the weld clearance that a valid node must meet (D16 / TST-002 C2)."""

    weld_clearance_mm: float = 5.0
    clearance_levels_mm: Tuple[float, ...] = (5.0, 10.0)

    def __post_init__(self) -> None:
        levels = tuple(float(x) for x in self.clearance_levels_mm)
        if list(levels) != sorted(set(levels)) or not levels:
            raise ValueError('clearance_levels_mm must be strictly ascending and non-empty')
        if float(self.weld_clearance_mm) not in levels:
            raise ValueError('weld_clearance_mm must be one of the clearance levels')
        object.__setattr__(self, 'clearance_levels_mm', levels)


def scene_columns(params: 'SceneScoreParams') -> List[str]:
    """Names of the scene columns added to ``candidates.csv`` (one ``clear_<level>mm`` column per level)."""
    return list(_COLUMNS_HEAD) + [_level_column(lv) for lv in params.clearance_levels_mm] + list(_COLUMNS_TAIL)


@dataclass
class ScoredLayers:
    """Scores per candidate, the ``candidates.csv`` scene columns, contact result per point, timing record."""

    scores: List[List[NodeScore]]
    columns: Dict[str, List[List[Any]]]
    contact: List[ContactResult]
    timing: Dict[str, Any]


def score_candidates(layers: Sequence[Sequence[Candidate]], frames_base_mm: Sequence[np.ndarray],
                     base_T_part_mm: np.ndarray, theta_deg: float, scene, part: PartMesh,
                     seam_faces: Tuple[Sequence[str], Sequence[str]], contact_params: ContactParams,
                     params: SceneScoreParams = SceneScoreParams()) -> ScoredLayers:
    """Score every candidate. ``frames_base_mm`` are the seam frames ``base_link_T_seam`` of the sweep."""
    if len(frames_base_mm) != len(layers):
        raise ValueError('one seam frame per candidate layer is required')
    t_start = time.perf_counter()
    part_T_base = np.linalg.inv(np.asarray(base_T_part_mm, dtype=float))

    # F9 contact, once per point
    t0 = time.perf_counter()
    contact = [contact_check(part, part_T_base @ tcp_pose(f, 0.0), seam_faces, contact_params)
               for f in frames_base_mm]
    t_contact = time.perf_counter() - t0

    index: List[Tuple[int, int]] = [(li, ci) for li, layer in enumerate(layers) for ci, c in enumerate(layer)
                                    if c.reason is None]
    states = [layers[li][ci].q_deg for li, ci in index]
    f4: Dict[Tuple[int, int], Any] = {}
    t0 = time.perf_counter()
    if states:
        for key, res in zip(index, scene.check(states, theta_deg)):
            f4[key] = res
    t_valid = time.perf_counter() - t0

    passed: Dict[Tuple[int, int], Optional[float]] = {k: None for k in index}
    level_ok: Dict[float, Dict[Tuple[int, int], bool]] = {lv: {} for lv in params.clearance_levels_mm}
    blocking: Dict[Tuple[int, int], str] = {}
    alive = [k for k in index if f4[k].valid]
    t0 = time.perf_counter()
    level_times = {}
    for lv in params.clearance_levels_mm:
        if not alive:
            break
        tl = time.perf_counter()
        res = scene.check_clearance([layers[li][ci].q_deg for li, ci in alive], theta_deg, lv)
        level_times[lv] = time.perf_counter() - tl
        nxt = []
        for k, r in zip(alive, res):
            level_ok[lv][k] = bool(r.valid)
            if r.valid:
                passed[k] = lv
                nxt.append(k)
            else:
                blocking[k] = ';'.join(r.contacts)
        alive = nxt
    t_clear = time.perf_counter() - t0

    scores: List[List[NodeScore]] = []
    names = scene_columns(params)
    cols: Dict[str, List[List[Any]]] = {name: [] for name in names}
    for li, layer in enumerate(layers):
        cres = contact[li]
        row_s: List[NodeScore] = []
        row = {name: [] for name in names}
        for ci, c in enumerate(layer):
            k = (li, ci)
            checked = c.reason is None
            v4 = f4[k].valid if checked else None
            lb = passed.get(k) if checked else None
            weld_ok = lb is not None and lb >= params.weld_clearance_mm
            valid = bool(checked and v4 and weld_ok and cres.ok)
            row_s.append(NodeScore(valid, lb))
            row['scene_valid'].append(v4)
            row['scene_contacts'].append(';'.join(f4[k].contacts) if checked else None)
            for lv in params.clearance_levels_mm:
                row[_level_column(lv)].append(level_ok[lv].get(k))
            row['clearance_lb_mm'].append(lb)
            row['clearance_blocking'].append(blocking.get(k))
            row['contact_ok'].append(cres.ok)
            row['contact_reason'].append(cres.reason)
            row['node_valid'].append(valid)
        scores.append(row_s)
        for name in names:
            cols[name].append(row[name])
    total = time.perf_counter() - t_start
    n = len(index)
    timing = {'points': len(layers), 'candidates': sum(len(l) for l in layers), 'states_checked': n,
              'contact_checks': len(contact), 'validity_s': t_valid, 'clearance_s': t_clear,
              'clearance_level_s': {f'{k:g}': v for k, v in level_times.items()}, 'contact_s': t_contact,
              'total_s': total, 'per_state_s': (t_valid + t_clear) / n if n else 0.0,
              'contact_per_point_s': t_contact / len(contact) if contact else 0.0}
    return ScoredLayers(scores, cols, contact, timing)


__all__ = ['scene_columns', 'SceneScoreParams', 'ScoredLayers', 'score_candidates']
