"""S.5 helpers: pitch refinement (CP-53 / TST-002 F1), failing-point classes with colliding pairs, C3 metrics
measured on a path, longest continuous window (C8 note) (WP-S S.5; TST-002 Rev 0.3 §5.1 F1, F3, F11, §5.3 C2, C3,
C8; DES-001 Rev 0.6 draft §4.3 steps 4–5).

Pure Python + numpy, mm / degrees (B-06). Nothing here makes an infeasible path feasible: refinement only changes
the pitch below the F1 ceiling, the metrics are read-only.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence

import numpy as np

from awm_path.core.kinematics import wrap_deg
from awm_path.core.ladder import NodeScore, solve
from awm_path.core.sweep import Candidate, PlannerParams, required_points

#: Point classes of a point without a valid node (reported per failing point).
POINT_CLASSES = ('contact', 'IK', 'F4', 'clearance')


def max_label_steps(layers: Sequence[Sequence[Candidate]], scores: Optional[Sequence[Sequence[NodeScore]]] = None,
                    closed: bool = False) -> List[float]:
    """Per step i → i+1: the largest wrapped |Δq| (deg, any joint) between candidates of the same ψ index and the same
    configuration label that are kinematically valid at both points (0 when no such pair exists).

    ``scores`` given (CP-53 rule ``scene_valid``, lead decision 2026-11-04): only candidates whose node score is valid
    (F4 ∧ F5 ∧ F9, the nodes the DP can use) are paired. ``closed``: the step from the last point back to the first
    is appended. On a non-singular pose one ψ has at most one solution per label, so the pair is the continuation of
    that state.
    """
    def keyed(li: int, layer: Sequence[Candidate]) -> Dict[Any, np.ndarray]:
        return {(c.psi_index, c.label): np.asarray(c.q_deg, dtype=float) for ci, c in enumerate(layer)
                if c.reason is None and (scores is None or scores[li][ci].valid)}

    def step(a: Dict[Any, np.ndarray], b: Dict[Any, np.ndarray]) -> float:
        common = sorted(set(a) & set(b))
        return max((float(np.max(np.abs(wrap_deg(b[k] - a[k])))) for k in common), default=0.0)

    if scores is not None and (len(scores) != len(layers) or any(len(s) != len(l) for s, l in zip(scores, layers))):
        raise ValueError('scores do not match the candidate layers')
    keys = [keyed(li, layer) for li, layer in enumerate(layers)]
    out = [step(a, b) for a, b in zip(keys[:-1], keys[1:])]
    if closed and len(keys) > 1:
        out.append(step(keys[-1], keys[0]))
    return out


#: CP-53 pitch rules: ``kinematic`` = all kinematically valid candidates (S.5 records); ``scene_valid`` = only
#: scene-valid nodes (lead decision 2026-11-04, S.5b onwards).
PITCH_RULES = ('kinematic', 'scene_valid')


@dataclass(frozen=True)
class PitchResult:
    """Outcome of :func:`refine_pitch`. ``ok`` False = floor reached with steps still > Δq_max (reported).
    ``scores`` = the node scores of the final sample (rule ``scene_valid`` only)."""

    ok: bool
    points: int
    pitch_mm: float
    pitch_ceiling_mm: float
    max_step_deg: float
    iterations: int
    layers: List[List[Candidate]]
    rule: str = 'kinematic'
    closed: bool = False
    scores: Optional[List[List[NodeScore]]] = None

    def as_record(self) -> Dict[str, Any]:
        return {'ok': self.ok, 'points': self.points, 'pitch_mm': self.pitch_mm,
                'pitch_ceiling_mm': self.pitch_ceiling_mm, 'max_label_step_deg': self.max_step_deg,
                'iterations': self.iterations, 'rule': self.rule, 'closed': self.closed}


def refine_pitch(length_mm: float, rate_deg_per_mm: float, sample: Callable[[int], Any],
                 params: PlannerParams, pitch_min_mm: float, rule: str = 'kinematic', closed: bool = False
                 ) -> PitchResult:
    """CP-53 pitch rule: start at the F1 ceiling Δs ≤ min(pitch_max, Δq_max / rate), then halve the pitch until every
    step of :func:`max_label_steps` is ≤ Δq_max, or the next pitch would fall below ``pitch_min_mm`` (Assumption,
    recorded).

    ``rule`` ``kinematic``: ``sample(n)`` returns the kinematic sweep at ``n`` points and all kinematically valid
    candidates are paired (S.5). ``scene_valid``: ``sample(n)`` returns ``(layers, scores)`` and only scene-valid
    nodes are paired. Open arcs: ``n`` points from start to end (pitch L / (n − 1)); ``closed`` seams: ``n`` points
    at pitch L / n, the closing step included; doubling keeps every earlier point.
    """
    if rule not in PITCH_RULES:
        raise ValueError(f'unknown pitch rule {rule!r}; expected one of {PITCH_RULES}')
    n = required_points(length_mm, rate_deg_per_mm, params, closed=closed)
    span = (lambda m: length_mm / m) if closed else (lambda m: length_mm / (m - 1))
    ceiling = span(n)
    it = 0
    while True:
        it += 1
        got = sample(n)
        layers, scores = (got if rule == 'scene_valid' else (got, None))
        steps = max_label_steps(layers, scores=scores, closed=closed)
        worst = max(steps, default=0.0)
        pitch = span(n)
        done = worst <= params.dq_max_deg + 1e-9
        if done or pitch / 2.0 < pitch_min_mm:
            return PitchResult(done, n, pitch, ceiling, worst, it, layers, rule, closed, scores)
        n = 2 * n if closed else 2 * (n - 1) + 1


def _pairs(text: Optional[str]) -> List[str]:
    return [p for p in (text or '').split(';') if p]


def classify_points(layers: Sequence[Sequence[Candidate]], columns: Mapping[str, Sequence[Sequence[Any]]]
                    ) -> List[Dict[str, Any]]:
    """Per point: number of valid nodes, counts per candidate outcome and the colliding / blocking pairs.

    Outcomes: the kinematic reason (``no_ik_*``, ``joint_limit``, ``singular_*``), ``f4_invalid``, ``clearance``
    (F4 valid but below the weld clearance), ``contact`` (F9 failed at the point), ``valid``. ``pairs`` counts the
    ``link<->object`` pairs of F4 contacts and of the clearance check. ``contact_ok`` / ``contact_reason`` per point.
    ``columns`` are the scene columns of :func:`awm_path.core.scoring.score_candidates`.
    """
    out = []
    for li, layer in enumerate(layers):
        counts: Counter = Counter()
        pairs: Counter = Counter()
        contact_ok = bool(columns['contact_ok'][li][0]) if layer else True
        for ci, c in enumerate(layer):
            if c.reason is not None:
                counts[c.reason] += 1
            elif columns['node_valid'][li][ci]:
                counts['valid'] += 1
            elif not columns['scene_valid'][li][ci]:
                counts['f4_invalid'] += 1
                pairs.update(_pairs(columns['scene_contacts'][li][ci]))
            elif not columns['contact_ok'][li][ci] and columns['clearance_lb_mm'][li][ci] is not None:
                counts['contact'] += 1
            else:
                counts['clearance'] += 1
                pairs.update(_pairs(columns['clearance_blocking'][li][ci]))
        out.append({'point': li, 'valid_nodes': counts.get('valid', 0), 'counts': dict(sorted(counts.items())),
                    'pairs': dict(sorted(pairs.items(), key=lambda kv: (-kv[1], kv[0]))),
                    'contact_ok': contact_ok,
                    'contact_reason': columns['contact_reason'][li][0] if layer else None})
    return out


def point_classes(points: Sequence[Mapping[str, Any]]) -> List[Optional[str]]:
    """Class of each point without a valid node (None = covered): ``contact`` if F9 fails at the point; else ``IK``
    if no candidate is kinematically valid; else ``F4`` if no candidate passes F4; else ``clearance``."""
    out: List[Optional[str]] = []
    for p in points:
        if p['valid_nodes'] > 0:
            out.append(None)
        elif not p['contact_ok']:
            out.append('contact')
        else:
            c = p['counts']
            kin_valid = sum(v for k, v in c.items() if k in ('f4_invalid', 'clearance', 'contact', 'valid'))
            if kin_valid == 0:
                out.append('IK')
            elif c.get('clearance', 0) == 0 and c.get('contact', 0) == 0:
                out.append('F4')
            else:
                out.append('clearance')
    return out


def path_metrics(path: Sequence[Candidate]) -> Dict[str, Any]:
    """C3 values measured on a returned path (unwrapped joint values): labels used, largest joint step, wrist-3
    travel from the start and largest |q6|."""
    q = np.array([c.q_deg for c in path], dtype=float)
    steps = np.abs(np.diff(q, axis=0)) if len(q) > 1 else np.zeros((0, 6))
    return {'labels': sorted({c.label for c in path}),
            'max_step_deg': float(steps.max()) if steps.size else 0.0,
            'max_step_per_joint_deg': [float(v) for v in (steps.max(axis=0) if steps.size else np.zeros(6))],
            'wrist3_travel_deg': float(np.max(np.abs(q[:, 5] - q[0, 5]))) if len(q) else 0.0,
            'wrist3_abs_max_deg': float(np.max(np.abs(q[:, 5]))) if len(q) else 0.0,
            'psi_deg': sorted({float(c.psi_deg) for c in path})}


def longest_window(layers: Sequence[Sequence[Candidate]], scores: Optional[Sequence[Sequence[NodeScore]]],
                   params: PlannerParams) -> Dict[str, Any]:
    """Longest run of consecutive layers that one continuous DP path covers (open; all F3 / F11 rules). Diagnostic
    for C8 only — never used as a path. Returns start, end (inclusive) and the number of layers (0 if none)."""
    n = len(layers)
    if scores is None:
        scores = [[NodeScore(True)] * len(l) for l in layers]
    best = {'start': None, 'end': None, 'layers': 0}
    for i in range(n):
        if n - i <= best['layers']:
            break
        if not any(s.valid and c.reason is None for c, s in zip(layers[i], scores[i])):
            continue
        res = solve(layers[i:], scores[i:], params, closed=False)
        length = res.max_reached_layer + 1
        if length > best['layers']:
            best = {'start': i, 'end': i + length - 1, 'layers': length}
    return best


__all__ = ['POINT_CLASSES', 'PITCH_RULES', 'max_label_steps', 'PitchResult', 'refine_pitch', 'classify_points', 'point_classes',
           'path_metrics', 'longest_window']
