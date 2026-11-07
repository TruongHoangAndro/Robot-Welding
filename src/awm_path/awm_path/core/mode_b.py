"""Mode B screening helpers (WP-S S.5b; DES-001 Rev 0.6 draft §4.3 step 3, §6.3 Mode B, B-12; TST-002 Rev 0.3 draft
§5.1 F3, F11, §5.3 C2, C8).

Mode B = rotate the positioner with emission off, stop, weld one segment with the robot alone. The positioner angles
and split points are **Engineer data** (B-12); automatic Mode B search is R3. These helpers only *screen*: per angle
θ they report where one continuous robot path exists on the closed seam, and list angle sets that would cover the
seam as **suggestions** for the Engineer. Nothing here plans or accepts a path.

Pure Python + numpy, mm / degrees (B-06).
"""

from __future__ import annotations

import itertools
import math
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from awm_path.core.ladder import NodeScore, solve
from awm_path.core.sweep import Candidate, PlannerParams

_TOL = 1e-9

Interval = Tuple[float, float]


# ---------------------------------------------------------------------------------------------------------------
# θ grid
# ---------------------------------------------------------------------------------------------------------------

def screening_thetas(lower_deg: float, upper_deg: float, step_deg: float) -> List[float]:
    """Every multiple of ``step_deg`` inside [lower, upper] (0 included; 0 must lie inside the limits)."""
    if step_deg <= 0.0:
        raise ValueError('step_deg must be > 0')
    if not lower_deg - _TOL <= 0.0 <= upper_deg + _TOL:
        raise ValueError('0 deg must lie inside the positioner limits')
    k0 = math.ceil(lower_deg / step_deg - _TOL)
    k1 = math.floor(upper_deg / step_deg + _TOL)
    return [float(k * step_deg) + 0.0 for k in range(k0, k1 + 1)]


def theta_representative(theta_deg: float) -> float:
    """Geometric class of a positioner angle: θ and θ ± 360° place the part and the holders identically. Returns the
    representative in (−180°, 180°]."""
    r = math.fmod(float(theta_deg), 360.0)
    if r <= -180.0 + _TOL:
        r += 360.0
    elif r > 180.0 + _TOL:
        r -= 360.0
    return float(round(r, 9)) + 0.0


def theta_classes(thetas: Sequence[float]) -> Dict[float, List[float]]:
    """Representative → the given angles of that class (both sorted ascending)."""
    out: Dict[float, List[float]] = {}
    for t in sorted(float(x) for x in thetas):
        out.setdefault(theta_representative(t), []).append(t)
    return dict(sorted(out.items()))


# ---------------------------------------------------------------------------------------------------------------
# runs and windows on a closed seam
# ---------------------------------------------------------------------------------------------------------------

def circular_runs(flags: Sequence[bool]) -> List[Tuple[int, int]]:
    """Runs of True on a circle as (start, end) inclusive; a run over the seam start has end ≥ n (unwrapped index).
    All True → [(0, n − 1)]."""
    n = len(flags)
    if n == 0 or not any(flags):
        return []
    if all(flags):
        return [(0, n - 1)]
    first_false = list(flags).index(False)
    runs: List[Tuple[int, int]] = []
    i = first_false + 1
    while i < first_false + 1 + n:
        if flags[i % n]:
            j = i
            while flags[(j + 1) % n] and j + 1 < first_false + n:
                j += 1
            s, e = i % n, j - i + i % n
            runs.append((s, e))
            i = j + 1
        else:
            i += 1
    return sorted(runs)


def _valid_layer(layer: Sequence[Candidate], scores: Optional[Sequence[NodeScore]]) -> bool:
    return any(c.reason is None and (scores is None or scores[k].valid) for k, c in enumerate(layer))


def window_lengths(layers: Sequence[Sequence[Candidate]], scores: Optional[Sequence[Sequence[NodeScore]]],
                   params: PlannerParams, stride: int = 1) -> List[int]:
    """Per start point i of a closed seam: the number of consecutive points (≤ n) that one continuous DP path
    covers from i in travel direction, over the seam start if needed (``ladder.solve`` on the doubled layers, open:
    same configuration label, |Δq| ≤ Δq_max, joint limits on unwrapped values, wrist-3 travel from the window start
    — F3, F11). 0 = no valid node at i, or i not a start (``stride`` > 1: only every stride-th point is a start;
    recorded by the caller, a coverage lower bound)."""
    n = len(layers)
    if scores is None:
        scores = [[NodeScore(True)] * len(l) for l in layers]
    lay2 = list(layers) + list(layers)
    sc2 = list(scores) + list(scores)
    out = [0] * n
    for i in range(0, n, max(1, int(stride))):
        if not _valid_layer(layers[i], scores[i]):
            continue
        res = solve(lay2[i:i + n], sc2[i:i + n], params, closed=False)
        out[i] = res.max_reached_layer + 1
    return out


def maximal_windows(lengths: Sequence[int]) -> List[Tuple[int, int]]:
    """Windows (start, length) that no other window contains on the circle of n = len(lengths) points."""
    n = len(lengths)
    lens = np.asarray(lengths, dtype=np.int64)
    idx = np.nonzero(lens > 0)[0]
    out = []
    for a in idx:
        la = lens[a]
        d = (a - idx) % n
        contained = (d + la <= lens[idx]) & (idx != a)
        if not np.any(contained):
            out.append((int(a), int(la)))
    return out


def window_intervals(windows: Sequence[Tuple[int, int]], pitch_mm: float) -> List[Interval]:
    """Windows → s intervals (mm) from the first to the last covered point; the end may exceed the seam length."""
    return [(float(a * pitch_mm), float(a * pitch_mm + (la - 1) * pitch_mm)) for a, la in windows]


def _pieces(intervals: Sequence[Interval], length: float) -> List[Interval]:
    out: List[Interval] = []
    for a, b in intervals:
        if b - a >= length - _TOL:
            return [(0.0, length)]
        a0 = a % length
        b0 = a0 + (b - a)
        if b0 > length:
            out += [(a0, length), (0.0, b0 - length)]
        else:
            out.append((a0, b0))
    return sorted(out)


def _union_length(intervals: Sequence[Interval], length: float) -> Tuple[float, bool]:
    total, cur_s, cur_e = 0.0, None, None
    for a, b in _pieces(intervals, length):
        if cur_e is None or a > cur_e + _TOL:
            if cur_e is not None:
                total += cur_e - cur_s
            cur_s, cur_e = a, b
        else:
            cur_e = max(cur_e, b)
    if cur_e is not None:
        total += cur_e - cur_s
    return total, total >= length - 1e-6


def bins_covered(intervals: Sequence[Interval], length: float, bin_mm: float) -> List[bool]:
    """Per bin [k·bin, min((k+1)·bin, L)] of a closed seam: True when the whole bin lies inside **one** interval
    (one continuous window = one weld segment). Two intervals that touch or overlap are not merged (review M4: a bin
    straddling two windows needs a stop between them); only the pieces of one interval split by the seam start are
    joined, and a bin never crosses the seam start."""
    nb = max(1, int(math.ceil(length / bin_mm - _TOL)))
    pieces = [p for iv in intervals for p in _pieces([iv], length)]
    out = []
    for k in range(nb):
        lo, hi = k * bin_mm, min((k + 1) * bin_mm, length)
        out.append(any(a <= lo + _TOL and hi <= b + _TOL for a, b in pieces))
    return out


def circle_cover(intervals: Sequence[Interval], length: float, overlap_mm: float) -> Dict[str, Any]:
    """Do the intervals cover the closed seam with ≥ ``overlap_mm`` at every join? (each interval shrunk by half the
    overlap at both ends must still cover). Also: covered length (raw union) and the smallest join overlap between
    circularly consecutive non-nested intervals (single interval: its length − L)."""
    covered, _ = _union_length(intervals, length)
    half = overlap_mm / 2.0
    shrunk = [(a + half, b - half) for a, b in intervals if b - a >= overlap_mm - _TOL]
    covers = bool(shrunk) and _union_length(shrunk, length)[1]
    norm = sorted(((a % length), b - a) for a, b in intervals)
    keep = [(a, la) for i, (a, la) in enumerate(norm)
            if not any(j != i and ((a - b) % length) + la <= lb + _TOL and (lb > la or j < i)
                       for j, (b, lb) in enumerate(norm))]
    if len(keep) == 1:
        joins = [keep[0][1] - length]
    else:
        joins = [(keep[i][0] + keep[i][1]) - (keep[(i + 1) % len(keep)][0] + (length if i + 1 == len(keep) else 0.0))
                 for i in range(len(keep))] if keep else []
    return {'covers': covers, 'covered_mm': covered, 'min_join_overlap_mm': min(joins) if joins else None}


def cover_sets(windows_by_theta: Mapping[float, Sequence[Interval]], length: float, overlap_mm: float, k: int,
               limit: Optional[int] = None) -> List[Dict[str, Any]]:
    """All sets of ``k`` distinct angles, one window each, that cover the closed seam with ≥ ``overlap_mm`` at every
    join and contain no covering subset (redundant angle). Ranked by the smallest join overlap (largest first), then
    the angles. **Suggestions only** — the Engineer enters the angles (B-12)."""
    out = []
    thetas = sorted(windows_by_theta)
    for combo in itertools.combinations(thetas, k):
        for wins in itertools.product(*(windows_by_theta[t] for t in combo)):
            cov = circle_cover(wins, length, overlap_mm)
            if not cov['covers']:
                continue
            if any(circle_cover(sub, length, overlap_mm)['covers']
                   for r in range(1, k) for sub in itertools.combinations(wins, r)):
                continue
            out.append({'thetas': list(combo), 'min_join_overlap_mm': cov['min_join_overlap_mm'],
                        'segments': [{'theta_deg': t, 's_from_mm': w[0], 's_to_mm': w[1]} for t, w in zip(combo, wins)]})
    out.sort(key=lambda d: (-round(d['min_join_overlap_mm'], 9), d['thetas'],
                            [s['s_from_mm'] for s in d['segments']]))
    return out[:limit] if limit else out


def best_cover(windows_by_theta: Mapping[float, Sequence[Interval]], length: float, k: int) -> Optional[Dict[str, Any]]:
    """The set of ``k`` angles (one window each) with the largest covered length (raw union, no overlap rule); ties
    by the smallest join overlap (largest first), then the angles. For the report when no set covers."""
    best = None
    for combo in itertools.combinations(sorted(windows_by_theta), k):
        for wins in itertools.product(*(windows_by_theta[t] for t in combo)):
            cov = circle_cover(wins, length, 0.0)
            key = (-round(cov['covered_mm'], 6), -round(cov['min_join_overlap_mm'] or -math.inf, 9), list(combo))
            if best is None or key < best[0]:
                best = (key, {'thetas': list(combo), 'covered_mm': cov['covered_mm'],
                              'covered_pct': 100.0 * min(cov['covered_mm'], length) / length,
                              'min_join_overlap_mm': cov['min_join_overlap_mm'],
                              'segments': [{'theta_deg': t, 's_from_mm': w[0], 's_to_mm': w[1]}
                                           for t, w in zip(combo, wins)]})
    return None if best is None else best[1]


__all__ = ['screening_thetas', 'theta_representative', 'theta_classes', 'circular_runs', 'window_lengths',
           'maximal_windows', 'window_intervals', 'bins_covered', 'circle_cover', 'cover_sets', 'best_cover']
