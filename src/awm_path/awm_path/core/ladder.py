"""Ladder-graph dynamic programme over the candidate layers (WP-S S.3; TST-002 Rev 0.3 §5.1 F2, F3, F7, F11;
DES-001 Rev 0.6 draft §4.3 steps 4–5).

Graph (exact, no approximation):

* node = (layer, candidate, **unwrapped** joint vector). At the first layer every kinematically and scene-valid
  candidate is expanded into all its 2π representatives inside the effective joint limits (``unwrapped_variants``);
  each of these is a *start*.
* edge = same configuration label and max |Δq| ≤ Δq_max on every joint; the successor takes, per joint, the
  representative nearest the predecessor (``nearest_representative``), so the unwrapped step equals the wrapped step.
* Every DP state carries its start. This makes the two path-dependent rules exact instead of approximated:
  wrist 3 travel |q6 − q6(start)| ≤ ``wrist3_travel_deg`` (F11) and, for closed seams, the reconnection of the last
  layer to the **same** first-layer state (same unwrapped values, not a 2π-shifted copy; F3). Absolute limits
  (joint limits, |q6| ≤ ``wrist3_abs_deg``) are checked on the unwrapped values of every state.
* cost = Σ nodes [w_dev·|ψ − ψ_nominal| + w_clear·max(0, target − clearance) + w_sing·singularity penalty]
  + Σ edges [w_dq·Σ|Δq| + w_wrist3·|Δq6|]; singularity penalty = sin ε_w/|sin q5| + sin ε_e/|sin q3| + d_s/d_wc
  (≤ 1 each for a valid node, 0 when the candidate has no metrics).
* tie-break F7: lowest cost (rounded to 1e-9), then lowest first-layer ψ index, then configuration-label order, then
  the first-layer variant (smallest Σ|q| first, then lexicographic); all sorts are stable.

A seam is accepted only at 100 %: on failure no path is returned, only the failing layer(s) and reason counts.
"""

from __future__ import annotations

import math
import time
from collections import Counter
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from awm_core import units
from awm_path.core.kinematics import wrap_deg
from awm_path.core.states import LABEL_ORDER, unwrapped_variants
from awm_path.core.sweep import Candidate, PlannerParams

_TOL_DEG = 1e-9
_TURN_BASE = 7  # turn counts -3..3 per joint encoded in base 7
_TURN_OFFSET = 3


@dataclass(frozen=True)
class NodeScore:
    """Scene score of one candidate (S.4 supplies it): validity (F4, F9) and unpadded clearance in mm (F5)."""

    valid: bool
    clearance_mm: Optional[float] = None


@dataclass(frozen=True)
class LadderResult:
    """Result of ``solve``. ``path`` is empty unless ``ok`` (no partial path).

    ``reason_counts[i]`` = sorted (reason, count) pairs of layer i: candidate reasons of the sweep,
    ``scene_invalid``, and DP drops ``joint_limit_unwrapped`` / ``wrist3_travel`` (transitions into layer i) and
    ``no_closure`` (layer 0, closed seams). ``states_per_layer`` = DP states kept per layer.
    """

    ok: bool
    path: Tuple[Candidate, ...]
    cost: Optional[float]
    failing_layers: Tuple[int, ...]
    reason_counts: Tuple[Tuple[Tuple[str, int], ...], ...]
    max_reached_layer: int
    closure_failed: bool
    states_per_layer: Tuple[int, ...]
    n_nodes: int
    n_edges: int
    n_graph_edges: int
    wall_time_s: float = field(default=0.0, compare=False)


@dataclass
class _Layer:
    idx: np.ndarray          # index into the candidate layer
    q: np.ndarray            # (m, 6) wrapped joint values
    label: np.ndarray        # (m,) label order
    psi: np.ndarray          # (m,) psi index
    cost: np.ndarray         # (m,) node cost


def _sin_deg(x: float) -> float:
    return math.sin(units.deg_to_rad(x))


def _node_cost(c: Candidate, s: NodeScore, params: PlannerParams) -> float:
    cost = 0.0
    if params.psi_nominal_deg is not None:
        cost += params.w_dev * abs(float(wrap_deg(c.psi_deg - params.psi_nominal_deg)))
    if s.clearance_mm is not None:
        cost += params.w_clear * max(0.0, params.clearance_target_mm - s.clearance_mm)
    m = c.metrics
    if m is not None:
        pen = 0.0
        for thr, val in ((_sin_deg(params.singular_wrist_deg), m.sin_q5),
                         (_sin_deg(params.singular_elbow_deg), m.sin_q3),
                         (params.singular_shoulder_mm, m.wrist_centre_mm)):
            pen += thr / val if val > 0.0 else math.inf
        cost += params.w_sing * pen
    return cost


def node_cost(c: Candidate, s: NodeScore, params: PlannerParams) -> float:
    """Node cost of the ladder (public; S.6 ranks single Mode C poses with it): ψ deviation + low clearance +
    singularity penalty, as in the module docstring."""
    return _node_cost(c, s, params)


def _build_layer(cands: Sequence[Candidate], scores: Sequence[NodeScore], params: PlannerParams,
                 counts: Counter) -> _Layer:
    rows = []
    for k, (c, s) in enumerate(zip(cands, scores)):
        if c.reason is not None:
            counts[c.reason] += 1
            continue
        if not s.valid:
            counts['scene_invalid'] += 1
            continue
        rows.append((c.psi_index, LABEL_ORDER[c.label], c.q_deg, k, _node_cost(c, s, params)))
    rows.sort(key=lambda r: (r[0], r[1], r[2], r[3]))
    if not rows:
        return _Layer(np.zeros(0, int), np.zeros((0, 6)), np.zeros(0, int), np.zeros(0, int), np.zeros(0))
    # ``solve`` is public: joint values are wrapped into (-180, 180] so the window search never misses an edge.
    return _Layer(idx=np.array([r[3] for r in rows]), q=wrap_deg(np.array([r[2] for r in rows], dtype=float)),
                  label=np.array([r[1] for r in rows]), psi=np.array([r[0] for r in rows]),
                  cost=np.array([r[4] for r in rows], dtype=float))


def _edges(a: _Layer, b: _Layer, params: PlannerParams) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """CSR adjacency a -> b: (ptr, nbr, delta (E, 6), edge cost), sorted by (source, target).

    Bucketed per configuration label; inside a label the targets are sorted by q6 and only those inside the
    window |Δq6| ≤ Δq_max (with the ±360° wrap) are tested on all joints — no all-pairs test.
    """
    dq = params.dq_max_deg + _TOL_DEG
    src_l: List[np.ndarray] = []
    dst_l: List[np.ndarray] = []
    for lab in sorted(set(a.label.tolist()) & set(b.label.tolist())):
        ia = np.nonzero(a.label == lab)[0]
        ib = np.nonzero(b.label == lab)[0]
        order = ib[np.argsort(b.q[ib, 5], kind='stable')]
        keys = b.q[order, 5]
        for shift in (-360.0, 0.0, 360.0):
            lo = np.searchsorted(keys, a.q[ia, 5] + shift - dq, side='left')
            hi = np.searchsorted(keys, a.q[ia, 5] + shift + dq, side='right')
            cnt = hi - lo
            if not np.any(cnt):
                continue
            src = np.repeat(ia, cnt)
            pos = np.repeat(lo, cnt) + (np.arange(int(cnt.sum())) - np.repeat(np.cumsum(cnt) - cnt, cnt))
            src_l.append(src)
            dst_l.append(order[pos])
    if src_l:
        src = np.concatenate(src_l)
        dst = np.concatenate(dst_l)
        pair = np.unique(src * max(1, len(b.idx)) + dst)  # sorted, duplicates of overlapping windows removed
        src, dst = pair // max(1, len(b.idx)), pair % max(1, len(b.idx))
        delta = wrap_deg(b.q[dst] - a.q[src])
        keep = np.all(np.abs(delta) <= dq, axis=1)
        src, dst, delta = src[keep], dst[keep], delta[keep]
    else:
        src = dst = np.zeros(0, dtype=np.int64)
        delta = np.zeros((0, 6))
    ptr = np.zeros(len(a.idx) + 1, dtype=np.int64)
    np.add.at(ptr, src + 1, 1)
    ptr = np.cumsum(ptr)
    ecost = params.w_dq * np.abs(delta).sum(axis=1) + params.w_wrist3 * np.abs(delta[:, 5])
    return ptr, dst.astype(np.int64), delta, ecost


def _in_limits(u: np.ndarray, lims: np.ndarray) -> np.ndarray:
    return np.all((u >= lims[:, 0] - _TOL_DEG) & (u <= lims[:, 1] + _TOL_DEG), axis=1)


def _turn_code(u: np.ndarray, q: np.ndarray) -> np.ndarray:
    turns = np.rint((u - q) / 360.0).astype(np.int64) + _TURN_OFFSET
    if np.any(turns < 0) or np.any(turns >= _TURN_BASE):
        raise ValueError('unwrapped joint value outside the encodable range')
    return (turns * (_TURN_BASE ** np.arange(6))).sum(axis=1)


def solve(layers: Sequence[Sequence[Candidate]], scores: Optional[Sequence[Sequence[NodeScore]]],
          params: PlannerParams, closed: bool) -> LadderResult:
    """Find the lowest-cost continuous path through all layers, or report why none exists.

    ``scores`` has the shape of ``layers`` (``None`` = every candidate scene-valid, no clearance). For a closed seam
    the layers must not repeat the first point: the closing edge goes from the last layer back to the start state.
    """
    t0 = time.perf_counter()
    n = len(layers)
    if n == 0:
        raise ValueError('no layers')
    if scores is None:
        scores = [[NodeScore(True)] * len(l) for l in layers]
    if len(scores) != n or any(len(s) != len(l) for s, l in zip(scores, layers)):
        raise ValueError('scores do not match the candidate layers')
    lims = np.array(params.effective_limits_deg(), dtype=float)
    travel = params.wrist3_travel_deg + _TOL_DEG
    counts = [Counter() for _ in range(n)]
    built = [_build_layer(l, s, params, c) for l, s, c in zip(layers, scores, counts)]
    gaps = tuple(i for i, b in enumerate(built) if len(b.idx) == 0)

    # First layer: all unwrapped variants, ordered by (psi index, label order, q, variant order).
    l0 = built[0]
    s_node, s_u = [], []
    for k in range(len(l0.idx)):
        # Variant order: nearest to zero first (sum of |q|), then lexicographic.
        for v in sorted(unwrapped_variants(l0.q[k], lims), key=lambda v: (float(np.abs(v).sum()), tuple(v))):
            s_node.append(k)
            s_u.append(v)
    start_node = np.array(s_node, dtype=np.int64)
    start_u = np.array(s_u, dtype=float).reshape(-1, 6)
    node = start_node.copy()
    u = start_u.copy()
    start = np.arange(len(node), dtype=np.int64)
    cost = l0.cost[node] if len(node) else np.zeros(0)
    back_hist: List[np.ndarray] = [np.full(len(node), -1, dtype=np.int64)]
    node_hist: List[np.ndarray] = [node]
    u_hist: List[np.ndarray] = [u]
    states = [len(node)]
    n_nodes, n_edges, n_graph = len(node), 0, 0
    reached = 0 if len(node) else -1
    died_at: Optional[int] = None if len(node) else 0

    for i in range(n - 1):
        if len(node) == 0:
            break
        a, b = built[i], built[i + 1]
        ptr, nbr, delta, ecost = _edges(a, b, params)
        n_graph += len(nbr)
        deg = ptr[node + 1] - ptr[node]
        rep = np.repeat(np.arange(len(node)), deg)
        first = np.repeat(ptr[node], deg)
        within = np.arange(len(rep)) - np.repeat(np.cumsum(deg) - deg, deg)
        e = first + within
        n_edges += len(e)
        nu = u[rep] + delta[e]
        ok_lim = _in_limits(nu, lims)
        ok_w3 = np.abs(nu[:, 5] - start_u[start[rep], 5]) <= travel
        for reason, n_drop in (('joint_limit_unwrapped', int(np.sum(~ok_lim))),
                               ('wrist3_travel', int(np.sum(ok_lim & ~ok_w3)))):
            if n_drop:
                counts[i + 1][reason] += n_drop
        keep = ok_lim & ok_w3
        rep, e, nu = rep[keep], e[keep], nu[keep]
        nn = nbr[e]
        nc = cost[rep] + ecost[e] + b.cost[nn]
        if len(rep) == 0:
            node = np.zeros(0, dtype=np.int64)
            died_at = i + 1
            break
        key_node = start[rep] * len(b.idx) + nn
        tcode = _turn_code(nu, b.q[nn])
        order = np.lexsort((rep, np.round(nc, 9), tcode, key_node))
        k1, k2 = key_node[order], tcode[order]
        firsts = np.ones(len(order), dtype=bool)
        firsts[1:] = (k1[1:] != k1[:-1]) | (k2[1:] != k2[:-1])
        sel = order[firsts]
        back_hist.append(rep[sel])
        start = start[rep[sel]]
        node = nn[sel]
        u = nu[sel]
        cost = nc[sel]
        node_hist.append(node)
        u_hist.append(u)
        states.append(len(node))
        n_nodes += len(node)
        reached = i + 1

    closure_failed = False
    best: Optional[int] = None
    total = cost
    if len(node) and reached == n - 1:
        if closed:
            d = start_u[start] - u
            ok_close = np.all(np.abs(d) <= params.dq_max_deg + _TOL_DEG, axis=1)
            total = cost + params.w_dq * np.abs(d).sum(axis=1) + params.w_wrist3 * np.abs(d[:, 5])
            if not np.any(ok_close):
                closure_failed = True
                counts[0]['no_closure'] += len(node)
            else:
                cand = np.nonzero(ok_close)[0]
                best = int(cand[np.lexsort((cand, start[cand], np.round(total[cand], 9)))[0]])
        else:
            cand = np.arange(len(node))
            best = int(np.lexsort((cand, start, np.round(total, 9)))[0])

    rc = tuple(tuple(sorted(c.items())) for c in counts)
    wall = time.perf_counter() - t0
    if best is None:
        if gaps:
            failing = gaps
        elif closure_failed:
            failing = (0,)
        else:
            failing = (died_at if died_at is not None else reached + 1,)
        return LadderResult(False, (), None, failing, rc, reached, closure_failed, tuple(states),
                            n_nodes, n_edges, n_graph, wall)

    path: List[Candidate] = []
    s = best
    for li in range(reached, -1, -1):
        k = int(node_hist[li][s])
        c = layers[li][int(built[li].idx[k])]
        path.append(replace(c, q_deg=tuple(float(x) for x in u_hist[li][s])))
        s = int(back_hist[li][s])
    path.reverse()
    return LadderResult(True, tuple(path), float(total[best]), (), rc, reached, False, tuple(states),
                        n_nodes, n_edges, n_graph, wall)
