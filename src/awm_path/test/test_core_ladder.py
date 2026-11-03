"""Tests of the ladder-graph DP (WP-S S.3; TST-002 F2, F3, F7, F11). Synthetic ladders need no kinematics."""
import math
import sys
import time
from pathlib import Path
from typing import List, Sequence

import numpy as np
import pytest

from awm_path.core.ladder import LadderResult, NodeScore, solve
from awm_path.core.sweep import Candidate, PlannerParams

sys.path.insert(0, str(Path(__file__).parent))

A, B = 'S+E+W+', 'S+E+W-'
P = PlannerParams()


def cand(point: int, q: Sequence[float], label: str = A, psi_index: int = 0) -> Candidate:
    return Candidate(point, psi_index, 10.0 * psi_index, label, tuple(float(x) for x in q), None, None)


def chain(q1_values: Sequence[float], label: str = A, psi_index: int = 0, q6=None) -> List[Candidate]:
    out = []
    for i, x in enumerate(q1_values):
        q6v = 0.0 if q6 is None else q6[i]
        out.append(cand(i, [x, -90.0, 90.0, -90.0, 90.0, q6v], label, psi_index))
    return out


def ladder(*chains: List[Candidate]) -> List[List[Candidate]]:
    n = len(chains[0])
    return [[c[i] for c in chains] for i in range(n)]


def counts(res: LadderResult, layer: int) -> dict:
    return dict(res.reason_counts[layer])


def test_unique_feasible_path() -> None:
    good = chain([0, 4, 8, 12, 16])
    bad1 = chain([50, 60, 70, 80, 90])
    bad2 = chain([-100, -100, -80, -100, -100])
    res = solve(ladder(bad1, good, bad2), None, P, closed=False)
    assert res.ok and len(res.path) == 5
    assert [c.q_deg[0] for c in res.path] == [0, 4, 8, 12, 16]
    assert res.path[0].q_deg == (0.0, -90.0, 90.0, -90.0, 90.0, 0.0)  # variant nearest zero
    assert res.cost == pytest.approx(16.0)


def test_label_change_never_used_even_when_cheaper() -> None:
    layers = [[cand(0, [0, -90, 90, -90, 90, 0], A)],
              [cand(1, [0, -90, 90, -90, 90, 0], B), cand(1, [4, -90, 90, -90, 90, 0], A)],
              [cand(2, [0, -90, 90, -90, 90, 0], B), cand(2, [8, -90, 90, -90, 90, 0], A)]]
    res = solve(layers, None, P, closed=False)
    assert res.ok and {c.label for c in res.path} == {A}
    only_b = [layers[0], [layers[1][0]], [layers[2][0]]]
    res2 = solve(only_b, None, P, closed=False)
    assert not res2.ok and res2.failing_layers == (1,) and res2.path == ()


def test_dq_bound_on_every_joint() -> None:
    assert solve(ladder(chain([0, 5, 10])), None, P, closed=False).ok
    assert not solve(ladder(chain([0, 5.1, 10])), None, P, closed=False).ok
    assert not solve(ladder(chain([0, 0, 0], q6=[0, 5.01, 10])), None, P, closed=False).ok
    res = solve(ladder(chain([0, 3, 6, 9]), chain([0, 2, 7.5, 9])), None, P, closed=False)
    q = np.array([c.q_deg for c in res.path])
    assert np.all(np.abs(np.diff(q, axis=0)) <= 5.0)


def test_gap_layer_reported_no_partial_path() -> None:
    layers = ladder(chain([0, 1, 2, 3, 4]))
    scores = [[NodeScore(True)] for _ in layers]
    scores[2] = [NodeScore(False)]
    res = solve(layers, scores, P, closed=False)
    assert not res.ok and res.path == () and res.cost is None
    assert res.failing_layers == (2,)
    assert counts(res, 2) == {'scene_invalid': 1}
    assert res.max_reached_layer == 1
    rejected = [list(l) for l in layers]
    rejected[3] = [Candidate(3, 0, 0.0, None, None, None, 'no_ik_unreachable')]
    res2 = solve(rejected, None, P, closed=False)
    assert res2.failing_layers == (3,) and counts(res2, 3) == {'no_ik_unreachable': 1}


def test_closed_seam_reconnects_to_same_state() -> None:
    there_and_back = [0, 4, 8, 12, 8, 4]
    res = solve(ladder(chain(there_and_back)), None, P, closed=True)
    assert res.ok and res.cost == pytest.approx(4 * 6)  # 5 steps + closing step 4 -> 0
    # q1 turns once: the last layer meets the start only as a 2π-shifted copy -> rejected.
    turning = [5.0 * k for k in range(72)]
    res2 = solve(ladder(chain(turning)), None, P, closed=True)
    assert not res2.ok and res2.closure_failed and res2.failing_layers == (0,)
    assert counts(res2, 0)['no_closure'] > 0
    assert solve(ladder(chain(turning)), None, P, closed=False).ok
    # Closing edge itself must obey Δq_max.
    assert not solve(ladder(chain([0, 4, 8])), None, P, closed=True).ok


def test_wrist3_travel_and_absolute_limit() -> None:
    q6_180 = [5.0 * k for k in range(37)]   # 0 .. 180
    q6_185 = [5.0 * k for k in range(38)]   # 0 .. 185
    zeros37, zeros38 = [0.0] * 37, [0.0] * 38
    assert solve(ladder(chain(zeros37, q6=q6_180)), None, P, closed=False).ok
    res = solve(ladder(chain(zeros38, q6=q6_185)), None, P, closed=False)
    assert not res.ok and res.failing_layers == (37,) and counts(res, 37).get('wrist3_travel', 0) > 0
    tight = PlannerParams(wrist3_abs_deg=90.0)
    res2 = solve(ladder(chain([0.0] * 20, q6=[5.0 * k for k in range(20)])), None, tight, closed=False)
    assert not res2.ok and res2.failing_layers == (19,)
    assert counts(res2, 19).get('joint_limit_unwrapped', 0) > 0
    # 0 -> 200 wrapped = -160: the unwrapped path keeps 200, so |q6| is checked on 200, not on -160.
    path = [100.0 + 5.0 * k for k in range(21)]
    res3 = solve(ladder(chain([0.0] * 21, q6=path)), None, PlannerParams(wrist3_abs_deg=190.0), closed=False)
    assert not res3.ok
    res4 = solve(ladder(chain([0.0] * 21, q6=path)), None, PlannerParams(), closed=False)
    assert res4.ok and res4.path[-1].q_deg[5] in (200.0, -160.0)
    assert abs(res4.path[-1].q_deg[5] - res4.path[0].q_deg[5]) == pytest.approx(100.0)


def test_tie_break_f7() -> None:
    c_psi1 = chain([0, 1, 2], psi_index=1)
    c_psi0 = chain([0, 1, 2], psi_index=0)
    res = solve(ladder(c_psi1, c_psi0), None, P, closed=False)
    assert res.ok and all(c.psi_index == 0 for c in res.path)
    c_b = chain([0, 1, 2], label=B)
    c_a = chain([0, 1, 2], label=A)
    res2 = solve(ladder(c_b, c_a), None, P, closed=False)
    assert all(c.label == A for c in res2.path)
    cheaper_psi1 = chain([0, 0.2, 0.4], psi_index=1)
    res3 = solve(ladder(c_psi0, cheaper_psi1), None, P, closed=False)
    # Layer 0 is the same joint state for both ψ (equal cost): lowest ψ index; then the cheaper ψ 1 nodes.
    assert res3.cost == pytest.approx(0.4)
    assert [c.psi_index for c in res3.path] == [0, 1, 1]


def test_cost_terms() -> None:
    p = PlannerParams(psi_nominal_deg=0.0, w_dev=1.0, w_clear=2.0, clearance_target_mm=10.0, w_wrist3=0.0)
    layers = ladder(chain([0, 1], psi_index=1))
    scores = [[NodeScore(True, 4.0)], [NodeScore(True, None)]]
    res = solve(layers, scores, p, closed=False)
    assert res.cost == pytest.approx(1.0 + 2 * 10.0 + 2.0 * 6.0)


def test_deterministic() -> None:
    rng = np.random.default_rng(7)
    chains = [chain(np.cumsum(rng.uniform(-3, 3, 30)), label=[A, B][k % 2], psi_index=k // 2) for k in range(12)]
    r1 = solve(ladder(*chains), None, P, closed=False)
    r2 = solve(ladder(*chains), None, P, closed=False)
    assert r1 == r2 and r1.ok


def _synthetic_j2_ladder(n_layers: int = 72, n_psi: int = 36, seed: int = 11) -> List[List[Candidate]]:
    """J2-sized synthetic ladder: 72 layers × 36 ψ × 8 labels, smooth joint drift ≤ 2°/step (all chains feasible)."""
    from awm_path.core.states import LABELS
    rng = np.random.default_rng(seed)
    base = rng.uniform(-170.0, 170.0, size=(n_psi, 8, 6))
    base[:, :, 2] = rng.uniform(-150.0, 150.0, size=(n_psi, 8))
    drift = rng.uniform(-2.0, 2.0, size=(n_psi, 8, 6))
    drift[:, :, 5] = rng.uniform(-1.0, 1.0, size=(n_psi, 8))
    layers = []
    for i in range(n_layers):
        q = base + i * drift
        q = q - 360.0 * np.ceil((q - 180.0) / 360.0)
        layers.append([cand(i, q[j, k], LABELS[k], j) for j in range(n_psi) for k in range(8)])
    return layers


def test_j2_sized_ladder_timing() -> None:
    layers = _synthetic_j2_ladder()
    t = time.perf_counter()
    res = solve(layers, None, P, closed=False)
    wall = time.perf_counter() - t
    print(f'J2-sized synthetic ladder: layers {len(layers)}, candidates/layer {len(layers[0])}, '
          f'start states {res.states_per_layer[0]}, nodes {res.n_nodes}, transitions {res.n_edges}, '
          f'graph edges {res.n_graph_edges}, wall {wall:.2f} s')
    assert res.ok
    assert wall < 30.0  # C5 budget, excluding scene calls


# ---- end-to-end on real kinematics (Task 3 synthetic circle) ------------------------------------------------------

@pytest.fixture(scope='module')
def kin():
    from awm_path.core.kinematics import UrAnalyticKinematics
    return UrAnalyticKinematics()


def test_e2e_open_arc_fine_pitch(kin) -> None:
    """Open 150° arc of the r 13.3 mm circle at pitch 0.29 mm (≤ F1 limit 1.16 mm): one continuous path, steps ≤ 5°."""
    from test_core_sweep import AXIS, CENTRE, R_MM, circle_frames, tool
    from awm_path.core.sweep import pitch_points, sweep
    frames = circle_frames(CENTRE, AXIS, R_MM, 288)[:121]
    assert pitch_points(frames, P, closed=False).ok
    layers = sweep(frames, tool(), kin, P, P.psi_free)
    res = solve(layers, None, P, closed=False)
    print(f'open arc: layers {len(layers)}, nodes {res.n_nodes}, transitions {res.n_edges}, '
          f'graph edges {res.n_graph_edges}, wall {res.wall_time_s:.2f} s')
    assert res.ok
    q = np.array([c.q_deg for c in res.path])
    assert np.all(np.abs(np.diff(q, axis=0)) <= 5.0 + 1e-9)
    assert len({c.label for c in res.path}) == 1


def test_e2e_closed_circle_is_rejected_with_reasons(kin) -> None:
    """Closed 360° seam whose frame turns once about a fixed axis, robot-only, fixed ψ.

    A joint loop that returns to the same unwrapped state maps to a contractible loop of tool orientations, but a
    single 360° turn about a fixed axis is not contractible in SO(3). So with ψ fixed some joint must turn by 360°:
    F11 (wrist-3 travel ≤ 180°) or F3 (no 2π-shifted closure) rejects every path. This test records that result
    (it is a property of the geometry, not of the solver).
    """
    from test_core_sweep import AXIS, CENTRE, R_MM, circle_frames, tool
    from awm_path.core.sweep import sweep
    frames = circle_frames(CENTRE, AXIS, R_MM, 288)
    layers = sweep(frames, tool(), kin, P, (90.0,))
    res = solve(layers, None, P, closed=True)
    assert not res.ok and res.path == ()
    assert sum(dict(rc).get('wrist3_travel', 0) for rc in res.reason_counts) > 0
    no_w3 = PlannerParams(wrist3_travel_deg=720.0, wrist3_abs_deg=360.0)  # isolates F3: not a planner setting
    res2 = solve(layers, None, no_w3, closed=True)
    assert not res2.ok and res2.closure_failed
    res3 = solve(layers, None, no_w3, closed=False)
    assert res3.ok
    q = np.array([c.q_deg for c in res3.path])
    net = q[-1] - q[0] + (np.mod(q[0] - q[-1] + 180.0, 360.0) - 180.0)
    assert np.any(np.isclose(np.abs(net), 360.0, atol=1e-6))
    print(f'closed circle (psi 90): net joint turn after closing step {np.round(net, 3)}')


def test_unwrapped_input_is_wrapped_not_lost() -> None:
    """``solve`` is public: q outside (-180, 180] is wrapped before the window search (edge found, not lost)."""
    layers = [[cand(0, [0, -90, 90, -90, 90, 362.0])], [cand(1, [0, -90, 90, -90, 90, 2.0])]]
    res = solve(layers, None, P, closed=False)
    assert res.ok and res.path[1].q_deg[5] - res.path[0].q_deg[5] == pytest.approx(0.0)
    layers2 = [[cand(0, [0, -90, 90, -90, 90, 540.0])], [cand(1, [0, -90, 90, -90, 90, 182.0])]]
    res2 = solve(layers2, None, P, closed=False)
    assert res2.ok and res2.path[1].q_deg[5] - res2.path[0].q_deg[5] == pytest.approx(2.0)
    layers_far = [[cand(0, [0, -90, 90, -90, 90, 722.0])], [cand(1, [0, -90, 90, -90, 90, -718.0])]]
    res_far = solve(layers_far, None, P, closed=False)
    assert res_far.ok and res_far.path[0].q_deg[5] == pytest.approx(2.0)
    layers3 = [[cand(0, [0, -90, 90, -90, 90, 540.0])], [cand(1, [0, -90, 90, -90, 90, 2.0])]]
    res3 = solve(layers3, None, P, closed=False)  # 540 = 180 (mod 360): 178 deg away from 2 -> no edge
    assert not res3.ok and res3.failing_layers == (1,)


def test_e2e_closed_circle_psi_free_is_rejected(kin) -> None:
    """Same closed circle, ψ-free (36 values), 288 points: rejected; failing layer and reason recorded."""
    from test_core_sweep import AXIS, CENTRE, R_MM, circle_frames, tool
    from awm_path.core.sweep import sweep
    layers = sweep(circle_frames(CENTRE, AXIS, R_MM, 288), tool(), kin, P, P.psi_free)
    res = solve(layers, None, P, closed=True)
    reasons = {k: sum(dict(rc).get(k, 0) for rc in res.reason_counts)
               for k in ('wrist3_travel', 'joint_limit_unwrapped', 'no_closure')}
    print(f'closed circle psi-free: failing layers {res.failing_layers}, max reached {res.max_reached_layer}, '
          f'DP drops {reasons}, nodes {res.n_nodes}, wall {res.wall_time_s:.2f} s')
    assert not res.ok and res.path == ()
    assert res.failing_layers == (145,)
    assert reasons['wrist3_travel'] > 0
