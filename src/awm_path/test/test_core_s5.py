"""WP-S S.5 Task 2: pitch refinement (CP-53 / F1), failing-point classes, C3 path metrics, coverage."""

import math

import numpy as np
import pytest

from awm_path.core.ladder import NodeScore, solve
from awm_path.core.s5 import (POINT_CLASSES, classify_points, longest_window, max_label_steps, path_metrics, point_classes,
                              refine_pitch)
from awm_path.core.sweep import Candidate, PlannerParams


def _c(point, psi_i, label, q, reason=None):
    return Candidate(point, psi_i, 10.0 * psi_i, label, None if q is None else tuple(float(x) for x in q), None,
                     reason)


def _ramp_layers(n, step_deg, psi=1):
    """n points; per ψ one candidate of label A whose q1 moves step_deg per point."""
    return [[_c(i, j, 'S+E+W+', [i * step_deg + 30 * j, 0, 0, 0, 0, 0]) for j in range(psi)] for i in range(n)]


def test_max_label_steps_only_pairs_same_psi_and_label():
    layers = [[_c(0, 0, 'S+E+W+', [0] * 6), _c(0, 1, 'S+E+W+', [50, 0, 0, 0, 0, 0]),
               _c(0, 2, 'S-E+W+', [0] * 6, reason='joint_limit')],
              [_c(1, 0, 'S+E+W+', [2, 0, 0, 0, 0, 179]), _c(1, 1, 'S-E+W+', [0] * 6),
               _c(1, 2, 'S-E+W+', [90] * 6)]]
    steps = max_label_steps(layers)
    # only (psi 0, S+E+W+) is kinematically valid at both points; its largest joint step is q6: 179 deg
    assert steps == [pytest.approx(179.0)]


def test_max_label_steps_wraps_joint_differences():
    layers = [[_c(0, 0, 'S+E+W+', [179, 0, 0, 0, 0, 0])], [_c(1, 0, 'S+E+W+', [-179, 0, 0, 0, 0, 0])]]
    assert max_label_steps(layers) == [pytest.approx(2.0)]


def test_refine_pitch_halves_until_steps_are_within_dq_max():
    params = PlannerParams()
    calls = []

    def sample(n):
        calls.append(n)
        # joint motion of 40 deg over the arc length -> step = 40 / (n - 1)
        return _ramp_layers(n, 40.0 / (n - 1))

    res = refine_pitch(length_mm=10.0, rate_deg_per_mm=1.0, sample=sample, params=params, pitch_min_mm=0.05)
    assert res.ok
    assert calls[0] == 5                              # F1 ceiling: min(3, 5 / 1) = 3 mm -> 4 steps, 5 points
    assert res.max_step_deg <= params.dq_max_deg
    assert res.points == calls[-1] == 9               # 40 / 8 = 5 deg
    assert res.pitch_mm == pytest.approx(10.0 / 8)
    assert res.iterations == len(calls)


def test_refine_pitch_stops_at_the_floor_and_reports_it():
    res = refine_pitch(length_mm=10.0, rate_deg_per_mm=1.0, sample=lambda n: _ramp_layers(n, 50.0),
                       params=PlannerParams(), pitch_min_mm=1.0)
    assert not res.ok
    assert res.pitch_mm >= 1.0 and res.max_step_deg == pytest.approx(50.0)


def _cols(layers, valid, f4=None, clr=None, contact=None, contacts='', blocking=''):
    shape = lambda v: [[v] * len(l) for l in layers]
    return {'scene_valid': f4 or shape(True), 'scene_contacts': shape(contacts), 'clearance_lb_mm': clr or shape(10.0),
            'clearance_blocking': shape(blocking), 'contact_ok': contact or shape(True),
            'contact_reason': shape(None), 'node_valid': valid}


def test_classify_points_gives_a_class_and_the_colliding_pairs():
    layers = [[_c(0, 0, 'S+E+W+', [0] * 6), _c(0, 1, None, None, reason='no_ik_unreachable')],
              [_c(1, 0, 'S+E+W+', [0] * 6)],
              [_c(2, 0, 'S+E+W+', [0] * 6)],
              [_c(3, 0, None, None, reason='singular_wrist')],
              [_c(4, 0, 'S+E+W+', [0] * 6)]]
    valid = [[True, False], [False], [False], [False], [False]]
    f4 = [[True, None], [False], [True], [None], [True]]
    clr = [[10.0, None], [None], [None], [None], [10.0]]
    contact = [[True, True], [True], [True], [True], [False]]
    cols = _cols(layers, valid, f4=f4, clr=clr, contact=contact)
    cols['scene_contacts'][1] = ['head_body<->jig_1;nozzle_body<->part']
    cols['clearance_blocking'][2] = ['conduit<->part']
    pts = classify_points(layers, cols)
    assert [p['valid_nodes'] for p in pts] == [1, 0, 0, 0, 0]
    assert point_classes(pts) == [None, 'F4', 'clearance', 'IK', 'nozzle']   # F9 at point 4 → nozzle (CP-56)
    assert pts[1]['pairs'] == {'head_body<->jig_1': 1, 'nozzle_body<->part': 1}
    assert pts[2]['pairs'] == {'conduit<->part': 1}
    assert pts[3]['counts'] == {'singular_wrist': 1}
    assert pts[0]['counts'] == {'no_ik_unreachable': 1, 'valid': 1}


def test_path_metrics_measure_c3_on_the_path():
    p = PlannerParams()
    layers = _ramp_layers(5, 4.0)
    res = solve(layers, None, p, closed=False)
    m = path_metrics(res.path)
    assert m['labels'] == ['S+E+W+'] and m['max_step_deg'] == pytest.approx(4.0)
    assert m['wrist3_travel_deg'] == 0.0 and m['wrist3_abs_max_deg'] == 0.0


def test_longest_window_finds_the_longest_continuous_run():
    p = PlannerParams()
    layers = _ramp_layers(10, 2.0)
    scores = [[NodeScore(i not in (2, 7))] for i in range(10)]
    w = longest_window(layers, scores, p)
    assert (w['start'], w['end'], w['layers']) == (3, 6, 4)
    all_ok = longest_window(layers, None, p)
    assert all_ok['layers'] == 10


# --- S.5b: CP-53 pitch rule on scene-valid nodes (lead decision 2026-11-04), closed seams ---------------------

def test_max_label_steps_with_scores_pairs_scene_valid_nodes_only():
    layers = [[_c(0, 0, 'S+E+W+', [0] * 6), _c(0, 1, 'S+E+W+', [0] * 6)],
              [_c(1, 0, 'S+E+W+', [40, 0, 0, 0, 0, 0]), _c(1, 1, 'S+E+W+', [2, 0, 0, 0, 0, 0])]]
    assert max_label_steps(layers) == [pytest.approx(40.0)]
    scores = [[NodeScore(False), NodeScore(True)], [NodeScore(True), NodeScore(True)]]
    assert max_label_steps(layers, scores=scores) == [pytest.approx(2.0)]


def test_max_label_steps_closed_adds_the_step_back_to_the_first_point():
    layers = [[_c(0, 0, 'S+E+W+', [0] * 6)], [_c(1, 0, 'S+E+W+', [3, 0, 0, 0, 0, 0])],
              [_c(2, 0, 'S+E+W+', [20, 0, 0, 0, 0, 0])]]
    assert max_label_steps(layers, closed=True) == [pytest.approx(3.0), pytest.approx(17.0), pytest.approx(20.0)]


def test_refine_pitch_scene_valid_rule_ignores_invalid_nodes_and_records_the_rule():
    params = PlannerParams()

    def sample(n):
        # ψ 0 jumps 40 deg per step but is scene-invalid; ψ 1 moves 1 deg per step and is valid
        layers = [[_c(i, 0, 'S+E+W+', [40.0 * i, 0, 0, 0, 0, 0]), _c(i, 1, 'S+E+W+', [1.0 * i, 0, 0, 0, 0, 0])]
                  for i in range(n)]
        return layers, [[NodeScore(False), NodeScore(True)] for _ in range(n)]

    res = refine_pitch(10.0, 1.0, sample, params, 0.05, rule='scene_valid')
    assert res.ok and res.iterations == 1 and res.rule == 'scene_valid'
    assert res.scores is not None and len(res.scores) == res.points
    assert res.as_record()['rule'] == 'scene_valid'
    kin = refine_pitch(10.0, 1.0, lambda n: sample(n)[0], params, 0.05)
    assert kin.rule == 'kinematic' and kin.iterations > 1 and kin.scores is None
    with pytest.raises(ValueError):
        refine_pitch(10.0, 1.0, sample, params, 0.05, rule='nope')


def test_refine_pitch_closed_uses_pitch_length_over_n_and_doubles_n():
    params = PlannerParams()
    calls = []

    def sample(n):
        calls.append(n)
        # periodic joint motion q1 = 20 sin(2π i / n): largest step ≈ 125.7 / n, the closing step included
        return [[_c(i, 0, 'S+E+W+', [20.0 * math.sin(2 * math.pi * i / n), 0, 0, 0, 0, 0])] for i in range(n)]

    res = refine_pitch(10.0, 1.0, sample, params, 0.05, closed=True)
    assert calls == [4, 8, 16, 32] and res.ok and res.closed
    assert res.pitch_mm == pytest.approx(10.0 / 32) and res.pitch_ceiling_mm == pytest.approx(2.5)
    # an open ramp that does not return to its start fails the closing step at every pitch
    bad = refine_pitch(10.0, 1.0, lambda n: _ramp_layers(n, 40.0 / n), params, 0.5, closed=True)
    assert not bad.ok and bad.max_step_deg == pytest.approx(40.0 - 40.0 / bad.points)


def test_point_classes_give_the_cause_nozzle_for_f4_on_the_nozzle_only():
    """Review I3 / CP-56 decision 2: F4 failures only on nozzle_tip / nozzle_body (and F9) → cause 'nozzle'; a point
    where another head link also collides stays 'F4'."""
    layers = [[_c(0, 0, 'S+E+W+', [0] * 6), _c(0, 1, 'S+E+W+', [0] * 6)], [_c(1, 0, 'S+E+W+', [0] * 6)]]
    valid = [[False, False], [False]]
    f4 = [[False, False], [False]]
    cols = _cols(layers, valid, f4=f4, clr=[[None, None], [None]])
    cols['scene_contacts'][0] = ['nozzle_body<->part', 'jig_1<->nozzle_tip']
    cols['scene_contacts'][1] = ['nozzle_body<->part;head_body<->jig_1']
    pts = classify_points(layers, cols)
    assert point_classes(pts) == ['nozzle', 'F4']
    assert point_classes(pts, nozzle_links=('nozzle_tip',)) == ['F4', 'F4']
    assert POINT_CLASSES[0] == 'nozzle'


def test_coverage_reject_reason_names_the_causes():
    from awm_path.core.s5 import coverage_reject_reason
    r = coverage_reject_reason([None, 'nozzle', 'F4', 'nozzle', None])
    assert r == 'no path: C2 fails at 3 of 5 points (causes: nozzle 2, F4 1)'
    assert coverage_reject_reason([None, None]) is None


def test_unreachable_point_is_ik_even_when_f9_fails():
    """Review minor 7: no kinematically valid candidate → 'IK' (unreachable), even if F9 also fails there."""
    layers = [[_c(0, 0, None, None, reason='no_ik_unreachable')]]
    cols = _cols(layers, [[False]], f4=[[None]], clr=[[None]], contact=[[False]])
    pts = classify_points(layers, cols)
    assert point_classes(pts) == ['IK']
