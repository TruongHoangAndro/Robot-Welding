"""WP-S S.5 Task 2: pitch refinement (CP-53 / F1), failing-point classes, C3 path metrics, coverage."""

import math

import numpy as np
import pytest

from awm_path.core.ladder import NodeScore, solve
from awm_path.core.s5 import (classify_points, longest_window, max_label_steps, path_metrics, point_classes,
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
    assert point_classes(pts) == [None, 'F4', 'clearance', 'IK', 'contact']
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
