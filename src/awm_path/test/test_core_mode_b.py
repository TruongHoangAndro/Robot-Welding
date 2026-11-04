"""WP-S S.5b: Mode B screening helpers — θ grid and equivalence, circular runs, continuous windows on a closed
seam, coverage bins, covering angle sets (suggestions only). Pure, no ROS."""

import pytest

from awm_path.core.ladder import NodeScore
from awm_path.core.mode_b import (best_cover, bins_covered, circle_cover, circular_runs, cover_sets,
                                  maximal_windows, screening_thetas, theta_classes, theta_representative,
                                  window_intervals, window_lengths)
from awm_path.core.sweep import Candidate, PlannerParams


def _c(point, q1, label='S+E+W+', reason=None):
    return Candidate(point, 0, 0.0, label, None if reason else (float(q1), 0.0, 0.0, 0.0, 0.0, 0.0), None, reason)


def test_screening_thetas_are_step_multiples_inside_the_limits_with_zero():
    assert screening_thetas(-400.0, 400.0, 30.0) == [30.0 * k for k in range(-13, 14)]
    assert screening_thetas(-45.0, 100.0, 30.0) == [-30.0, 0.0, 30.0, 60.0, 90.0]
    with pytest.raises(ValueError):
        screening_thetas(10.0, 100.0, 30.0)        # 0 outside the limits
    with pytest.raises(ValueError):
        screening_thetas(-10.0, 10.0, 0.0)


def test_theta_representative_and_classes():
    assert theta_representative(390.0) == 30.0
    assert theta_representative(-180.0) == 180.0
    assert theta_representative(180.0) == 180.0
    assert theta_representative(-390.0) == -30.0
    cls = theta_classes([-390.0, -30.0, 0.0, 330.0, 360.0])
    assert cls == {-30.0: [-390.0, -30.0, 330.0], 0.0: [0.0, 360.0]}


def test_circular_runs_merge_across_the_seam_start():
    assert circular_runs([True, False, True, True, False, True]) == [(2, 3), (5, 6)]
    assert circular_runs([False, True, True, False]) == [(1, 2)]
    assert circular_runs([True, True, True]) == [(0, 2)]
    assert circular_runs([False, False]) == []


def test_window_lengths_follow_the_dp_on_the_doubled_closed_seam():
    params = PlannerParams()
    # 8 points; q1 steps of 2 deg except a jump of 30 deg between points 5 and 6; point 2 has no valid node
    q = [0, 2, 4, 6, 8, 10, 40, 42]
    layers = [[_c(i, v)] for i, v in enumerate(q)]
    scores = [[NodeScore(i != 2)] for i in range(8)]
    lengths = window_lengths(layers, scores, params)
    # start 3: 3,4,5 then jump -> 3 layers; start 6: 6,7,0(q 0 vs 42 -> jump) -> 2 layers; start 2: invalid
    assert lengths[2] == 0 and lengths[3] == 3 and lengths[6] == 2
    assert lengths[0] == 2                          # 0,1 then the invalid point 2
    full = window_lengths([[_c(i, 2 * i)] for i in range(4)][:2] * 2, None, params)
    assert all(v <= 4 for v in full)                # never longer than the seam


def test_window_lengths_stride_only_starts_at_stride_points():
    params = PlannerParams()
    layers = [[_c(i, 0.0)] for i in range(6)]
    assert window_lengths(layers, None, params, stride=2) == [6, 0, 6, 0, 6, 0]


def test_maximal_windows_drop_contained_ones_on_the_circle():
    # n 8: (6, len 4) covers 6,7,0,1 and contains (7, len 2) and (0, len 2)
    lengths = [2, 0, 0, 0, 0, 0, 4, 2]
    assert maximal_windows(lengths) == [(6, 4)]
    assert maximal_windows([3, 3, 0, 0]) == [(0, 3), (1, 3)]


def test_window_intervals_in_mm():
    assert window_intervals([(6, 4), (1, 1)], pitch_mm=0.5) == [(3.0, 4.5), (0.5, 0.5)]


def test_bins_covered_handles_the_wrap_and_partial_last_bin():
    # seam 10 mm, interval 8.5 .. 12.0 covers bins [8,9)? no (starts 8.5); [9,10) yes; [0,1), [1,2) yes
    assert bins_covered([(8.5, 12.0)], 10.0, 1.0) == [True, True, False, False, False, False, False, False, False,
                                                      True]
    assert bins_covered([(0.0, 9.5)], 9.5, 2.0) == [True] * 5      # last bin 8 .. 9.5


def test_circle_cover_requires_the_overlap_at_every_join():
    ok = circle_cover([(0.0, 6.0), (4.0, 12.0)], 10.0, 2.0)
    assert ok['covers'] and ok['min_join_overlap_mm'] == pytest.approx(2.0)
    short = circle_cover([(0.0, 6.0), (4.5, 12.0)], 10.0, 2.0)
    assert not short['covers'] and short['covered_mm'] == pytest.approx(10.0)
    gap = circle_cover([(0.0, 4.0), (5.0, 9.0)], 10.0, 0.0)
    assert not gap['covers'] and gap['covered_mm'] == pytest.approx(8.0)
    assert circle_cover([(2.0, 13.0)], 10.0, 1.0)['covers']


def test_cover_sets_one_window_per_angle_ranked_by_join_overlap():
    win = {0.0: [(0.0, 6.0)], 30.0: [(4.0, 12.0)], 60.0: [(3.0, 13.0)], 90.0: [(1.0, 2.0)]}
    sets2 = cover_sets(win, 10.0, 2.0, 2)
    assert [s['thetas'] for s in sets2] == [[0.0, 60.0], [0.0, 30.0]]   # 60: joins 3 and 3 > 30: joins 2 and 2
    assert sets2[0]['min_join_overlap_mm'] == pytest.approx(3.0)
    assert sets2[0]['segments'][0] == {'theta_deg': 0.0, 's_from_mm': 0.0, 's_to_mm': 6.0}
    sets3 = cover_sets(win, 10.0, 2.0, 3)
    assert sets3 == []                              # every covering 3-set contains a covering 2-set (redundant)
    assert cover_sets(win, 10.0, 5.0, 2) == []
    best = best_cover(win, 10.0, 2)
    assert best['thetas'] == [0.0, 60.0] and best['covered_pct'] == pytest.approx(100.0)
