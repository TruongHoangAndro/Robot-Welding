"""WP-S S.6 Task 1: Mode C core — B-03 angular step, θ grid, rotation sign, coaxiality (F8), pose ranking."""

import math

import numpy as np
import pytest

from awm_path.core.ladder import NodeScore
from awm_path.core.mode_c import (angular_step_max_deg, closing_overlap_deg, coaxiality, failing_ranges,
                                  interval_bounds, part_T_tcp_at, radial_extent_mm, rot_x_deg, select_pose,
                                  theta_grid, travel_sign, world_T_flange_at)
from awm_path.core.states import SingularityMetrics
from awm_path.core.sweep import Candidate, PlannerParams


def test_angular_step_is_the_b03_bound():
    # Δθ = 2 (c - margin) / r_max  (rad)
    assert angular_step_max_deg(5.0, 3.0, 130.0) == pytest.approx(math.degrees(4.0 / 130.0))
    with pytest.raises(ValueError):
        angular_step_max_deg(5.0, 5.0, 130.0)
    with pytest.raises(ValueError):
        angular_step_max_deg(5.0, 1.0, 0.0)
    with pytest.raises(ValueError):
        angular_step_max_deg(5.0, -1.0, 10.0)


def test_theta_grid_uniform_and_not_coarser_than_the_bound():
    g = theta_grid(368.6, 1.762, direction=-1)
    assert g[0] == 0.0 and g[-1] == pytest.approx(-368.6)
    steps = np.abs(np.diff(g))
    assert np.allclose(steps, steps[0]) and steps[0] <= 1.762 + 1e-12
    assert len(g) == math.ceil(368.6 / 1.762) + 1
    assert len(theta_grid(10.0, 5.0)) == 3          # exact multiple: no extra step
    with pytest.raises(ValueError):
        theta_grid(10.0, 1.0, direction=0)


def test_closing_overlap_deg():
    assert closing_overlap_deg(2.0, 13.335) == pytest.approx(math.degrees(2.0 / 13.335))
    assert closing_overlap_deg(0.0, 13.335) == 0.0
    with pytest.raises(ValueError):
        closing_overlap_deg(-1.0, 13.0)


def test_radial_extent_is_distance_from_the_axis_line():
    pts = np.array([[100.0, 3.0, 4.0], [-50.0, 0.0, 1.0], [0.0, -6.0, 8.0]])
    assert radial_extent_mm(pts, np.zeros(3), np.array([2.0, 0.0, 0.0])) == pytest.approx(10.0)
    assert radial_extent_mm(pts, np.array([0.0, 0.0, 1.0]), np.array([1.0, 0, 0])) == pytest.approx(math.hypot(6, 7))


def test_coaxiality_offset_angle_and_f8_verdict():
    ok = coaxiality(np.array([5.0, 0.03, 0.0]), np.array([-1.0, 0, 0]), np.zeros(3), np.array([1.0, 0, 0]))
    assert ok['offset_mm'] == pytest.approx(0.03) and ok['angle_deg'] == pytest.approx(0.0, abs=1e-9)
    assert ok['pass'] is True and ok['tol_mm'] == 0.05 and ok['tol_deg'] == 0.1
    tilt = np.array([math.cos(math.radians(0.2)), math.sin(math.radians(0.2)), 0.0])
    bad = coaxiality(np.zeros(3), tilt, np.zeros(3), np.array([1.0, 0, 0]))
    assert bad['angle_deg'] == pytest.approx(0.2) and bad['pass'] is False
    assert coaxiality(np.array([0, 0, 0.06]), np.array([1.0, 0, 0]), np.zeros(3),
                      np.array([1.0, 0, 0]))['pass'] is False


def _seam_frame(p, t, z):
    f = np.eye(4)
    f[:3, 0], f[:3, 2] = t, z
    f[:3, 1] = np.cross(z, t)
    f[:3, 3] = p
    return f


@pytest.mark.parametrize('t', [(0.0, 1.0, 0.0), (0.0, -1.0, 0.0)])
def test_travel_sign_advances_the_contact_along_seam_x(t):
    r = 13.0
    t = np.array(t)
    p = np.array([7.0, 0.0, r])
    world_T_seam = _seam_frame(p, t, np.array([0.0, 0.0, 1.0]))
    flange0 = np.eye(4)
    sign = travel_sign(world_T_seam, flange0[:3, 3], flange0[:3, 0])
    # finite difference: contact point (fixed in world) seen in the part frame after a small turn sign·dθ
    world_T_tcp = world_T_seam
    a = part_T_tcp_at(flange0, np.eye(4), world_T_tcp, 0.0)[:3, 3]
    b = part_T_tcp_at(flange0, np.eye(4), world_T_tcp, sign * 0.5)[:3, 3]
    assert float((b - a) @ t) > 0.0


def test_travel_sign_refuses_a_point_on_the_axis():
    with pytest.raises(ValueError):
        travel_sign(_seam_frame(np.array([3.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])),
                    np.zeros(3), np.array([1.0, 0, 0]))


def test_world_T_flange_turns_about_flange_x_and_part_pose_follows():
    w0 = np.eye(4)
    w0[:3, 3] = [1.0, 2.0, 3.0]
    w = world_T_flange_at(w0, 90.0)
    assert np.allclose(w[:3, 0], [1, 0, 0]) and np.allclose(w[:3, 1], [0, 0, 1]) and np.allclose(w[:3, 3], [1, 2, 3])
    assert np.allclose(rot_x_deg(90.0) @ rot_x_deg(-90.0), np.eye(4))
    flange_T_part = np.eye(4)
    flange_T_part[:3, 3] = [0.0, 0.0, 10.0]
    world_T_tcp = np.eye(4)
    world_T_tcp[:3, 3] = [1.0, 2.0, 13.0]
    # at θ = 0 the TCP is at the part origin; after 180° it is 20 mm from it along part -Z
    assert np.allclose(part_T_tcp_at(w0, flange_T_part, world_T_tcp, 0.0)[:3, 3], 0.0)
    assert np.allclose(part_T_tcp_at(w0, flange_T_part, world_T_tcp, 180.0)[:3, 3], [0, 0, -20.0])


def _cand(point, psi_i, label, sin_q5=0.9):
    m = SingularityMetrics(sin_q5=sin_q5, sin_q3=0.9, wrist_centre_mm=300.0)
    return Candidate(point, psi_i, 10.0 * psi_i, label, (0.0,) * 6, m, None)


def test_select_pose_valid_only_cost_then_psi_then_label_then_point():
    p = PlannerParams()
    entries = [
        (0, _cand(0, 3, 'S+E+W+'), NodeScore(True, 10.0)),
        (1, _cand(1, 1, 'S-E+W+'), NodeScore(True, 10.0)),
        (1, _cand(1, 1, 'S+E+W+'), NodeScore(True, 10.0)),      # same cost and ψ: label order decides
        (2, _cand(2, 0, 'S+E+W+'), NodeScore(False, None)),     # invalid: never ranked
        (3, _cand(3, 0, 'S+E+W+'), NodeScore(True, 5.0)),       # lower clearance → higher cost
        (4, _cand(4, 1, 'S+E+W+'), NodeScore(True, 10.0)),      # tie with point 1: lower point first
        (5, Candidate(5, 0, 0.0, None, None, None, 'no_ik_unreachable'), NodeScore(False)),   # kinematic reject
    ]
    ranked = select_pose(entries, p)
    assert [(r['point'], r['psi_index'], r['label']) for r in ranked] == [
        (1, 1, 'S+E+W+'), (4, 1, 'S+E+W+'), (1, 1, 'S-E+W+'), (0, 3, 'S+E+W+'), (3, 0, 'S+E+W+')]
    assert ranked[0]['rank'] == 1 and ranked[-1]['cost'] > ranked[0]['cost']


def test_interval_bounds_and_failing_ranges():
    b = interval_bounds([10.0, 5.0, None, 10.0], 2.0, 100.0)
    half = 100.0 * math.radians(2.0) / 2
    assert b[0] == pytest.approx(5.0 - half) and b[1] is None and b[2] is None
    th = [0.0, -1.0, -2.0, -3.0, -4.0, -5.0]
    assert failing_ranges(th, [True, False, False, True, False, True]) == [
        {'theta_from_deg': -1.0, 'theta_to_deg': -2.0, 'states': 2},
        {'theta_from_deg': -4.0, 'theta_to_deg': -4.0, 'states': 1}]
    assert failing_ranges(th, [True] * 6) == []
