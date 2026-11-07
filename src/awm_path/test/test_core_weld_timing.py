"""G1.9: acceleration-aware maximum travel speed of a weld arc (A47; DES-001 Rev 0.5 §4.3 step 5; CP-59 follow-up of
2026-11-06: the travel speed is the Engineer's input, refused with the feasible maximum, never slowed, no limit raised).

Synthetic 7-joint paths (test data, not part data). The positioner angle oscillates along the arc,
θ(s) = A·cos(2π s / λ): the joint velocity grows with v, the acceleration with v², so a speed that is inside every
velocity limit can break the positioner acceleration limit.
"""
import math

import numpy as np
import pytest

from awm_core import frames
from awm_path.core import weld_timing as wt
from awm_path.core.sync import SyncCell, SyncParams

VLIM = np.array([180.0] * 6 + [30.0])
ALIM = np.array([286.0] * 6 + [20.0])
RAMP = 5.0
RES = 0.01


def _wavy(amp=10.0, lam=20.0, length=100.0, step=0.5):
    s = np.arange(0.0, length + 1e-9, step)
    q7 = np.zeros((len(s), 7))
    q7[:, 0] = 0.01 * s                     # a slow robot joint, so every point differs
    q7[:, 6] = amp * np.cos(2.0 * math.pi * s / lam)
    return s, q7


def _velocity_only_max(s, q7, vlim):
    """The G1.3 rule: constant speed, finite differences against the velocity limits."""
    rate = (np.abs(np.diff(q7, axis=0)) / np.diff(s)[:, None]).max(axis=0)
    return 1.0 / float((rate / vlim).max())


def _cell(acc=tuple(ALIM[:6]), theta_acc=float(ALIM[6])):
    return SyncCell(base_link_T_axis=np.eye(4), world_z_base=np.array([0.0, 0.0, 1.0]),
                    tool0_T_tcp=frames.make_transform([0.0, 0.0, 150.0], np.eye(3)),
                    joint_limits_deg=((-360.0, 360.0),) * 6, theta_limits_deg=(-400.0, 400.0),
                    joint_velocity_deg_s=tuple(VLIM[:6]), theta_velocity_deg_s=float(VLIM[6]),
                    joint_acceleration_deg_s2=acc, theta_acceleration_deg_s2=theta_acc)


def test_velocity_only_would_accept_but_acceleration_refuses():
    s, q7 = _wavy()
    v_vel = _velocity_only_max(s, q7, VLIM)
    speed = 6.0
    assert speed < v_vel                                    # the G1.3 rule says "within limits"
    rep = wt.speed_report(s, q7, speed, VLIM, ALIM, RAMP, RES)
    assert rep['speed_within_limits'] is False
    assert rep['limited_by'] == 'joint 7 acceleration'
    vmax = rep['max_travel_speed_mm_s']
    assert 0.0 < vmax < speed < v_vel
    # the reported maximum is accepted by the spline model, 0.02 mm/s more is refused (resolution 0.01 mm/s)
    assert wt.within_limits(s, q7, vmax, VLIM, ALIM, RAMP)
    assert not wt.within_limits(s, q7, vmax + 2 * RES, VLIM, ALIM, RAMP)
    assert abs(vmax / RES - round(vmax / RES)) < 1e-6      # rounded down to the resolution


def test_feasible_speed_reports_true_maximum_above_the_request():
    s, q7 = _wavy()
    rep = wt.speed_report(s, q7, 1.0, VLIM, ALIM, RAMP, RES)
    assert rep['speed_within_limits'] is True
    assert rep['max_travel_speed_mm_s'] > 1.0
    assert wt.within_limits(s, q7, rep['max_travel_speed_mm_s'], VLIM, ALIM, RAMP)
    assert len(rep['max_joint_speed_deg_s']) == 7
    assert rep['max_joint_speed_deg_s'][6] <= VLIM[6]


def test_velocity_bound_still_applies():
    """With huge acceleration limits the maximum is the velocity bound of the spline (≈ the G1.3 value)."""
    s, q7 = _wavy()
    big = np.full(7, 1e9)
    rep = wt.speed_report(s, q7, 100.0, VLIM, big, RAMP, RES)
    v_vel = _velocity_only_max(s, q7, VLIM)
    assert rep['speed_within_limits'] is False
    assert abs(rep['max_travel_speed_mm_s'] - v_vel) < 0.05 * v_vel
    assert wt.within_limits(s, q7, rep['max_travel_speed_mm_s'], VLIM, big, RAMP)


def test_no_feasible_speed_reports_zero():
    """A positioner acceleration limit no speed ≥ the resolution can meet → maximum 0.0 (refused, never slowed)."""
    s, q7 = _wavy()
    alim = ALIM.copy()
    alim[6] = 1e-6
    rep = wt.speed_report(s, q7, 1.0, VLIM, alim, RAMP, RES)
    assert rep['speed_within_limits'] is False and rep['max_travel_speed_mm_s'] == 0.0
    assert rep['limited_by'] == 'joint 7 acceleration'


def test_spline_model_rejects_bad_input():
    s, q7 = _wavy()
    with pytest.raises(wt.TimingError):
        wt.weld_times(s, 0.0, RAMP)
    with pytest.raises(wt.TimingError):
        wt.weld_times(s, 100.0, RAMP)                       # ramps longer than the arc
    with pytest.raises(ValueError):
        wt.speed_report(s, q7, 1.0, VLIM, ALIM[:6], RAMP, RES)


def test_sync_cell_requires_acceleration_limits():
    with pytest.raises(TypeError):
        SyncCell(base_link_T_axis=np.eye(4), world_z_base=np.array([0.0, 0.0, 1.0]), tool0_T_tcp=np.eye(4),
                 joint_limits_deg=((-360.0, 360.0),) * 6, theta_limits_deg=(-400.0, 400.0),
                 joint_velocity_deg_s=(180.0,) * 6, theta_velocity_deg_s=30.0)
    with pytest.raises(ValueError):
        _cell(acc=(286.0,) * 5)
    with pytest.raises(ValueError):
        _cell(theta_acc=0.0)
    c = _cell()
    assert np.allclose(c.velocity_limits_deg_s, VLIM) and np.allclose(c.acceleration_limits_deg_s2, ALIM)


def test_params_record_the_ramp_and_resolution():
    rec = SyncParams().record()
    assert rec['weld_ramp_accel_mm_s2'] == 5.0 and rec['speed_search_resolution_mm_s'] == 0.01
    with pytest.raises(ValueError):
        SyncParams(weld_ramp_accel_mm_s2=0.0)


def _random_path(seed):
    """Seeded smooth random 7-joint path (synthetic test data; seeds found by a search where feasibility is not
    monotonic in the travel speed)."""
    rng = np.random.default_rng(seed)
    length = rng.uniform(20, 120)
    n = int(length / 1.0) + 1
    s = np.linspace(0, length, n)
    q = np.zeros((n, 7))
    for j in range(7):
        for _ in range(3):
            q[:, j] += rng.uniform(-20, 20) * np.sin(2 * np.pi * s / rng.uniform(5, 60) + rng.uniform(0, 6.28))
    return s, q


VL2 = np.array([180.0] * 6 + [30.0])
AL2 = np.array([286.0] * 6 + [50.0])


def test_non_monotonic_feasibility_island_is_found_by_the_coarse_scan():
    """Review minor 1: seed 8 (L 52.7 mm) is feasible up to ~2.05 mm/s and again at 3.48–3.67 mm/s; the plain
    bisection reported 2.05. The coarse scan finds the island; the maximum is feasible."""
    s, q = _random_path(8)
    vmax = wt.max_feasible_speed(s, q, VL2, AL2, RAMP, RES)
    assert vmax >= 3.48 - 1e-9
    assert wt.within_limits(s, q, vmax, VL2, AL2, RAMP)


def test_feasible_request_is_never_reported_above_the_maximum():
    """Review minor 1: seed 178 has a narrow island 2.89–2.94 mm/s (narrower than the scan step): a feasible request
    of 2.9 mm/s must give speed_within_limits true and a maximum ≥ the request (no self-contradicting record)."""
    s, q = _random_path(178)
    assert wt.within_limits(s, q, 2.9, VL2, AL2, RAMP)
    rep = wt.speed_report(s, q, 2.9, VL2, AL2, RAMP, RES)
    assert rep['speed_within_limits'] is True
    assert rep['max_travel_speed_mm_s'] >= 2.9 - 1e-9
    assert wt.within_limits(s, q, rep['max_travel_speed_mm_s'], VL2, AL2, RAMP)
