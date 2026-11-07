"""G1.12: the program generator uses the planner's weld timing model (``awm_path.core.weld_timing``), so the maximum
travel speed in D07 and the one the program accepts cannot differ (decision Truong 2026-11-07: one source).

Synthetic 7-joint paths only (test data, not part data): the seeded random paths of
``awm_path/test/test_core_weld_timing.py`` where feasibility is not monotonic in the travel speed.
"""
import numpy as np
import pytest

from awm_core import frames
from awm_path.core import weld_timing as wt
from awm_path.core.sync import SyncCell
from awm_program import timing as legacy_timing
from awm_program.core import program7, spline

VL = np.array([180.0] * 6 + [30.0])
AL = np.array([286.0] * 6 + [50.0])
RAMP = 5.0
RES = 0.01


def _random_path(seed):
    """Same generator as awm_path/test/test_core_weld_timing.py::_random_path (seeds 8 and 178 are non-monotonic)."""
    rng = np.random.default_rng(seed)
    length = rng.uniform(20, 120)
    n = int(length / 1.0) + 1
    s = np.linspace(0, length, n)
    q = np.zeros((n, 7))
    for j in range(7):
        for _ in range(3):
            q[:, j] += rng.uniform(-20, 20) * np.sin(2 * np.pi * s / rng.uniform(5, 60) + rng.uniform(0, 6.28))
    return s, q


def _pcell():
    sync = SyncCell(base_link_T_axis=np.eye(4), world_z_base=np.array([0.0, 0.0, 1.0]),
                    tool0_T_tcp=frames.make_transform([0.0, 0.0, 150.0], np.eye(3)),
                    joint_limits_deg=((-360.0, 360.0),) * 6, theta_limits_deg=(-400.0, 400.0),
                    joint_velocity_deg_s=tuple(VL[:6]), theta_velocity_deg_s=float(VL[6]),
                    joint_acceleration_deg_s2=tuple(AL[:6]), theta_acceleration_deg_s2=float(AL[6]))
    return program7.ProgramCell(sync, tuple(AL), (0.0,) * 7, tuple(f'j{k}' for k in range(6)), ('p',))


PARAMS = program7.ProgramParams(weld_ramp_accel_mm_s2=RAMP, speed_search_resolution_mm_s=RES)


@pytest.mark.parametrize('seed,request_mm_s', [(8, 6.0), (178, 6.0), (178, 2.9)])
def test_program_maximum_equals_planner_maximum_on_non_monotonic_paths(seed, request_mm_s):
    """Seed 8: feasible up to ~2.05 mm/s and again at 3.48–3.67 mm/s (a plain bisection on [0, request] gives 2.05);
    seed 178: narrow island 2.89–2.94 mm/s, request 2.9 feasible. Program max == D07 max, exactly."""
    s, q = _random_path(seed)
    pcell = _pcell()
    planner = wt.speed_report(s, q, request_mm_s, VL, AL, RAMP, RES)
    prog_max = program7.max_feasible_speed(s, q, request_mm_s, pcell, PARAMS)
    assert prog_max == planner['max_travel_speed_mm_s']
    assert program7.speed_within_limits(s, q, request_mm_s, pcell, PARAMS) is planner['speed_within_limits']
    assert wt.within_limits(s, q, prog_max, VL, AL, RAMP)
    if seed == 8:
        assert prog_max >= 3.48 - 1e-9


def test_program_helpers_are_the_planner_functions():
    """No copy left: the program's weld timing names are the planner's objects (thin wrapper only for max_abs)."""
    assert spline.TimingError is wt.TimingError
    assert legacy_timing.TimingError is wt.TimingError
    assert legacy_timing.constant_speed_times is wt.constant_speed_times
    assert spline.clamped_cubic is wt.clamped_cubic
    assert spline.weld_times is wt.weld_times
    s, q = _random_path(8)
    t, _ = wt.weld_times(s, 2.0, RAMP)
    v, a = wt.clamped_cubic(t, q)
    for x, y in zip(spline.max_abs(t, q, v, a), wt.max_abs(t, v, a)):
        assert np.array_equal(x, y)


def test_weld_spline_maxima_equal_the_planner():
    s, q = _random_path(178)
    t, v, a, vmax, amax, ramp = program7.weld_spline(s, q, 2.9, PARAMS)
    pv, pa = wt.weld_spline_maxima(s, q, 2.9, RAMP)
    assert np.array_equal(vmax, pv) and np.array_equal(amax, pa)


def test_program_cell_refuses_acceleration_limits_other_than_the_planner_cell():
    pc = _pcell()
    with pytest.raises(ValueError, match='acceleration limits'):
        program7.ProgramCell(pc.sync, tuple(AL * 2.0), pc.home_deg, pc.robot_joints, pc.positioner_joints)


def test_refusal_advice_on_a_maximum_above_the_refused_request():
    """Review 1.a, seed 8: 3.0 mm/s is refused but the maximum is 3.67 mm/s (feasibility not monotonic). The advice
    must be 'a travel speed ≤ the maximum', never 'a lower travel speed'."""
    s, q = _random_path(8)
    pcell = _pcell()
    assert not program7.speed_within_limits(s, q, 3.0, pcell, PARAMS)
    vmax = program7.max_feasible_speed(s, q, 3.0, pcell, PARAMS)
    assert vmax > 3.0
    text = program7.weld_speed_refusal('S01 arc 1/1', s, q, 3.0, pcell, PARAMS, 3.67) + program7.SPEED_ADVICE
    assert f'max feasible travel speed {vmax:.2f} mm/s' in text
    assert '≤ the maximum' in text and 'not monotonic' in text
    assert 'lower travel speed' not in text and 're-plan with a lower' not in text
