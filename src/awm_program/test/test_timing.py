"""Time parameterisation (pure)."""

import numpy as np
import pytest

from awm_path.params import ChainParams
from awm_program.generator import velocity_limits
from awm_program.timing import (TimingError, arc_length, constant_speed_times, joint_velocities,
                                max_velocity_ratio, trapezoid_times)


def test_constant_speed_profile():
    s = np.linspace(0.0, 0.16, 81)
    t = constant_speed_times(s, 0.006, 0.5)
    v = np.diff(s) / np.diff(t)
    assert np.all(np.diff(t) > 0)
    # Cruise at the travel speed; ramp distance v^2/2a = 36 um, inside the first step.
    assert np.allclose(v[2:-2], 0.006, rtol=1e-9)
    assert t[-1] == pytest.approx(0.16 / 0.006 + 0.006 / 0.5, rel=1e-9)


def test_triangular_profile():
    s = np.linspace(0.0, 0.001, 11)
    t = constant_speed_times(s, 1.0, 0.5)
    assert t[-1] == pytest.approx(2 * np.sqrt(0.001 / 0.5))


def test_timing_errors():
    with pytest.raises(TimingError):
        constant_speed_times(np.array([0.0, 0.0]), 0.01, 0.5)
    with pytest.raises(TimingError):
        constant_speed_times(np.array([0.0, 0.01, 0.01, 0.02]), 0.01, 0.5)
    with pytest.raises(TimingError):
        constant_speed_times(np.array([0.0, 0.01]), 0.0, 0.5)


def test_arc_length():
    pts = np.array([[0, 0, 0], [3, 4, 0], [3, 4, 12]], float)
    assert np.allclose(arc_length(pts), [0, 5, 17])


def test_trapezoid_respects_limits():
    q = np.linspace([0, 0, 0, 0, 0, 0], [1.0, -0.5, 0.2, 0.0, 0.3, 2.0], 50)
    q = np.vstack([q, q[-1]])  # a repeated point is dropped
    vmax = np.full(6, 0.6)
    amax = np.full(6, 1.0)
    t, keep = trapezoid_times(q, vmax, amax)
    assert keep.sum() == 50 and t[0] == 0.0
    ratio, joint = max_velocity_ratio(q[keep], t, vmax)
    assert ratio <= 1.0 + 1e-9 and joint == 5
    v = joint_velocities(q[keep], t)
    assert np.allclose(v[0], 0) and np.allclose(v[-1], 0)


def test_segment_limits():
    p = ChainParams()
    assert np.allclose(velocity_limits(p, 'weld'), p.max_joint_velocity_rad_s)
    assert np.allclose(velocity_limits(p, 'air'),
                       np.array(p.max_joint_velocity_rad_s) * p.velocity_scaling)
