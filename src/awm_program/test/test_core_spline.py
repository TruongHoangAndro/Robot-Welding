"""Pure tests of the G1.5 spline / timing helpers (no ROS)."""
import numpy as np
import pytest

from awm_program.core import spline


def _data(n=12, m=3, seed=1):
    rng = np.random.default_rng(seed)
    t = np.cumsum(np.r_[0.0, rng.uniform(0.05, 0.6, n - 1)])
    y = np.cumsum(rng.normal(0.0, 2.0, (n, m)), axis=0)
    return t, y


def test_clamped_spline_interpolates_with_rest_ends_and_c2():
    t, y = _data()
    v, a = spline.clamped_cubic(t, y)
    assert v.shape == y.shape and a.shape == y.shape
    assert np.allclose(v[0], 0.0) and np.allclose(v[-1], 0.0)
    tq = np.linspace(t[0], t[-1], 4001)
    p, vq, aq = spline.evaluate(t, y, v, a, tq)
    assert np.allclose(spline.evaluate(t, y, v, a, t)[0], y, atol=1e-12)
    # velocity = derivative of position, acceleration = derivative of velocity (numerically), continuous
    assert np.allclose(np.gradient(p, tq, axis=0)[5:-5], vq[5:-5], atol=2e-2 * np.abs(vq).max())
    eps = 1e-9
    for k in range(1, len(t) - 1):
        left = spline.evaluate(t, y, v, a, np.array([t[k] - eps]))
        right = spline.evaluate(t, y, v, a, np.array([t[k] + eps]))
        assert np.allclose(left[1], right[1], atol=1e-6) and np.allclose(left[2], right[2], atol=1e-5)


def test_two_points_rest_to_rest():
    t = np.array([0.0, 2.0])
    y = np.array([[0.0], [10.0]])
    v, a = spline.clamped_cubic(t, y)
    assert np.allclose(v, 0.0)
    vmax, amax = spline.max_abs(t, y, v, a)
    assert vmax[0] == pytest.approx(1.5 * 10.0 / 2.0)     # cubic rest-to-rest: 1.5 · Δy / T
    assert amax[0] == pytest.approx(6.0 * 10.0 / 4.0)     # 6 · Δy / T²


def test_max_abs_is_exact_against_dense_sampling():
    t, y = _data(seed=7)
    v, a = spline.clamped_cubic(t, y)
    vmax, amax = spline.max_abs(t, y, v, a)
    _, vq, aq = spline.evaluate(t, y, v, a, np.sort(np.r_[np.linspace(t[0], t[-1], 200001), t]))
    assert np.all(vmax >= np.abs(vq).max(axis=0) - 1e-9)
    assert np.allclose(vmax, np.abs(vq).max(axis=0), rtol=1e-6)
    assert np.allclose(amax, np.abs(aq).max(axis=0), rtol=1e-9)


def test_weld_times_constant_part_and_ramp():
    s = np.linspace(0.0, 20.0, 41)
    t, d = spline.weld_times(s, 3.0, 5.0)
    assert d == pytest.approx(0.9)
    inner = (s[:-1] >= d) & (s[1:] <= s[-1] - d)
    assert np.allclose(np.diff(s)[inner] / np.diff(t)[inner], 3.0)
    assert np.all(np.diff(s)[~inner] / np.diff(t)[~inner] < 3.0)
    with pytest.raises(spline.TimingError, match='constant-speed'):
        spline.weld_times(np.linspace(0.0, 1.0, 3), 3.0, 5.0)


def test_rest_to_rest_respects_scaled_limits():
    q = np.column_stack([np.linspace(0.0, 90.0, 80), np.linspace(0.0, -30.0, 80)])
    vlim = np.array([180.0, 30.0])
    alim = np.array([286.0, 50.0])
    t, v, a = spline.rest_to_rest(q, vlim, alim, 0.25)
    vmax, amax = spline.max_abs(t, q, v, a)
    assert np.all(vmax <= 0.25 * vlim * (1 + 1e-9)) and np.all(amax <= 0.25 * alim * (1 + 1e-9))
    assert max(vmax / (0.25 * vlim)) > 0.9 or max(amax / (0.25 * alim)) > 0.9   # not needlessly slow
    assert np.allclose(v[0], 0) and np.allclose(v[-1], 0)
    with pytest.raises(spline.TimingError):
        spline.rest_to_rest(np.zeros((3, 2)), vlim, alim, 0.25)
