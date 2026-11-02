"""Tests of the single mm <-> m / deg <-> rad adapter (ADR-14)."""
import math

import numpy as np
import pytest

from awm_core import units


def test_length_round_trip() -> None:
    assert units.m_to_mm(units.mm_to_m(123.456)) == pytest.approx(123.456, abs=1e-12)
    assert units.mm_to_m(1000.0) == 1.0


def test_angle_round_trip() -> None:
    assert units.rad_to_deg(units.deg_to_rad(-270.0)) == pytest.approx(-270.0, abs=1e-12)
    assert units.deg_to_rad(180.0) == pytest.approx(math.pi)


def test_arrays_are_converted_elementwise() -> None:
    out = units.mm_to_m([1000, 2000])
    assert isinstance(out, np.ndarray)
    np.testing.assert_array_equal(out, np.array([1.0, 2.0]))


def test_transform_scales_translation_only() -> None:
    c, s = math.cos(math.pi / 6), math.sin(math.pi / 6)
    t = np.array([[c, -s, 0.0, 100.0],
                  [s, c, 0.0, -200.0],
                  [0.0, 0.0, 1.0, 300.0],
                  [0.0, 0.0, 0.0, 1.0]])
    t_before = t.copy()
    out = units.transform_mm_to_m(t)
    np.testing.assert_array_equal(out[:3, :3], t[:3, :3])
    np.testing.assert_allclose(out[:3, 3], [0.1, -0.2, 0.3], atol=1e-15)
    np.testing.assert_array_equal(out[3], [0.0, 0.0, 0.0, 1.0])
    np.testing.assert_array_equal(t, t_before)
    back = units.transform_m_to_mm(out)
    np.testing.assert_allclose(back, t, atol=1e-12)


def test_transform_rejects_non_4x4() -> None:
    with pytest.raises(ValueError):
        units.transform_mm_to_m(np.eye(3))
    with pytest.raises(ValueError):
        units.transform_m_to_mm(np.eye(3))
