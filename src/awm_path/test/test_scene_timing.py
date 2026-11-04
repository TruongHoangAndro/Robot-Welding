"""WP-S S.4 C5: seam frames on the circle through the D20 preview frames (timing runner helper)."""

import math

import numpy as np
import pytest

from awm_path.scene_timing import circle_frames


def _preview(c, r, n_axis, start):
    n_axis = np.asarray(n_axis, float) / np.linalg.norm(n_axis)
    e0 = np.asarray(start, float) / np.linalg.norm(start)
    e1 = np.cross(n_axis, e0)
    out = []
    for k in range(4):
        a = k * math.pi / 2
        z = math.cos(a) * e0 + math.sin(a) * e1
        out.append({'point_mm': list(c + r * z), 'z': list(z), 'x': list(np.cross(n_axis, z))})
    return out


def test_circle_frames_reproduce_the_preview_frames():
    prev = _preview(np.array([10.0, -5.0, 3.0]), 13.335, [0, 0.1, 1.0], [1.0, 0.0, 0.0])
    frames = circle_frames(prev, 0.0, 270.0, 4)
    for f, p in zip(frames, prev):
        assert np.allclose(f[:3, 3], p['point_mm'], atol=1e-9)
        assert np.allclose(f[:3, 2], p['z'], atol=1e-9)
        assert np.allclose(f[:3, 0], p['x'], atol=1e-9)
        assert np.allclose(f[:3, :3].T @ f[:3, :3], np.eye(3), atol=1e-12)


def test_circle_frames_respect_the_travel_direction_of_the_preview():
    prev = _preview(np.zeros(3), 5.0, [0, 0, 1], [1, 0, 0])
    rev = [dict(p, x=list(-np.asarray(p['x']))) for p in prev]   # reversed travel direction
    f = circle_frames(rev, 0.0, 90.0, 2)
    assert np.allclose(f[0][:3, 0], rev[0]['x'], atol=1e-9)
    assert np.allclose(f[1][:3, 3], rev[3]['point_mm'], atol=1e-9)  # phi follows the travel direction
    with pytest.raises(ValueError):
        circle_frames(prev[:2], 0.0, 90.0, 3)
