"""Convex hull (pure numpy, runs in any interpreter)."""

import numpy as np
import pytest

from awm_core import stl
from awm_geometry.hull import HullError, contains, convex_hull, write_stl


def _closed(verts, tris):
    edges = {}
    for t in tris:
        for u, v in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
            edges[(int(u), int(v))] = edges.get((int(u), int(v)), 0) + 1
    return all(c == 1 and (v, u) in edges for (u, v), c in edges.items())


def _volume(verts, tris):
    a, b, c = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    return float(np.einsum('ij,ij->i', a, np.cross(b, c)).sum() / 6.0)


def test_cube_with_interior_and_face_points():
    rng = np.random.default_rng(1)
    corners = np.array([[x, y, z] for x in (0, 10) for y in (0, 20) for z in (0, 30)], float)
    inner = rng.uniform([0, 0, 0], [10, 20, 30], size=(500, 3))
    on_face = np.column_stack([rng.uniform(0, 10, 50), np.zeros(50), rng.uniform(0, 30, 50)])
    verts, tris = convex_hull(np.vstack([inner, corners, on_face]))
    assert len(verts) == 8 and len(tris) == 12
    assert _closed(verts, tris)
    assert _volume(verts, tris) == pytest.approx(6000.0)  # positive: outward orientation
    assert contains(verts, tris, np.vstack([inner, on_face]), 1e-9).all()


def test_sphere_points_all_on_hull():
    rng = np.random.default_rng(2)
    p = rng.normal(size=(2000, 3))
    p = 50.0 * p / np.linalg.norm(p, axis=1)[:, None] + [100.0, -200.0, 300.0]
    verts, tris = convex_hull(p)
    assert len(verts) == 2000 and _closed(verts, tris)
    assert len(verts) - len(tris) * 3 // 2 + len(tris) == 2
    assert contains(verts, tris, p, 1e-6).all()
    assert not contains(verts, tris, [[100.0, -200.0, 351.0]], 1e-6).any()


def test_degenerate_inputs_fail():
    with pytest.raises(HullError, match='at least 4'):
        convex_hull(np.zeros((3, 3)))
    with pytest.raises(HullError, match='coplanar'):
        convex_hull(np.random.default_rng(3).uniform(size=(50, 3)) * [1, 1, 0])


def test_write_stl_round_trip(tmp_path):
    corners = np.array([[x, y, z] for x in (0, 1) for y in (0, 2) for z in (0, 3)], float)
    verts, tris = convex_hull(corners)
    write_stl(tmp_path / 'h.stl', verts, tris)
    v2, t2 = stl.read_stl(tmp_path / 'h.stl')
    assert len(t2) == 12 and len(v2) == 8
    assert np.allclose(np.sort(v2, axis=0), np.sort(verts, axis=0))
