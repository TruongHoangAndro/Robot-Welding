"""Mesh operations for the export guard (pure numpy, runs in any interpreter)."""

import numpy as np
import pytest

from awm_geometry import meshops

_DIRS = [((1, 0, 0), [(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)]),
         ((-1, 0, 0), [(0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0)]),
         ((0, 1, 0), [(0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0)]),
         ((0, -1, 0), [(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)]),
         ((0, 0, 1), [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]),
         ((0, 0, -1), [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)])]


def voxel_mesh(cells, size=1.0):
    """Closed outward mesh of a union of unit cubes (test helper; duplicated vertices, welded later)."""
    cells = set(map(tuple, cells))
    verts, tris = [], []
    for c in cells:
        for d, quad in _DIRS:
            if tuple(np.add(c, d)) in cells:
                continue
            base = len(verts)
            verts += [tuple(size * (np.add(c, q))) for q in quad]
            tris += [(base, base + 1, base + 2), (base, base + 2, base + 3)]
    return meshops.weld(np.array(verts, float), np.array(tris), 1e-9)


U_CELLS = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (0, 1, 0), (2, 1, 0)]


def test_weld_closed_and_volume():
    verts, tris = voxel_mesh(U_CELLS, 10.0)
    assert meshops.is_closed(tris)
    assert meshops.volume(verts, tris) == pytest.approx(5000.0)


def test_inside_non_convex():
    verts, tris = voxel_mesh(U_CELLS, 10.0)
    pts = np.array([[5, 5, 5], [15, 15, 5], [25, 15, 5], [35, 5, 5], [15, 5, 5]], float)
    assert meshops.inside(verts, tris, pts).tolist() == [True, False, True, False, True]


def test_penetration_depth():
    verts, tris = voxel_mesh(U_CELLS, 10.0)
    pts = np.array([[5, 5, 5], [15, 15, 5], [15, 9, 5], [1, 5, 5], [40, 40, 40]], float)
    depth = meshops.penetration(verts, tris, pts)
    np.testing.assert_allclose(depth, [5.0, 0.0, 1.0, 1.0, 0.0], atol=1e-9)


def test_distance_to_triangles():
    verts = np.array([[0, 0, 0], [10, 0, 0], [0, 10, 0]], float)
    tris = np.array([[0, 1, 2]])
    pts = np.array([[1, 1, 3], [-3, -4, 0], [20, 0, 0], [5, 5, 0]], float)
    np.testing.assert_allclose(meshops.distance(verts, tris, pts), [3.0, 5.0, 10.0, 0.0], atol=1e-12)


def test_open_mesh_is_refused_for_inside():
    verts, tris = voxel_mesh([(0, 0, 0)])
    with pytest.raises(ValueError, match='closed'):
        meshops.inside(verts, tris[:-1], np.zeros((1, 3)))


def test_distance_matches_dense_sampling():
    rng = np.random.default_rng(0)
    verts, tris = rng.normal(size=(3, 3)), np.array([[0, 1, 2]])
    pts = rng.normal(size=(300, 3)) * 2
    u, v = np.meshgrid(np.linspace(0, 1, 300), np.linspace(0, 1, 300))
    m = u + v <= 1
    samples = verts[0] + np.outer(u[m], verts[1] - verts[0]) + np.outer(v[m], verts[2] - verts[0])
    brute = np.array([np.linalg.norm(samples - p, axis=1).min() for p in pts])
    d = meshops.distance(verts, tris, pts)
    assert (d <= brute + 1e-12).all() and np.abs(d - brute).max() < 1e-3


def test_surface_samples_spacing_and_owner():
    verts = np.array([[0, 0, 0], [30, 0, 0], [0, 0.5, 0], [0, 0, 1], [1, 0, 1], [0, 1, 1]], float)
    tris = np.array([[0, 1, 2], [3, 4, 5]])
    pts, owner = meshops.sample_surface(verts, tris, 1.0)
    assert set(owner.tolist()) == {0, 1}
    long = pts[owner == 0]
    # along the 30 mm edge no gap wider than the spacing
    xs = np.sort(long[np.abs(long[:, 1]) < 1e-12][:, 0])
    assert xs[0] == 0 and xs[-1] == 30 and np.diff(xs).max() <= 1.0 + 1e-12
    # every vertex is a sample
    for v in verts:
        assert np.min(np.linalg.norm(pts - v, axis=1)) < 1e-12
