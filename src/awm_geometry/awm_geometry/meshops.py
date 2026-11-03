"""Triangle-mesh operations for the export guard of ``export-scene`` (pure numpy, any interpreter).

Inside test = generalised winding number (exact for closed, consistently oriented meshes, also non-convex);
depth = distance to the nearest triangle. Units are those of the caller (mm in the scene export).
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

_BUDGET = 400_000  # point x triangle pairs per numpy block (bounds memory to a few hundred MB)


def _chunk(n_tris: int) -> int:
    return max(1, _BUDGET // max(1, n_tris))


def weld(verts: np.ndarray, tris: np.ndarray, tol: float) -> Tuple[np.ndarray, np.ndarray]:
    """Merge vertices closer than about ``tol`` (grid rounding) and drop triangles that become degenerate."""
    verts = np.asarray(verts, dtype=float)
    keys = np.round(verts / tol).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    out_tris = inverse.reshape(-1)[np.asarray(tris, dtype=np.int64)]
    keep = ((out_tris[:, 0] != out_tris[:, 1]) & (out_tris[:, 1] != out_tris[:, 2])
            & (out_tris[:, 0] != out_tris[:, 2]))
    return verts[first], out_tris[keep]


def is_closed(tris: np.ndarray) -> bool:
    """Every directed edge appears once and its reverse appears once (closed, consistently oriented)."""
    tris = np.asarray(tris, dtype=np.int64)
    edges = np.concatenate([tris[:, [0, 1]], tris[:, [1, 2]], tris[:, [2, 0]]])
    directed = {tuple(e) for e in edges.tolist()}
    return len(directed) == len(edges) and all((b, a) in directed for a, b in directed)


def volume(verts: np.ndarray, tris: np.ndarray) -> float:
    """Signed volume (positive for an outward-oriented closed mesh)."""
    a, b, c = (np.asarray(verts, float)[np.asarray(tris)[:, k]] for k in range(3))
    return float(np.einsum('ij,ij->i', a, np.cross(b, c)).sum() / 6.0)


def inside(verts: np.ndarray, tris: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Boolean mask of ``points`` inside the closed mesh (winding number > 0.5)."""
    if not is_closed(tris):
        raise ValueError('inside test needs a closed, consistently oriented mesh')
    verts = np.asarray(verts, float)
    tris = np.asarray(tris, dtype=np.int64)
    points = np.asarray(points, float).reshape(-1, 3)
    out = np.zeros(len(points), dtype=bool)
    step = _chunk(len(tris))
    for s in range(0, len(points), step):
        p = points[s:s + step, None, :]
        a, b, c = (verts[tris[:, k]][None, :, :] - p for k in range(3))
        la, lb, lc = (np.linalg.norm(x, axis=2) for x in (a, b, c))
        det = np.einsum('ptk,ptk->pt', a, np.cross(b, c))
        den = (la * lb * lc + np.einsum('ptk,ptk->pt', a, b) * lc + np.einsum('ptk,ptk->pt', b, c) * la
               + np.einsum('ptk,ptk->pt', c, a) * lb)
        winding = np.arctan2(det, den).sum(axis=1) / (2.0 * np.pi)
        out[s:s + step] = winding > 0.5
    return out


def distance(verts: np.ndarray, tris: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Distance from each point to the nearest triangle (Ericson, closest point on triangle)."""
    verts = np.asarray(verts, float)
    tris = np.asarray(tris, dtype=np.int64)
    points = np.asarray(points, float).reshape(-1, 3)
    A, B, C = (verts[tris[:, k]] for k in range(3))
    best = np.full(len(points), np.inf)
    step = max(1, _chunk(len(tris)) // 4)
    for s in range(0, len(points), step):
        p = points[s:s + step, None, :]
        q = _closest_on_triangles(p, A[None], B[None], C[None])
        best[s:s + p.shape[0]] = np.linalg.norm(q - p, axis=2).min(axis=1)
    return best


def _closest_on_triangles(p, a, b, c):
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
    bp = p - b
    d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1)
    cp = p - c
    d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
    va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
    with np.errstate(divide='ignore', invalid='ignore'):
        denom = va + vb + vc
        v = np.where(denom != 0, vb / denom, 0.0)
        w = np.where(denom != 0, vc / denom, 0.0)
        res = a + ab * v[..., None] + ac * w[..., None]
        # edge regions
        t_ab = np.where(d1 - d3 != 0, d1 / (d1 - d3), 0.0)
        e_ab = (vc <= 0) & (d1 >= 0) & (d3 <= 0)
        res = np.where(e_ab[..., None], a + ab * t_ab[..., None], res)
        t_ac = np.where(d2 - d6 != 0, d2 / (d2 - d6), 0.0)
        e_ac = (vb <= 0) & (d2 >= 0) & (d6 <= 0)
        res = np.where(e_ac[..., None], a + ac * t_ac[..., None], res)
        t_bc = np.where((d4 - d3) + (d5 - d6) != 0, (d4 - d3) / ((d4 - d3) + (d5 - d6)), 0.0)
        e_bc = (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)
        res = np.where(e_bc[..., None], b + (c - b) * t_bc[..., None], res)
    # vertex regions
    res = np.where(((d1 <= 0) & (d2 <= 0))[..., None], a, res)
    res = np.where(((d3 >= 0) & (d4 <= d3))[..., None], b, res)
    res = np.where(((d6 >= 0) & (d5 <= d6))[..., None], c, res)
    return res


def penetration(verts: np.ndarray, tris: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Depth of each point inside the closed mesh (0 outside); points outside the mesh bbox are skipped."""
    verts = np.asarray(verts, float)
    points = np.asarray(points, float).reshape(-1, 3)
    depth = np.zeros(len(points))
    lo, hi = verts.min(axis=0), verts.max(axis=0)
    cand = np.nonzero(np.all((points >= lo) & (points <= hi), axis=1))[0]
    if len(cand) == 0:
        return depth
    ins = cand[inside(verts, tris, points[cand])]
    if len(ins):
        depth[ins] = distance(verts, tris, points[ins])
    return depth


def sample_surface(verts: np.ndarray, tris: np.ndarray, spacing: float) -> Tuple[np.ndarray, np.ndarray]:
    """Points on every triangle on a barycentric grid whose step along each edge is <= ``spacing``.

    Returns ``(points, owner)`` with ``owner`` = triangle index of each point. The triangle vertices are samples.
    """
    verts = np.asarray(verts, float)
    tris = np.asarray(tris, dtype=np.int64)
    a, b, c = (verts[tris[:, k]] for k in range(3))
    longest = np.max([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1),
                      np.linalg.norm(a - c, axis=1)], axis=0)
    steps = np.maximum(1, np.ceil(longest / spacing).astype(np.int64))
    points, owners = [], []
    for n in np.unique(steps):
        idx = np.nonzero(steps == n)[0]
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing='ij')
        keep = (i + j) <= n
        w1, w2 = i[keep] / n, j[keep] / n
        w0 = 1.0 - w1 - w2
        pts = (a[idx, None, :] * w0[None, :, None] + b[idx, None, :] * w1[None, :, None]
               + c[idx, None, :] * w2[None, :, None])
        points.append(pts.reshape(-1, 3))
        owners.append(np.repeat(idx, len(w0)))
    return np.vstack(points), np.concatenate(owners)
