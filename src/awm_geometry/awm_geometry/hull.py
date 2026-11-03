"""3D convex hull (Quickhull) in numpy, for the collision envelopes of cell components (DES-001 §5.4).

Pure numpy (no OCC, no scipy: neither is in the conda env ``awm_occ`` and nothing is installed for the spike), so
it is unit-tested in any interpreter. Points within ``eps`` of a hull face count as inside; coplanar points
are not kept as hull vertices.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np


_BUDGET = 2_000_000  # point x face pairs per block when points are assigned to new faces


class HullError(ValueError):
    """The points do not span a volume, or the hull failed its consistency check."""


def _plane(pts: np.ndarray, a: int, b: int, c: int) -> Tuple[np.ndarray, float]:
    n = np.cross(pts[b] - pts[a], pts[c] - pts[a])
    norm = np.linalg.norm(n)
    if norm == 0.0:
        raise HullError('degenerate hull face')
    n = n / norm
    return n, float(n @ pts[a])


def convex_hull(points: np.ndarray, eps: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray]:
    """Convex hull of ``points`` (N x 3): ``(vertices V x 3, triangles T x 3)``, outward counter-clockwise.

    ``eps`` (same unit as the points) defaults to 1e-9 of the bounding-box diagonal (at least 1e-12).
    Raises :class:`HullError` if the points are (nearly) coplanar or the result is not a closed hull.
    """
    pts = np.unique(np.asarray(points, dtype=float).reshape(-1, 3), axis=0)
    if len(pts) < 4:
        raise HullError(f'{len(pts)} distinct point(s): a hull needs at least 4')
    diag = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))
    eps = max(1e-9 * diag, 1e-12) if eps is None else float(eps)

    # Initial tetrahedron: extreme pair on the widest axis, farthest from that line, farthest from the plane.
    axis = int(np.argmax(pts.max(axis=0) - pts.min(axis=0)))
    i0, i1 = int(np.argmin(pts[:, axis])), int(np.argmax(pts[:, axis]))
    line = pts[i1] - pts[i0]
    d_line = np.linalg.norm(np.cross(pts - pts[i0], line), axis=1) / np.linalg.norm(line)
    i2 = int(np.argmax(d_line))
    if d_line[i2] <= eps:
        raise HullError('points are collinear')
    normal = np.cross(pts[i1] - pts[i0], pts[i2] - pts[i0])
    normal /= np.linalg.norm(normal)
    d_plane = (pts - pts[i0]) @ normal
    i3 = int(np.argmax(np.abs(d_plane)))
    if abs(d_plane[i3]) <= eps:
        raise HullError('points are coplanar')
    centroid = pts[[i0, i1, i2, i3]].mean(axis=0)

    faces: Dict[int, dict] = {}
    edges: Dict[Tuple[int, int], int] = {}
    next_id = [0]

    def add_face(a: int, b: int, c: int) -> int:
        n, d = _plane(pts, a, b, c)
        if n @ centroid - d > 0:  # centroid must be inside (behind every face)
            b, c = c, b
            n, d = -n, -d
        fid = next_id[0]
        next_id[0] += 1
        faces[fid] = {'v': (a, b, c), 'n': n, 'd': d, 'out': np.empty(0, dtype=np.int64)}
        for e in ((a, b), (b, c), (c, a)):
            edges[e] = fid
        return fid

    def assign(candidates: np.ndarray, fids: List[int]) -> None:
        if len(candidates) == 0 or not fids:
            return
        normals = np.array([faces[f]['n'] for f in fids])
        offsets = np.array([faces[f]['d'] for f in fids])
        best = np.empty(len(candidates), dtype=np.int64)
        outside = np.empty(len(candidates), dtype=bool)
        step = max(1, _BUDGET // len(fids))  # bounded point x face block (memory)
        for s in range(0, len(candidates), step):
            dist = pts[candidates[s:s + step]] @ normals.T - offsets
            k = np.argmax(dist, axis=1)
            best[s:s + step] = k
            outside[s:s + step] = dist[np.arange(len(k)), k] > eps
        for k, fid in enumerate(fids):
            faces[fid]['out'] = candidates[outside & (best == k)]

    first = [add_face(i0, i1, i2), add_face(i0, i1, i3), add_face(i0, i2, i3), add_face(i1, i2, i3)]
    rest = np.setdiff1d(np.arange(len(pts)), [i0, i1, i2, i3])
    assign(rest, first)
    pending = [f for f in first if len(faces[f]['out'])]

    while pending:
        fid = pending.pop()
        face = faces.get(fid)
        if face is None or len(face['out']) == 0:
            continue
        out = face['out']
        apex = int(out[np.argmax(pts[out] @ face['n'] - face['d'])])
        p = pts[apex]
        visible = {fid}
        stack = [fid]
        while stack:
            g = faces[stack.pop()]
            a, b, c = g['v']
            for u, v in ((a, b), (b, c), (c, a)):
                h = edges[(v, u)]
                if h not in visible and p @ faces[h]['n'] - faces[h]['d'] > eps:
                    visible.add(h)
                    stack.append(h)
        horizon = []
        orphans = []
        for g in visible:
            a, b, c = faces[g]['v']
            for u, v in ((a, b), (b, c), (c, a)):
                if edges[(v, u)] not in visible:
                    horizon.append((u, v))
            orphans.append(faces[g]['out'])
        for g in visible:
            a, b, c = faces[g]['v']
            for e in ((a, b), (b, c), (c, a)):
                if edges.get(e) == g:
                    del edges[e]
            del faces[g]
        new = []
        for u, v in horizon:
            n, d = _plane(pts, u, v, apex)
            fid_new = next_id[0]
            next_id[0] += 1
            faces[fid_new] = {'v': (u, v, apex), 'n': n, 'd': d, 'out': np.empty(0, dtype=np.int64)}
            for e in ((u, v), (v, apex), (apex, u)):
                edges[e] = fid_new
            new.append(fid_new)
        candidates = np.unique(np.concatenate(orphans)) if orphans else np.empty(0, dtype=np.int64)
        assign(candidates[candidates != apex], new)
        pending.extend(f for f in new if len(faces[f]['out']))

    tris = np.array([f['v'] for f in faces.values()], dtype=np.int64)
    used, inverse = np.unique(tris, return_inverse=True)
    tris = inverse.reshape(-1, 3)
    verts = pts[used]
    n_edges = len(edges)
    if n_edges != 3 * len(tris) or any((v, u) not in edges for u, v in edges):
        raise HullError('hull is not a closed 2-manifold')
    if len(verts) - n_edges // 2 + len(tris) != 2:
        raise HullError('hull fails the Euler check V - E + F = 2')
    return verts, tris


def contains(verts: np.ndarray, tris: np.ndarray, points: np.ndarray, tol: float) -> np.ndarray:
    """Boolean mask: which ``points`` lie inside the closed hull ``(verts, tris)`` within ``tol``."""
    a, b, c = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    n = np.cross(b - a, c - a)
    n /= np.linalg.norm(n, axis=1)[:, None]
    d = np.einsum('ij,ij->i', n, a)
    return np.all(np.asarray(points, float) @ n.T - d <= tol, axis=1)


def write_stl(path, verts: np.ndarray, tris: np.ndarray) -> None:
    """Binary STL of an indexed triangle mesh (coordinates written as given, float32)."""
    import struct

    verts = np.asarray(verts, dtype=float)
    tris = np.asarray(tris, dtype=np.int64)
    a, b, c = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    n = np.cross(b - a, c - a)
    norm = np.linalg.norm(n, axis=1)
    n = np.divide(n, norm[:, None], out=np.zeros_like(n), where=norm[:, None] > 0)
    rec = np.zeros(len(tris), dtype=np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)),
                                              ('attr', '<u2')]))
    rec['normal'] = n
    rec['v'] = np.stack([a, b, c], axis=1)
    with open(path, 'wb') as fh:
        fh.write(b'awm_geometry binary STL'.ljust(80, b' '))
        fh.write(struct.pack('<I', len(tris)))
        fh.write(rec.tobytes())
