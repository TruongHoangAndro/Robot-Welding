"""Seam sampling for path planning (A41 input): arc-length samples with tangent and face normals.

Output per sample: ``s_m`` (arc length from the weld start), ``p_m`` (point), ``t`` (unit tangent in
the weld direction), ``n1`` / ``n2`` (outward unit normals of the two faces that meet at the seam;
``n1`` is the reference face = ``weld_side`` when it names a face). Part frame, metres.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

import numpy as np
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE
from OCC.Core.TopExp import topexp
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCC.Core.TopoDS import TopoDS_Shape, topods
from OCC.Core.gp import gp_Pnt, gp_Vec

from awm_geometry.topology import face_normal_at, index_map

DENSE_STEP_M = 5.0e-4
CHAIN_TOL_M = 1.0e-5


class SeamGeometryError(RuntimeError):
    """The seam's edges cannot be sampled (not a chain, unknown id, ...)."""


def _dense_edge(edge) -> np.ndarray:
    """Dense samples of one edge in curve-parameter direction.

    Rows: (x, y, z, tx, ty, tz, u, sign) where u is the curve parameter and sign (+1 / -1) tells
    whether the weld direction follows increasing u.
    """
    curve = BRepAdaptor_Curve(edge)
    u0, u1 = curve.FirstParameter(), curve.LastParameter()
    # Upper bound of the curve length from a coarse polyline, for the sample count.
    coarse = [curve.Value(u0 + (u1 - u0) * k / 16) for k in range(17)]
    approx = sum(coarse[k].Distance(coarse[k + 1]) for k in range(16))
    n = int(min(20000, max(8, math.ceil(approx / DENSE_STEP_M) + 1)))
    rows = []
    for k in range(n):
        u = u0 + (u1 - u0) * k / (n - 1)
        p, v = gp_Pnt(), gp_Vec()
        curve.D1(u, p, v)
        tangent = np.array([v.X(), v.Y(), v.Z()])
        norm = np.linalg.norm(tangent)
        if norm < 1e-12:
            raise SeamGeometryError('degenerate tangent on seam edge')
        rows.append([p.X(), p.Y(), p.Z(), *(tangent / norm), u, 1.0])
    return np.array(rows)


def _chain(segments: List[np.ndarray]) -> List[np.ndarray]:
    """Orient each dense edge so consecutive edges connect end-to-start."""
    if len(segments) == 1:
        return segments
    first, second = segments[0], segments[1]
    ends0 = [first[0, :3], first[-1, :3]]
    ends1 = [second[0, :3], second[-1, :3]]
    best = min(((i, j) for i in range(2) for j in range(2)),
               key=lambda ij: np.linalg.norm(ends0[ij[0]] - ends1[ij[1]]))
    out = [_flip(first) if best[0] == 0 else first]
    for seg in segments[1:]:
        tail = out[-1][-1, :3]
        d_start = np.linalg.norm(seg[0, :3] - tail)
        d_end = np.linalg.norm(seg[-1, :3] - tail)
        if min(d_start, d_end) > CHAIN_TOL_M * 100:
            raise SeamGeometryError(
                f'seam edges are not connected (gap {min(d_start, d_end) * 1000:.3f} mm)')
        out.append(seg if d_start <= d_end else _flip(seg))
    return out


def _flip(seg: np.ndarray) -> np.ndarray:
    out = seg[::-1].copy()
    out[:, 3:6] *= -1.0
    out[:, 7] *= -1.0
    return out


def sample_seam(shape: TopoDS_Shape, seam: Dict, spacing_m: float) -> Dict:
    """Sample one seam (a ``seams.json`` entry) of the healed shape in metres."""
    if spacing_m <= 0:
        raise SeamGeometryError('spacing must be positive')
    edges_map = index_map(shape, TopAbs_EDGE)
    faces_map = index_map(shape, TopAbs_FACE)
    ancestors = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, ancestors)

    dense, edge_faces, curves = [], [], []
    for eid in seam['edges']:
        index = int(eid[1:])
        if not 1 <= index <= edges_map.Size():
            raise SeamGeometryError(f'{seam["seam_id"]}: unknown edge {eid}')
        edge = topods.Edge(edges_map.FindKey(index))
        dense.append(_dense_edge(edge))
        curves.append(BRepAdaptor_Curve(edge))
        fids = []
        for f in ancestors.FindFromKey(edge):
            fid = faces_map.FindIndex(f)
            if fid not in fids:
                fids.append(fid)
        edge_faces.append(fids)

    segs = _chain(dense)
    # _chain keeps the edge order, so faces / curves stay aligned with segs.
    # Weld direction: chain start must be the seam start; closed chains use the reversed flag.
    chain_start, chain_end = segs[0][0, :3], segs[-1][-1, :3]
    closed = bool(seam.get('closed')) or bool(np.linalg.norm(chain_start - chain_end) < 1e-6)
    if closed:
        flip = bool(seam.get('reversed'))
    else:
        start = np.asarray(seam['start_m'])
        flip = np.linalg.norm(chain_end - start) < np.linalg.norm(chain_start - start)
    if flip:
        segs = [_flip(s) for s in reversed(segs)]
        edge_faces = list(reversed(edge_faces))
        curves = list(reversed(curves))

    ref_face = _reference_face(seam.get('weld_side', ''), edge_faces)
    rows, owner = [], []
    for k, seg in enumerate(segs):
        rows.append(seg)
        owner += [k] * len(seg)
    dense_all = np.vstack(rows)
    owner = np.array(owner)
    steps = np.linalg.norm(np.diff(dense_all[:, :3], axis=0), axis=1)
    s_dense = np.concatenate([[0.0], np.cumsum(steps)])
    length = float(s_dense[-1])
    if length < spacing_m:
        raise SeamGeometryError(f'{seam["seam_id"]}: seam length {length * 1000:.2f} mm is '
                                f'shorter than the sample spacing')
    n = int(math.ceil(length / spacing_m)) + 1
    s_out = np.linspace(0.0, length, n)
    points = []
    pair_faces: Optional[List[int]] = None
    for s in s_out:
        j = int(np.clip(np.searchsorted(s_dense, s) - 1, 0, len(s_dense) - 2))
        w = 0.0 if s_dense[j + 1] == s_dense[j] else (s - s_dense[j]) / (s_dense[j + 1] - s_dense[j])
        k = int(owner[j])
        if owner[j + 1] != k:  # zero-length step between two edges of the chain
            u = dense_all[j, 6]
        else:
            u = dense_all[j, 6] * (1 - w) + dense_all[j + 1, 6] * w
        p, t = _evaluate(curves[k], u, dense_all[j, 7])
        fids = _ordered_faces(edge_faces[k], ref_face, pair_faces)
        pair_faces = fids
        normals = [face_normal_at(topods.Face(faces_map.FindKey(f)), p) for f in fids[:2]]
        points.append({'s_m': float(s), 'p_m': p.tolist(), 't': t.tolist(),
                       'n1': normals[0].tolist(),
                       'n2': normals[1].tolist() if len(normals) > 1 else None})
    faces_used = sorted({f for fl in edge_faces for f in fl})
    return {'seam_id': seam['seam_id'], 'length_m': length, 'closed': closed,
            'faces': [f'F{f}' for f in faces_used], 'points': points}


def _evaluate(curve: BRepAdaptor_Curve, u: float, sign: float):
    """Exact point and unit tangent (weld direction) at curve parameter u."""
    pnt, vec = gp_Pnt(), gp_Vec()
    curve.D1(u, pnt, vec)
    t = sign * np.array([vec.X(), vec.Y(), vec.Z()])
    return np.array([pnt.X(), pnt.Y(), pnt.Z()]), t / np.linalg.norm(t)


def _reference_face(weld_side: str, edge_faces: Sequence[Sequence[int]]) -> Optional[int]:
    if weld_side and weld_side.startswith('F') and weld_side[1:].isdigit():
        fid = int(weld_side[1:])
        if not any(fid in fl for fl in edge_faces):
            raise SeamGeometryError(f'weld_side {weld_side} is not adjacent to the seam edges')
        return fid
    return None


def _ordered_faces(fids: Sequence[int], ref: Optional[int],
                   previous: Optional[List[int]]) -> List[int]:
    """Keep n1 on the reference face, else on the face that was n1 before, else the lowest id."""
    fids = list(fids)
    if ref is not None and ref in fids:
        return [ref] + [f for f in fids if f != ref]
    if previous and previous[0] in fids:
        return [previous[0]] + [f for f in fids if f != previous[0]]
    return sorted(fids)
