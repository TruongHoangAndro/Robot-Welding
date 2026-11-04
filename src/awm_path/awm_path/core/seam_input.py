"""Seam input for the S.5 spike run: exact seam geometry of a joint edge from the D20 view model, seam frames along
the arc length, and the Engineer-entered arc split (WP-S S.5; TST-002 Rev 0.3 §3.2, §5.1 F1, §5.3 C8; DES-001
Rev 0.6 draft §4.3 step 1; CP-53; open item m4 of S.4).

Why this module exists: the D04 v2 seam geometry (``geometry.points``: s, p, t, n1, n2) is produced by M2 (T2.2),
which does not exist yet. For the spike the geometry is built from the D20 view model of the job:

* the polyline of the joint edge in ``view.glb`` — its vertices are points of the B-Rep edge (tessellation with chord
  ≤ 0.05 mm, float32 in mm);
* the D20 ``curve_type`` of that edge; only ``circle`` is supported here (the golden-part joints are circles). The
  circle through the vertices is the exact curve: the fit residual is checked (``tol_mm``), otherwise refused;
* the D20 preview frames of the joint (exact joint-aware B-Rep frames, Z = bisector of the exterior-face normals):
  start point, travel direction and the side of Z are taken from preview frame 0, and **every** preview frame is
  checked against the circle frame (point ≤ ``tol_mm``; Z, X ≤ ``tol_deg``). A mismatch refuses the input.
* n1 = n2 = the radial direction (DES-001 §4.3: butt joint between tubes, n1 = n2, Z = surface normal; G1 measured
  0.000° on J1 / J2). Assumption recorded with the result: valid for tube butt joints whose exterior faces are
  tangent at the seam; checked only at the preview frames.

The result has the shape of D04 v2 ``geometry`` so that the planner later reads the real D04 unchanged.
Pure Python + numpy, mm / degrees (B-06).
"""

from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple, Union

import numpy as np

from awm_core import units

PathLike = Union[str, Path]

_GLB_MAGIC = 0x46546C67
_CHUNK_JSON = 0x4E4F534A
_CHUNK_BIN = 0x004E4942
_MODE_LINE_STRIP = 3
_FLOAT = 5126


class SeamInputError(ValueError):
    """The seam input cannot be built exactly (missing edge, not a circle, preview mismatch)."""


def read_glb_edge_polyline(glb: PathLike, edge_id: str) -> np.ndarray:
    """Vertices (n, 3) in mm of the D20 polyline node ``edge_id`` (line strip, float32 POSITION)."""
    data = Path(glb).read_bytes()
    if len(data) < 20 or struct.unpack('<I', data[:4])[0] != _GLB_MAGIC:
        raise SeamInputError(f'{glb}: not a GLB file')
    jlen, jtype = struct.unpack('<II', data[12:20])
    if jtype != _CHUNK_JSON:
        raise SeamInputError(f'{glb}: first chunk is not JSON')
    doc = json.loads(data[20:20 + jlen])
    off = 20 + jlen
    blen, btype = struct.unpack('<II', data[off:off + 8])
    if btype != _CHUNK_BIN:
        raise SeamInputError(f'{glb}: second chunk is not BIN')
    binary = data[off + 8:off + 8 + blen]
    nodes = [n for n in doc.get('nodes', []) if n.get('name') == edge_id and 'mesh' in n]
    if len(nodes) != 1:
        raise SeamInputError(f'{glb}: edge node {edge_id!r} found {len(nodes)} times')
    prims = doc['meshes'][nodes[0]['mesh']]['primitives']
    if len(prims) != 1 or prims[0].get('mode') != _MODE_LINE_STRIP:
        raise SeamInputError(f'{glb}: node {edge_id!r} is not one line strip')
    acc = doc['accessors'][prims[0]['attributes']['POSITION']]
    if acc.get('componentType') != _FLOAT or acc.get('type') != 'VEC3':
        raise SeamInputError(f'{glb}: node {edge_id!r} positions are not float VEC3')
    view = doc['bufferViews'][acc['bufferView']]
    start = int(view.get('byteOffset', 0)) + int(acc.get('byteOffset', 0))
    pts = np.frombuffer(binary, dtype='<f4', count=int(acc['count']) * 3, offset=start)
    return pts.reshape(-1, 3).astype(np.float64)


@dataclass(frozen=True)
class CircleFit:
    """Least-squares circle: centre (mm), unit axis (sign arbitrary), radius (mm), max residual (mm, radial and
    out-of-plane)."""

    centre: np.ndarray
    axis: np.ndarray
    radius: float
    max_residual_mm: float


def fit_circle(points_mm: Sequence[Sequence[float]]) -> CircleFit:
    """Plane by SVD, then the algebraic circle fit in the plane; residual over all points."""
    p = np.asarray(points_mm, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 3:
        raise SeamInputError('a circle fit needs at least 3 points (n, 3)')
    m = p.mean(axis=0)
    _, _, vt = np.linalg.svd(p - m)
    axis = vt[2] / np.linalg.norm(vt[2])
    u, v = vt[0], vt[1]
    x, y = (p - m) @ u, (p - m) @ v
    a = np.stack([x, y, np.ones_like(x)], axis=1)
    sol, *_ = np.linalg.lstsq(a, x * x + y * y, rcond=None)
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    r = math.sqrt(sol[2] + cx * cx + cy * cy)
    c = m + cx * u + cy * v
    d = p - c
    out_of_plane = np.abs(d @ axis)
    radial = np.abs(np.linalg.norm(d - np.outer(d @ axis, axis), axis=1) - r)
    return CircleFit(c, axis, float(r), float(max(out_of_plane.max(), radial.max())))


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    c = float(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1.0, 1.0))
    return float(units.rad_to_deg(math.acos(c)))


def circle_joint_geometry(edge_points_mm: Sequence[Sequence[float]], preview_frames: Sequence[Mapping[str, Any]],
                          spacing_max_mm: float, length_mm: float, tol_mm: float = 1e-3,
                          tol_deg: float = 0.01) -> Dict[str, Any]:
    """D04 v2 ``geometry``-like dict of a closed circular joint edge: ``spacing_mm``, ``length_mm``, ``closed``,
    ``points`` (s_mm, p_mm, t, n1, n2) from the edge start (preview frame 0) in its travel direction, plus ``fit``.

    ``length_mm`` is the D20 joint length (checked against 2πr within ``tol_mm``). Raises ``SeamInputError``.
    """
    if len(preview_frames) < 1:
        raise SeamInputError('no preview frames: start point and travel direction unknown')
    fit = fit_circle(edge_points_mm)
    if fit.max_residual_mm > tol_mm:
        raise SeamInputError(f'edge points are not a circle: residual {fit.max_residual_mm:.4g} mm > {tol_mm} mm')
    circ = 2.0 * math.pi * fit.radius
    if abs(circ - float(length_mm)) > tol_mm:
        raise SeamInputError(f'circle length {circ:.6f} mm differs from the joint length {length_mm:.6f} mm')
    p0 = np.asarray(preview_frames[0]['point_mm'], dtype=float)
    e0 = p0 - fit.centre
    e0 -= (e0 @ fit.axis) * fit.axis
    e0 /= np.linalg.norm(e0)
    x0 = np.asarray(preview_frames[0]['x'], dtype=float)
    e1 = x0 - (x0 @ e0) * e0 - (x0 @ fit.axis) * fit.axis
    e1 /= np.linalg.norm(e1)
    axis = np.cross(e0, e1)
    sign = 1.0 if float(np.asarray(preview_frames[0]['z'], dtype=float) @ e0) > 0.0 else -1.0

    def at(phi: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        er = math.cos(phi) * e0 + math.sin(phi) * e1
        return fit.centre + fit.radius * er, np.cross(axis, er), sign * er

    for k, pf in enumerate(preview_frames):
        q = np.asarray(pf['point_mm'], dtype=float)
        d = q - fit.centre
        phi = math.atan2(float(d @ e1), float(d @ e0))
        p, t, z = at(phi)
        errs = (float(np.linalg.norm(p - q)), _angle_deg(z, np.asarray(pf['z'], float)),
                _angle_deg(t, np.asarray(pf['x'], float)))
        if errs[0] > tol_mm or errs[1] > tol_deg or errs[2] > tol_deg:
            raise SeamInputError(f'preview frame {k} does not match the circle frame: point {errs[0]:.4g} mm, '
                                 f'Z {errs[1]:.4g} deg, X {errs[2]:.4g} deg (tolerance {tol_mm} mm / {tol_deg} deg)')
    n = max(3, math.ceil(circ / float(spacing_max_mm) - 1e-12))
    spacing = circ / n
    pts: List[Dict[str, Any]] = []
    for i in range(n):
        p, t, z = at(2.0 * math.pi * i / n)
        pts.append({'s_mm': float(i * spacing), 'p_mm': [float(v) for v in p], 't': [float(v) for v in t],
                    'n1': [float(v) for v in z], 'n2': [float(v) for v in z]})
    return {'spacing_mm': float(spacing), 'length_mm': float(circ), 'closed': True, 'points': pts,
            'fit': {'centre_mm': [float(v) for v in fit.centre], 'axis': [float(v) for v in axis],
                    'radius_mm': fit.radius, 'max_residual_mm': fit.max_residual_mm,
                    'edge_points': int(len(np.asarray(edge_points_mm))), 'tol_mm': tol_mm, 'tol_deg': tol_deg,
                    'preview_frames_checked': len(preview_frames)}}


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise SeamInputError('zero vector in the seam geometry')
    return v / n


def frames_at(points: Sequence[Mapping[str, Any]], s_values: Sequence[float], closed: bool,
              length_mm: float) -> List[np.ndarray]:
    """Seam frames (part frame, 4x4, mm) at arc lengths ``s_values``: linear interpolation of p, t, n1, n2 between
    the geometry points; X = t ⟂ Z, Z = bisector(n1, n2), Y = Z × X. Closed seams wrap s modulo the length."""
    s = np.array([float(p['s_mm']) for p in points])
    if np.any(np.diff(s) <= 0.0):
        raise SeamInputError('geometry s_mm must be strictly increasing')
    arr = {k: np.array([p[k] for p in points], dtype=float) for k in ('p_mm', 't', 'n1', 'n2')}
    if closed:
        s = np.append(s, float(length_mm))
        arr = {k: np.vstack([v, v[:1]]) for k, v in arr.items()}
    out = []
    for sv in s_values:
        q = float(sv) % float(length_mm) if closed else float(sv)
        if q < s[0] - 1e-9 or q > s[-1] + 1e-9:
            raise SeamInputError(f's = {sv} mm outside the seam geometry')
        i = int(np.clip(np.searchsorted(s, q, side='right') - 1, 0, len(s) - 2))
        w = (q - s[i]) / (s[i + 1] - s[i])
        lerp = {k: (1.0 - w) * v[i] + w * v[i + 1] for k, v in arr.items()}
        z = _unit(_unit(lerp['n1']) + _unit(lerp['n2']))
        x = _unit(lerp['t'] - (lerp['t'] @ z) * z)
        t = np.eye(4)
        t[:3, :3] = np.stack([x, np.cross(z, x), z], axis=1)
        t[:3, 3] = lerp['p_mm']
        out.append(t)
    return out


def arc_ranges(length_mm: float, arcs: int, overlap_mm: float, closed: bool) -> List[Tuple[float, float]]:
    """CP-53 arc split (Engineer data ``arcs``, ``arc_overlap_mm``): equal lengths from the seam start; every arc but
    the last extends by the overlap into the next one; on a closed seam the last arc also extends over the start
    (end > length = wrap). Values in mm, unwrapped."""
    if int(arcs) < 1 or overlap_mm < 0.0 or length_mm <= 0.0:
        raise ValueError('arcs >= 1, overlap >= 0 and length > 0 are required')
    step = float(length_mm) / int(arcs)
    out = []
    for k in range(int(arcs)):
        end = (k + 1) * step
        if k < int(arcs) - 1 or (closed and int(arcs) > 1):
            end += float(overlap_mm)
        out.append((float(k * step), float(end)))
    return out


__all__ = ['SeamInputError', 'read_glb_edge_polyline', 'CircleFit', 'fit_circle', 'circle_joint_geometry',
           'frames_at', 'arc_ranges']
