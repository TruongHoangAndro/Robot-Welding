"""Nozzle contact check (WP-S S.4 C3; TST-002 Rev 0.3 draft §5.1 F9; DES-001 Rev 0.6 draft §5.4, B-04).

There is no ACM entry between any head link and the part (B-04): the ``nozzle_tip`` collision geometry stops the
contact standoff s_c short of ``tcp_laser`` and is collision-checked like every other link (F4). Contact itself is
checked here, explicitly, on the part collision mesh with its per-triangle ``face_id`` (``collision_faces.json``):

* the samples are the nozzle end disc and its rim in the ``tcp_laser`` plane (tool Z = beam axis): the centre (the
  contact point = ``tcp_laser``), rings every ``ring_spacing_mm`` with ``ring_samples`` points, and the rim with
  ``rim_samples`` points;
* rule (a) **contact point on the seam**: the distance from ``tcp_laser`` to the seam faces (D04 ``faces.n1`` ∪
  ``faces.n2``) is ≤ ``point_tolerance_mm`` and no other face is closer to it (seam-edge exception as in (b));
* rule (b) **no other face rising towards the nozzle within s_c** (CP-56 decision 1, ``rule_b = 'rising'``): only
  the part of a non-seam face that lies on the nozzle side of the **contact plane** (plane through ``tcp_laser``,
  normal to the beam axis) by more than the contact tolerance ``point_tolerance_mm`` counts, i.e. the triangles are
  clipped to ``h > point_tolerance_mm`` with h = height above the plane towards the nozzle (tool −Z). That part
  fails if it is within ``standoff_mm`` of the samples — the end disc / rim in the plane and the rim lifted to
  h = ``point_tolerance_mm`` (the nozzle side wall at the lowest height that counts, so a wall rising at a lateral
  gap < s_c is not excused by the tolerance). Faces that only fall away from the plane (a chamfer next to a seam
  face) do not count; they are listed in ``excused`` when they are within s_c of the disc / rim (S.4 measure).
  Thread crests, walls and ledges that rise towards the nozzle by more than the tolerance still fail.
  Seam-edge exception (both rules): a face is excused where its closest point lies on the **seam edge**, i.e.
  within ``point_tolerance_mm`` of an n1 face *and* of an n2 face (end faces of the two solids of a butt joint).
  ``rule_b = 'unsigned'`` is the S.4 rule (every non-seam face within s_c of the disc / rim, unsigned distance),
  kept only to reproduce records made before CP-56 (S.4 … S.6 of 2026-11-03 / 04, S.5b).

Penetration of the nozzle into a seam face (e.g. a tilted flat tip) is not judged here: the unpadded ``nozzle_tip``
collision geometry in the F4 validity check catches it. A node is valid only if both checks pass.

The nozzle model (TST-002 §3.1 Ø 20 mm cylinder, Assumption until IPG data, I2 / OI-15) is a body of revolution
about the beam axis and the sample set is invariant under rotations by multiples of 360° / gcd(ring_samples,
rim_samples) (5° with the defaults), so the verdict does not depend on ψ (on the 5° / 10° grids of TST-002 §3.3) nor
on the IK branch. A wire guide that turns with ψ (OI-15) would break this and must be added as a sample set.

Pure Python + numpy, mm, no rclpy (B-06).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple, Union

import numpy as np

from awm_core import units
from awm_core.hashing import sha256_file
from awm_path.core.kinematics import check_rigid

PathLike = Union[str, Path]

#: Reasons of a failed contact check (``None`` = passed).
CONTACT_REASONS = ('contact_off_seam', 'other_face_within_standoff')
#: Rule (b) variants: CP-56 (only faces rising towards the nozzle) and the S.4 unsigned rule (reproduction only).
RULE_B = ('rising', 'unsigned')

_CHUNK = 64


def min_rim_samples(radius_mm: float, standoff_mm: float, ring_samples: int) -> int:
    """Smallest multiple of ``ring_samples`` with rim spacing 2π·r / n ≤ 2·s_c (multiple: the sample set keeps the
    rotation invariance of the rings, 360° / ``ring_samples``)."""
    need = math.ceil(math.pi * float(radius_mm) / float(standoff_mm) - 1e-9)
    return int(ring_samples * max(1, math.ceil(need / ring_samples)))


@dataclass(frozen=True)
class ContactParams:
    """Contact check parameters (mm).

    * ``nozzle_radius_mm`` 10, ``standoff_mm`` (s_c) 0.3: TST-002 §3.1 / F9 — Assumption until IPG data (I2);
    * ``point_tolerance_mm`` 0.15: "within tolerance" of F9 is not given a value by TST-002; chosen here as the part
      chord near the seams (0.1 mm, B-Rep → mesh) + 0.05 mm — Assumption, recorded with every result;
    * sample density: the rim spacing 2π·r / ``rim_samples`` must be ≤ 2·s_c (checked here; H-BOX 2π·10 / 360 =
      0.17 mm), so a face crossing the rim is never missed; :func:`awm_path.core.head.contact_params_for` derives
      ``rim_samples`` from the configured r and s_c (review M1). ``point_tolerance_mm`` is also the seam-edge tolerance
      of rule (b) and the height tolerance of the contact plane (CP-56: "allowing the contact tolerance");
      ``tie_mm`` is numerical only;
    * **interior blind spot (Assumption, review M1):** inside the rim the disc is sampled on rings every
      ``ring_spacing_mm`` (2 mm) with ``ring_samples`` points, much coarser than 2·s_c. A non-seam feature that rises
      into the nozzle footprint by h ∈ (``point_tolerance_mm``, s_c) and stays farther than s_c from every disc
      sample (between two rings, not reaching the rim) is not seen by F9. Features rising by h ≥ s_c inside the
      footprint hit the ``nozzle_tip`` collision cylinder (F4, it stops s_c short of ``tcp_laser``), so the blind
      spot is limited to that height band; denser rings would close it at a higher sample cost;
    * ``rule_b``: ``'rising'`` (CP-56 decision 1, default) or ``'unsigned'`` (S.4 rule, reproduction only).

    The nozzle values (radius, s_c) are cell data (CP-56 decision 2): runners build them from the robot description
    in use with :func:`awm_path.core.head.contact_params_for`; the defaults here equal the H-BOX defaults.
    """

    nozzle_radius_mm: float = 10.0
    standoff_mm: float = 0.3
    point_tolerance_mm: float = 0.15
    ring_spacing_mm: float = 2.0
    ring_samples: int = 72
    rim_samples: int = 360
    tie_mm: float = 1e-6
    rule_b: str = 'rising'

    def __post_init__(self) -> None:
        if self.rule_b not in RULE_B:
            raise ValueError(f'rule_b must be one of {RULE_B}, got {self.rule_b!r}')
        if self.nozzle_radius_mm <= 0.0 or self.standoff_mm <= 0.0 or self.rim_samples < 3:
            raise ValueError('nozzle radius and standoff must be > 0 and rim_samples ≥ 3')
        spacing = 2.0 * math.pi * self.nozzle_radius_mm / self.rim_samples
        if spacing > 2.0 * self.standoff_mm + 1e-12:
            raise ValueError(f'rim sample spacing {spacing:.4f} mm > 2·s_c = {2.0 * self.standoff_mm:.4f} mm: a face '
                             f'crossing the rim could be missed; use rim_samples ≥ '
                             f'{min_rim_samples(self.nozzle_radius_mm, self.standoff_mm, self.ring_samples)}')

    def as_record(self) -> dict:
        """Parameters as a plain dict (run records)."""
        return {k: getattr(self, k) for k in self.__dataclass_fields__}


@dataclass(frozen=True)
class PartMesh:
    """Part collision mesh in the ``part`` frame: triangles (T, 3, 3) in mm and the face of each triangle."""

    triangles_mm: np.ndarray
    face_index: np.ndarray
    face_ids: Tuple[str, ...]

    def __post_init__(self) -> None:
        t = np.asarray(self.triangles_mm, dtype=float)
        idx = np.asarray(self.face_index, dtype=int)
        if t.ndim != 3 or t.shape[1:] != (3, 3) or len(t) == 0:
            raise ValueError(f'triangles must have shape (T, 3, 3), got {t.shape}')
        if idx.shape != (len(t),) or idx.min() < 0 or idx.max() >= len(self.face_ids):
            raise ValueError('face_index must give one valid face per triangle')
        if len(set(self.face_ids)) != len(self.face_ids):
            raise ValueError('duplicate face ids')
        object.__setattr__(self, 'triangles_mm', t)
        object.__setattr__(self, 'face_index', idx)
        centroid = t.mean(axis=1)
        object.__setattr__(self, '_centroid', centroid)
        object.__setattr__(self, '_radius', np.linalg.norm(t - centroid[:, None, :], axis=2).max(axis=1))

    @classmethod
    def from_files(cls, stl: PathLike, collision_faces: PathLike) -> 'PartMesh':
        """Read ``cache/collision.stl`` and ``cache/collision_faces.json`` (export-scene, S.4 A3).

        The faces file must name the STL in its ``inputs`` with the matching SHA-256, and its triangle ranges must
        cover every triangle exactly once, in order.
        """
        stl, collision_faces = Path(stl), Path(collision_faces)
        doc = json.loads(collision_faces.read_text(encoding='utf-8'))
        if doc.get('unit', 'mm') != 'mm':
            raise ValueError(f'{collision_faces}: unit {doc.get("unit")!r}, expected mm')
        digest = sha256_file(stl)
        entries = [e for e in doc.get('inputs', []) if Path(e.get('name', '')).name == stl.name]
        if len(entries) != 1 or entries[0].get('sha256') != digest:
            raise ValueError(f'{collision_faces}: sha256 of {stl.name} does not match its inputs entry')
        from awm_core.stl import read_stl
        verts, tris = read_stl(stl)
        triangles = verts[tris]
        faces = doc.get('faces', [])
        index = np.full(len(triangles), -1, dtype=int)
        expected_first = 0
        for k, f in enumerate(faces):
            first, count = int(f['first_triangle']), int(f['triangle_count'])
            if first != expected_first or count < 0:
                raise ValueError(f'{collision_faces}: face {f["face_id"]} range does not follow the previous one')
            index[first:first + count] = k
            expected_first = first + count
        if expected_first != len(triangles) or np.any(index < 0) or doc.get('triangles', len(triangles)) != len(
                triangles):
            raise ValueError(f'{collision_faces}: face ranges ({expected_first} triangles) do not cover the '
                             f'{len(triangles)} triangles of {stl.name}')
        return cls(triangles, index, tuple(str(f['face_id']) for f in faces))

    def local(self, centre_mm: np.ndarray, radius_mm: float) -> np.ndarray:
        """Indices of the triangles that may have a point within ``radius_mm`` of ``centre_mm`` (conservative)."""
        d = np.linalg.norm(self._centroid - centre_mm, axis=1) - self._radius
        return np.nonzero(d <= radius_mm)[0]


@dataclass(frozen=True)
class ContactResult:
    """Result of :func:`contact_check`.

    * ``tcp_seam_distance_mm``: distance from ``tcp_laser`` to the seam faces (inf if none is near);
    * ``tcp_nearest_other``: (face_id, distance) of the nearest non-seam face to ``tcp_laser`` (or None);
    * ``offending``: (face_id, min sample distance) of every non-seam face that fails rule (b), by face id;
    * ``excused``: rule ``'rising'`` only — (face_id, min distance to the disc / rim) of the non-seam faces within
      s_c of the disc / rim (S.4 measure, seam-edge exception applied) that rule (b) does not count (they do not
      rise above the contact plane by more than the tolerance); empty for ``'unsigned'``;
    * ``samples`` / ``triangles``: sample count and local triangle count (for timing records).
    """

    ok: bool
    reason: Optional[str]
    tcp_seam_distance_mm: float
    tcp_nearest_other: Optional[Tuple[str, float]]
    offending: Tuple[Tuple[str, float], ...]
    samples: int
    triangles: int
    excused: Tuple[Tuple[str, float], ...] = ()


def disc_samples(params: ContactParams) -> np.ndarray:
    """Sample points of the nozzle end disc and rim in the ``tcp_laser`` frame (z = 0), mm. Row 0 = centre."""
    r_max = float(params.nozzle_radius_mm)
    pts = [np.zeros((1, 3))]
    n_rings = int(math.floor(r_max / params.ring_spacing_mm - 1e-9))
    for k in range(1, n_rings + 1):
        pts.append(_circle(k * params.ring_spacing_mm, params.ring_samples))
    pts.append(_circle(r_max, params.rim_samples))
    return np.concatenate(pts)


def rule_b_samples(params: ContactParams) -> np.ndarray:
    """Rule (b) ``'rising'`` samples in the ``tcp_laser`` frame: :func:`disc_samples` plus the rim lifted to the
    tolerance height on the nozzle side (z = −``point_tolerance_mm``; tool +Z = beam direction, the nozzle is on −Z).
    """
    rim = _circle(float(params.nozzle_radius_mm), params.rim_samples)
    rim[:, 2] = -float(params.point_tolerance_mm)
    return np.concatenate([disc_samples(params), rim])


def clip_triangles_above(triangles: np.ndarray, origin: np.ndarray, up: np.ndarray, h_min: float
                         ) -> Tuple[np.ndarray, np.ndarray]:
    """Clip triangles (T, 3, 3) to the open half-space ``(x − origin)·up > h_min`` (Sutherland–Hodgman, exact).

    Returns the pieces (K, 3, 3) (a clipped quad is split in two) and the index of the source triangle of each.
    """
    tri = np.asarray(triangles, dtype=float).reshape(-1, 3, 3)
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    h = (tri - np.asarray(origin, dtype=float)) @ up - float(h_min)        # (T, 3)
    inside = h > 0.0
    n_in = inside.sum(axis=1)
    src = np.arange(len(tri))
    pieces, owners = [tri[n_in == 3]], [src[n_in == 3]]

    def cut(p, hp, q, hq):
        t = (hp / (hp - hq))[:, None]
        return p + (q - p) * t

    one = n_in == 1
    if np.any(one):
        k = np.argmax(inside[one], axis=1)                                 # the vertex inside
        order = (k[:, None] + np.arange(3)[None]) % 3
        v = np.take_along_axis(tri[one], order[..., None], axis=1)
        hv = np.take_along_axis(h[one], order, axis=1)
        a, b, c = v[:, 0], v[:, 1], v[:, 2]
        pieces.append(np.stack([a, cut(a, hv[:, 0], b, hv[:, 1]), cut(a, hv[:, 0], c, hv[:, 2])], axis=1))
        owners.append(src[one])
    two = n_in == 2
    if np.any(two):
        k = np.argmin(inside[two], axis=1)                                 # the vertex outside
        order = (k[:, None] + 1 + np.arange(3)[None]) % 3                  # (in, in, out), orientation kept
        v = np.take_along_axis(tri[two], order[..., None], axis=1)
        hv = np.take_along_axis(h[two], order, axis=1)
        a, b, c = v[:, 0], v[:, 1], v[:, 2]
        pb, pa = cut(b, hv[:, 1], c, hv[:, 2]), cut(a, hv[:, 0], c, hv[:, 2])
        pieces += [np.stack([a, b, pb], axis=1), np.stack([a, pb, pa], axis=1)]
        owners += [src[two], src[two]]
    return np.concatenate(pieces).reshape(-1, 3, 3), np.concatenate(owners).astype(int)


def _circle(radius: float, n: int) -> np.ndarray:
    a = units.deg_to_rad(np.arange(n) * 360.0 / n)
    return np.stack([radius * np.cos(a), radius * np.sin(a), np.zeros(n)], axis=1)


def point_triangle_distances(points: np.ndarray, triangles: np.ndarray) -> np.ndarray:
    """Euclidean distances (P, T) from points (P, 3) to triangles (T, 3, 3)."""
    return closest_points_on_triangles(points, triangles)[0]


def closest_points_on_triangles(points: np.ndarray, triangles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Distances (P, T) and closest points (P, T, 3) from points (P, 3) to triangles (T, 3, 3) — Ericson,
    "Real-Time Collision Detection" §5.1.5. Degenerate (zero-area) triangles: distance to their edges (the closest
    point is then approximated by the nearest vertex)."""
    p_all = np.asarray(points, dtype=float).reshape(-1, 3)
    tri = np.asarray(triangles, dtype=float).reshape(-1, 3, 3)
    out = np.empty((len(p_all), len(tri)))
    out_q = np.empty((len(p_all), len(tri), 3))
    if len(tri) == 0:
        return out, out_q
    a, b, c = tri[None, :, 0], tri[None, :, 1], tri[None, :, 2]
    ab, ac = b - a, c - a
    for s in range(0, len(p_all), _CHUNK):
        p = p_all[s:s + _CHUNK, None, :]
        ap, bp, cp = p - a, p - b, p - c
        d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
        d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1)
        d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
        va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
        with np.errstate(divide='ignore', invalid='ignore'):
            t_ab = d1 / (d1 - d3)
            t_ac = d2 / (d2 - d6)
            t_bc = (d4 - d3) / ((d4 - d3) + (d5 - d6))
            denom = va + vb + vc
            v, w = vb / denom, vc / denom
        conds = [(d1 <= 0) & (d2 <= 0),
                 (d3 >= 0) & (d4 <= d3),
                 (vc <= 0) & (d1 >= 0) & (d3 <= 0),
                 (d6 >= 0) & (d5 <= d6),
                 (vb <= 0) & (d2 >= 0) & (d6 <= 0),
                 (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)]
        shape = d1.shape + (3,)
        choices = [np.broadcast_to(a, shape), np.broadcast_to(b, shape), a + ab * t_ab[..., None],
                   np.broadcast_to(c, shape), a + ac * t_ac[..., None], b + (c - b) * t_bc[..., None]]
        q = np.select([x[..., None] for x in conds], choices, default=a + ab * v[..., None] + ac * w[..., None])
        d = np.linalg.norm(p - q, axis=-1)
        bad = ~np.isfinite(d)
        if np.any(bad):
            d = np.where(bad, _edge_distances(p, a, b, c), d)
            verts = np.stack(np.broadcast_arrays(a, b, c), axis=-2)            # (1, T, 3, 3)
            dv = np.linalg.norm(p[..., None, :] - verts, axis=-1)             # (P, T, 3)
            nearest = np.take_along_axis(np.broadcast_to(verts, dv.shape + (3,)),
                                         dv.argmin(-1)[..., None, None], axis=-2)[..., 0, :]
            q = np.where(bad[..., None], nearest, q)
        out[s:s + _CHUNK] = d
        out_q[s:s + _CHUNK] = q
    return out, out_q


def _segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ab = b - a
    ll = (ab * ab).sum(-1)
    with np.errstate(divide='ignore', invalid='ignore'):
        t = np.clip(np.where(ll > 0, ((p - a) * ab).sum(-1) / ll, 0.0), 0.0, 1.0)
    return np.linalg.norm(p - (a + ab * t[..., None]), axis=-1)


def _edge_distances(p: np.ndarray, a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    return np.minimum(np.minimum(_segment_distance(p, a, b), _segment_distance(p, b, c)), _segment_distance(p, c, a))


def contact_check(part: PartMesh, part_T_tcp: np.ndarray,
                  seam_faces: Tuple[Sequence[str], Sequence[str]], params: ContactParams) -> ContactResult:
    """F9 contact check of one ``tcp_laser`` pose in the ``part`` frame (mm).

    ``seam_faces`` = (D04 ``faces.n1``, D04 ``faces.n2``) of the seam. Raises ``ValueError`` for an unknown or
    missing face id or a non-rigid pose. Rules in the module docstring.
    """
    t = check_rigid(part_T_tcp)
    n1, n2 = (tuple(x) for x in seam_faces)
    if not n1 or not n2:
        raise ValueError('seam faces need at least one n1 and one n2 face id')
    overlap = sorted(set(n1) & set(n2))
    if overlap:
        raise ValueError(f'n1 and n2 share face id(s) {overlap}: the faces of the two sides must be distinct')
    unknown = sorted((set(n1) | set(n2)) - set(part.face_ids))
    if unknown:
        raise ValueError(f'unknown seam face id(s): {unknown}')
    n1_idx = [part.face_ids.index(f) for f in set(n1)]
    n2_idx = [part.face_ids.index(f) for f in set(n2)]
    samples = disc_samples(params) @ t[:3, :3].T + t[:3, 3]
    reach = (params.nozzle_radius_mm + max(params.standoff_mm, params.point_tolerance_mm)
             + params.point_tolerance_mm + params.tie_mm)
    local = part.local(t[:3, 3], reach)
    tris, faces = part.triangles_mm[local], part.face_index[local]
    in1, in2 = np.isin(faces, n1_idx), np.isin(faces, n2_idx)
    is_seam = in1 | in2
    seam_tris, other_tris, other_face = tris[is_seam], tris[~is_seam], faces[~is_seam]

    tol = params.point_tolerance_mm
    tris1, tris2 = tris[in1], tris[in2]

    def on_seam_edge(q: np.ndarray) -> np.ndarray:
        """True where a point lies within tol of an n1 face and of an n2 face (the seam edge)."""
        if not (len(tris1) and len(tris2)):
            return np.zeros(len(q), bool)
        return ((point_triangle_distances(q, tris1).min(axis=1) <= tol)
                & (point_triangle_distances(q, tris2).min(axis=1) <= tol))

    # (a) contact point on the seam faces, no other face closer (except at the seam edge)
    d_seam_tcp = float(point_triangle_distances(samples[:1], seam_tris).min()) if len(seam_tris) else math.inf
    nearest_other: Optional[Tuple[str, float]] = None
    off_seam = d_seam_tcp > tol
    if len(other_tris):
        d0, q0 = closest_points_on_triangles(samples[:1], other_tris)
        k = int(np.argmin(d0[0]))
        nearest_other = (part.face_ids[int(other_face[k])], float(d0[0, k]))
        if not off_seam and nearest_other[1] < d_seam_tcp - params.tie_mm:
            off_seam = not bool(on_seam_edge(q0[0, k:k + 1])[0])

    # (b) S.4 measure: non-seam faces within s_c of the disc / rim (unsigned), except at the seam edge
    near: dict = {}
    if len(other_tris):
        d_other, q_other = closest_points_on_triangles(samples, other_tris)
        rows, cols = np.nonzero(d_other <= params.standoff_mm)
        if len(rows):
            edge = on_seam_edge(q_other[rows, cols])
            for r, c in zip(rows[~edge], cols[~edge]):
                fid = part.face_ids[int(other_face[c])]
                near[fid] = min(near.get(fid, math.inf), float(d_other[r, c]))
    if params.rule_b == 'unsigned':
        offending, excused = near, {}
    else:
        # CP-56: only the part of a face above the contact plane (towards the nozzle) by more than the tolerance
        offending = {}
        up = -t[:3, 2]                                                     # nozzle side = −beam axis
        pieces, owner = clip_triangles_above(other_tris, t[:3, 3], up, tol)
        if len(pieces):
            rb = rule_b_samples(params) @ t[:3, :3].T + t[:3, 3]
            d_p, q_p = closest_points_on_triangles(rb, pieces)
            rows, cols = np.nonzero(d_p <= params.standoff_mm)
            if len(rows):
                edge = on_seam_edge(q_p[rows, cols])
                for r, c in zip(rows[~edge], cols[~edge]):
                    fid = part.face_ids[int(other_face[owner[c]])]
                    offending[fid] = min(offending.get(fid, math.inf), float(d_p[r, c]))
        excused = {f: d for f, d in near.items() if f not in offending}
    offending_t = tuple(sorted(offending.items()))
    reason = 'contact_off_seam' if off_seam else ('other_face_within_standoff' if offending_t else None)
    return ContactResult(reason is None, reason, d_seam_tcp, nearest_other, offending_t, len(samples), len(local),
                         tuple(sorted(excused.items())))


__all__ = ['CONTACT_REASONS', 'RULE_B', 'ContactParams', 'PartMesh', 'ContactResult', 'disc_samples',
           'rule_b_samples', 'clip_triangles_above',
           'point_triangle_distances', 'closest_points_on_triangles', 'contact_check']
