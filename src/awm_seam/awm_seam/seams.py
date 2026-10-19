"""Seam building, validation, export and re-matching."""

from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from awm_core import jsonio
from awm_core.repository import JobPaths

CREATED_BY = 'awm_seam 0.1.0'
CHAIN_TOL_M = 1.0e-5
PICK_MAX_DIST_M = 2.0e-3
SEAM_ID = re.compile(r'^S[0-9]{2,3}$')
JOINT_TYPES = ('fillet', 'butt', 'lap', 'groove')
# Joint types whose root edge must be concave (inside corner between the two parts).
CONCAVE_JOINTS = ('fillet',)


class SeamDefinitionError(ValueError):
    """A seam request is invalid (unknown edge, broken chain, wrong attribute...)."""


@dataclass
class SeamRequest:
    """What the user enters for one seam (Workbench / CLI / pick file)."""

    seam_id: str
    joint_type: str
    leg_mm: float
    sequence: int
    material: str
    thickness_mm: float
    edges: List[str] = field(default_factory=list)
    picks_m: List[List[float]] = field(default_factory=list)
    weld_side: str = 'bisector'
    process_hint: Optional[str] = None
    reversed: bool = False
    start_pick_m: Optional[List[float]] = None
    end_pick_m: Optional[List[float]] = None
    note: str = ''

    @classmethod
    def from_dict(cls, data: Dict) -> 'SeamRequest':
        picks = data.get('picks_m') or ([data['pick_m']] if 'pick_m' in data else [])
        return cls(seam_id=data['seam_id'], joint_type=data['joint_type'],
                   leg_mm=float(data['leg_mm']), sequence=int(data['sequence']),
                   material=data['material'], thickness_mm=float(data['thickness_mm']),
                   edges=list(data.get('edges', [])), picks_m=[list(p) for p in picks],
                   weld_side=data.get('weld_side', 'bisector'),
                   process_hint=data.get('process_hint'), reversed=bool(data.get('reversed', False)),
                   start_pick_m=data.get('start_pick_m'), end_pick_m=data.get('end_pick_m'),
                   note=data.get('note', ''))


class EdgeCatalogue:
    """Read-only view of ``cache/edges.json`` with picking helpers."""

    def __init__(self, doc: Dict) -> None:
        self.doc = doc
        self.edges = {e['id']: e for e in doc['edges']}
        self.faces = {f['id']: f for f in doc['faces']}

    @classmethod
    def load(cls, job: JobPaths) -> 'EdgeCatalogue':
        return cls(jsonio.read_json(job.edges_json, 'edges', base_dir=job.dir))

    def edge(self, edge_id: str) -> Dict:
        if edge_id not in self.edges:
            raise SeamDefinitionError(f'unknown edge {edge_id}')
        return self.edges[edge_id]

    def distance(self, edge_id: str, point: Sequence[float]) -> float:
        """Distance from a point to the edge's polyline."""
        poly = np.asarray(self.edges[edge_id]['polyline_m'])
        p = np.asarray(point, dtype=float)
        a, b = poly[:-1], poly[1:]
        ab = b - a
        denom = np.einsum('ij,ij->i', ab, ab)
        t = np.clip(np.einsum('ij,ij->i', p - a, ab) / np.where(denom > 0, denom, 1.0), 0.0, 1.0)
        closest = a + ab * t[:, None]
        return float(np.min(np.linalg.norm(closest - p, axis=1)))

    def pick(self, point: Sequence[float], max_dist_m: float = PICK_MAX_DIST_M) -> str:
        """A21 picking: the edge nearest to a 3D point (UI click). Ambiguous picks are refused."""
        ranked = sorted((self.distance(eid, point), eid) for eid in self.edges)
        best_d, best = ranked[0]
        if best_d > max_dist_m:
            raise SeamDefinitionError(f'no edge within {max_dist_m * 1000:.1f} mm of pick '
                                      f'{list(point)} (nearest {best} at {best_d * 1000:.2f} mm)')
        if len(ranked) > 1 and ranked[1][0] - best_d < 1e-6:
            raise SeamDefinitionError(f'pick {list(point)} is ambiguous: {best} and '
                                      f'{ranked[1][1]} are equally close; pick nearer the seam')
        return best


def order_chain(cat: EdgeCatalogue, edge_ids: Sequence[str]) -> Tuple[List[str], np.ndarray,
                                                                       np.ndarray, bool]:
    """Order edges into one connected chain. Return (ids, start, end, closed).

    Direction: from the free end of the first listed edge that has one (the first pick), so the
    order in which the user picks edges defines the weld direction; ``reversed`` flips it.
    """
    ids = list(dict.fromkeys(edge_ids))
    if len(ids) != len(edge_ids):
        raise SeamDefinitionError('an edge is listed twice')
    for eid in ids:
        cat.edge(eid)
    if len(ids) == 1:
        e = cat.edges[ids[0]]
        return ids, np.array(e['start_m']), np.array(e['end_m']), bool(e['closed'])
    ends = {eid: (np.array(cat.edges[eid]['start_m']), np.array(cat.edges[eid]['end_m']))
            for eid in ids}

    def touching(p, eid):
        a, b = ends[eid]
        return min(np.linalg.norm(a - p), np.linalg.norm(b - p)) < CHAIN_TOL_M

    # Start from an edge that has a free end (open chain) or the first edge (closed loop).
    start_id, start_pt = ids[0], ends[ids[0]][0]
    for eid in ids:
        for p in ends[eid]:
            if not any(touching(p, o) for o in ids if o != eid):
                start_id, start_pt = eid, p
                break
        else:
            continue
        break
    chain, remaining = [start_id], [i for i in ids if i != start_id]
    a, b = ends[start_id]
    tail = b if np.linalg.norm(a - start_pt) < CHAIN_TOL_M else a
    while remaining:
        nxt = next((o for o in remaining if touching(tail, o)), None)
        if nxt is None:
            raise SeamDefinitionError(f'edges {sorted(remaining)} are not connected to the chain '
                                      f'{chain}')
        a, b = ends[nxt]
        tail = b if np.linalg.norm(a - tail) < CHAIN_TOL_M else a
        chain.append(nxt)
        remaining.remove(nxt)
    closed = bool(np.linalg.norm(tail - start_pt) < CHAIN_TOL_M)
    return chain, start_pt, tail, closed


def chain_polyline(cat: EdgeCatalogue, chain: Sequence[str], start: np.ndarray,
                   reversed_single: bool) -> np.ndarray:
    """Polyline of the chain in weld direction (same convention as awm_geometry sampling).

    The first edge is oriented to meet the second one (or to start at ``start`` for an open
    single edge); a single closed edge runs in curve direction, against it if ``reversed_single``.
    """
    polys = [np.asarray(cat.edges[e]['polyline_m'], float) for e in chain]
    first = polys[0]
    if len(polys) > 1:
        nxt = polys[1]
        d_end = min(np.linalg.norm(first[-1] - nxt[0]), np.linalg.norm(first[-1] - nxt[-1]))
        d_start = min(np.linalg.norm(first[0] - nxt[0]), np.linalg.norm(first[0] - nxt[-1]))
        if d_start < d_end:
            first = first[::-1]
    elif cat.edges[chain[0]]['closed']:
        if reversed_single:
            first = first[::-1]
    elif np.linalg.norm(first[-1] - start) < np.linalg.norm(first[0] - start):
        first = first[::-1]
    out = [first]
    for poly in polys[1:]:
        tail = out[-1][-1]
        out.append(poly if np.linalg.norm(poly[0] - tail) <= np.linalg.norm(poly[-1] - tail)
                   else poly[::-1])
    return np.vstack([out[0]] + [p[1:] for p in out[1:]])


def project_on_polyline(poly: np.ndarray, point: Sequence[float]) -> Tuple[float, float]:
    """(arc length of the closest point, distance) of a point to a polyline."""
    p = np.asarray(point, float)
    a, b = poly[:-1], poly[1:]
    ab = b - a
    seg_len = np.linalg.norm(ab, axis=1)
    denom = np.where(seg_len > 0, seg_len ** 2, 1.0)
    t = np.clip(np.einsum('ij,ij->i', p - a, ab) / denom, 0.0, 1.0)
    closest = a + ab * t[:, None]
    dist = np.linalg.norm(closest - p, axis=1)
    k = int(np.argmin(dist))
    s0 = float(np.sum(seg_len[:k]))
    return s0 + float(t[k] * seg_len[k]), float(dist[k])


def point_at(poly: np.ndarray, s: float) -> np.ndarray:
    seg_len = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    s = float(np.clip(s, 0.0, cum[-1]))
    k = int(np.clip(np.searchsorted(cum, s) - 1, 0, len(seg_len) - 1))
    w = 0.0 if seg_len[k] == 0 else (s - cum[k]) / seg_len[k]
    return poly[k] * (1 - w) + poly[k + 1] * w


def seam_fingerprint(cat: EdgeCatalogue, edge_ids: Sequence[str],
                     range_m: Optional[Sequence[float]] = None) -> str:
    joined = '|'.join(sorted(cat.edges[e]['fingerprint'] for e in edge_ids))
    if range_m is not None:
        joined += f'|{round(range_m[0], 4)}:{round(range_m[1], 4)}'
    return hashlib.sha256(joined.encode()).hexdigest()


def build_seam(cat: EdgeCatalogue, req: SeamRequest) -> Dict:
    """A21 + A22: validate a request and return the seams.json entry."""
    if not SEAM_ID.match(req.seam_id):
        raise SeamDefinitionError(f'seam id {req.seam_id!r} must look like S01')
    if req.joint_type not in JOINT_TYPES:
        raise SeamDefinitionError(f'{req.seam_id}: joint type {req.joint_type!r} not in '
                                  f'{JOINT_TYPES}')
    if req.leg_mm <= 0 or req.thickness_mm <= 0 or req.sequence < 1:
        raise SeamDefinitionError(f'{req.seam_id}: leg_mm, thickness_mm must be > 0 and '
                                  f'sequence >= 1')
    if req.process_hint not in (None, 'MIG', 'LASER'):
        raise SeamDefinitionError(f'{req.seam_id}: process_hint must be MIG, LASER or empty')
    edge_ids = list(req.edges) + [cat.pick(p) for p in req.picks_m]
    if not edge_ids:
        raise SeamDefinitionError(f'{req.seam_id}: no edges and no picks given')
    chain, start, end, closed = order_chain(cat, edge_ids)
    for eid in chain:
        e = cat.edges[eid]
        if req.joint_type in CONCAVE_JOINTS and e['convexity'] != 'concave':
            raise SeamDefinitionError(
                f'{req.seam_id}: edge {eid} is {e["convexity"]}, a {req.joint_type} root must be '
                f'a concave edge between two faces')
    faces = {f for eid in chain for f in cat.edges[eid]['faces']}
    if req.weld_side != 'bisector' and req.weld_side not in faces:
        raise SeamDefinitionError(f'{req.seam_id}: weld_side {req.weld_side!r} must be '
                                  f'"bisector" or one of the adjacent faces {sorted(faces)}')
    if req.reversed:
        chain = list(reversed(chain))
        start, end = end, start
    length = float(sum(cat.edges[e]['length_m'] for e in chain))
    range_m = None
    if req.start_pick_m is not None or req.end_pick_m is not None:
        range_m, start, end = _seam_range(cat, req, chain, start, closed)
        length = range_m[1] - range_m[0]
    seam = {
        'seam_id': req.seam_id, 'edges': chain, 'reversed': req.reversed,
        'joint_type': req.joint_type, 'weld_side': req.weld_side, 'leg_mm': req.leg_mm,
        'sequence': req.sequence, 'process_hint': req.process_hint, 'material': req.material,
        'thickness_mm': req.thickness_mm,
        'start_m': [float(v) for v in start], 'end_m': [float(v) for v in end],
        'length_m': length, 'closed': closed and range_m is None,
        'fingerprint': seam_fingerprint(cat, chain, range_m),
    }
    if range_m is not None:
        seam['range_m'] = range_m
        seam['chain_length_m'] = float(sum(cat.edges[e]['length_m'] for e in chain))
    if req.note:
        seam['note'] = req.note
    return seam


MIN_SEAM_M = 0.005


def _seam_range(cat: EdgeCatalogue, req: SeamRequest, chain: List[str], start: np.ndarray,
                closed: bool) -> Tuple[List[float], np.ndarray, np.ndarray]:
    """A22 start / end: arc-length range of a partial seam from start / end picks."""
    poly = chain_polyline(cat, chain, start, req.reversed)
    total = float(np.sum(np.linalg.norm(np.diff(poly, axis=0), axis=1)))
    s0, s1 = 0.0, total
    for name, pick in (('start', req.start_pick_m), ('end', req.end_pick_m)):
        if pick is None:
            continue
        s, dist = project_on_polyline(poly, pick)
        if dist > PICK_MAX_DIST_M:
            raise SeamDefinitionError(f'{req.seam_id}: {name} pick is {dist * 1000:.1f} mm from '
                                      f'the seam edges')
        if name == 'start':
            s0 = s
        else:
            s1 = s
    if closed:
        if s1 <= s0 + 1e-9:
            s1 += total
    elif s1 <= s0:
        raise SeamDefinitionError(f'{req.seam_id}: end pick lies before the start pick in weld '
                                  f'direction (use reversed)')
    # Picks are located on the display polyline; express the range in true arc length.
    scale = float(sum(cat.edges[e]['length_m'] for e in chain)) / total
    s0, s1 = s0 * scale, s1 * scale
    poly_s0, poly_s1 = s0 / scale, s1 / scale
    if s1 - s0 < MIN_SEAM_M:
        raise SeamDefinitionError(f'{req.seam_id}: partial seam shorter than '
                                  f'{MIN_SEAM_M * 1000:.0f} mm')
    return [s0, s1], point_at(poly, poly_s0 % total if closed else poly_s0), \
        point_at(poly, poly_s1 % total if closed and poly_s1 > total else poly_s1)


def _interval(seam: Dict, flip: bool, total: float) -> Optional[Tuple[float, float]]:
    """Arc-length interval of a partial seam, expressed in the other seam's direction if ``flip``.

    Both directions of a chain share the same origin vertex, so s maps to total - s.
    """
    if 'range_m' not in seam:
        return None
    s0, s1 = seam['range_m']
    if flip:
        s0, s1 = total - s1, total - s0
    if s0 < 0:
        s0, s1 = s0 + total, s1 + total
    return s0, s1


def _overlap(a: Dict, b: Dict) -> bool:
    """True if two seams weld a common piece of edge."""
    if sorted(a['edges']) != sorted(b['edges']):
        return True
    total = a.get('chain_length_m') or b.get('chain_length_m')
    flip = (a['reversed'] != b['reversed']) if len(a['edges']) == 1 else a['edges'] != b['edges']
    ia, ib = _interval(a, False, total), _interval(b, flip, total)
    if ia is None or ib is None:
        return True

    def split(iv):
        s0, s1 = iv
        return [(s0, s1)] if s1 <= total else [(s0, total), (0.0, s1 - total)]

    return any(x0 < y1 - 1e-5 and y0 < x1 - 1e-5 for x0, x1 in split(ia) for y0, y1 in split(ib))


def check_seam_list(seams: Sequence[Dict]) -> None:
    """Unique ids, unique sequence numbers, no piece of edge welded by two seams."""
    ids = [s['seam_id'] for s in seams]
    dup = {i for i in ids if ids.count(i) > 1}
    if dup:
        raise SeamDefinitionError(f'duplicate seam id(s): {sorted(dup)}')
    seqs = [s['sequence'] for s in seams]
    dup_seq = {q for q in seqs if seqs.count(q) > 1}
    if dup_seq:
        raise SeamDefinitionError(f'duplicate sequence number(s): {sorted(dup_seq)}')
    for i, a in enumerate(seams):
        for b in seams[i + 1:]:
            shared = set(a['edges']) & set(b['edges'])
            if shared and _overlap(a, b):
                raise SeamDefinitionError(f'edge(s) {sorted(shared)} used by {a["seam_id"]} and '
                                          f'{b["seam_id"]} with overlapping ranges')


def load_seams(job: JobPaths, check: bool = True) -> Dict:
    return jsonio.read_json(job.seams_json, 'seams', base_dir=job.dir if check else None)


def export_seams(job: JobPaths, seams: Sequence[Dict]) -> Dict:
    """A24: write seams.json (sorted by sequence) with the part record and edge catalogue as inputs."""
    check_seam_list(seams)
    part = jsonio.read_json(job.part_json, 'part', base_dir=job.dir)
    doc = jsonio.make_header(CREATED_BY, [job.part_json, job.edges_json], job.dir)
    doc.update({'part_no': part['part_no'], 'part_rev': part['part_rev'],
                'seams': sorted(seams, key=lambda s: s['sequence'])})
    jsonio.write_json(job.seams_json, doc, 'seams')
    return doc


def define_seams(job: JobPaths, requests: Sequence[SeamRequest], replace: bool = False) -> Dict:
    """Add (or with ``replace`` overwrite) seams in the job's seam list and export it."""
    cat = EdgeCatalogue.load(job)
    existing: List[Dict] = []
    if job.seams_json.exists() and not replace:
        existing = load_seams(job, check=False)['seams']
    new = [build_seam(cat, r) for r in requests]
    new_ids = {s['seam_id'] for s in new}
    merged = [s for s in existing if s['seam_id'] not in new_ids] + new
    return export_seams(job, merged)


def remove_seam(job: JobPaths, seam_id: str) -> Dict:
    seams = load_seams(job, check=False)['seams']
    left = [s for s in seams if s['seam_id'] != seam_id]
    if len(left) == len(seams):
        raise SeamDefinitionError(f'no seam {seam_id}')
    return export_seams(job, left)


def rematch(old_seams: Sequence[Dict], old_cat: EdgeCatalogue,
            new_cat: EdgeCatalogue) -> Tuple[List[Dict], List[str]]:
    """Map seams of a previous STEP revision onto a new edge catalogue by edge fingerprints.

    Returns (re-matched seams, ids that could not be matched). Unmatched seams must be picked again;
    nothing is guessed.
    """
    by_fp: Dict[str, List[str]] = {}
    for e in new_cat.edges.values():
        by_fp.setdefault(e['fingerprint'], []).append(e['id'])
    matched, missing = [], []
    for seam in old_seams:
        new_ids = []
        for eid in seam['edges']:
            old = old_cat.edges.get(eid)
            hits = by_fp.get(old['fingerprint'], []) if old else []
            if len(hits) != 1:
                break
            new_ids.append(hits[0])
        if len(new_ids) != len(seam['edges']):
            missing.append(seam['seam_id'])
            continue
        updated = dict(seam)
        updated['edges'] = new_ids
        updated['fingerprint'] = seam_fingerprint(new_cat, new_ids, seam.get('range_m'))
        matched.append(updated)
    return matched, missing
