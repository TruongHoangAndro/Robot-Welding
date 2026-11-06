"""Joint seam sampling on the exact B-Rep for M2 (G1.2, PLAN T2.2; CP-52, CP-53; DES-001 §4.3 step 1).

``awm_seam`` resolves picked edge ids to a joint (``JE<n>`` of the D20 ``topology.json``) and asks this module, in
the conda process, for the seam samples: points at equal arc-length spacing <= the requested pitch, the unit
tangent in the weld direction, and the joint-aware outward normals n1 (exterior face of ``solid_a``) and n2
(exterior face of ``solid_b``) of :mod:`awm_geometry.joints`.

Shape: ``cache/model.brep`` of the job (metres, written by ``cli import`` from the same load + heal as D20, D20
rule 5); its SHA-256 must match ``part.json`` and the joint's edges must have the D20 lengths, otherwise the ids
could name other edges and the request is refused. Output lengths are mm through ``awm_core.units``.

Weld direction: curve direction of the joint edge of ``solid_a``; ``reversed`` flips it. Start of a closed joint:
``start_offset_mm`` (arc length from the first vertex in curve direction) or ``start_point_mm`` (projected on the
edge, at most ``START_POINT_MAX_MM`` away). Closed seams end on their start point (last sample at s = length).
"""

from __future__ import annotations

import json
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GCPnts import GCPnts_AbscissaPoint
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SOLID
from OCC.Core.TopoDS import topods
from OCC.Core.gp import gp_Pnt, gp_Vec

from awm_core import jsonio, units
from awm_core.hashing import sha256_file
from awm_core.repository import JobPaths
from awm_geometry import joints, occ_io, topology

#: Agreement between the B-Rep edge length and the D20 joint length (mm).
LENGTH_TOL_MM = 1e-4
#: A start point given as a 3D point must lie this close to the joint edge (mm, as the v1 pick tolerance).
START_POINT_MAX_MM = 2.0
JOINT_TOL_MM = 0.05      # same coincidence tolerance as the D20 export (view.JOINT_TOL_MM)
JOINT_PROBE_MM = 0.5     # same probe distance as the D20 export (view.JOINT_PROBE_MM)
_PROJECT_STEP_MM = 0.05  # coarse step of the start point projection before refinement


class JointSampleError(RuntimeError):
    """A joint cannot be sampled (stale cache, unknown id, start point off the edge, ...)."""


def _vec(v) -> np.ndarray:
    return np.array([v.X(), v.Y(), v.Z()], dtype=float)


def topology_path(job: JobPaths):
    return job.dir / 'cache' / 'view' / 'topology.json'


def load(job: JobPaths) -> Tuple[object, Dict, Dict]:
    """(shape in metres, part.json, topology.json) of a job; refuse a stale B-Rep cache or D20."""
    part = json.loads(job.part_json.read_text(encoding='utf-8'))
    brep = part.get('brep_cache') or {}
    if not job.brep_cache.is_file() or sha256_file(job.brep_cache) != brep.get('sha256'):
        raise JointSampleError('cache/model.brep is missing or does not match part.json; re-run import')
    topo = jsonio.read_json(topology_path(job), 'view_topology', base_dir=job.dir)
    step_sha = part.get('step_sha256')
    if step_sha and step_sha != sha256_file(job.model_step):
        raise JointSampleError('model.step does not match part.json; re-run import')
    return occ_io.read_brep(job.brep_cache), part, topo


class JobModel:
    """Id maps of the cached shape (same scheme as D20: 1-based TopTools_IndexedMapOfShape indices)."""

    def __init__(self, shape) -> None:
        self.shape = shape
        self.solids = topology.index_map(shape, TopAbs_SOLID)
        self.faces = topology.index_map(shape, TopAbs_FACE)
        self.edges = topology.index_map(shape, TopAbs_EDGE)
        self.solid_list = [topods.Solid(self.solids.FindKey(i)) for i in range(1, self.solids.Size() + 1)]

    def _index(self, imap, ident: str, prefix: str) -> int:
        if not ident.startswith(prefix) or not ident[len(prefix):].isdigit():
            raise JointSampleError(f'bad id {ident!r}')
        k = int(ident[len(prefix):])
        if not 1 <= k <= imap.Size():
            raise JointSampleError(f'{ident} not in the cached shape (1..{imap.Size()})')
        return k

    def joint(self, rec: Dict) -> joints.JointEdge:
        """JointEdge of a D20 joint record; checks that each edge lies on its solid and has the D20 length."""
        sa = self._index(self.solids, rec['solid_a'], 'B') - 1
        sb = self._index(self.solids, rec['solid_b'], 'B') - 1
        out = []
        for solid_i, eid in ((sa, rec['edge_a']), (sb, rec['edge_b'])):
            edge = topods.Edge(self.edges.FindKey(self._index(self.edges, eid, 'E')))
            faces = None
            for e, fs in joints._edge_faces(self.solid_list[solid_i]):
                if e.IsSame(edge):
                    faces = fs
                    break
            if faces is None:
                raise JointSampleError(f'{eid} is not an edge of B{solid_i + 1}: D20 and the B-Rep cache differ')
            length_mm = units.m_to_mm(GCPnts_AbscissaPoint.Length(BRepAdaptor_Curve(edge)))
            if abs(length_mm - float(rec['length_mm'])) > LENGTH_TOL_MM:
                raise JointSampleError(f'{eid}: length {length_mm:.5f} mm differs from D20 '
                                       f'{float(rec["length_mm"]):.5f} mm: D20 and the B-Rep cache differ')
            out.append((edge, faces, length_mm))
        (ea, fa, la), (eb, fb, _) = out
        return joints.JointEdge(sa, sb, ea, eb, fa, fb, units.mm_to_m(la))


def _param_at(curve: BRepAdaptor_Curve, s_m: float, total_m: float) -> float:
    u0, u1 = curve.FirstParameter(), curve.LastParameter()
    if s_m <= 0.0:
        return u0
    if s_m >= total_m:
        return u1
    return GCPnts_AbscissaPoint(curve, s_m, u0).Parameter()


def project_start(curve: BRepAdaptor_Curve, point_mm: Sequence[float]) -> Tuple[float, float]:
    """(arc length from the first vertex in mm, distance in mm) of the edge point closest to ``point_mm``."""
    target = units.mm_to_m(np.asarray(point_mm, float))
    u0, u1 = curve.FirstParameter(), curve.LastParameter()
    total = GCPnts_AbscissaPoint.Length(curve)
    n = max(16, int(math.ceil(units.m_to_mm(total) / _PROJECT_STEP_MM)) + 1)
    us = np.linspace(u0, u1, n)

    def dist(u: float) -> float:
        return float(np.linalg.norm(_vec(curve.Value(float(u))) - target))

    k = int(np.argmin([dist(u) for u in us]))
    lo, hi = us[max(k - 1, 0)], us[min(k + 1, n - 1)]
    g = (math.sqrt(5.0) - 1.0) / 2.0
    for _ in range(80):  # golden-section refinement
        a, b = hi - g * (hi - lo), lo + g * (hi - lo)
        if dist(a) < dist(b):
            hi = b
        else:
            lo = a
    u = 0.5 * (lo + hi)
    s = GCPnts_AbscissaPoint.Length(curve, u0, u) if u > u0 else 0.0
    return units.m_to_mm(s), units.m_to_mm(dist(u))


def sample_joint(model: JobModel, rec: Dict, pitch_mm: float, reversed_: bool = False,
                 start_offset_mm: float = 0.0, start_point_mm: Optional[Sequence[float]] = None) -> Dict:
    """Samples of one joint seam in mm (see the module docstring for direction and start)."""
    if not pitch_mm > 0.0:
        raise JointSampleError('pitch must be > 0 mm')
    joint = model.joint(rec)
    curve = BRepAdaptor_Curve(joint.edge_a)
    closed = topology.edge_closed(joint.edge_a)
    total_m = GCPnts_AbscissaPoint.Length(curve)
    length_mm = units.m_to_mm(total_m)
    start = float(start_offset_mm or 0.0)
    if start_point_mm is not None:
        start, dist = project_start(curve, start_point_mm)
        if dist > START_POINT_MAX_MM:
            raise JointSampleError(f'start point is {dist:.2f} mm from the joint edge (max {START_POINT_MAX_MM})')
        if start >= length_mm - 1e-9:
            start = 0.0
    if not closed and start > 0.0:
        raise JointSampleError('an open joint starts at its end vertex; use reversed')
    if not 0.0 <= start < length_mm:
        raise JointSampleError(f'start offset {start:.3f} mm outside [0, {length_mm:.3f}) mm')
    n = int(math.ceil(length_mm / pitch_mm - 1e-9)) + 1
    spacing = length_mm / (n - 1)
    sign = -1.0 if reversed_ else 1.0
    probe, tol = units.mm_to_m(JOINT_PROBE_MM), units.mm_to_m(JOINT_TOL_MM)
    points: List[Dict] = []
    faces_a, faces_b = [], []
    for k in range(n):
        s = k * spacing
        if closed:
            e = (start + sign * s) % length_mm
            if k == n - 1:
                e = start  # closed seam ends on its start point
        else:
            e = s if not reversed_ else length_mm - s
        u = _param_at(curve, units.mm_to_m(e), total_m)
        pnt, vec = gp_Pnt(), gp_Vec()
        curve.D1(u, pnt, vec)
        p = _vec(pnt)
        t = sign * _vec(vec)
        t /= np.linalg.norm(t)
        fa, fb = joints.exterior_faces(joint, model.solid_list, p, probe, tol)
        n1, n2 = topology.face_normal_at(fa, p), topology.face_normal_at(fb, p)
        if joints.bisector(n1, n2) is None:
            raise JointSampleError(f'opposite face normals at s = {s:.3f} mm: seam frame Z undefined')
        for face, acc in ((fa, faces_a), (fb, faces_b)):
            fid = f'F{model.faces.FindIndex(face)}'
            if fid not in acc:
                acc.append(fid)
        points.append({'s_mm': float(s), 'p_mm': [float(v) + 0.0 for v in units.m_to_mm(p)],
                       't': [float(v) + 0.0 for v in t], 'n1': [float(v) + 0.0 for v in n1],
                       'n2': [float(v) + 0.0 for v in n2]})
    return {'joint_edge_id': rec['joint_edge_id'], 'solid_a': rec['solid_a'], 'solid_b': rec['solid_b'],
            'edge_a': rec['edge_a'], 'edge_b': rec['edge_b'], 'closed': bool(closed), 'length_mm': length_mm,
            'start_offset_mm': start, 'spacing_mm': spacing, 'faces_n1': faces_a, 'faces_n2': faces_b,
            'points': points}


def sample_joints(job: JobPaths, request: Dict, created_by: str) -> Dict:
    """Run a sampler request of ``awm_seam`` (``{pitch_mm, seams: [{seam_id, joint_edge_id, reversed,
    start_offset_mm, start_point_mm}]}``) and return the output document (mm, part frame)."""
    shape, _part, topo = load(job)
    model = JobModel(shape)
    by_id = {j['joint_edge_id']: j for j in topo['joints']}
    pitch = float(request['pitch_mm'])
    out = []
    for req in request['seams']:
        rec = by_id.get(req['joint_edge_id'])
        if rec is None:
            raise JointSampleError(f'{req["seam_id"]}: joint {req["joint_edge_id"]} not in topology.json')
        sampled = sample_joint(model, rec, pitch, bool(req.get('reversed')),
                               float(req.get('start_offset_mm') or 0.0), req.get('start_point_mm'))
        out.append(dict(seam_id=req['seam_id'], **sampled))
    doc = jsonio.make_header(created_by, [job.part_json, topology_path(job), job.brep_cache], job.dir)
    doc.update({'schema_version': '2.0', 'unit': 'mm', 'frame': 'part', 'pitch_mm': pitch, 'seams': out})
    return doc
