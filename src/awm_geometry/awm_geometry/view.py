"""D20 3D view model export (A13): ``view.glb`` + ``topology.json`` (DES-001 Rev 0.5 §4.5, CP-52).

Contract: ``docs/generated/contracts/D20_view_model.md`` (v2.0). Part frame, **mm** (the OCCT session unit
of the STEP reader; no scaling, no rotation). Display only: the browser picks ids, M2 resolves them on the
exact B-Rep.

Ids: ``F<n>`` / ``E<n>`` / ``B<n>`` are 1-based ``topexp.MapShapes`` indices on the healed part compound
(the scheme of :mod:`awm_geometry.topology`); degenerated edges get no entry. Joint edges ``JE<n>`` follow
the order of :func:`awm_geometry.joints.find_joint_edges`.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCC.Core.BRepGProp import BRepGProp_Face, brepgprop
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.GCPnts import GCPnts_AbscissaPoint, GCPnts_QuasiUniformDeflection
from OCC.Core.GProp import GProp_GProps
from OCC.Core.TopAbs import (TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID,
                             TopAbs_VERTEX)
from OCC.Core.TopExp import topexp
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopTools import (TopTools_IndexedDataMapOfShapeListOfShape,
                               TopTools_IndexedMapOfShape)
from OCC.Core.TopoDS import TopoDS_Edge, TopoDS_Shape, topods
from OCC.Core.gp import gp_Pnt, gp_Vec

from awm_core import units
from awm_geometry import joints, topology
from awm_geometry.assembly import Component, compound
from awm_geometry.glb import GlbBuilder
from awm_geometry.occ_io import GeometryError

LOG = logging.getLogger('awm_geometry')

SCHEMA_VERSION = '2.0'
FACE_LINEAR_DEFLECTION_MM = 0.05
FACE_ANGULAR_DEFLECTION_DEG = 5.0
EDGE_CHORD_MM = 0.05            # contract maximum (D20: chord <= 0.05 mm)
JOINT_TOL_MM = 0.05             # coincidence tolerance of joint edges (as test_joints_occ)
JOINT_PROBE_MM = 0.5            # probe distance for the exterior-face test of a joint
G1_TOL_DEG = 1.0                # tangent continuity between edges at a shared vertex
CLOSED_TOL_MM = 1.0e-4          # start == end within this distance -> closed
PREVIEW_FRACTIONS = (0.0, 0.25, 0.5, 0.75)
UP_AXES = ('+X', '-X', '+Y', '-Y', '+Z', '-Z')


@dataclass
class PartModel:
    """Healed part shape with its id maps (shared by the exporter and the tests)."""

    shape: TopoDS_Shape
    solids: TopTools_IndexedMapOfShape
    faces: TopTools_IndexedMapOfShape
    edges: TopTools_IndexedMapOfShape
    names: List[str]          # per solid index (0-based), '' if unknown
    healed: bool


def build_model(component: Component) -> PartModel:
    """Heal the component's shape (A12) and build the F / E / B id maps."""
    shape = compound([component.shape])
    shape, healed, checks = topology.check_and_heal(shape, 'mm')
    failed = [c for c in checks if not c.ok and c.name in ('brep_valid', 'has_solid')]
    if failed:
        raise GeometryError('model check failed: ' + '; '.join(
            f'{c.name}: {c.detail}' for c in failed))
    solids = topology.index_map(shape, TopAbs_SOLID)
    names = []
    for i in range(1, solids.Size() + 1):
        solid = solids.FindKey(i)
        same = [n for s, n in component.solids if s.IsSame(solid)]
        partner = [n for s, n in component.solids if s.IsPartner(solid)]
        names.append((same or partner or [''])[0])
    return PartModel(shape, solids, topology.index_map(shape, TopAbs_FACE),
                     topology.index_map(shape, TopAbs_EDGE), names, healed)


def _vec(v) -> np.ndarray:
    return np.array([v.X(), v.Y(), v.Z()], dtype=float)


def _owner(model: PartModel, kind) -> Dict[int, int]:
    """Map face / edge index -> solid index (1-based)."""
    imap = model.faces if kind == TopAbs_FACE else model.edges
    out: Dict[int, int] = {}
    for b in range(1, model.solids.Size() + 1):
        sub = topology.index_map(model.solids.FindKey(b), kind)
        for k in range(1, sub.Size() + 1):
            out.setdefault(imap.FindIndex(sub.FindKey(k)), b)
    return out


def face_mesh(face) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Triangulation of a meshed face: positions (mm), unit normals (outward), uint32 indices."""
    loc = TopLoc_Location()
    tri = BRep_Tool.Triangulation(face, loc)
    if tri is None or tri.NbTriangles() == 0:
        raise GeometryError('face has no triangulation')
    trsf = loc.Transformation()
    n = tri.NbNodes()
    pos = np.array([_vec(tri.Node(i).Transformed(trsf)) for i in range(1, n + 1)])
    idx = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)],
                   dtype=np.int64) - 1
    if face.Orientation() == TopAbs_REVERSED:
        idx = idx[:, [0, 2, 1]]
    # Vertex normals from the exact surface (BRepGProp_Face follows the face orientation).
    tri_n = np.cross(pos[idx[:, 1]] - pos[idx[:, 0]], pos[idx[:, 2]] - pos[idx[:, 0]])
    fallback = np.zeros_like(pos)
    for k in range(3):
        np.add.at(fallback, idx[:, k], tri_n)
    normals = np.zeros_like(pos)
    prop = BRepGProp_Face(face)
    has_uv = tri.HasUVNodes()
    for i in range(n):
        normal = None
        if has_uv:
            uv = tri.UVNode(i + 1)
            p, v = gp_Pnt(), gp_Vec()
            prop.Normal(uv.X(), uv.Y(), p, v)
            if v.Magnitude() > 1e-12:
                normal = _vec(v)
        if normal is None:
            normal = fallback[i]
        norm = np.linalg.norm(normal)
        normals[i] = normal / norm if norm > 0 else (0.0, 0.0, 1.0)
    return pos, normals, idx.astype(np.uint32)


def edge_polyline(edge: TopoDS_Edge, chord: float) -> np.ndarray:
    """Points along the edge (curve-parameter direction), chordal deviation <= ``chord`` (mm)."""
    curve = BRepAdaptor_Curve(edge)
    sampler = GCPnts_QuasiUniformDeflection(curve, chord)
    if not sampler.IsDone() or sampler.NbPoints() < 2:
        raise GeometryError('edge discretisation failed')
    return np.array([_vec(sampler.Value(i)) for i in range(1, sampler.NbPoints() + 1)])


def _tangent(curve: BRepAdaptor_Curve, u: float) -> np.ndarray:
    p, v = gp_Pnt(), gp_Vec()
    curve.D1(u, p, v)
    t = _vec(v)
    return t / np.linalg.norm(t)


def _tangent_neighbours(model: PartModel) -> Dict[int, List[int]]:
    """Edges that share a vertex with G1 continuity (tangents parallel within G1_TOL_DEG)."""
    v2e = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(model.shape, TopAbs_VERTEX, TopAbs_EDGE, v2e)
    cos_tol = float(np.cos(units.deg_to_rad(G1_TOL_DEG)))
    out: Dict[int, set] = {}
    for vi in range(1, v2e.Size() + 1):
        vertex = topods.Vertex(v2e.FindKey(vi))
        ends = []  # (edge index, tangent at the vertex)
        seen = set()
        for e in v2e.FindFromIndex(vi):
            edge = topods.Edge(e)
            idx = model.edges.FindIndex(edge)
            if idx in seen or BRep_Tool.Degenerated(edge):
                continue
            seen.add(idx)
            curve = BRepAdaptor_Curve(edge)
            # The vertex is an end of the edge: take the end parameter nearest to it.
            here = _vec(BRep_Tool.Pnt(vertex))
            u = min((curve.FirstParameter(), curve.LastParameter()),
                    key=lambda w: float(np.linalg.norm(_vec(curve.Value(w)) - here)))
            ends.append((idx, _tangent(curve, u)))
        for a in range(len(ends)):
            for b in range(a + 1, len(ends)):
                if abs(float(ends[a][1] @ ends[b][1])) >= cos_tol:
                    out.setdefault(ends[a][0], set()).add(ends[b][0])
                    out.setdefault(ends[b][0], set()).add(ends[a][0])
    return {k: sorted(v) for k, v in out.items()}


def _edge_closed(edge: TopoDS_Edge, curve: BRepAdaptor_Curve) -> bool:
    start = _vec(curve.Value(curve.FirstParameter()))
    end = _vec(curve.Value(curve.LastParameter()))
    return bool(BRep_Tool.IsClosed(edge)) or float(np.linalg.norm(start - end)) < CLOSED_TOL_MM


def _fingerprint(ctype: str, curve: BRepAdaptor_Curve, length_mm: float) -> str:
    """Fingerprint of :func:`awm_geometry.topology.fingerprint` (metres, 0.1 mm resolution).

    Converted to metres so a D20 edge and the same edge of the A21 catalogue share one fingerprint.
    """
    u0, u1 = curve.FirstParameter(), curve.LastParameter()
    ends = [units.mm_to_m(_vec(curve.Value(u))).tolist() for u in (u0, u1)]
    mid = units.mm_to_m(_vec(curve.Value(0.5 * (u0 + u1)))).tolist()
    return topology.fingerprint(ctype, units.mm_to_m(length_mm), ends, mid)


def preview_frames(joint: joints.JointEdge, solids, n_frames=PREVIEW_FRACTIONS) -> List[Dict]:
    """Seam-frame samples at fractions of the joint length along edge_a (display check only).

    X = tangent of edge_a in curve-parameter direction, made orthogonal to Z; Z = bisector of the
    exterior-face normals (rule of :mod:`awm_geometry.joints`).
    """
    curve = BRepAdaptor_Curve(joint.edge_a)
    u0 = curve.FirstParameter()
    total = GCPnts_AbscissaPoint.Length(curve)
    frames = []
    for fraction in n_frames:
        u = u0 if fraction == 0.0 else GCPnts_AbscissaPoint(curve, fraction * total, u0).Parameter()
        p = _vec(curve.Value(u))
        n1, n2 = joints.joint_normals(joint, solids, p, JOINT_PROBE_MM, JOINT_TOL_MM)
        z = joints.bisector(n1, n2)
        if z is None:
            raise GeometryError('joint face normals are opposite: seam frame Z undefined')
        x = _tangent(curve, u)
        x = x - (x @ z) * z
        x = x / np.linalg.norm(x)
        frames.append({'point_mm': p.tolist(), 'x': x.tolist(), 'z': z.tolist()})
    return frames


def _clean(v) -> Any:
    """Round-trip-stable plain floats (no -0.0)."""
    if isinstance(v, (list, tuple, np.ndarray)):
        return [_clean(x) for x in v]
    return float(v) + 0.0


def export(component: Component, step: Path, component_name: Optional[str], up_axis: str,
           out_dir: Path, created_by: str, base_dir: Path) -> Dict[str, Any]:
    """Write ``view.glb`` and ``topology.json`` into ``out_dir``; return a summary."""
    from awm_core import jsonio

    if up_axis not in UP_AXES:
        raise GeometryError(f'up axis {up_axis!r} not in {UP_AXES}')
    model = build_model(component)
    LOG.info('A13 D20: %d solid(s), %d face(s), %d edge(s)%s', model.solids.Size(),
             model.faces.Size(), model.edges.Size(), ' (healed)' if model.healed else '')
    mesher = BRepMesh_IncrementalMesh(model.shape, FACE_LINEAR_DEFLECTION_MM, False,
                                      units.deg_to_rad(FACE_ANGULAR_DEFLECTION_DEG), True)
    mesher.Perform()
    if not mesher.IsDone():
        raise GeometryError('tessellation failed')

    face_owner = _owner(model, TopAbs_FACE)
    edge_owner = _owner(model, TopAbs_EDGE)
    solid_list = joints.solids_of(model.shape)
    solid_index = [model.solids.FindIndex(s) for s in solid_list]
    found = joints.find_joint_edges(solid_list, JOINT_TOL_MM)
    edge_joint: Dict[int, str] = {}
    joint_docs = []
    for k, joint in enumerate(found, start=1):
        jid = f'JE{k}'
        ia, ib = model.edges.FindIndex(joint.edge_a), model.edges.FindIndex(joint.edge_b)
        edge_joint[ia] = edge_joint[ib] = jid
        joint_docs.append({
            'joint_edge_id': jid,
            'solid_a': f'B{solid_index[joint.solid_a]}', 'solid_b': f'B{solid_index[joint.solid_b]}',
            'edge_a': f'E{ia}', 'edge_b': f'E{ib}',
            'length_mm': float(joint.length),
            'closed': _edge_closed(joint.edge_a, BRepAdaptor_Curve(joint.edge_a)),
            'preview_frames': [{k2: _clean(v) for k2, v in f.items()}
                               for f in preview_frames(joint, solid_list)],
        })
    neighbours = _tangent_neighbours(model)

    glb = GlbBuilder({'unit': 'mm', 'frame': 'part', 'schema': 'D20',
                      'schema_version': SCHEMA_VERSION}, created_by)
    root = glb.add_node('part')
    solid_nodes = {}
    solid_docs = []
    for b in range(1, model.solids.Size() + 1):
        solid = model.solids.FindKey(b)
        props = GProp_GProps()
        brepgprop.VolumeProperties(solid, props)
        lo, hi = topology.bbox(solid)
        solid_docs.append({'solid_id': f'B{b}', 'name': model.names[b - 1],
                           'volume_mm3': abs(float(props.Mass())),
                           'bbox_min_mm': _clean(lo), 'bbox_max_mm': _clean(hi)})
        solid_nodes[b] = glb.add_node(f'B{b}', root, extras={'solid_id': f'B{b}'})

    face_docs = []
    for i in range(1, model.faces.Size() + 1):
        face = topods.Face(model.faces.FindKey(i))
        fid, sid = f'F{i}', f'B{face_owner[i]}'
        stype = topology._SURFACE_NAMES.get(BRepAdaptor_Surface(face).GetType(), 'other')
        face_docs.append({'face_id': fid, 'solid_id': sid, 'surface_type': stype,
                          'area_mm2': float(topology.face_area(face))})
        try:
            pos, nor, idx = face_mesh(face)
        except GeometryError as exc:
            raise GeometryError(f'{fid}: {exc}') from exc
        mesh = glb.add_triangles(fid, pos, nor, idx)
        glb.add_node(fid, solid_nodes[face_owner[i]], mesh,
                     extras={'face_id': fid, 'solid_id': sid})

    ancestors = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(model.shape, TopAbs_EDGE, TopAbs_FACE, ancestors)
    edge_docs = []
    for i in range(1, model.edges.Size() + 1):
        edge = topods.Edge(model.edges.FindKey(i))
        if BRep_Tool.Degenerated(edge):
            continue
        eid, sid = f'E{i}', f'B{edge_owner[i]}'
        curve = BRepAdaptor_Curve(edge)
        ctype = topology._CURVE_NAMES.get(curve.GetType(), 'other')
        length = float(topology.edge_length(edge))
        face_ids: List[str] = []
        a = ancestors.FindIndex(edge)
        if a:
            for f in ancestors.FindFromIndex(a):
                fid = f'F{model.faces.FindIndex(f)}'
                if fid not in face_ids:
                    face_ids.append(fid)
        jid = edge_joint.get(i)
        edge_docs.append({
            'edge_id': eid, 'solid_id': sid, 'curve_type': ctype, 'length_mm': length,
            'closed': _edge_closed(edge, curve), 'faces': face_ids,
            'tangent_neighbours': [f'E{n}' for n in neighbours.get(i, [])],
            'fingerprint': _fingerprint(ctype, curve, length), 'joint_edge_id': jid,
        })
        mesh = glb.add_line_strip(eid, edge_polyline(edge, EDGE_CHORD_MM))
        glb.add_node(eid, solid_nodes[edge_owner[i]], mesh,
                     extras={'edge_id': eid, 'solid_id': sid, 'joint_edge_id': jid})

    doc = jsonio.make_header(created_by, [step], base_dir)
    doc.update({
        'schema_version': SCHEMA_VERSION, 'unit': 'mm', 'frame': 'part',
        'source': {'file': doc['inputs'][0]['name'], 'component': component_name,
                   'up_axis': up_axis},
        'tessellation': {'face_linear_deflection_mm': FACE_LINEAR_DEFLECTION_MM,
                         'face_angular_deflection_deg': FACE_ANGULAR_DEFLECTION_DEG,
                         'edge_chord_mm': EDGE_CHORD_MM},
        'solids': solid_docs, 'faces': face_docs, 'edges': edge_docs, 'joints': joint_docs,
    })
    jsonio.validate(doc, 'view_topology')
    out_dir.mkdir(parents=True, exist_ok=True)
    glb_path = out_dir / 'view.glb'
    tmp = out_dir / '.view.glb.tmp'
    tmp.write_bytes(glb.to_bytes())
    tmp.replace(glb_path)
    topo_path = jsonio.write_json(out_dir / 'topology.json', doc, 'view_topology')
    return {'view_glb': str(glb_path), 'topology_json': str(topo_path),
            'solids': len(solid_docs), 'faces': len(face_docs), 'edges': len(edge_docs),
            'joints': [{k: j[k] for k in ('joint_edge_id', 'solid_a', 'solid_b', 'edge_a',
                                          'edge_b', 'length_mm', 'closed')}
                       for j in joint_docs]}
