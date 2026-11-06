"""D20 3D view model export (A13): ``view.glb`` + ``topology.json`` (DES-001 Rev 0.5 §4.5, CP-52).

Contract: ``docs/generated/contracts/D20_view_model.md`` (v2.0, rules 1-7). Output in the part frame, **mm**,
no rotation. Display only: the browser picks ids, M2 resolves them on the exact B-Rep.

Rule 5 (catalogue parity): the shape is :func:`awm_geometry.part.load_part` - the same healed shape in
metres as the A21 edge catalogue - so ``F<n>`` / ``E<n>`` / ``B<n>``, closed flags and fingerprints are
computed by the same functions on the same evaluation. Every length leaving this module is converted to mm
through ``awm_core.units``. Degenerated edges get no entry. Joint edges ``JE<n>`` follow the order of
:func:`awm_geometry.joints.find_joint_edges`; an edge in two joints is an error (rule 6).
"""

from __future__ import annotations

import logging
import os
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
from awm_geometry.glb import GlbBuilder
from awm_geometry.occ_io import GeometryError
from awm_geometry.part import LoadedPart

LOG = logging.getLogger('awm_geometry')

SCHEMA_VERSION = '2.0'
FACE_LINEAR_DEFLECTION_MM = 0.05
FACE_ANGULAR_DEFLECTION_DEG = 5.0
EDGE_CHORD_MM = 0.05            # contract maximum (D20: chord <= 0.05 mm)
JOINT_TOL_MM = 0.05             # coincidence tolerance of joint edges (as test_joints_occ)
JOINT_PROBE_MM = 0.5            # probe distance for the exterior-face test of a joint
G1_TOL_DEG = 1.0                # tangent continuity between edges at a shared vertex
PREVIEW_FRACTIONS = (0.0, 0.25, 0.5, 0.75)
UP_AXES = ('+X', '-X', '+Y', '-Y', '+Z', '-Z')


class PartModel:
    """Id maps of a loaded part (shared by the exporter and the tests)."""

    def __init__(self, loaded: LoadedPart) -> None:
        self.shape: TopoDS_Shape = loaded.shape
        self.solids: TopTools_IndexedMapOfShape = topology.index_map(loaded.shape, TopAbs_SOLID)
        self.faces: TopTools_IndexedMapOfShape = topology.index_map(loaded.shape, TopAbs_FACE)
        self.edges: TopTools_IndexedMapOfShape = topology.index_map(loaded.shape, TopAbs_EDGE)
        self.names: List[str] = loaded.solid_names


def _vec(v) -> np.ndarray:
    return np.array([v.X(), v.Y(), v.Z()], dtype=float)


def _mm(x) -> List[float]:
    """Metres -> mm as plain floats (no -0.0) for JSON."""
    return [float(v) + 0.0 for v in np.atleast_1d(units.m_to_mm(x))]


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
    """Triangulation of a meshed face: positions (shape unit), unit outward normals, uint32 indices."""
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
    # Vertex normals from the exact surface (BRepGProp_Face follows the face orientation);
    # area-weighted triangle normals where the surface normal is undefined.
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
    """Points along the edge (curve-parameter direction), chordal deviation <= ``chord`` (shape unit)."""
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


def preview_frames(joint: joints.JointEdge, solids, fractions=PREVIEW_FRACTIONS) -> List[Dict]:
    """Seam-frame samples at fractions of the joint length along edge_a (display check only).

    X = tangent of edge_a in its curve-parameter direction, made orthogonal to Z; Z = bisector of the
    exterior-face normals (rule of :mod:`awm_geometry.joints`). Point in mm. Raises ``JointError`` /
    ``GeometryError`` if the frame is undefined.
    """
    curve = BRepAdaptor_Curve(joint.edge_a)
    u0 = curve.FirstParameter()
    total = GCPnts_AbscissaPoint.Length(curve)
    probe, tol = units.mm_to_m(JOINT_PROBE_MM), units.mm_to_m(JOINT_TOL_MM)
    frames = []
    for fraction in fractions:
        u = u0 if fraction == 0.0 else GCPnts_AbscissaPoint(curve, fraction * total, u0).Parameter()
        p = _vec(curve.Value(u))
        n1, n2 = joints.joint_normals(joint, solids, p, probe, tol)
        z = joints.bisector(n1, n2)
        if z is None:
            raise GeometryError('joint face normals are opposite: seam frame Z undefined')
        x = _tangent(curve, u)
        x = x - (x @ z) * z
        x = x / np.linalg.norm(x)
        frames.append({'point_mm': _mm(p), 'x': [float(v) + 0.0 for v in x],
                       'z': [float(v) + 0.0 for v in z]})
    return frames


def _joints(model: PartModel) -> Tuple[List[Dict], Dict[int, str]]:
    """Joint records (rule 6: preview frames best effort; an edge in two joints is an error)."""
    solid_list = joints.solids_of(model.shape)
    solid_index = [model.solids.FindIndex(s) for s in solid_list]
    found = joints.find_joint_edges(solid_list, units.mm_to_m(JOINT_TOL_MM))
    edge_joint: Dict[int, str] = {}
    docs = []
    for k, joint in enumerate(found, start=1):
        jid = f'JE{k}'
        ia, ib = model.edges.FindIndex(joint.edge_a), model.edges.FindIndex(joint.edge_b)
        for i in (ia, ib):
            if i in edge_joint:
                raise GeometryError(f'edge E{i} belongs to two joints ({edge_joint[i]} and {jid}); '
                                    'joint_edge_id holds one id')
            edge_joint[i] = jid
        try:
            frames = preview_frames(joint, solid_list)
        except (joints.JointError, GeometryError) as exc:
            LOG.warning('%s (E%d / E%d): no preview frames: %s', jid, ia, ib, exc)
            frames = []
        docs.append({
            'joint_edge_id': jid,
            'solid_a': f'B{solid_index[joint.solid_a]}', 'solid_b': f'B{solid_index[joint.solid_b]}',
            'edge_a': f'E{ia}', 'edge_b': f'E{ib}',
            'length_mm': units.m_to_mm(joint.length),
            'closed': topology.edge_closed(joint.edge_a),
            'preview_frames': frames,
        })
    return docs, edge_joint


def source_name(step: Path, job_dir: Optional[Path]) -> Tuple[Path, str]:
    """Rule 7: (base dir, name) of the STEP file; relative to the job folder, else the bare name."""
    step = step.resolve()
    if job_dir is None:
        return step.parent, step.name
    job = Path(job_dir).resolve()
    try:
        rel = step.relative_to(job)
    except ValueError:
        raise GeometryError(f'STEP file {step} is not inside the job folder {job}') from None
    return job, rel.as_posix()


def export(loaded: LoadedPart, step: Path, component_name: Optional[str], up_axis: str,
           out_dir: Path, created_by: str, job_dir: Optional[Path] = None) -> Dict[str, Any]:
    """Write ``view.glb`` and ``topology.json`` into ``out_dir``; return a summary."""
    from awm_core import jsonio

    if up_axis not in UP_AXES:
        raise GeometryError(f'up axis {up_axis!r} not in {UP_AXES}')
    base_dir, name = source_name(step, job_dir)
    model = PartModel(loaded)
    LOG.info('A13 D20: %d solid(s), %d face(s), %d edge(s)%s', model.solids.Size(),
             model.faces.Size(), model.edges.Size(), ' (healed)' if loaded.healed else '')
    mesher = BRepMesh_IncrementalMesh(model.shape, units.mm_to_m(FACE_LINEAR_DEFLECTION_MM), False,
                                      units.deg_to_rad(FACE_ANGULAR_DEFLECTION_DEG), True)
    mesher.Perform()
    if not mesher.IsDone():
        raise GeometryError('tessellation failed')

    face_owner = _owner(model, TopAbs_FACE)
    edge_owner = _owner(model, TopAbs_EDGE)
    joint_docs, edge_joint = _joints(model)
    neighbours = _tangent_neighbours(model)

    glb = GlbBuilder({'unit': 'mm', 'frame': 'part', 'schema': 'D20',
                      'schema_version': SCHEMA_VERSION}, created_by)
    root = glb.add_node('part')
    solid_nodes = {}
    solid_docs = []
    comps = loaded.solid_components
    for b in range(1, model.solids.Size() + 1):
        solid = model.solids.FindKey(b)
        props = GProp_GProps()
        brepgprop.VolumeProperties(solid, props)
        lo, hi = topology.bbox(solid)
        volume_mm3 = units.m_to_mm(units.m_to_mm(units.m_to_mm(abs(props.Mass()))))
        solid_docs.append({'solid_id': f'B{b}', 'name': model.names[b - 1],
                           'volume_mm3': volume_mm3, 'bbox_min_mm': _mm(lo),
                           'bbox_max_mm': _mm(hi)})
        if comps is not None and comps[b - 1] is not None:
            solid_docs[-1].update(component_index=comps[b - 1],
                                  component_path=loaded.component_paths[comps[b - 1]])
        solid_nodes[b] = glb.add_node(f'B{b}', root, extras={'solid_id': f'B{b}'})

    shells: List[int] = []

    def _parent(owner: Optional[int]) -> int:
        """glb parent node: the owning solid, or one 'shells' node for faces / edges without a solid."""
        if owner is not None:
            return solid_nodes[owner]
        if not shells:
            shells.append(glb.add_node('shells', root, extras={'solid_id': None}))
        return shells[0]

    face_docs = []
    for i in range(1, model.faces.Size() + 1):
        face = topods.Face(model.faces.FindKey(i))
        owner = face_owner.get(i)
        fid, sid = f'F{i}', (None if owner is None else f'B{owner}')
        stype = topology._SURFACE_NAMES.get(BRepAdaptor_Surface(face).GetType(), 'other')
        area_mm2 = units.m_to_mm(units.m_to_mm(topology.face_area(face)))
        face_docs.append({'face_id': fid, 'solid_id': sid, 'surface_type': stype,
                          'area_mm2': area_mm2})
        try:
            pos, nor, idx = face_mesh(face)
        except GeometryError as exc:
            raise GeometryError(f'{fid}: {exc}') from exc
        mesh = glb.add_triangles(fid, units.m_to_mm(pos), nor, idx)
        glb.add_node(fid, _parent(owner), mesh, extras={'face_id': fid, 'solid_id': sid})

    ancestors = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(model.shape, TopAbs_EDGE, TopAbs_FACE, ancestors)
    chord_m = units.mm_to_m(EDGE_CHORD_MM)
    edge_docs = []
    for i in range(1, model.edges.Size() + 1):
        edge = topods.Edge(model.edges.FindKey(i))
        if BRep_Tool.Degenerated(edge):
            continue
        owner = edge_owner.get(i)
        eid, sid = f'E{i}', (None if owner is None else f'B{owner}')
        curve = BRepAdaptor_Curve(edge)
        jid = edge_joint.get(i)
        edge_docs.append({
            'edge_id': eid, 'solid_id': sid,
            'curve_type': topology._CURVE_NAMES.get(curve.GetType(), 'other'),
            'length_mm': units.m_to_mm(topology.edge_length(edge)),
            'closed': topology.edge_closed(edge),
            'faces': topology.edge_face_ids(ancestors, model.faces, edge),
            'tangent_neighbours': [f'E{n}' for n in neighbours.get(i, [])],
            'fingerprint': topology.edge_fingerprint(edge), 'joint_edge_id': jid,
        })
        mesh = glb.add_line_strip(eid, units.m_to_mm(edge_polyline(edge, chord_m)))
        glb.add_node(eid, _parent(owner), mesh, extras={'edge_id': eid, 'solid_id': sid, 'joint_edge_id': jid})

    doc = jsonio.make_header(created_by, [step], base_dir)
    doc.update({
        'schema_version': SCHEMA_VERSION, 'unit': 'mm', 'frame': 'part',
        'source': dict({'file': name, 'component': component_name, 'up_axis': up_axis},
                       **({'components': True} if comps is not None else {})),
        'tessellation': {'face_linear_deflection_mm': FACE_LINEAR_DEFLECTION_MM,
                         'face_angular_deflection_deg': FACE_ANGULAR_DEFLECTION_DEG,
                         'edge_chord_mm': EDGE_CHORD_MM},
        'solids': solid_docs, 'faces': face_docs, 'edges': edge_docs, 'joints': joint_docs,
    })
    jsonio.validate(doc, 'view_topology')
    out_dir.mkdir(parents=True, exist_ok=True)
    glb_path = out_dir / 'view.glb'
    tmp = out_dir / f'.view.glb.{os.getpid()}.tmp'
    tmp.write_bytes(glb.to_bytes())
    tmp.replace(glb_path)
    topo_path = jsonio.write_json(out_dir / 'topology.json', doc, 'view_topology')
    return {'view_glb': str(glb_path), 'topology_json': str(topo_path),
            'solids': len(solid_docs), 'faces': len(face_docs), 'edges': len(edge_docs),
            'joints': [{k: j[k] for k in ('joint_edge_id', 'solid_a', 'solid_b', 'edge_a',
                                          'edge_b', 'length_mm', 'closed')}
                       for j in joint_docs]}
