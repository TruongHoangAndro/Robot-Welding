"""Model check / heal (A12) and the B-Rep edge catalogue used for seam picking (A21).

Edge and face ids (``E<n>``, ``F<n>``) are the 1-based indices of ``TopTools_IndexedMapOfShape``
built by ``topexp.MapShapes`` on the healed shape stored in ``cache/model.brep``. They are stable
for that cache file. Each edge also gets a geometric fingerprint so seams can be re-matched when
the STEP revision changes (DES-001 §5.2).
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepOffset import BRepOffset_Analyse
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.ChFiDS import ChFiDS_Concave, ChFiDS_Convex, ChFiDS_Tangential
from OCC.Core.GProp import GProp_GProps
from OCC.Core.GeomAPI import GeomAPI_ProjectPointOnSurf
from OCC.Core.GeomAbs import (GeomAbs_BSplineCurve, GeomAbs_BSplineSurface, GeomAbs_BezierCurve,
                              GeomAbs_Circle, GeomAbs_Cone, GeomAbs_Cylinder, GeomAbs_Ellipse,
                              GeomAbs_Line, GeomAbs_Plane, GeomAbs_Sphere, GeomAbs_Torus)
from OCC.Core.BRepLProp import BRepLProp_SLProps
from OCC.Core.ShapeAnalysis import ShapeAnalysis_FreeBounds
from OCC.Core.ShapeFix import ShapeFix_Shape
from OCC.Core.TopAbs import (TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED, TopAbs_SHELL,
                             TopAbs_SOLID)
from OCC.Core.TopTools import (TopTools_IndexedDataMapOfShapeListOfShape,
                               TopTools_IndexedMapOfShape)
from OCC.Core.TopExp import topexp
from OCC.Core.TopoDS import TopoDS_Edge, TopoDS_Face, TopoDS_Shape, topods
from OCC.Core.gp import gp_Pnt

_CURVE_NAMES = {GeomAbs_Line: 'line', GeomAbs_Circle: 'circle', GeomAbs_Ellipse: 'ellipse',
                GeomAbs_BSplineCurve: 'bspline', GeomAbs_BezierCurve: 'bezier'}
_SURFACE_NAMES = {GeomAbs_Plane: 'plane', GeomAbs_Cylinder: 'cylinder', GeomAbs_Cone: 'cone',
                  GeomAbs_Sphere: 'sphere', GeomAbs_Torus: 'torus',
                  GeomAbs_BSplineSurface: 'bspline'}

SMALL_EDGE_M = 1.0e-4
FINGERPRINT_DECIMALS = 4  # 0.1 mm


@dataclass
class CheckResult:
    name: str
    ok: bool
    detail: str

    def as_dict(self) -> Dict[str, object]:
        return {'name': self.name, 'ok': self.ok, 'detail': self.detail}


def index_map(shape: TopoDS_Shape, kind) -> TopTools_IndexedMapOfShape:
    imap = TopTools_IndexedMapOfShape()
    topexp.MapShapes(shape, kind, imap)
    return imap


def count(shape: TopoDS_Shape, kind) -> int:
    return index_map(shape, kind).Size()


def bbox(shape: TopoDS_Shape) -> Tuple[List[float], List[float]]:
    box = Bnd_Box()
    box.SetGap(0.0)
    brepbndlib.Add(shape, box, False)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return [xmin, ymin, zmin], [xmax, ymax, zmax]


def check_and_heal(shape: TopoDS_Shape, unit_source: str) -> Tuple[TopoDS_Shape, bool,
                                                                   List[CheckResult]]:
    """A12: run ShapeFix if the shape is not valid; return (shape, healed, checks)."""
    checks: List[CheckResult] = []
    valid = BRepCheck_Analyzer(shape).IsValid()
    healed = False
    if not valid:
        fixer = ShapeFix_Shape(shape)
        fixer.Perform()
        shape = fixer.Shape()
        healed = True
        valid = BRepCheck_Analyzer(shape).IsValid()
    checks.append(CheckResult('brep_valid', valid,
                              'valid' + (' after ShapeFix' if healed else '') if valid
                              else 'BRepCheck_Analyzer reports an invalid shape after ShapeFix'))
    n_solids = count(shape, TopAbs_SOLID)
    checks.append(CheckResult('has_solid', n_solids > 0, f'{n_solids} solid(s)'))
    free = ShapeAnalysis_FreeBounds(shape)
    n_free = count(free.GetClosedWires(), TopAbs_EDGE) + count(free.GetOpenWires(), TopAbs_EDGE)
    checks.append(CheckResult('closed_shells', n_free == 0,
                              f'{n_free} free (unshared) edge(s)'))
    lo, hi = bbox(shape)
    size = max(h - l for h, l in zip(hi, lo))
    plausible = 0.005 <= size <= 5.0
    checks.append(CheckResult('size_plausible', plausible,
                              f'largest dimension {size * 1000:.1f} mm (expected 5 mm - 5 m)'))
    checks.append(CheckResult('units_declared', unit_source in ('mm', 'm', 'inch'),
                              f'file length unit: {unit_source}'))
    edges = index_map(shape, TopAbs_EDGE)
    small = 0
    for i in range(1, edges.Size() + 1):
        edge = topods.Edge(edges.FindKey(i))
        if BRep_Tool.Degenerated(edge):
            continue
        if edge_length(edge) < SMALL_EDGE_M:
            small += 1
    checks.append(CheckResult('no_small_edges', small == 0,
                              f'{small} edge(s) shorter than {SMALL_EDGE_M * 1000:.2f} mm'))
    return shape, healed, checks


def edge_length(edge: TopoDS_Edge) -> float:
    props = GProp_GProps()
    brepgprop.LinearProperties(edge, props)
    return props.Mass()


def face_area(face: TopoDS_Face) -> float:
    props = GProp_GProps()
    brepgprop.SurfaceProperties(face, props)
    return props.Mass()


def _pnt(p: gp_Pnt) -> List[float]:
    return [p.X(), p.Y(), p.Z()]


def face_normal_at(face: TopoDS_Face, point: np.ndarray) -> np.ndarray:
    """Outward unit normal of ``face`` at the surface point closest to ``point``."""
    surface = BRep_Tool.Surface(face)
    proj = GeomAPI_ProjectPointOnSurf(gp_Pnt(*point), surface)
    if proj.NbPoints() == 0:
        raise RuntimeError('point projection on face failed')
    u, v = proj.LowerDistanceParameters()
    props = BRepLProp_SLProps(BRepAdaptor_Surface(face), u, v, 1, 1e-9)
    if not props.IsNormalDefined():
        raise RuntimeError('face normal undefined at seam point')
    n = props.Normal()
    normal = np.array([n.X(), n.Y(), n.Z()])
    if face.Orientation() == TopAbs_REVERSED:
        normal = -normal
    return normal / np.linalg.norm(normal)


def sample_edge(edge: TopoDS_Edge, n: int) -> np.ndarray:
    """``n`` points uniform in curve parameter (for display / picking only)."""
    curve = BRepAdaptor_Curve(edge)
    u0, u1 = curve.FirstParameter(), curve.LastParameter()
    return np.array([_pnt(curve.Value(u0 + (u1 - u0) * k / (n - 1))) for k in range(n)])


def fingerprint(curve_type: str, length: float, ends: List[List[float]], mid: List[float]) -> str:
    """Orientation-independent hash of an edge's geometry (0.1 mm resolution)."""
    r = FINGERPRINT_DECIMALS
    ends_sorted = sorted(tuple(round(c, r) + 0.0 for c in p) for p in ends)
    text = f'{curve_type}|{round(length, r)}|{ends_sorted}|{tuple(round(c, r) + 0.0 for c in mid)}'
    return hashlib.sha256(text.encode()).hexdigest()


def edge_catalogue(shape: TopoDS_Shape) -> Dict[str, list]:
    """Build the ``edges.json`` payload (edges + faces) for a healed shape in metres."""
    faces = index_map(shape, TopAbs_FACE)
    edges = index_map(shape, TopAbs_EDGE)
    ancestors = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, ancestors)
    analyse = BRepOffset_Analyse(shape, math.radians(1.0))

    face_list = []
    for i in range(1, faces.Size() + 1):
        face = topods.Face(faces.FindKey(i))
        stype = _SURFACE_NAMES.get(BRepAdaptor_Surface(face).GetType(), 'other')
        face_list.append({'id': f'F{i}', 'surface_type': stype, 'area_m2': face_area(face)})

    edge_list = []
    for i in range(1, edges.Size() + 1):
        edge = topods.Edge(edges.FindKey(i))
        if BRep_Tool.Degenerated(edge):
            continue
        curve = BRepAdaptor_Curve(edge)
        ctype = _CURVE_NAMES.get(curve.GetType(), 'other')
        u0, u1 = curve.FirstParameter(), curve.LastParameter()
        start, end = _pnt(curve.Value(u0)), _pnt(curve.Value(u1))
        mid = _pnt(curve.Value(0.5 * (u0 + u1)))
        length = edge_length(edge)
        n_poly = int(min(64, max(2, math.ceil(length / 0.002) + 1)))
        face_ids: List[str] = []
        idx = ancestors.FindIndex(edge)
        if idx:
            for f in ancestors.FindFromIndex(idx):
                fid = f'F{faces.FindIndex(f)}'
                if fid not in face_ids:
                    face_ids.append(fid)
        convexity = _convexity(analyse, edge, len(face_ids))
        if convexity == 'free' and BRep_Tool.IsClosed(
                edge, topods.Face(faces.FindKey(int(face_ids[0][1:])))):
            convexity = 'smooth'  # seam line of a periodic face (cylinder, cone)
        dihedral = _dihedral_deg(faces, face_ids, np.array(mid)) if len(face_ids) == 2 else None
        edge_list.append({
            'id': f'E{i}', 'curve_type': ctype, 'length_m': length,
            'start_m': start, 'end_m': end, 'mid_m': mid,
            'closed': bool(BRep_Tool.IsClosed(edge)) or float(np.linalg.norm(
                np.subtract(start, end))) < 1e-7,
            'faces': face_ids, 'convexity': convexity, 'dihedral_deg': dihedral,
            'polyline_m': sample_edge(edge, n_poly).tolist(),
            'fingerprint': fingerprint(ctype, length, [start, end], mid),
        })
    return {'edges': edge_list, 'faces': face_list}


def _convexity(analyse: BRepOffset_Analyse, edge: TopoDS_Edge, n_faces: int) -> str:
    if n_faces == 1:
        return 'free'
    if n_faces > 2:
        return 'nonmanifold'
    kinds = {iv.Type() for iv in analyse.Type(edge)}
    if kinds == {ChFiDS_Concave}:
        return 'concave'
    if kinds == {ChFiDS_Convex}:
        return 'convex'
    if kinds <= {ChFiDS_Tangential}:
        return 'smooth'
    return 'mixed'


def _dihedral_deg(faces: TopTools_IndexedMapOfShape, face_ids: List[str],
                  point: np.ndarray) -> Optional[float]:
    """Angle between the two outward normals at ``point`` (0 = tangent faces)."""
    try:
        n1 = face_normal_at(topods.Face(faces.FindKey(int(face_ids[0][1:]))), point)
        n2 = face_normal_at(topods.Face(faces.FindKey(int(face_ids[1][1:]))), point)
    except RuntimeError:
        return None
    return float(np.degrees(np.arccos(np.clip(np.dot(n1, n2), -1.0, 1.0))))


def counts(shape: TopoDS_Shape) -> Dict[str, int]:
    return {'solids': count(shape, TopAbs_SOLID), 'shells': count(shape, TopAbs_SHELL),
            'faces': count(shape, TopAbs_FACE), 'edges': count(shape, TopAbs_EDGE)}
