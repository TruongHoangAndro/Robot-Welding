"""Weld joints between separate solids (A41 input, DES-001 Rev 0.6 B-02, TST-002 Rev 0.3 §3.2).

A weldment such as the golden part is an assembly of separate solids. A seam edge then exists once in
each solid, and the two faces of one solid at that edge are its outer surface and its end (contact)
face. Taking n1 / n2 from one solid therefore gives a seam frame that is wrong by the angle between
those faces (45° on a tube butt joint).

Joint-aware rule: a joint edge is an edge of solid A that coincides with an edge of solid B. At a point
of the joint, n1 is the normal of the face of A next to the edge that does not touch B (its exterior
face), n2 the same for B. The seam frame Z is the bisector of n1 / n2 (out of the material).

All lengths are in the unit of the given shapes; tolerances are passed by the caller.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.GCPnts import GCPnts_AbscissaPoint
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_SOLID
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCC.Core.TopoDS import TopoDS_Edge, TopoDS_Face, TopoDS_Shape, TopoDS_Solid, topods
from OCC.Core.gp import gp_Pnt, gp_Vec

from awm_geometry.topology import face_normal_at

EDGE_SAMPLES = 9


class JointError(RuntimeError):
    """The exterior faces of a joint cannot be determined."""


@dataclass
class JointEdge:
    """One edge shared (geometrically) by two solids."""

    solid_a: int
    solid_b: int
    edge_a: TopoDS_Edge
    edge_b: TopoDS_Edge
    faces_a: List[TopoDS_Face]
    faces_b: List[TopoDS_Face]
    length: float


def solids_of(shape: TopoDS_Shape) -> List[TopoDS_Solid]:
    """All solids of a shape, in explorer order (the solid index used by this module)."""
    out = []
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        out.append(topods.Solid(exp.Current()))
        exp.Next()
    return out


def _distance(a: TopoDS_Shape, b: TopoDS_Shape) -> float:
    dist = BRepExtrema_DistShapeShape(a, b)
    if not dist.IsDone():
        raise JointError('distance computation failed')
    return dist.Value()


def _vertex(p: np.ndarray) -> TopoDS_Shape:
    return BRepBuilderAPI_MakeVertex(gp_Pnt(*map(float, p))).Vertex()


def _closest_on(shape: TopoDS_Shape, p: np.ndarray) -> np.ndarray:
    dist = BRepExtrema_DistShapeShape(_vertex(p), shape)
    if not dist.IsDone() or dist.NbSolution() == 0:
        raise JointError('point projection failed')
    q = dist.PointOnShape2(1)
    return np.array([q.X(), q.Y(), q.Z()])


def _edge_points(edge: TopoDS_Edge, n: int) -> np.ndarray:
    curve = BRepAdaptor_Curve(edge)
    u0, u1 = curve.FirstParameter(), curve.LastParameter()
    pts = []
    for k in range(n):
        p = curve.Value(u0 + (u1 - u0) * k / (n - 1))
        pts.append([p.X(), p.Y(), p.Z()])
    return np.array(pts)


def _edge_faces(solid: TopoDS_Solid) -> List[Tuple[TopoDS_Edge, List[TopoDS_Face]]]:
    ancestors = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(solid, TopAbs_EDGE, TopAbs_FACE, ancestors)
    out = []
    for i in range(1, ancestors.Size() + 1):
        faces: List[TopoDS_Face] = []
        for f in ancestors.FindFromIndex(i):
            face = topods.Face(f)
            if not any(face.IsSame(g) for g in faces):
                faces.append(face)
        out.append((topods.Edge(ancestors.FindKey(i)), faces))
    return out


def find_joint_edges(solids: Sequence[TopoDS_Solid], tol: float) -> List[JointEdge]:
    """Edges of one solid that coincide with an edge of another solid within ``tol``.

    Coincidence: equal length within ``tol`` and every sample point of edge A within ``tol`` of
    edge B. Solid pairs farther apart than ``tol`` are skipped.
    """
    per_solid = [_edge_faces(s) for s in solids]
    lengths = [[GCPnts_AbscissaPoint.Length(BRepAdaptor_Curve(e)) for e, _ in edges]
               for edges in per_solid]
    joints = []
    for a in range(len(solids)):
        for b in range(a + 1, len(solids)):
            if _distance(solids[a], solids[b]) > tol:
                continue
            for (ea, fa), la in zip(per_solid[a], lengths[a]):
                pts = _edge_points(ea, EDGE_SAMPLES)
                if any(_distance(_vertex(p), solids[b]) > tol for p in pts[::4]):
                    continue
                for (eb, fb), lb in zip(per_solid[b], lengths[b]):
                    if abs(la - lb) > tol:
                        continue
                    if all(_distance(_vertex(p), eb) <= tol for p in pts):
                        joints.append(JointEdge(a, b, ea, eb, fa, fb, la))
                        break
    return joints


def _exterior_face(faces: Sequence[TopoDS_Face], edge: TopoDS_Edge, other: TopoDS_Solid,
                   p: np.ndarray, probe: float, tol: float) -> TopoDS_Face:
    """The face next to ``edge`` at ``p`` that does not touch ``other``.

    A probe point is taken on each face about ``probe`` away from the edge; the face whose probe
    point is farther than ``tol`` from ``other`` is exterior.
    """
    curve = BRepAdaptor_Curve(edge)
    exterior = []
    for face in faces:
        best, far = None, 0.0
        normal = face_normal_at(face, p)
        tangent = _tangent_at(curve, p)
        side = np.cross(normal, tangent)
        for direction in (side, -side):
            q = _closest_on(face, p + probe * direction)
            if np.linalg.norm(q - p) > far:
                best, far = q, float(np.linalg.norm(q - p))
        if best is None or far < 0.5 * probe:
            continue
        if _distance(_vertex(best), other) > tol:
            exterior.append(face)
    if len(exterior) != 1:
        raise JointError(f'{len(exterior)} exterior faces found at the joint (expected 1)')
    return exterior[0]


def _tangent_at(curve: BRepAdaptor_Curve, p: np.ndarray) -> np.ndarray:
    u0, u1 = curve.FirstParameter(), curve.LastParameter()
    us = np.linspace(u0, u1, 64)
    pts = np.array([[q.X(), q.Y(), q.Z()] for q in (curve.Value(u) for u in us)])
    k = int(np.argmin(np.linalg.norm(pts - p, axis=1)))
    pnt, vec = gp_Pnt(), gp_Vec()
    curve.D1(float(us[k]), pnt, vec)
    t = np.array([vec.X(), vec.Y(), vec.Z()])
    return t / np.linalg.norm(t)


def joint_normals(joint: JointEdge, solids: Sequence[TopoDS_Solid], p: Sequence[float],
                  probe: float, tol: float) -> Tuple[np.ndarray, np.ndarray]:
    """Outward normals (n1 of solid A, n2 of solid B) of the exterior faces at joint point ``p``."""
    point = np.asarray(p, float)
    fa = _exterior_face(joint.faces_a, joint.edge_a, solids[joint.solid_b], point, probe, tol)
    fb = _exterior_face(joint.faces_b, joint.edge_b, solids[joint.solid_a], point, probe, tol)
    return face_normal_at(fa, point), face_normal_at(fb, point)


def bisector(n1: np.ndarray, n2: np.ndarray) -> Optional[np.ndarray]:
    """Seam frame Z: unit bisector of the two normals (None if they are opposite)."""
    z = np.asarray(n1, float) + np.asarray(n2, float)
    norm = np.linalg.norm(z)
    return None if norm < 1e-9 else z / norm
