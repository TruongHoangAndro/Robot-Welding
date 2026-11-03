"""Joint-aware seam normals between separate solids (B-02; TST-002 Rev 0.3 gate G1).

Run with ``$AWM_OCC_PYTHON -m pytest``; skipped in an interpreter without OCC. The gate test on the
real cell CAD runs only when ``AWM_CELL_STEP`` names the file.
"""

import math
import os

import numpy as np
import pytest

OUTER_R = 13.335  # mm, golden-part tube (test data)
INNER_R = 10.465
G1_LIMIT_DEG = 0.5


def _angle_deg(a, b):
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))


def _tube(z0, height, axis=(0.0, 0.0, 1.0)):
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt

    d = np.asarray(axis, float)
    base = gp_Pnt(*(d * z0))
    ax2 = gp_Ax2(base, gp_Dir(*d))
    outer = BRepPrimAPI_MakeCylinder(ax2, OUTER_R, height).Shape()
    inner = BRepPrimAPI_MakeCylinder(ax2, INNER_R, height).Shape()
    return BRepAlgoAPI_Cut(outer, inner).Shape()


def _radial(p, centre, axis):
    v = np.asarray(p) - centre
    v = v - axis * (v @ axis)
    return v / np.linalg.norm(v)


def _circle_points(edge, n=12):
    from OCC.Core.BRepAdaptor import BRepAdaptor_Curve

    curve = BRepAdaptor_Curve(edge)
    u0, u1 = curve.FirstParameter(), curve.LastParameter()
    out = []
    for k in range(n):
        q = curve.Value(u0 + (u1 - u0) * (k + 0.5) / n)
        out.append(np.array([q.X(), q.Y(), q.Z()]))
    return out, curve


def _check_outer_joints(shape, tol=0.05, probe=0.5, outer_r=OUTER_R):
    """Return the worst joint-aware and per-solid angles to the radial direction on outer joints."""
    from OCC.Core.GeomAbs import GeomAbs_Circle

    from awm_geometry import joints
    from awm_geometry.topology import face_normal_at

    solids = joints.solids_of(shape)
    found = joints.find_joint_edges(solids, tol)
    worst_new, worst_old, n_outer = 0.0, 0.0, 0
    for joint in found:
        points, curve = _circle_points(joint.edge_a)
        if curve.GetType() != GeomAbs_Circle or abs(curve.Circle().Radius() - outer_r) > tol:
            continue
        n_outer += 1
        circle = curve.Circle()
        centre = np.array([circle.Location().X(), circle.Location().Y(), circle.Location().Z()])
        d = circle.Axis().Direction()
        axis = np.array([d.X(), d.Y(), d.Z()])
        for p in points:
            radial = _radial(p, centre, axis)
            n1, n2 = joints.joint_normals(joint, solids, p, probe, tol)
            worst_new = max(worst_new, _angle_deg(joints.bisector(n1, n2), radial))
            old = joints.bisector(*[face_normal_at(f, p) for f in joint.faces_a[:2]])
            worst_old = max(worst_old, _angle_deg(old, radial))
    return n_outer, worst_new, worst_old, len(found)


def test_two_tubes_butt_joint(occ):
    from awm_geometry.assembly import compound

    shape = compound([_tube(-30.0, 30.0), _tube(0.0, 40.0)])
    n_outer, worst_new, worst_old, n_joints = _check_outer_joints(shape)
    assert n_joints == 2  # outer and bore circle
    assert n_outer == 1
    assert worst_new < G1_LIMIT_DEG
    assert worst_old == pytest.approx(45.0, abs=0.5)  # the per-solid rule of Rev 0.2


def test_tilted_tubes_butt_joint(occ):
    from awm_geometry.assembly import compound

    axis = np.array([0.0, 0.9948, 0.1021])
    axis /= np.linalg.norm(axis)
    shape = compound([_tube(-30.0, 30.0, axis), _tube(0.0, 25.0, axis)])
    n_outer, worst_new, _, _ = _check_outer_joints(shape)
    assert n_outer == 1
    assert worst_new < G1_LIMIT_DEG


def test_no_joint_between_separated_solids(occ):
    from awm_geometry import joints
    from awm_geometry.assembly import compound

    shape = compound([_tube(-30.0, 30.0), _tube(1.0, 40.0)])
    assert joints.find_joint_edges(joints.solids_of(shape), 0.05) == []


def test_bisector_of_opposite_normals_is_none(occ):
    from awm_geometry import joints

    assert joints.bisector(np.array([1.0, 0, 0]), np.array([-1.0, 0, 0])) is None


@pytest.mark.skipif(not os.environ.get('AWM_CELL_STEP'), reason='AWM_CELL_STEP not set')
def test_gate_g1_golden_part(occ):
    """TST-002 Rev 0.3 G1: joint seam frames of the golden part are radial within 0.5°."""
    from awm_geometry.assembly import compound, read_components
    from awm_geometry.joints import solids_of

    from conftest import golden_record

    rec = golden_record()
    name = rec['component']
    components = read_components(os.environ['AWM_CELL_STEP'], name)
    assert components, f'component {name} not found'
    shape = compound([components[0].shape])
    assert len(solids_of(shape)) == rec['solids']
    n_outer, worst_new, worst_old, _ = _check_outer_joints(shape, outer_r=rec['outer_radius_mm'])
    assert n_outer == rec['outer_joints']  # J1 and J2
    assert worst_new < G1_LIMIT_DEG
    assert worst_old > 40.0
