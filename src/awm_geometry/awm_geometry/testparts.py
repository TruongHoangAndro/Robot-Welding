"""Generated test parts for the end-to-end chain (no customer STEP files are available).

Each part comes with a *pick request*: the seam picks a user would make in the Workbench (a point
near the seam edge plus the drawing attributes). The picks are test data, not source constants.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Union

from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Shape
from OCC.Core.BRep import BRep_Builder
from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt

from awm_core import units
from awm_geometry.occ_io import GeometryError, scale_shape


def _fuse(a: TopoDS_Shape, b: TopoDS_Shape) -> TopoDS_Shape:
    op = BRepAlgoAPI_Fuse(a, b)
    op.SimplifyResult()
    if not op.IsDone():
        raise GeometryError('boolean fuse failed')
    return op.Shape()


def _cut(a: TopoDS_Shape, b: TopoDS_Shape) -> TopoDS_Shape:
    op = BRepAlgoAPI_Cut(a, b)
    op.SimplifyResult()
    if not op.IsDone():
        raise GeometryError('boolean cut failed')
    return op.Shape()


def _cylinder(r: float, h: float, z0: float) -> TopoDS_Shape:
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, z0), gp_Dir(0, 0, 1)), r, h).Shape()


def plate_tjoint() -> Tuple[TopoDS_Shape, Dict]:
    """Base plate 200x150x10 mm with a 160x10x80 mm web: two 160 mm fillet seams."""
    plate = BRepPrimAPI_MakeBox(gp_Pnt(-0.100, -0.075, 0.0), gp_Pnt(0.100, 0.075, 0.010)).Shape()
    web = BRepPrimAPI_MakeBox(gp_Pnt(-0.080, -0.005, 0.010), gp_Pnt(0.080, 0.005, 0.090)).Shape()
    shape = _fuse(plate, web)
    common = {'joint_type': 'fillet', 'weld_side': 'bisector', 'leg_mm': 4.0,
              'process_hint': 'MIG', 'material': 'S235JR', 'thickness_mm': 10.0}
    picks = {'part_no': 'TEST-PLATE-T', 'part_rev': 'A', 'seams': [
        dict(common, seam_id='S01', sequence=1, pick_m=[0.0, 0.005, 0.010]),
        dict(common, seam_id='S02', sequence=2, pick_m=[0.0, -0.005, 0.010], reversed=True),
    ]}
    return shape, picks


def _on_circle(r: float, deg: float, z: float) -> list:
    return [r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg)), z]


def pipe_flange() -> Tuple[TopoDS_Shape, Dict]:
    """Flange D150x12 mm with a pipe D50x2.5 mm, 100 mm high: one circular fillet joint.

    A full 360 deg orbit is not reachable from a fixed UR5e base with the 45 deg torch (the
    robot-side arc puts the wrist over the base axis), so the joint is welded as three partial seams
    (A22 start / end): S01, S02 counter-clockwise -120 -> 60 -> 180 deg, S03 clockwise
    -120 -> 180 deg. Arc limits come from a reach map of the NOMINAL cell (PROGRESS.md,
    2026-10-19); arcs start where the approach pose is reachable.
    """
    flange = _cylinder(0.075, 0.012, 0.0)
    pipe = _cut(_cylinder(0.025, 0.100, 0.012), _cylinder(0.0225, 0.100, 0.012))
    shape = _fuse(flange, pipe)
    common = {'joint_type': 'fillet', 'weld_side': 'bisector', 'leg_mm': 3.0,
              'process_hint': 'MIG', 'material': 'S235JR', 'thickness_mm': 2.5,
              'pick_m': _on_circle(0.025, 90.0, 0.012)}
    arc = [  # (seam, sequence, reversed, start deg, end deg)
        ('S01', 1, False, -120.0, 60.0),
        ('S02', 2, False, 60.0, 180.0),
        ('S03', 3, True, -120.0, 180.0),
    ]
    picks = {'part_no': 'TEST-PIPE-FL', 'part_rev': 'A', 'seams': [
        dict(common, seam_id=sid, sequence=seq, reversed=rev,
             start_pick_m=_on_circle(0.025, a0, 0.012), end_pick_m=_on_circle(0.025, a1, 0.012))
        for sid, seq, rev, a0, a1 in arc]}
    return shape, picks


# Separate-solid weldments (CP-52): the solids touch but are not fused, as in a Fusion assembly.
_TUBE_R, _BORE_R = 0.013335, 0.010465  # golden-part tube section (test data)


def _assembly(*shapes: TopoDS_Shape) -> TopoDS_Shape:
    result = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(result)
    for shape in shapes:
        builder.Add(result, shape)
    return result


def _tube(z0: float, h: float) -> TopoDS_Shape:
    return _cut(_cylinder(_TUBE_R, h, z0), _cylinder(_BORE_R, h, z0))


def tube_rod() -> Tuple[TopoDS_Shape, Dict]:
    """Two solids: tube D26.67 x 2.87 mm (z -30..0 mm) butt against a rod D26.67 mm (z 0..40 mm).

    One joint edge: the outer circle at z = 0 (the bore circle has no partner edge on the rod).
    """
    return _assembly(_tube(-0.030, 0.030), _cylinder(_TUBE_R, 0.040, 0.0)), {
        'part_no': 'TEST-TUBE-ROD', 'part_rev': 'A', 'seams': []}


def tube_tube() -> Tuple[TopoDS_Shape, Dict]:
    """Two coaxial tubes D26.67 x 2.87 mm butt-jointed at z = 0: joint edges at the outer and bore circles."""
    return _assembly(_tube(-0.030, 0.030), _tube(0.0, 0.040)), {
        'part_no': 'TEST-TUBE-TUBE', 'part_rev': 'A', 'seams': []}


PARTS: Dict[str, Callable[[], Tuple[TopoDS_Shape, Dict]]] = {
    'plate_tjoint': plate_tjoint,
    'pipe_flange': pipe_flange,
    'tube_rod': tube_rod,
    'tube_tube': tube_tube,
}


def box(lo: Sequence[float], hi: Sequence[float]) -> TopoDS_Shape:
    """Axis-aligned box between two corners (metres, like every test part here)."""
    return BRepPrimAPI_MakeBox(gp_Pnt(*lo), gp_Pnt(*hi)).Shape()


def separate(*shapes: TopoDS_Shape) -> TopoDS_Shape:
    """Compound of separate (not fused) solids."""
    return _assembly(*shapes)


def write_assembly(path: Union[str, Path],
                   components: List[Tuple[str, List[TopoDS_Shape]]],
                   placements_mm: Optional[Sequence[Optional[Sequence[Sequence[float]]]]] = None,
                   instance_names: bool = False) -> None:
    """Write a STEP assembly (mm): one sub-assembly per ``(name, solids in metres)``, one part per solid.

    Part names are ``<name>-body<k>``. ``placements_mm`` (optional, one entry per component) gives the
    placement of each sub-assembly in the root as a 4x4 rigid transform with the translation in mm
    (``None`` = identity). ``instance_names``: name each instance ``<name>:<k>`` (k counts equal names) and
    each part instance ``<name>-body<j>:1``, as a Fusion export does. Used to test component reading
    (A15 / D20 / A18) without the cell CAD.
    """
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.Interface import Interface_Static
    from OCC.Core.STEPCAFControl import STEPCAFControl_Writer
    from OCC.Core.STEPControl import STEPControl_AsIs
    from OCC.Core.TCollection import TCollection_ExtendedString
    from OCC.Core.TDataStd import TDataStd_Name
    from OCC.Core.TDocStd import TDocStd_Document
    from OCC.Core.TopLoc import TopLoc_Location
    from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool

    doc = TDocStd_Document('XmlOcaf')
    tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    root = tool.NewShape()
    TDataStd_Name.Set(root, TCollection_ExtendedString('assembly'))
    seen: Dict[str, int] = {}
    for index, (name, solids) in enumerate(components):
        sub = tool.NewShape()
        TDataStd_Name.Set(sub, TCollection_ExtendedString(name))
        for k, solid in enumerate(solids, start=1):
            label = tool.AddShape(scale_shape(solid, units.m_to_mm(1.0)), False)
            TDataStd_Name.Set(label, TCollection_ExtendedString(f'{name}-body{k}'))
            body = tool.AddComponent(sub, label, TopLoc_Location())
            if instance_names:
                TDataStd_Name.Set(body, TCollection_ExtendedString(f'{name}-body{k}:1'))
        placement = placements_mm[index] if placements_mm else None
        inst = tool.AddComponent(root, sub, TopLoc_Location() if placement is None
                                 else TopLoc_Location(_trsf(placement)))
        if instance_names:
            seen[name] = seen.get(name, 0) + 1
            TDataStd_Name.Set(inst, TCollection_ExtendedString(f'{name}:{seen[name]}'))
    tool.UpdateAssemblies()
    Interface_Static.SetCVal('write.step.unit', 'MM')
    writer = STEPCAFControl_Writer()
    writer.SetNameMode(True)
    if not writer.Transfer(doc, STEPControl_AsIs) or writer.Write(str(path)) != IFSelect_RetDone:
        raise GeometryError(f'cannot write STEP assembly {path}')


def _trsf(matrix: Sequence[Sequence[float]]):
    """gp_Trsf from a 4x4 rigid transform (rotation rows + translation column, same length unit)."""
    from OCC.Core.gp import gp_Trsf

    m = [list(map(float, row)) for row in matrix]
    trsf = gp_Trsf()
    trsf.SetValues(m[0][0], m[0][1], m[0][2], m[0][3],
                   m[1][0], m[1][1], m[1][2], m[1][3],
                   m[2][0], m[2][1], m[2][2], m[2][3])
    return trsf


def weldment_assembly() -> List[Tuple[str, List[TopoDS_Shape]]]:
    """Small assembly: component ``weldment`` (tube + rod, one joint), ``other`` (a box) and two
    components both named ``dup`` (ambiguous name)."""
    return [
        ('weldment', [_tube(-0.030, 0.030), _cylinder(_TUBE_R, 0.040, 0.0)]),
        ('other', [box((0.100, 0.0, 0.0), (0.120, 0.020, 0.020))]),
        ('dup', [box((0.200, 0.0, 0.0), (0.210, 0.010, 0.010))]),
        ('dup', [box((0.300, 0.0, 0.0), (0.310, 0.010, 0.010))]),
    ]


def write_tree_assembly(path: Union[str, Path], root_name: str, nodes: List[Dict]) -> None:
    """Write a nested STEP assembly (mm) for component-tree tests (A18 head export, ADR-16).

    ``nodes``: list of ``{'name', 'shape' (TopoDS_Shape in mm, leaf) | 'children' (list of nodes),
    'placement_mm' (optional 4x4 rigid transform in the parent, translation in mm)}``. Each instance is named
    ``<name>:<k>`` (k counts equal names under one parent) and each product ``<name>``, as a Fusion export does.
    A leaf shape may be a solid or a compound of open faces (surface model).
    """
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.Interface import Interface_Static
    from OCC.Core.STEPCAFControl import STEPCAFControl_Writer
    from OCC.Core.STEPControl import STEPControl_AsIs
    from OCC.Core.TCollection import TCollection_ExtendedString
    from OCC.Core.TDataStd import TDataStd_Name
    from OCC.Core.TDocStd import TDocStd_Document
    from OCC.Core.TopLoc import TopLoc_Location
    from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool

    doc = TDocStd_Document('XmlOcaf')
    tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())

    def name(label, text: str) -> None:
        TDataStd_Name.Set(label, TCollection_ExtendedString(text))

    def add(parent, items: List[Dict]) -> None:
        seen: Dict[str, int] = {}
        for node in items:
            if 'children' in node:
                label = tool.NewShape()
                add(label, node['children'])
            else:
                label = tool.AddShape(node['shape'], False)
            name(label, node['name'])
            placement = node.get('placement_mm')
            inst = tool.AddComponent(parent, label, TopLoc_Location() if placement is None
                                     else TopLoc_Location(_trsf(placement)))
            seen[node['name']] = seen.get(node['name'], 0) + 1
            name(inst, f"{node['name']}:{seen[node['name']]}")

    root = tool.NewShape()
    name(root, root_name)
    add(root, nodes)
    tool.UpdateAssemblies()
    Interface_Static.SetCVal('write.step.unit', 'MM')
    writer = STEPCAFControl_Writer()
    writer.SetNameMode(True)
    if not writer.Transfer(doc, STEPControl_AsIs) or writer.Write(str(path)) != IFSelect_RetDone:
        raise GeometryError(f'cannot write STEP assembly {path}')
