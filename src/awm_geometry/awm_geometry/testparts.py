"""Generated test parts for the end-to-end chain (no customer STEP files are available).

Each part comes with a *pick request*: the seam picks a user would make in the Workbench (a point
near the seam edge plus the drawing attributes). The picks are test data, not source constants.
"""

from __future__ import annotations

import math
from typing import Callable, Dict, Tuple

from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt

from awm_geometry.occ_io import GeometryError


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


PARTS: Dict[str, Callable[[], Tuple[TopoDS_Shape, Dict]]] = {
    'plate_tjoint': plate_tjoint,
    'pipe_flange': pipe_flange,
}
