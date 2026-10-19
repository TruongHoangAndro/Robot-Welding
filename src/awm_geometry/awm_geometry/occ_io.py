"""STEP / BREP / STL input-output with explicit unit handling (A11, A13).

OCCT returns STEP geometry in its session length unit (millimetres). The demo assumed metres
(DES-004 finding on units), and ``xstep.cascade.unit`` reads back empty in OCCT 8.0, so this module
scales explicitly: every shape that leaves ``read_step`` is in metres, every shape written to STEP
by ``write_step_mm`` is scaled back to millimetres.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Tuple, Union

from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepTools import breptools
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.Interface import Interface_Static
from OCC.Core.STEPControl import STEPControl_AsIs, STEPControl_Reader, STEPControl_Writer
from OCC.Core.StlAPI import StlAPI_Writer
from OCC.Core.TColStd import TColStd_SequenceOfAsciiString
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.gp import gp_Pnt, gp_Trsf

PathLike = Union[str, Path]
LOG = logging.getLogger('awm_geometry')

MM_TO_M = 1.0e-3


class GeometryError(RuntimeError):
    """A geometry operation failed (bad file, invalid shape, ...)."""


def scale_shape(shape: TopoDS_Shape, factor: float) -> TopoDS_Shape:
    """Uniform scale about the origin (geometry is copied)."""
    trsf = gp_Trsf()
    trsf.SetScale(gp_Pnt(0.0, 0.0, 0.0), factor)
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


def _file_length_unit(reader: STEPControl_Reader) -> str:
    names = TColStd_SequenceOfAsciiString()
    angles = TColStd_SequenceOfAsciiString()
    solid = TColStd_SequenceOfAsciiString()
    try:
        reader.FileUnits(names, angles, solid)
    except Exception:  # noqa: BLE001 - informative only
        return 'unknown'
    units = {names.Value(i).ToCString().lower() for i in range(1, names.Length() + 1)}
    if not units:
        return 'unknown'
    if len(units) > 1:
        return 'unknown'
    unit = units.pop()
    if unit in ('millimetre', 'millimeter', 'mm'):
        return 'mm'
    if unit in ('metre', 'meter', 'm'):
        return 'm'
    if unit in ('inch', 'in'):
        return 'inch'
    return 'unknown'


def read_step(path: PathLike) -> Tuple[TopoDS_Shape, str]:
    """Read a STEP file; return ``(shape in metres, unit declared in the file)``."""
    path = Path(path)
    if not path.is_file():
        raise GeometryError(f'STEP file not found: {path}')
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(path))
    if status != IFSelect_RetDone:
        raise GeometryError(f'cannot read STEP file {path} (status {status})')
    unit = _file_length_unit(reader)
    if reader.TransferRoots() == 0:
        raise GeometryError(f'STEP file {path} contains no transferable shape')
    shape = reader.OneShape()
    if shape is None or shape.IsNull():
        raise GeometryError(f'STEP file {path} produced a null shape')
    # The OCCT session unit is millimetres (STEP reader default); convert to SI.
    return scale_shape(shape, MM_TO_M), unit


def write_step_mm(shape_m: TopoDS_Shape, path: PathLike) -> None:
    """Write a shape given in metres as a STEP AP214 file in millimetres."""
    Interface_Static.SetCVal('write.step.unit', 'MM')
    Interface_Static.SetCVal('write.step.schema', 'AP214IS')
    writer = STEPControl_Writer()
    if writer.Transfer(scale_shape(shape_m, 1.0 / MM_TO_M), STEPControl_AsIs) != IFSelect_RetDone:
        raise GeometryError('STEP transfer failed')
    if writer.Write(str(path)) != IFSelect_RetDone:
        raise GeometryError(f'cannot write STEP file {path}')


def write_brep(shape: TopoDS_Shape, path: PathLike) -> None:
    if not breptools.Write(shape, str(path)):
        raise GeometryError(f'cannot write BREP file {path}')


def read_brep(path: PathLike) -> TopoDS_Shape:
    shape = TopoDS_Shape()
    if not breptools.Read(shape, str(path), BRep_Builder()):
        raise GeometryError(f'cannot read BREP file {path}')
    return shape


def tessellate_to_stl(shape: TopoDS_Shape, path: PathLike, linear_deflection_m: float,
                      angular_deflection_rad: float) -> int:
    """Mesh the shape (A13) and write a binary STL in metres. Return the triangle count."""
    mesher = BRepMesh_IncrementalMesh(shape, linear_deflection_m, False, angular_deflection_rad, True)
    mesher.Perform()
    if not mesher.IsDone():
        raise GeometryError('tessellation failed')
    writer = StlAPI_Writer()
    writer.SetASCIIMode(False)
    if not writer.Write(shape, str(path)):
        raise GeometryError(f'cannot write STL {path}')
    data = Path(path).read_bytes()
    return int.from_bytes(data[80:84], 'little') if len(data) >= 84 else 0
