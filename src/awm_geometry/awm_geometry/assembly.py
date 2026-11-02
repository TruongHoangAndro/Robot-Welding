"""Read components of a STEP assembly with XCAF (A18 / A15 input, DES-001 ADR-16).

The Fusion cell CAD is one STEP assembly; a job part (for example the golden part) is one component
of it. This module finds a component by name and returns its solids placed in the file's frame,
with no unit conversion (OCCT session unit = millimetres).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Union

from OCC.Core.BRep import BRep_Builder
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
from OCC.Core.TDF import TDF_Label, TDF_LabelSequence
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Shape
from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool

from awm_geometry.occ_io import GeometryError

PathLike = Union[str, Path]


@dataclass
class Component:
    """One assembly component placed in the file frame."""

    path: str
    shape: TopoDS_Shape


def _name(label: TDF_Label) -> str:
    return label.GetLabelName() or ''


def read_components(step: PathLike, name: str) -> List[Component]:
    """All components whose instance or product name equals ``name`` (instance suffix ``:n`` ignored).

    The search stops at the first match on each branch. Raises if the file cannot be read.
    """
    path = Path(step)
    if not path.is_file():
        raise GeometryError(f'STEP file not found: {path}')
    doc = TDocStd_Document('XmlOcaf')
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone or not reader.Transfer(doc):
        raise GeometryError(f'cannot read STEP assembly {path}')
    tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    found: List[Component] = []

    def matches(text: str) -> bool:
        return text == name or text.rsplit(':', 1)[0] == name

    def walk(label: TDF_Label, location: TopLoc_Location, trail: List[str]) -> None:
        ref = label
        if tool.IsReference(label):
            ref = TDF_Label()
            tool.GetReferredShape(label, ref)
            location = location.Multiplied(tool.GetLocation(label))
        here = _name(label) or _name(ref)
        trail = trail + [here]
        if matches(_name(label)) or matches(_name(ref)):
            found.append(Component(' / '.join(trail), tool.GetShape(ref).Moved(location)))
            return
        if tool.IsAssembly(ref):
            children = TDF_LabelSequence()
            tool.GetComponents(ref, children)
            for i in range(1, children.Length() + 1):
                walk(children.Value(i), location, trail)

    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    return found


def compound(shapes: List[TopoDS_Shape]) -> TopoDS_Compound:
    """One compound of the given shapes (solids stay separate)."""
    result = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(result)
    for shape in shapes:
        builder.Add(result, shape)
    return result
