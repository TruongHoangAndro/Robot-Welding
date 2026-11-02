"""Read components of a STEP assembly with XCAF (A18 / A15 input, DES-001 ADR-16).

The Fusion cell CAD is one STEP assembly; a job part (for example the golden part) is one component
of it. This module finds a component by name and returns its solids placed in the file's frame,
with no unit conversion (OCCT session unit = millimetres).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple, Union

from OCC.Core.BRep import BRep_Builder
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
from OCC.Core.TDF import TDF_Label, TDF_LabelSequence
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.TopAbs import TopAbs_SOLID
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Shape
from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool

from awm_geometry.occ_io import GeometryError, file_length_unit

PathLike = Union[str, Path]


@dataclass
class Component:
    """One assembly component placed in the file frame."""

    path: str
    shape: TopoDS_Shape
    #: (solid, name) for every solid below the component; name = nearest instance or product name.
    solids: List[Tuple[TopoDS_Shape, str]] = field(default_factory=list)
    #: Length unit declared in the STEP file ('mm', 'm', 'inch', 'unknown'); shapes are in mm regardless.
    unit: str = 'unknown'


def _name(label: TDF_Label) -> str:
    """Label name; '' for none and for the automatic reference names ``=>[0:1:...]`` of XCAF."""
    text = label.GetLabelName() or ''
    return '' if text.startswith('=>[') else text


def _open(path: Path):
    """Read a STEP file into an XCAF document; return ``(document, shape tool, file length unit)``.

    Callers keep the document referenced while they use the tool's labels.
    """
    if not path.is_file():
        raise GeometryError(f'STEP file not found: {path}')
    doc = TDocStd_Document('XmlOcaf')
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone or not reader.Transfer(doc):
        raise GeometryError(f'cannot read STEP assembly {path}')
    tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    return doc, tool, file_length_unit(reader.Reader())


def _named_solids(tool, label: TDF_Label, location: TopLoc_Location,
                  parent_name: str) -> List[Tuple[TopoDS_Shape, str]]:
    """Solids below ``label`` placed with ``location``, each with its nearest non-empty name."""
    ref = label
    if tool.IsReference(label):
        ref = TDF_Label()
        tool.GetReferredShape(label, ref)
        location = location.Multiplied(tool.GetLocation(label))
    name = _name(label) or _name(ref) or parent_name
    if tool.IsAssembly(ref):
        out: List[Tuple[TopoDS_Shape, str]] = []
        children = TDF_LabelSequence()
        tool.GetComponents(ref, children)
        for i in range(1, children.Length() + 1):
            out.extend(_named_solids(tool, children.Value(i), location, name))
        return out
    out = []
    exp = TopExp_Explorer(tool.GetShape(ref).Moved(location), TopAbs_SOLID)
    while exp.More():
        out.append((exp.Current(), name))
        exp.Next()
    return out


def read_components(step: PathLike, name: str) -> List[Component]:
    """All components whose instance or product name equals ``name`` (instance suffix ``:n`` ignored).

    The search stops at the first match on each branch. Raises if the file cannot be read.
    """
    path = Path(step)
    _doc, tool, unit = _open(path)
    found: List[Component] = []

    def matches(text: str) -> bool:
        return text == name or text.rsplit(':', 1)[0] == name

    def walk(label: TDF_Label, location: TopLoc_Location, trail: List[str]) -> None:
        parent_location = location
        ref = label
        if tool.IsReference(label):
            ref = TDF_Label()
            tool.GetReferredShape(label, ref)
            location = location.Multiplied(tool.GetLocation(label))
        here = _name(label) or _name(ref)
        trail = trail + [here]
        if matches(_name(label)) or matches(_name(ref)):
            found.append(Component(' / '.join(trail), tool.GetShape(ref).Moved(location),
                                   _named_solids(tool, label, parent_location, ''), unit))
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


def read_part(step: PathLike, name: Optional[str]) -> Component:
    """The part to export: the single component called ``name``, or the whole file if ``name`` is None.

    Raises if the component is missing or the name is ambiguous (no silent choice between instances).
    """
    path = Path(step)
    if name is not None:
        found = read_components(path, name)
        if len(found) != 1:
            raise GeometryError(f'{len(found)} components named {name!r} in {path} (expected 1)'
                                + ('' if not found else ': ' + '; '.join(c.path for c in found)))
        return found[0]
    _doc, tool, unit = _open(path)
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    shapes, solids = [], []
    for i in range(1, roots.Length() + 1):
        shapes.append(tool.GetShape(roots.Value(i)))
        solids.extend(_named_solids(tool, roots.Value(i), TopLoc_Location(), ''))
    if not shapes:
        raise GeometryError(f'STEP file {path} contains no shape')
    return Component(path.name, compound(shapes), solids, unit)


def compound(shapes: List[TopoDS_Shape]) -> TopoDS_Compound:
    """One compound of the given shapes (solids stay separate)."""
    result = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(result)
    for shape in shapes:
        builder.Add(result, shape)
    return result
