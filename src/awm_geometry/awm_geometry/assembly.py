"""Read components of a STEP assembly with XCAF (A18 / A15 input, DES-001 ADR-16).

The Fusion cell CAD is one STEP assembly; a job part (for example the golden part) is one component
of it. This module finds a component by name and returns its solids placed in the file's frame,
with no unit conversion (OCCT session unit = millimetres).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union

from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.GeomAbs import GeomAbs_Cylinder
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
from OCC.Core.TDF import TDF_Label, TDF_LabelSequence
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Shape, topods
from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool
from OCC.Core.gp import gp_Trsf

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


# --- Component tree (inspect-assembly, input to the A18 CAD map, ADR-16) -------------------------

def _matrix(trsf: gp_Trsf) -> List[List[float]]:
    """4x4 homogeneous matrix of a transform (rows; translation in the session unit, mm)."""
    rows = [[trsf.Value(r, c) for c in range(1, 5)] for r in range(1, 4)]
    return rows + [[0.0, 0.0, 0.0, 1.0]]


def _box(shape: TopoDS_Shape) -> Optional[Bnd_Box]:
    """Bounding box of a placed shape (exact B-Rep bounds, no gap); None if the shape is empty."""
    box = Bnd_Box()
    box.SetGap(0.0)
    brepbndlib.AddOptimal(shape, box, False, False)
    return None if box.IsVoid() else box


def _box_dict(box: Optional[Bnd_Box]) -> Optional[Dict[str, List[float]]]:
    if box is None:
        return None
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return {'min': [xmin, ymin, zmin], 'max': [xmax, ymax, zmax]}


def _count_solids(shape: TopoDS_Shape) -> int:
    n = 0
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        n += 1
        exp.Next()
    return n


def main_cylinder(shape: TopoDS_Shape) -> Optional[Dict[str, object]]:
    """Axis of the largest-area cylindrical face of a placed shape (file frame), or None.

    Used to find rotation axes (for example a positioner shaft) in an assembly without markers.
    ``point_mm`` is the cylinder's axis location; the axis sign is the face's own (not oriented).
    """
    from awm_geometry.topology import face_area

    best = None
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = topods.Face(exp.Current())
        exp.Next()
        surf = BRepAdaptor_Surface(face)
        if surf.GetType() != GeomAbs_Cylinder:
            continue
        area = face_area(face)
        if best is None or area > best[0]:
            best = (area, surf.Cylinder())
    if best is None:
        return None
    area, cyl = best
    ax = cyl.Axis()
    d, p = ax.Direction(), ax.Location()
    return {'axis': [d.X(), d.Y(), d.Z()], 'point_mm': [p.X(), p.Y(), p.Z()],
            'radius_mm': cyl.Radius(), 'area_mm2': area}


def component_tree(step: PathLike, cylinders: bool = True) -> Tuple[List[Dict[str, object]], str]:
    """Every node of the assembly tree in depth-first order (parents first), placed in the file frame.

    Returns ``(components, file length unit)``. Each component dict has: ``index``, ``parent`` (index
    or None), ``depth``, ``path`` (names joined by `` / ``), ``instance_name``, ``product_name``,
    ``is_assembly``, ``children``, ``local_placement_mm`` and ``placement_mm`` (4x4, translation in
    mm, file frame), ``bbox_mm`` (``{min, max}`` or None), ``solids`` and, for leaf parts when
    ``cylinders`` is set, ``main_cylinder`` (see :func:`main_cylinder`). No unit conversion: OCCT
    reads STEP into its session unit (mm).
    """
    path = Path(step)
    _doc, tool, unit = _open(path)
    out: List[Dict[str, object]] = []
    boxes: List[Optional[Bnd_Box]] = []

    def walk(label: TDF_Label, location: TopLoc_Location, parent: Optional[int],
             depth: int, trail: List[str]) -> None:
        ref, local = label, TopLoc_Location()
        instance = ''
        if tool.IsReference(label):
            ref = TDF_Label()
            tool.GetReferredShape(label, ref)
            local = tool.GetLocation(label)
            location = location.Multiplied(local)
            instance = _name(label)
        product = _name(ref)
        trail = trail + [instance or product]
        index = len(out)
        node: Dict[str, object] = {
            'index': index, 'parent': parent, 'depth': depth, 'path': ' / '.join(trail),
            'instance_name': instance, 'product_name': product,
            'is_assembly': bool(tool.IsAssembly(ref)), 'children': 0,
            'local_placement_mm': _matrix(local.Transformation()),
            'placement_mm': _matrix(location.Transformation()),
            'bbox_mm': None, 'solids': 0}
        out.append(node)
        boxes.append(None)
        if node['is_assembly']:
            children = TDF_LabelSequence()
            tool.GetComponents(ref, children)
            node['children'] = children.Length()
            box = Bnd_Box()
            for i in range(1, children.Length() + 1):
                child = len(out)
                walk(children.Value(i), location, index, depth + 1, trail)
                node['solids'] += out[child]['solids']
                if boxes[child] is not None:
                    box.Add(boxes[child])
            boxes[index] = None if box.IsVoid() else box
        else:
            shape = tool.GetShape(ref).Moved(location)
            node['solids'] = _count_solids(shape)
            boxes[index] = _box(shape)
            if cylinders:
                node['main_cylinder'] = main_cylinder(shape)
        node['bbox_mm'] = _box_dict(boxes[index])

    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), None, 0, [])
    if not out:
        raise GeometryError(f'STEP file {path} contains no shape')
    return out, unit
