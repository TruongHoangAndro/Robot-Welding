"""Shared load + heal of a part (A11 / A12) for the A21 edge catalogue and the D20 view model.

D20 contract rule 5: ``E<n>`` / ``F<n>`` / ``B<n>`` of ``edges.json`` (``cli import``) and of ``topology.json``
(``cli export-view``) must be equal, and so must the fingerprints. Both commands therefore take their shape from
:func:`load_part`: the same reader, the same scaling to metres (the unit of the existing catalogue, see
:mod:`awm_geometry.occ_io`) and the same healing. D20 converts its outputs back to mm through ``awm_core.units``.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple, Union

from OCC.Core.TopAbs import TopAbs_SOLID
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Shape

from awm_geometry import assembly, occ_io, topology

LOG = logging.getLogger('awm_geometry')
PathLike = Union[str, Path]


@dataclass
class LoadedPart:
    """Healed part shape in metres with its checks and the names of its solids."""

    shape: TopoDS_Shape
    unit: str
    healed: bool
    checks: List[topology.CheckResult]
    #: Name per solid, index = B id - 1 (map order of the healed shape); '' if unknown.
    solid_names: List[str]
    #: Components mode (CP-60): inspect-assembly index of the leaf component per solid (None if unmapped) and the
    #: component path below the root per index; both None for a single component or a plain whole-file read.
    solid_components: Optional[List[Optional[int]]] = None
    component_paths: Optional[List[str]] = None


def _solids(shape: TopoDS_Shape) -> List[TopoDS_Shape]:
    out = []
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        out.append(exp.Current())
        exp.Next()
    return out


def _scaled(shape_mm: TopoDS_Shape, tagged: List[tuple]) -> Tuple[TopoDS_Shape, List[tuple]]:
    """Scale to metres; map every ``(solid, tag)`` to its scaled solid (None if the history loses it)."""
    op = occ_io.scale_shape_tracked(shape_mm, occ_io.MM_TO_M)
    originals = _solids(shape_mm)
    out = []
    for solid, tag in tagged:
        match = [s for s in originals if s.IsSame(solid)] or [s for s in originals if s.IsPartner(solid)]
        try:
            out.append((op.ModifiedShape(match[0]), tag) if match else (None, tag))
        except Exception:  # noqa: BLE001 - OCC raises Standard_NoSuchObject: name unmapped
            out.append((None, tag))
    return op.Shape(), out


def _read_components(path: Path) -> Tuple[TopoDS_Shape, List[tuple], str, List[str]]:
    """Whole file per assembly leaf (CP-60): compound of the placed leaf shapes (mm), ``(solid, (name, index))``
    for every solid, the file unit and the component path of every tree node (index of inspect-assembly)."""
    from awm_core.cad_roles import rel_paths

    tree, unit = assembly.component_tree(path, cylinders=False, shapes=True)
    leaves = [n for n in tree if not n['is_assembly']]
    tagged = []
    for node in leaves:
        for solid in _solids(node['_shape']):
            tagged.append((solid, (node['instance_name'] or node['product_name'], node['index'])))
    if not leaves:
        raise occ_io.GeometryError(f'STEP file {path} contains no shape')
    return assembly.compound([n['_shape'] for n in leaves]), tagged, unit, rel_paths(tree)


def load_part(step: PathLike, component: Optional[str] = None, components: bool = False) -> LoadedPart:
    """Read ``step`` (whole file, or the single assembly component ``component``), scale to metres, heal.

    Whole file: :func:`occ_io.read_step` (unchanged catalogue behaviour); solid names are not available
    from that reader and stay ''. Component: :func:`assembly.read_part`; solid names are mapped through
    the scaling and healing history, and a name that cannot be mapped stays '' (logged). ``components``
    (CP-60): the whole file read per assembly leaf, so every solid also knows its component (index and path);
    shells without a solid (surface models) are kept. Every reader of one job must use the same arguments so that
    the B / F / E ids agree (D20 rule 5).
    """
    path = Path(step)
    tagged: List[tuple] = []
    paths: Optional[List[str]] = None
    if components and component is not None:
        raise occ_io.GeometryError('give either a component or the components mode, not both')
    if components:
        shape_mm, raw, unit, paths = _read_components(path)
        shape, tagged = _scaled(shape_mm, raw)
    elif component is None:
        shape, unit = occ_io.read_step(path)
    else:
        comp = assembly.read_part(path, component)
        unit = comp.unit
        shape, tagged = _scaled(comp.shape, [(s, (n, None)) for s, n in comp.solids])
    history: list = []
    shape, healed, checks = topology.check_and_heal(shape, unit, history)
    for ctx in history:
        tagged = [(None if s is None else ctx.Value(s), t) for s, t in tagged]
    solids = topology.index_map(shape, TopAbs_SOLID)
    tags = []
    for i in range(1, solids.Size() + 1):
        solid = solids.FindKey(i)
        hit = [t for s, t in tagged if s is not None and s.IsSame(solid)]
        tags.append(hit[0] if hit else ('', None))
    names = [t[0] for t in tags]
    if component is None and not components:
        LOG.info('solid names are not read for a whole-file import (no component given)')
    elif '' in names:
        LOG.warning('no name found for solid(s) %s after scaling / healing',
                    ', '.join(f'B{i + 1}' for i, n in enumerate(names) if not n))
    if not components:
        return LoadedPart(shape, unit, healed, checks, names)
    return LoadedPart(shape, unit, healed, checks, names, [t[1] for t in tags], paths)
