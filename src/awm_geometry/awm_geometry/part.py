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
from typing import List, Optional, Union

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


def _solids(shape: TopoDS_Shape) -> List[TopoDS_Shape]:
    out = []
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        out.append(exp.Current())
        exp.Next()
    return out


def load_part(step: PathLike, component: Optional[str] = None) -> LoadedPart:
    """Read ``step`` (whole file, or the single assembly component ``component``), scale to metres, heal.

    Whole file: :func:`occ_io.read_step` (unchanged catalogue behaviour); solid names are not available
    from that reader and stay ''. Component: :func:`assembly.read_part`; solid names are mapped through
    the scaling and healing history, and a name that cannot be mapped stays '' (logged).
    """
    path = Path(step)
    named: List[tuple] = []
    if component is None:
        shape, unit = occ_io.read_step(path)
    else:
        comp = assembly.read_part(path, component)
        unit = comp.unit
        op = occ_io.scale_shape_tracked(comp.shape, occ_io.MM_TO_M)
        shape = op.Shape()
        originals = _solids(comp.shape)
        for solid, name in comp.solids:
            match = [s for s in originals if s.IsSame(solid)] or \
                [s for s in originals if s.IsPartner(solid)]
            try:
                named.append((op.ModifiedShape(match[0]), name) if match else (None, name))
            except Exception:  # noqa: BLE001 - OCC raises Standard_NoSuchObject: name unmapped
                named.append((None, name))
    history: list = []
    shape, healed, checks = topology.check_and_heal(shape, unit, history)
    for ctx in history:
        named = [(None if s is None else ctx.Value(s), n) for s, n in named]
    solids = topology.index_map(shape, TopAbs_SOLID)
    names = []
    for i in range(1, solids.Size() + 1):
        solid = solids.FindKey(i)
        hit = [n for s, n in named if s is not None and s.IsSame(solid)]
        names.append(hit[0] if hit else '')
    if component is None:
        LOG.info('solid names are not read for a whole-file import (no component given)')
    elif '' in names:
        LOG.warning('no name found for solid(s) %s after scaling / healing',
                    ', '.join(f'B{i + 1}' for i, n in enumerate(names) if not n))
    return LoadedPart(shape, unit, healed, checks, names)
