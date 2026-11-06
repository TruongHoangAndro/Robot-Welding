"""Circle pick on the exact B-Rep (CP-60 positioner axis, CP-61 nozzle mouth).

The browser picks an edge id of a D20 view; this module re-loads the same shape with the same arguments
(:func:`awm_geometry.part.load_part`, D20 rule 5), checks that the edge lies on a circle and returns its centre,
axis and radius in the file frame (mm). An arc of a circle is accepted (same circle); any other curve is refused.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GeomAbs import GeomAbs_Circle
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.TopoDS import TopoDS_Edge, topods

from awm_core import jsonio, units
from awm_core.hashing import sha256_file
from awm_geometry import part, topology
from awm_geometry.occ_io import GeometryError

SCHEMA = 'v2/axis_pick'


def circle_frame(edge: TopoDS_Edge) -> Dict[str, Any]:
    """``{center_mm, axis, radius_mm}`` of a circular edge of a loaded shape (metres); GeometryError otherwise."""
    curve = BRepAdaptor_Curve(edge)
    if curve.GetType() != GeomAbs_Circle:
        kind = topology._CURVE_NAMES.get(curve.GetType(), 'other')
        raise GeometryError(f'edge is not a circle ({kind})')
    circ = curve.Circle()
    loc, d = circ.Location(), circ.Axis().Direction()
    center = units.m_to_mm(np.array([loc.X(), loc.Y(), loc.Z()], dtype=float))
    axis = np.array([d.X(), d.Y(), d.Z()], dtype=float)
    return {'center_mm': [float(v) + 0.0 for v in center], 'axis': [float(v) + 0.0 for v in axis / np.linalg.norm(axis)],
            'radius_mm': float(units.m_to_mm(circ.Radius()))}


def edge_by_id(shape, edge_id: str) -> TopoDS_Edge:
    """Edge ``E<n>`` of a loaded shape (same index map as the D20 export)."""
    if not edge_id.startswith('E') or not edge_id[1:].isdigit():
        raise GeometryError(f'bad edge id {edge_id!r} (expected E<n>)')
    edges = topology.index_map(shape, TopAbs_EDGE)
    k = int(edge_id[1:])
    if not 1 <= k <= edges.Size():
        raise GeometryError(f'edge {edge_id} not in the model ({edges.Size()} edges)')
    return topods.Edge(edges.FindKey(k))


def pick(step: Path, edge_id: str, up_axis: str, out: Path, created_by: str, component: Optional[str] = None,
         components: bool = False, reverse: bool = False) -> Dict[str, Any]:
    """Write ``axis_pick.json`` (``v2/axis_pick``, file frame, mm) for edge ``edge_id`` of ``step``."""
    loaded = part.load_part(step, component, components=components)
    try:
        frame = circle_frame(edge_by_id(loaded.shape, edge_id))
    except GeometryError as exc:
        raise GeometryError(f'{edge_id}: {exc}') from exc
    axis = [-v + 0.0 for v in frame['axis']] if reverse else frame['axis']
    out.parent.mkdir(parents=True, exist_ok=True)
    doc = jsonio.make_header(created_by, [step], out.parent)
    doc.update({'schema_version': '2.0', 'source': {'file': step.name, 'sha256': sha256_file(step),
                                                    'up_axis': up_axis},
                'component_path': component, 'edge_id': edge_id, 'center_file_mm': frame['center_mm'],
                'axis_file': axis, 'radius_mm': frame['radius_mm'], 'reversed': bool(reverse), 'method': 'pick'})
    jsonio.write_json(out, doc, SCHEMA)
    return {'out': str(out), 'edge_id': edge_id, 'center_file_mm': frame['center_mm'], 'axis_file': axis,
            'radius_mm': frame['radius_mm']}
