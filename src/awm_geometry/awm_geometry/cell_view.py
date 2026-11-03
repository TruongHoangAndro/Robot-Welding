"""Cell display model ``cell_view.glb`` for the S10-lite role editor (contract
``docs/generated/contracts/S10_lite_cell_model.md`` v0.1, CP-54).

One glTF node per leaf component of ``cell_components.json`` (``inspect-assembly``), named ``C<index>`` with
``extras = {index, instance_name, path}``; positions in the file frame, mm; coarse tessellation (display only,
never used for collision or paths).
"""

from __future__ import annotations

import json
import logging
import os
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepTools import breptools
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Shape, topods

from awm_core import units
from awm_core.hashing import sha256_file
from awm_geometry import assembly
from awm_geometry.glb import GlbBuilder
from awm_geometry.occ_io import GeometryError
from awm_geometry.scene import _face_triangles

LOG = logging.getLogger('awm_geometry')

SCHEMA = 'cell_view'
SCHEMA_VERSION = '0.1'
UP_AXES = ('+X', '-X', '+Y', '-Y', '+Z', '-Z')
LINEAR_DEFLECTION_MM = 2.0   # contract v0.1 (2026-11-03): linear <= 2.0 mm, angular <= 30 deg, < 15 MB
ANGULAR_DEFLECTION_DEG = 30.0


def _mesh(shape: TopoDS_Shape):
    """(positions mm, unit normals, uint32 indices) of a shape in the session unit (mm); None if no triangles.

    Normals are area-weighted triangle normals accumulated per face (creases between faces stay sharp).
    """
    breptools.Clean(shape)
    BRepMesh_IncrementalMesh(shape, LINEAR_DEFLECTION_MM, False,
                             units.deg_to_rad(ANGULAR_DEFLECTION_DEG), True).Perform()
    pos_all, nor_all, idx_all, offset = [], [], [], 0
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        pos, idx, _ = _face_triangles(topods.Face(exp.Current()))
        exp.Next()
        if idx is None:
            continue
        tri_n = np.cross(pos[idx[:, 1]] - pos[idx[:, 0]], pos[idx[:, 2]] - pos[idx[:, 0]])
        nor = np.zeros_like(pos)
        for k in range(3):
            np.add.at(nor, idx[:, k], tri_n)
        norm = np.linalg.norm(nor, axis=1)
        nor = np.where(norm[:, None] > 0, nor / np.where(norm > 0, norm, 1.0)[:, None], [0.0, 0.0, 1.0])
        pos_all.append(pos)
        nor_all.append(nor)
        idx_all.append(idx + offset)
        offset += len(pos)
    if not idx_all:
        return None
    return np.vstack(pos_all), np.vstack(nor_all), np.vstack(idx_all).astype(np.uint32)


def export(step: Path, components_json: Path, out_dir: Path, up_axis: str, created_by: str) -> Dict[str, Any]:
    """Write ``out_dir/cell_view.glb``; return a summary. The components JSON must belong to ``step``."""
    if up_axis not in UP_AXES:
        raise GeometryError(f'up axis {up_axis!r} not in {UP_AXES}')
    doc = json.loads(Path(components_json).read_text())
    digest = sha256_file(step)
    if doc.get('source', {}).get('sha256') != digest:
        raise GeometryError(f'{components_json} was made from another file (SHA-256 '
                            f'{doc.get("source", {}).get("sha256")} vs {step.name} {digest})')
    tree, _unit = assembly.component_tree(step, cylinders=False, shapes=True)
    listed: List[Dict[str, Any]] = doc['components']
    if [(c['index'], c['path']) for c in listed] != [(c['index'], c['path']) for c in tree]:
        raise GeometryError(f'{components_json} does not list the components of {step.name} in the same order')
    glb = GlbBuilder({'unit': 'mm', 'frame': 'file', 'schema': SCHEMA, 'schema_version': SCHEMA_VERSION,
                      'up_axis': up_axis}, created_by)
    nodes = triangles = empty = 0
    for node in tree:
        if node['is_assembly']:
            continue
        name = f'C{node["index"]}'
        mesh = _mesh(node['_shape'])
        mesh_index = None
        if mesh is None:
            empty += 1
            LOG.warning('%s (%s) has no triangles: node without mesh', name, node['path'])
        else:
            mesh_index = glb.add_triangles(name, *mesh)
            triangles += len(mesh[2])
        glb.add_node(name, None, mesh_index, extras={'index': node['index'],
                                                    'instance_name': node['instance_name'],
                                                    'path': node['path']})
        nodes += 1
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / 'cell_view.glb'
    tmp = out_dir / f'.cell_view.glb.{os.getpid()}.tmp'
    tmp.write_bytes(glb.to_bytes())
    tmp.replace(path)
    return {'cell_view_glb': str(path), 'nodes': nodes, 'nodes_without_mesh': empty, 'triangles': triangles,
            'bytes': path.stat().st_size}
