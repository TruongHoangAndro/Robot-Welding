"""Cell collision scene from the Fusion cell CAD and its role map (WP-S S.4 A3; A15 / A18, ADR-16, DES-001 §5.3–5.4).

Inputs: the cell STEP assembly (D21) and a role map ``cad_map.yaml`` (data, never source: component names, roles,
frames). Outputs, all in mm:

* ``<repo>/cell/model/frames.yaml`` — ``world_T_file``, ``world_T_base``, ``world_T_positioner_axis``;
* ``<repo>/cell/model/<object_id>.stl`` — per mapped ``cell_static`` / ``positioner_rotating`` component, in the
  component's own frame: its convex hull (default) or, where the map's ``collision`` says ``mesh``, its closed
  tessellated mesh; ``scene_manifest.json`` lists them with pose, parent, hash, padding, collision mode. The folder
  is rewritten on every export (stale meshes removed);
* ``<repo>/jobs/<job>/`` — job import of the ``part`` component (model.step, part.json, edges.json, …), D20 view
  model in ``cache/view/``, collision mesh ``cache/collision.stl`` + ``cache/collision_faces.json`` (triangle →
  face_id), ``placement.json`` (v2 D19).

Frames (every one an **Assumption** until ``REF_*`` markers exist, OI-23):

* ``world``: the file frame rotated so that the map's file up axis becomes +Z (fixed per-axis rotation), origin at
  the ``ur_base`` component's placement origin.
* ``base`` (ur_description ``base_link``): at the ``ur_base`` placement; its axes are the UR component's own axes
  rotated like the file (the UR CAD is assumed to share the file's up axis, with its X along ``base_link`` X).
* ``positioner_flange`` at θ = 0 (= ``positioner_axis``): origin at the map's axis point, x = axis direction,
  z = world up made orthogonal to x, y = z × x.
* ``part``: the part component is read placed in the file frame (as ``import`` / ``export-view`` do), so the part
  frame is the file frame and ``flange_T_part = flange_T_file``. D20 / edges.json coordinates are in it.

Default deny: every component with geometry (solids or surfaces) must be covered by a map entry (itself or an
ancestor). Export guard: points sampled on the part collision mesh (spacing ``GUARD_SPACING_MM``, every vertex
included) must not lie deeper than the map's ``contact_tolerance_mm`` inside any cell object at θ = 0, measured
against the object's collision mesh; otherwise nothing is written and the object, part faces and depth are listed.
"""

from __future__ import annotations

import json
import logging
import os
import re
import shutil
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepTools import breptools
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCC.Core.TopoDS import TopoDS_Shape, topods

from awm_core import frames, jsonio, units
from awm_core.hashing import sha256_bytes, sha256_file
from awm_core.repository import Repository
from awm_geometry import assembly, hull, joints, meshops, miniyaml, topology, view
from awm_geometry.occ_io import GeometryError

LOG = logging.getLogger('awm_geometry')

MAP_SCHEMA = 'cad_map/0.1'
SCHEMA_VERSION = '2.0'  # header of the cell outputs (v2 set)
ROLES = ('part', 'positioner_rotating', 'cell_static', 'head_cad', 'ignore')
PARENT = {'cell_static': 'world', 'positioner_rotating': 'positioner_flange'}
ENTRY_KEYS = {'name', 'path', 'depth'}
#: Tessellation feeding the hulls. Chords lie inside convex surfaces, so a hull may be up to this much smaller
#: than the exact envelope — covered by the 3 mm padding (DES-001 §5.4).
HULL_LINEAR_DEFLECTION_MM = 0.2
HULL_ANGULAR_DEFLECTION_DEG = 15.0
MESH_LINEAR_DEFLECTION_MM = 0.5   # collision: mesh (S.4 A review: linear <= 0.5 mm)
MESH_ANGULAR_DEFLECTION_DEG = 15.0
MESH_WELD_MM = 1.0e-4             # coincident tessellation nodes of neighbouring faces are merged
COLLISION_MODES = ('hull', 'mesh')
RETRY_ANGULAR_DEG = (20.0, 30.0, 45.0, 60.0)  # face-alone retries of a face left empty (same chord)
DEFAULT_CONTACT_TOLERANCE_MM = 1.0  # cad_map default (Assumption, scene decision D8)
GUARD_SPACING_MM = 1.0            # sample spacing on the part surface for the export guard
PART_NEAR_ANGULAR_DEG = 5.0
PART_FAR_ANGULAR_DEG = 15.0
PART_PADDING_MM = 0.0  # the nozzle tip may touch the part; padding sits on the head links (B-04)
PART_IMPORT_LINEAR_DEFLECTION_M = 1.0e-4  # same as `cli import` default (mesh.stl in the part record)
PART_IMPORT_ANGULAR_DEFLECTION_DEG = 10.0
_ASSUMPTION = ('Assumption: from component placements of the cell CAD and the role map; no REF_* markers yet '
               '(OI-23)')

# Fixed rotation per file up axis that brings it to +Z (right-handed).
_UP_TO_Z = {'+Z': ([1, 0, 0], 0.0), '-Z': ([1, 0, 0], 180.0), '+Y': ([1, 0, 0], 90.0),
            '-Y': ([1, 0, 0], -90.0), '+X': ([0, 1, 0], -90.0), '-X': ([0, 1, 0], 90.0)}


class MapError(GeometryError):
    """The role map is invalid or does not cover the assembly."""


def object_id(name: str) -> str:
    """Scene object id from a component instance name: ``^[a-z][a-z0-9_]*$`` (``c_`` prefix if needed)."""
    oid = re.sub(r'[^a-z0-9]+', '_', name.lower()).strip('_')
    if not oid or not oid[0].isalpha():
        oid = f'c_{oid}'
    return oid


def world_R_file(up_axis: str) -> np.ndarray:
    if up_axis not in _UP_TO_Z:
        raise MapError(f'up axis {up_axis!r} not in {sorted(_UP_TO_Z)}')
    axis, deg = _UP_TO_Z[up_axis]
    rot = frames.axis_angle_to_matrix(axis, units.deg_to_rad(deg))
    return np.where(np.abs(rot) < 1e-15, 0.0, rot)


def pose_mm(t: np.ndarray) -> Dict[str, List[float]]:
    """4x4 (mm) -> ``{position_mm, quat_xyzw}`` (v2 ``pose_mm``)."""
    t = np.asarray(t, dtype=float)
    return {'position_mm': [float(v) for v in t[:3, 3]],
            'quat_xyzw': [float(v) for v in frames.matrix_to_quat(t[:3, :3])]}


def canonical_sha256(objects: List[Dict[str, Any]]) -> str:
    """``scene_sha256``: SHA-256 of the canonical JSON (sorted keys, no spaces) of the objects (mesh hashes in)."""
    return sha256_bytes(json.dumps(objects, sort_keys=True, separators=(',', ':')).encode())


# --- role map -------------------------------------------------------------------------------------------------

def load_map(path: Path, step: Path) -> Dict[str, Any]:
    """Read and check the role map against the STEP file (schema, match rule, default deny, SHA-256, roles)."""
    if not path.is_file():
        raise MapError(f'role map not found: {path}')
    cmap = miniyaml.load(path)
    try:
        jsonio.validate(cmap, 'v2/cad_map')
    except jsonio.SchemaValidationError as exc:
        raise MapError(f'{path}: {exc}') from exc
    if not isinstance(cmap, dict) or cmap.get('schema') != MAP_SCHEMA:
        raise MapError(f'{path}: schema must be {MAP_SCHEMA!r}')
    if cmap.get('match') != 'instance_name' or cmap.get('default') != 'deny':
        raise MapError(f'{path}: only "match: instance_name" and "default: deny" are supported')
    source = cmap.get('source') or {}
    if source.get('unit') != 'mm':
        raise MapError(f'{path}: source.unit must be mm')
    digest = sha256_file(step)
    if source.get('sha256') != digest:
        raise MapError(f'{path}: source SHA-256 {source.get("sha256")} does not match {step.name} ({digest})')
    world_R_file(source.get('up_axis'))
    unknown = set(cmap.get('roles') or {}) - set(ROLES)
    if unknown:
        raise MapError(f'{path}: unknown role(s) {sorted(unknown)}; allowed {list(ROLES)}')
    return cmap


def _rel_paths(tree: List[Dict[str, Any]]) -> List[str]:
    """Path of every node below its root (instance names, as in cell_components.md)."""
    rel: List[str] = []
    for node in tree:
        name = node['instance_name'] or node['product_name']
        parent = node['parent']
        if parent is None:
            rel.append('')
        else:
            rel.append(name if rel[parent] == '' else f'{rel[parent]} / {name}')
    return rel


def _find(tree, rel, crit: Dict[str, Any]) -> List[int]:
    return [n['index'] for n in tree if n['parent'] is not None
            and ('name' not in crit or n['instance_name'] == crit['name'])
            and ('path' not in crit or rel[n['index']] == crit['path'])
            and ('depth' not in crit or n['depth'] == crit['depth'])]


def resolve_roles(tree: List[Dict[str, Any]], cmap: Dict[str, Any]) -> Dict[int, Tuple[str, str]]:
    """``{node index: (role, entry label)}`` for every node named by the map; default deny for the rest."""
    rel = _rel_paths(tree)
    direct: Dict[int, Tuple[str, str]] = {}
    problems: List[str] = []
    for role, entries in (cmap.get('roles') or {}).items():
        for entry in entries or []:
            if isinstance(entry, str):
                crit = {'name': entry}
            elif isinstance(entry, dict) and set(entry) <= ENTRY_KEYS and ({'name', 'path'} & set(entry)):
                crit = dict(entry)
            else:
                problems.append(f'{role}: bad entry {entry!r} (a name, or a mapping with name / path [+ depth])')
                continue
            hits = _find(tree, rel, crit)
            if len(hits) != 1:
                where = '; '.join(rel[i] for i in hits[:5])
                problems.append(f'{role}: {entry!r} matches no component' if not hits else
                                f'{role}: {entry!r} matches {len(hits)} components ({where}); add path / depth')
                continue
            if hits[0] in direct:
                problems.append(f'{role}: {entry!r} is already mapped as {direct[hits[0]][0]}')
                continue
            direct[hits[0]] = (role, rel[hits[0]])
    covered: Dict[int, str] = {}
    for node in tree:
        i, parent = node['index'], node['parent']
        inherited = covered.get(parent) if parent is not None else None
        if i in direct:
            if inherited:
                problems.append(f'{rel[i]!r} is mapped as {direct[i][0]} inside a component mapped as {inherited}')
            covered[i] = direct[i][0]
        elif inherited:
            covered[i] = inherited
    unmapped = [rel[n['index']] for n in tree
                if not n['is_assembly'] and n['bbox_mm'] is not None and n['index'] not in covered]
    if unmapped:
        problems.append(f'{len(unmapped)} component(s) with geometry not mapped (default deny): '
                        + '; '.join(unmapped[:20]) + (' …' if len(unmapped) > 20 else ''))
    if problems:
        raise MapError('role map does not fit the assembly:\n  ' + '\n  '.join(problems))
    return direct


# --- frames ---------------------------------------------------------------------------------------------------

def cell_frames(tree: List[Dict[str, Any]], cmap: Dict[str, Any]) -> Dict[str, Any]:
    """world_T_file, world_T_base, world_T_flange (θ = 0) as 4x4 (mm) plus the axis in world."""
    fr = cmap.get('frames') or {}
    if (fr.get('world') or {}).get('origin_from') != 'ur_base':
        raise MapError('frames.world.origin_from must be ur_base (the only rule until REF_WORLD exists)')
    ur_name = (fr.get('ur_base') or {}).get('component')
    hits = [n for n in tree if n['parent'] is not None and n['instance_name'] == ur_name]
    if len(hits) != 1:
        raise MapError(f'frames.ur_base.component {ur_name!r} matches {len(hits)} components (expected 1)')
    file_T_ur = np.asarray(hits[0]['placement_mm'], dtype=float)
    if not np.isfinite(file_T_ur).all():
        raise MapError(f'placement of {ur_name!r} is not finite')
    rot = world_R_file(cmap['source']['up_axis'])
    world_T_file = frames.make_transform(-rot @ file_T_ur[:3, 3], rot)
    world_T_base = world_T_file @ file_T_ur @ frames.make_transform([0, 0, 0], rot.T)
    axis = fr.get('positioner_axis') or {}
    point = np.asarray(axis.get('point_file_mm'), dtype=float)
    direction = np.asarray(axis.get('direction_file'), dtype=float)
    if point.shape != (3,) or direction.shape != (3,):
        raise MapError('frames.positioner_axis needs point_file_mm and direction_file (3 numbers each)')
    if not (np.isfinite(point).all() and np.isfinite(direction).all()):
        raise MapError('frames.positioner_axis: point_file_mm and direction_file must be finite')
    if abs(np.linalg.norm(direction) - 1.0) > 1e-6:
        raise MapError(f'frames.positioner_axis.direction_file {direction.tolist()} must have unit length')
    p_w = (world_T_file @ np.append(point, 1.0))[:3]
    x = rot @ direction / np.linalg.norm(direction)
    up = np.array([0.0, 0.0, 1.0])
    z = up - (up @ x) * x
    if np.linalg.norm(z) < 1e-6:
        raise MapError('positioner axis is vertical: flange frame z (world up) undefined')
    z /= np.linalg.norm(z)
    world_T_flange = frames.make_transform(p_w, np.column_stack([x, np.cross(z, x), z]))
    return {'world_T_file': world_T_file, 'world_T_base': world_T_base, 'world_T_flange': world_T_flange,
            'axis_point_mm': p_w, 'axis_direction': x, 'ur_component': ur_name}


def write_frames(path: Path, fr: Dict[str, Any], step: Path, map_path: Path, cmap: Dict[str, Any],
                 created_by: str, base_dir: Path) -> None:
    """``frames.yaml`` with the full header; ``inputs`` are relative to ``base_dir`` (the final model folder)."""
    doc = dict(jsonio.make_header(created_by, [step, map_path], base_dir), schema_version=SCHEMA_VERSION)
    doc.update({
        'schema': 'awm_cell_frames/0.1',
        'unit': 'mm',
        'source': {'file': step.name, 'sha256': sha256_file(step), 'map': map_path.name,
                   'map_sha256': sha256_file(map_path), 'file_up_axis': cmap['source']['up_axis']},
        'world_T_file': dict(pose_mm(fr['world_T_file']),
                             note='world = file rotated so that the file up axis is +Z, origin at the ur_base '
                                  'component origin'),
        'world_T_base': dict(pose_mm(fr['world_T_base']), component=fr['ur_component'],
                             note=f'{_ASSUMPTION}. world -> ur_description base_link: the UR component placement '
                                  'is taken as base_link; the UR CAD is assumed to share the file up axis, with its '
                                  'X along base_link X'),
        'world_T_positioner_axis': dict(pose_mm(fr['world_T_flange']),
                                        point_mm=[float(v) for v in fr['axis_point_mm']],
                                        direction=[float(v) for v in fr['axis_direction']],
                                        note=f'{_ASSUMPTION}. positioner_flange at theta = 0; x = axis '
                                             'direction, z = world up orthogonal to x'),
    })
    path.parent.mkdir(parents=True, exist_ok=True)
    miniyaml.dump(doc, path, header='Cell frames for the TST-002 spike (generated by awm_geometry export-scene; '
                                    'do not edit). Lengths mm, quaternions xyzw.')


# --- meshes ---------------------------------------------------------------------------------------------------

def _face_triangles(face) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], float]:
    """(positions, triangles 0-based, deflection) of a meshed face in the shape unit; (None, None, 0) if unmeshed."""
    loc = TopLoc_Location()
    tri = BRep_Tool.Triangulation(face, loc)
    if tri is None or tri.NbTriangles() == 0:
        return None, None, 0.0
    trsf = loc.Transformation()
    pos = np.array([tri.Node(i).Transformed(trsf).Coord() for i in range(1, tri.NbNodes() + 1)], dtype=float)
    idx = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)], dtype=np.int64) - 1
    if face.Orientation() == TopAbs_REVERSED:
        idx = idx[:, [0, 2, 1]]
    return pos, idx, float(tri.Deflection())


def _tessellate_mm(shape: TopoDS_Shape, linear_mm: float, angular_deg: float, label: str,
                   retried: Optional[List[Dict[str, Any]]] = None) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Triangles (file frame, mm, not welded) per solid of a component shape read from the STEP (session unit
    mm); a component without solids (surfaces only) gives one entry for all its faces."""
    breptools.Clean(shape)
    mesher = BRepMesh_IncrementalMesh(shape, linear_mm, False, units.deg_to_rad(angular_deg), True)
    mesher.Perform()
    if not mesher.IsDone():
        raise GeometryError('tessellation failed')
    parts = []
    solids = TopExp_Explorer(shape, TopAbs_SOLID)
    while solids.More():
        parts.append(solids.Current())
        solids.Next()
    out = []
    face_no = 0
    for piece in parts or [shape]:
        verts, tris, offset = [], [], 0
        exp = TopExp_Explorer(piece, TopAbs_FACE)
        while exp.More():
            face_no += 1
            face = topods.Face(exp.Current())
            pos, idx, _ = _face_triangles(face)
            for retry_deg in RETRY_ANGULAR_DEG if pos is None else ():
                # BRepMesh sometimes leaves a small valid face empty (cell CAD: a 6 mm2 cone); mesh it alone
                # with the same chord and a coarser angle (accepted only if the achieved chord holds). Logged; an error
                # if every retry fails.
                breptools.Clean(face)
                BRepMesh_IncrementalMesh(face, linear_mm, False, units.deg_to_rad(retry_deg), False).Perform()
                pos, idx, deflection = _face_triangles(face)
                if pos is not None and deflection > linear_mm:
                    pos = idx = None  # the chord must still hold
                if pos is not None:
                    LOG.warning('%r: face %d meshed again alone (angular %.0f deg, chord %.2f mm, achieved '
                                '%.3f mm)', label, face_no, retry_deg, linear_mm, deflection)
                    if retried is not None:
                        retried.append({'face': face_no, 'angular_deflection_deg': float(retry_deg),
                                        'achieved_chord_mm': float(deflection)})
                    break
            if pos is None:
                raise GeometryError(f'{label!r}: face {face_no} has no triangulation: its collision geometry '
                                    'would have a hole')
            verts.append(pos)
            tris.append(idx + offset)
            offset += len(pos)
            exp.Next()
        if verts:
            out.append((np.vstack(verts), np.vstack(tris)))
    if not out:
        raise GeometryError('component has no tessellated face')
    return out


def closed_shell(verts: np.ndarray, tris: np.ndarray, label: str, k: int) -> Tuple[np.ndarray, np.ndarray]:
    """Weld one solid's tessellation into a closed, outward shell; raise if it is open or inside-out."""
    v, t = meshops.weld(verts, tris, MESH_WELD_MM)  # welded per solid: touching solids are not merged
    if not meshops.is_closed(t):
        raise GeometryError(f'tessellated mesh of {label!r} (solid {k}) is not closed: the export guard cannot '
                            'test inside / outside (use collision: hull or fix the CAD)')
    vol = meshops.volume(v, t)
    if not vol > 0.0:
        raise GeometryError(f'tessellated mesh of {label!r} (solid {k}) is inside-out (volume {vol:.3f} mm3)')
    return v, t


def _solid_volume_mm3(shape: TopoDS_Shape) -> float:
    props = GProp_GProps()
    brepgprop.VolumeProperties(shape, props)
    return abs(float(props.Mass()))


def part_collision_mesh(shape_m: TopoDS_Shape, chord_near_mm: float, chord_far_mm: float
                        ) -> Tuple[np.ndarray, np.ndarray, List[Dict[str, Any]]]:
    """Part collision mesh in mm with the chord ``chord_near_mm`` on the faces at joint edges and
    ``chord_far_mm`` elsewhere. Returns ``(vertices, triangles, face records)``; triangles are grouped per face in
    ``F<n>`` order (D20 ids), each record ``{face_id, first_triangle, triangle_count, near_joint, deflection_mm}``.
    """
    faces = topology.index_map(shape_m, TopAbs_FACE)
    ancestors = TopTools_IndexedDataMapOfShapeListOfShape()
    topexp.MapShapesAndAncestors(shape_m, TopAbs_EDGE, TopAbs_FACE, ancestors)
    near = set()
    for joint in joints.find_joint_edges(joints.solids_of(shape_m), units.mm_to_m(view.JOINT_TOL_MM)):
        for edge in (joint.edge_a, joint.edge_b):
            near.update(topology.edge_face_ids(ancestors, faces, edge))
    breptools.Clean(shape_m)
    for i in range(1, faces.Size() + 1):
        if f'F{i}' in near:
            BRepMesh_IncrementalMesh(faces.FindKey(i), units.mm_to_m(chord_near_mm), False,
                                     units.deg_to_rad(PART_NEAR_ANGULAR_DEG), True).Perform()
    mesher = BRepMesh_IncrementalMesh(shape_m, units.mm_to_m(chord_far_mm), False,
                                      units.deg_to_rad(PART_FAR_ANGULAR_DEG), True)
    mesher.Perform()
    if not mesher.IsDone():
        raise GeometryError('tessellation failed')
    verts, tris, records = [], [], []
    n_verts = n_tris = 0
    for i in range(1, faces.Size() + 1):
        pos, idx, deflection = _face_triangles(topods.Face(faces.FindKey(i)))
        count = 0 if idx is None else len(idx)
        if count == 0:
            raise GeometryError(f'part face F{i} has no triangulation: the collision mesh would have a hole')
        else:
            verts.append(units.m_to_mm(pos))
            tris.append(idx + n_verts)
            n_verts += len(pos)
        records.append({'face_id': f'F{i}', 'first_triangle': n_tris, 'triangle_count': count,
                        'near_joint': f'F{i}' in near, 'deflection_mm': float(units.m_to_mm(deflection))})
        n_tris += count
    return np.vstack(verts), np.vstack(tris), records


# --- export ---------------------------------------------------------------------------------------------------

def collision_modes(direct, tree, cmap) -> Dict[int, str]:
    """Collision mode per cell object (``hull`` default); every map entry must name a mapped cell object."""
    names = {tree[i]['instance_name']: i for i, (role, _) in direct.items() if role in PARENT}
    modes = {i: 'hull' for i in names.values()}
    problems = []
    for name, mode in (cmap.get('collision') or {}).items():
        if name not in names:
            problems.append(f'collision: {name!r} is not a cell_static / positioner_rotating component of the map')
        elif mode not in COLLISION_MODES:
            problems.append(f'collision: {name!r} mode {mode!r} not in {COLLISION_MODES}')
        else:
            modes[names[name]] = mode
    if problems:
        raise MapError('\n  '.join(['role map collision entries:'] + problems))
    return modes


def _cell_objects(tree, direct, modes, fr, padding_mm: float) -> List[Dict[str, Any]]:
    """Scene objects with their meshes in memory (``_verts`` in the component frame, ``_tris``, ``_file_T``)."""
    flange_T_file = frames.invert(fr['world_T_flange']) @ fr['world_T_file']
    parent_T_file = {'world': fr['world_T_file'], 'positioner_flange': flange_T_file}
    objects, ids = [], set()
    for index, (role, label) in sorted(direct.items()):
        if role not in PARENT:
            continue
        node = tree[index]
        oid = object_id(node['instance_name'] or node['product_name'])
        if oid in ids:
            raise MapError(f'object id {oid!r} of {label!r} is not unique')
        ids.add(oid)
        file_T_comp = np.asarray(node['placement_mm'], dtype=float)
        mode = modes[index]
        extra: Dict[str, Any] = {}
        retried: List[Dict[str, Any]] = []
        if mode == 'hull':
            pieces = _tessellate_mm(node['_shape'], HULL_LINEAR_DEFLECTION_MM, HULL_ANGULAR_DEFLECTION_DEG, label,
                                    retried)
            pts = np.vstack([v for v, _ in pieces])
            local = (pts - file_T_comp[:3, 3]) @ file_T_comp[:3, :3]  # inverse of a rigid placement
            verts, tris = hull.convex_hull(local)
            diag = float(np.linalg.norm(local.max(axis=0) - local.min(axis=0)))
            if not hull.contains(verts, tris, local, 1e-6 * diag + 1e-9).all():
                raise GeometryError(f'hull of {label!r} does not contain all its tessellation nodes')
            solid = _solid_volume_mm3(node['_shape'])
            ratio = meshops.volume(verts, tris) / solid if solid > 0 else None
            extra['hull_volume_ratio'] = ratio
            LOG.info('A18 %s (%s, %s): hull %d vertices, %d triangles, hull / solid volume %s', oid, label, role,
                     len(verts), len(tris), 'n/a (no solid)' if ratio is None else f'{ratio:.2f}')
        else:
            verts_list, tris_list, offset = [], [], 0
            for k, (pts, raw) in enumerate(_tessellate_mm(node['_shape'], MESH_LINEAR_DEFLECTION_MM,
                                                          MESH_ANGULAR_DEFLECTION_DEG, label, retried), start=1):
                local = (pts - file_T_comp[:3, 3]) @ file_T_comp[:3, :3]
                v, t = closed_shell(local, raw, label, k)
                verts_list.append(v)
                tris_list.append(t + offset)
                offset += len(v)
            verts, tris = np.vstack(verts_list), np.vstack(tris_list)
            LOG.info('A18 %s (%s, %s): mesh %d vertices, %d triangles, %d closed shell(s)', oid, label, role,
                     len(verts), len(tris), len(verts_list))
        if retried:
            extra['retried_faces'] = retried
        parent = PARENT[role]
        obj = {'object_id': oid, 'role': role, 'component': label, 'parent_frame': parent,
               'pose_mm': pose_mm(parent_T_file[parent] @ file_T_comp), 'collision': mode,
               'mesh': {'file': f'{oid}.stl', 'sha256': None, 'face_ids': False,
                        'vertices': int(len(verts)), 'triangles': int(len(tris))},
               'padding_mm': float(padding_mm), **extra,
               '_verts': verts, '_tris': tris, '_file_T': file_T_comp}
        objects.append(obj)
    return objects


def export_guard(objects, part_verts, part_tris, face_records, tolerance_mm: float) -> List[Dict[str, Any]]:
    """Penetration of the part surface into every object (part frame = file frame, θ = 0).

    Sets ``part_max_depth_mm`` on each object; raises if any sample is deeper than ``tolerance_mm``.
    """
    samples, owner = meshops.sample_surface(part_verts, part_tris, GUARD_SPACING_MM)
    firsts = np.array([r['first_triangle'] for r in face_records])
    face_of = np.array([r['face_id'] for r in face_records])[np.searchsorted(firsts, owner, side='right') - 1]
    problems, table = [], []
    for obj in objects:
        t = obj['_file_T']
        verts_file = obj['_verts'] @ t[:3, :3].T + t[:3, 3]
        depth = meshops.penetration(verts_file, obj['_tris'], samples)
        deepest = float(depth.max()) if len(depth) else 0.0
        obj['part_max_depth_mm'] = deepest
        faces = sorted(set(face_of[depth > 0].tolist()), key=lambda f: int(f[1:]))
        table.append({'object_id': obj['object_id'], 'max_depth_mm': deepest, 'part_faces': faces})
        if deepest > tolerance_mm:
            deep = sorted(set(face_of[depth > tolerance_mm].tolist()), key=lambda f: int(f[1:]))
            problems.append(f'{obj["object_id"]} ({obj["component"]}, {obj["collision"]}): part points up to '
                            f'{deepest:.3f} mm deep, deeper than {tolerance_mm} mm on faces {", ".join(deep)}')
        LOG.info('guard %s: max part depth %.3f mm%s', obj['object_id'], deepest,
                 f' (faces {", ".join(faces)})' if faces else '')
    if problems:
        raise GeometryError('export refused, part inside cell objects (contact_tolerance_mm '
                            f'{tolerance_mm}):\n  ' + '\n  '.join(problems))
    return table


def _job_from_name(repo: Repository, job_name: str) -> Tuple[str, str]:
    if '_' not in job_name:
        raise GeometryError(f'job {job_name!r} must be <PartNo>_<PartRev>')
    part_no, part_rev = job_name.rsplit('_', 1)
    try:
        if repo.job_id(part_no, part_rev) != job_name:
            raise ValueError(job_name)
    except ValueError as exc:
        raise GeometryError(f'job {job_name!r} is not a valid job id: {exc}') from exc
    return part_no, part_rev


STAGING_PREFIX = '.staging-export-scene-'


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def remove_orphaned_staging(root: Path) -> List[Path]:
    """Remove staging folders of earlier runs whose process is gone (or of this pid); return them."""
    removed = []
    if root.is_dir():
        for path in root.glob(f'{STAGING_PREFIX}*'):
            suffix = path.name[len(STAGING_PREFIX):]
            if not suffix.isdigit():
                continue
            pid = int(suffix)
            if pid == os.getpid() or not _pid_alive(pid):
                shutil.rmtree(path, ignore_errors=True)
                removed.append(path)
                LOG.warning('removed orphaned staging folder %s', path)
    return removed


def _keep_foreign_files(old_dir: Path, staged_dir: Path) -> None:
    """Copy files of other modules (in the old job folder, not produced by this export) into the staged folder."""
    if not old_dir.is_dir():
        return
    for src in old_dir.rglob('*'):
        if src.is_file():
            dst = staged_dir / src.relative_to(old_dir)
            if not dst.exists():
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dst)


def swap_dirs(pairs: List[Tuple[Path, Path]]) -> None:
    """Put each staged folder in place of its final folder by renames: old aside, staged in, old deleted.

    If any rename fails, every step already done is undone (staged folders are removed, old folders restored).
    """
    done: List[Dict[str, Any]] = []
    try:
        for staged, final in pairs:
            step: Dict[str, Any] = {'final': final, 'old': None, 'placed': False}
            done.append(step)
            final.parent.mkdir(parents=True, exist_ok=True)
            if final.exists():
                step['old'] = final.with_name(f'.{final.name}.old-{os.getpid()}')
                os.replace(final, step['old'])
            os.replace(staged, final)
            step['placed'] = True
    except BaseException:
        for step in reversed(done):
            if step['placed']:
                shutil.rmtree(step['final'], ignore_errors=True)
            if step['old'] is not None and step['old'].exists():
                os.replace(step['old'], step['final'])
        raise
    for step in done:
        if step['old'] is not None:
            shutil.rmtree(step['old'], ignore_errors=True)


def guard_record(tolerance: float, chord_near_mm: float, chord_far_mm: float) -> Dict[str, Any]:
    """``export_guard`` block of the manifest (v2/cell_scene; scene decision D9)."""
    return {
        'contact_tolerance_mm': tolerance, 'theta_deg': 0.0, 'sample_spacing_mm': GUARD_SPACING_MM,
        'part_chord_near_mm': float(chord_near_mm), 'part_chord_far_mm': float(chord_far_mm),
        'object_chord_hull_mm': HULL_LINEAR_DEFLECTION_MM, 'object_chord_mesh_mm': MESH_LINEAR_DEFLECTION_MM,
        'method': 'part surface samples (all vertices + longest-edge bisection to the sample spacing) inside the '
                  'object collision mesh (winding number); depth = distance to the mesh; positioner at theta = 0',
        'note': 'sampled check: the true overlap can exceed contact_tolerance_mm by up to about the sample spacing '
                'plus the part and object chords; static objects vs the rotating part are checked only at '
                'theta = 0 (other angles: scene check at theta, S.4 Part C)',
    }


def export_scene(step: Path, map_path: Path, repo_root: Path, job_name: str, chord_near_mm: float,
                 chord_far_mm: float, padding_mm: float, created_by: str) -> Dict[str, Any]:
    """Write the cell part and the job part of the spike scene (see module docstring); return a summary.

    Everything is computed and checked first (map, frames, meshes, model checks, export guard); then every output
    is written into a staging folder of the repository (job import, D20 export and their own checks included)
    and only moved into ``cell/model`` and ``jobs/<job>`` when all of it succeeded.
    """
    from awm_geometry import cli, part

    if not step.is_file():
        raise GeometryError(f'STEP file not found: {step}')
    if not 0 < chord_near_mm <= chord_far_mm:
        raise GeometryError('need 0 < chord-near-mm <= chord-far-mm')
    repo = Repository(repo_root)
    part_no, part_rev = _job_from_name(repo, job_name)
    remove_orphaned_staging(repo.root)
    cmap = load_map(map_path, step)
    tolerance = float(cmap.get('contact_tolerance_mm', DEFAULT_CONTACT_TOLERANCE_MM))
    LOG.info('A18 reading assembly %s', step)
    tree, _unit = assembly.component_tree(step, cylinders=False, shapes=True)
    direct = resolve_roles(tree, cmap)
    parts = [(i, label) for i, (role, label) in direct.items() if role == 'part']
    if len(parts) != 1:
        raise MapError(f'the map must name exactly one part component, found {len(parts)}')
    modes = collision_modes(direct, tree, cmap)
    fr = cell_frames(tree, cmap)
    objects = _cell_objects(tree, direct, modes, fr, padding_mm)
    part_node = tree[parts[0][0]]
    component = part_node['instance_name'] or part_node['product_name']
    roles: Dict[str, int] = {}
    for role, _label in direct.values():
        roles[role] = roles.get(role, 0) + 1
    del tree, part_node  # release the cell shapes before the part read

    LOG.info('A11 reading part component %r', component)
    loaded = part.load_part(step, component)
    failed = [c for c in loaded.checks if not c.ok and c.name in ('brep_valid', 'has_solid')]
    if failed:
        raise GeometryError('model check failed: ' + '; '.join(f'{c.name}: {c.detail}' for c in failed))
    verts, tris, face_records = part_collision_mesh(loaded.shape, chord_near_mm, chord_far_mm)
    guard = export_guard(objects, verts, tris, face_records, tolerance)

    # --- all checks passed: write into a staging folder, then move into place --------------------------------
    model_dir = repo.cell_dir / 'model'
    job_final = repo.job(part_no, part_rev)
    staging_root = repo.root / f'{STAGING_PREFIX}{os.getpid()}'
    if staging_root.exists():
        shutil.rmtree(staging_root)
    stage = Repository(staging_root)
    stage_model = stage.cell_dir / 'model'
    stage_model.mkdir(parents=True)
    try:
        for obj in objects:
            mesh = stage_model / obj['mesh']['file']
            hull.write_stl(mesh, obj.pop('_verts'), obj.pop('_tris'))
            obj.pop('_file_T')
            obj['mesh']['sha256'] = sha256_file(mesh)
        write_frames(stage_model / 'frames.yaml', fr, step, map_path, cmap, created_by, model_dir)
        manifest = dict(jsonio.make_header(created_by, [step, map_path], model_dir), schema_version=SCHEMA_VERSION)
        manifest.update({
            'unit': 'mm', 'frames_file': 'frames.yaml',
            'hull_tessellation': {'linear_deflection_mm': HULL_LINEAR_DEFLECTION_MM,
                                  'angular_deflection_deg': HULL_ANGULAR_DEFLECTION_DEG},
            'mesh_tessellation': {'linear_deflection_mm': MESH_LINEAR_DEFLECTION_MM,
                                  'angular_deflection_deg': MESH_ANGULAR_DEFLECTION_DEG,
                                  'weld_mm': MESH_WELD_MM},
            'export_guard': guard_record(tolerance, chord_near_mm, chord_far_mm),
            'note': f'{_ASSUMPTION}. Meshes in the component frame; pose = parent_T_component; positioner_flange '
                    'objects at theta = 0; collision = hull (convex hull, hull_volume_ratio = hull / solid volume) '
                    'or mesh (closed tessellation)',
            'objects': objects, 'scene_sha256': canonical_sha256(objects),
        })
        jsonio.write_json(stage_model / 'scene_manifest.json', manifest, 'v2/cell_scene')

        job = stage.job(part_no, part_rev)
        cli.copy_step(step, job)
        breptools.Clean(loaded.shape)
        record = cli.write_part_record(job, step.name, loaded, part_no, part_rev,
                                       PART_IMPORT_LINEAR_DEFLECTION_M, PART_IMPORT_ANGULAR_DEFLECTION_DEG)
        breptools.Clean(loaded.shape)
        view_summary = view.export(loaded, job.model_step, component, cmap['source']['up_axis'],
                                   job.cache_dir / 'view', created_by, job.dir)
        collision = job.cache_dir / 'collision.stl'
        hull.write_stl(collision, verts, tris)
        faces_doc = jsonio.make_header(created_by, [job.model_step, collision], job.dir)
        faces_doc.update({'schema_version': '2.0', 'unit': 'mm', 'frame': 'part', 'mesh': 'cache/collision.stl',
                          'component': component, 'triangles': int(len(tris)),
                          'chord_near_mm': float(chord_near_mm), 'chord_far_mm': float(chord_far_mm),
                          'angular_near_deg': PART_NEAR_ANGULAR_DEG, 'angular_far_deg': PART_FAR_ANGULAR_DEG,
                          'faces': face_records})
        faces_json = job.cache_dir / 'collision_faces.json'
        faces_json.write_text(json.dumps(faces_doc, indent=1) + '\n')

        flange_T_part = frames.invert(fr['world_T_flange']) @ fr['world_T_file']
        part_objects = [{'object_id': 'part', 'role': 'part', 'parent_frame': 'positioner_flange',
                         'pose_mm': pose_mm(flange_T_part),
                         'mesh': {'file': 'cache/collision.stl', 'sha256': sha256_file(collision),
                                  'face_ids': True},
                         'padding_mm': PART_PADDING_MM}]
        # the cell manifest is an input: a new cell with an old job (or the reverse) fails the hash check
        placement = jsonio.make_header(created_by, [job.model_step, collision, faces_json,
                                                    stage_model / 'scene_manifest.json'], job.dir)
        placement.update({'schema_version': '2.0', 'source': 'fixture_cad', 'verified': False,
                          'flange_T_part': pose_mm(flange_T_part), 'objects': part_objects,
                          'scene_sha256': canonical_sha256(part_objects)})
        jsonio.write_json(job.dir / 'placement.json', placement, 'v2/placement')

        _keep_foreign_files(job_final.dir, job.dir)
        swap_dirs([(stage_model, model_dir), (job.dir, job_final.dir)])
    finally:
        shutil.rmtree(staging_root, ignore_errors=True)

    near = [r for r in face_records if r['near_joint']]
    return {
        'cell_model': str(model_dir), 'job': str(job_final.dir), 'mapped_components_per_role': roles,
        'objects': {o['object_id']: {'collision': o['collision'], 'triangles': o['mesh']['triangles'],
                                     'hull_volume_ratio': o.get('hull_volume_ratio'),
                                     'part_max_depth_mm': o['part_max_depth_mm']} for o in objects},
        'guard': guard,
        'retried_faces': sum(len(o.get('retried_faces', [])) for o in objects),
        'part': {'component': component, 'triangles': int(len(tris)), 'faces': len(face_records),
                 'near_joint_faces': len(near), 'edges': record['edges'], 'joints': len(view_summary['joints'])},
        'world_T_base': pose_mm(fr['world_T_base']),
        'world_T_positioner_axis': dict(pose_mm(fr['world_T_flange']),
                                        direction=[float(v) for v in fr['axis_direction']]),
        'flange_T_part': pose_mm(flange_T_part),
    }
