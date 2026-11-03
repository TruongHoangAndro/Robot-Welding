"""Cell collision scene from the Fusion cell CAD and its role map (WP-S S.4 A3; A15 / A18, ADR-16, DES-001 §5.3–5.4).

Inputs: the cell STEP assembly (D21) and a role map ``cad_map.yaml`` (data, never source: component names, roles,
frames). Outputs, all in mm:

* ``<repo>/cell/model/frames.yaml`` — ``world_T_file``, ``world_T_base``, ``world_T_positioner_axis``;
* ``<repo>/cell/model/<object_id>.stl`` — convex hull per mapped ``cell_static`` / ``positioner_rotating``
  component, in the component's own frame; ``scene_manifest.json`` lists them with pose, parent, hash, padding;
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

Default deny: every component that carries solids must be covered by a map entry (itself or an ancestor).
"""

from __future__ import annotations

import json
import logging
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepTools import breptools
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
from OCC.Core.TopoDS import TopoDS_Shape, topods

from awm_core import frames, jsonio, units
from awm_core.hashing import sha256_bytes, sha256_file
from awm_core.repository import JobPaths, Repository
from awm_geometry import assembly, hull, joints, miniyaml, topology, view
from awm_geometry.occ_io import GeometryError

LOG = logging.getLogger('awm_geometry')

MAP_SCHEMA = 'cad_map/0.1'
ROLES = ('part', 'positioner_rotating', 'cell_static', 'head_cad', 'ignore')
PARENT = {'cell_static': 'world', 'positioner_rotating': 'positioner_flange'}
ENTRY_KEYS = {'name', 'path', 'depth'}
#: Tessellation feeding the hulls. Chords lie inside convex surfaces, so a hull may be up to this much smaller
#: than the exact envelope — covered by the 3 mm padding (DES-001 §5.4).
HULL_LINEAR_DEFLECTION_MM = 0.2
HULL_ANGULAR_DEFLECTION_DEG = 15.0
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
                if not n['is_assembly'] and n['solids'] > 0 and n['index'] not in covered]
    if unmapped:
        problems.append(f'{len(unmapped)} component(s) with solids not mapped (default deny): '
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
    rot = world_R_file(cmap['source']['up_axis'])
    world_T_file = frames.make_transform(-rot @ file_T_ur[:3, 3], rot)
    world_T_base = world_T_file @ file_T_ur @ frames.make_transform([0, 0, 0], rot.T)
    axis = fr.get('positioner_axis') or {}
    point = np.asarray(axis.get('point_file_mm'), dtype=float)
    direction = np.asarray(axis.get('direction_file'), dtype=float)
    if point.shape != (3,) or direction.shape != (3,) or np.linalg.norm(direction) == 0:
        raise MapError('frames.positioner_axis needs point_file_mm and direction_file (3 numbers each)')
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


def write_frames(path: Path, fr: Dict[str, Any], step: Path, map_path: Path, cmap: Dict[str, Any]) -> None:
    doc = {
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
    }
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


def shape_nodes_mm(shape: TopoDS_Shape) -> np.ndarray:
    """Tessellation nodes (file frame, mm) of a component shape read from the STEP (session unit mm)."""
    breptools.Clean(shape)
    mesher = BRepMesh_IncrementalMesh(shape, HULL_LINEAR_DEFLECTION_MM, False,
                                      units.deg_to_rad(HULL_ANGULAR_DEFLECTION_DEG), True)
    mesher.Perform()
    if not mesher.IsDone():
        raise GeometryError('tessellation failed')
    chunks = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        pos, _, _ = _face_triangles(topods.Face(exp.Current()))
        if pos is not None:
            chunks.append(pos)
        exp.Next()
    if not chunks:
        raise GeometryError('component has no tessellated face')
    return np.vstack(chunks)


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
            LOG.warning('F%d has no triangulation (degenerate face): no collision triangles', i)
        else:
            verts.append(units.m_to_mm(pos))
            tris.append(idx + n_verts)
            n_verts += len(pos)
        records.append({'face_id': f'F{i}', 'first_triangle': n_tris, 'triangle_count': count,
                        'near_joint': f'F{i}' in near, 'deflection_mm': float(units.m_to_mm(deflection))})
        n_tris += count
    return np.vstack(verts), np.vstack(tris), records


# --- export ---------------------------------------------------------------------------------------------------

def _cell_objects(tree, direct, fr, model_dir: Path, padding_mm: float) -> List[Dict[str, Any]]:
    flange_T_file = frames.invert(fr['world_T_flange']) @ fr['world_T_file']
    parent_T_file = {'world': fr['world_T_file'], 'positioner_flange': flange_T_file}
    objects, ids = [], set()
    for index, (role, label) in sorted(direct.items()):
        if role not in PARENT:
            continue
        node = tree[index]
        name = node['instance_name'] or node['product_name']
        oid = object_id(name)
        if oid in ids:
            raise MapError(f'object id {oid!r} of {label!r} is not unique')
        ids.add(oid)
        file_T_comp = np.asarray(node['placement_mm'], dtype=float)
        pts = shape_nodes_mm(node['_shape'])
        local = (pts - file_T_comp[:3, 3]) @ file_T_comp[:3, :3]  # inverse of a rigid placement
        verts, tris = hull.convex_hull(local)
        mesh = model_dir / f'{oid}.stl'
        hull.write_stl(mesh, verts, tris)
        parent = PARENT[role]
        LOG.info('A18 %s (%s, %s): %d points -> hull %d vertices, %d triangles', oid, label, role, len(pts),
                 len(verts), len(tris))
        objects.append({'object_id': oid, 'role': role, 'component': label, 'parent_frame': parent,
                        'pose_mm': pose_mm(parent_T_file[parent] @ file_T_comp),
                        'mesh': {'file': mesh.name, 'sha256': sha256_file(mesh), 'face_ids': False,
                                 'vertices': int(len(verts)), 'triangles': int(len(tris))},
                        'padding_mm': float(padding_mm)})
    return objects


def export_scene(step: Path, map_path: Path, repo_root: Path, job_name: str, chord_near_mm: float,
                 chord_far_mm: float, padding_mm: float, created_by: str) -> Dict[str, Any]:
    """Write the cell part and the job part of the spike scene (see module docstring); return a summary."""
    from awm_geometry import cli, part

    if not step.is_file():
        raise GeometryError(f'STEP file not found: {step}')
    if not 0 < chord_near_mm <= chord_far_mm:
        raise GeometryError('need 0 < chord-near-mm <= chord-far-mm')
    cmap = load_map(map_path, step)
    LOG.info('A18 reading assembly %s', step)
    tree, _unit = assembly.component_tree(step, cylinders=False, shapes=True)
    direct = resolve_roles(tree, cmap)
    parts = [(i, label) for i, (role, label) in direct.items() if role == 'part']
    if len(parts) != 1:
        raise MapError(f'the map must name exactly one part component, found {len(parts)}')
    fr = cell_frames(tree, cmap)

    repo = Repository(repo_root)
    model_dir = repo.cell_dir / 'model'
    model_dir.mkdir(parents=True, exist_ok=True)
    objects = _cell_objects(tree, direct, fr, model_dir, padding_mm)
    write_frames(model_dir / 'frames.yaml', fr, step, map_path, cmap)
    manifest = {
        'schema': 'awm_cell_scene/0.1', 'created_by': created_by, 'created_at': jsonio.utc_now_iso(),
        'inputs': [{'name': step.name, 'sha256': sha256_file(step)},
                   {'name': map_path.name, 'sha256': sha256_file(map_path)}],
        'unit': 'mm', 'frames_file': 'frames.yaml',
        'hull_tessellation': {'linear_deflection_mm': HULL_LINEAR_DEFLECTION_MM,
                              'angular_deflection_deg': HULL_ANGULAR_DEFLECTION_DEG},
        'note': f'{_ASSUMPTION}. Meshes: convex hull per component in its own frame; pose = parent_T_component; '
                'positioner_flange objects at theta = 0',
        'objects': objects, 'scene_sha256': canonical_sha256(objects),
    }
    (model_dir / 'scene_manifest.json').write_text(json.dumps(manifest, indent=1) + '\n')

    part_node = tree[parts[0][0]]
    component = part_node['instance_name'] or part_node['product_name']
    del tree  # release the cell shapes before the second read
    if '_' not in job_name:
        raise GeometryError(f'job {job_name!r} must be <PartNo>_<PartRev>')
    part_no, part_rev = job_name.rsplit('_', 1)
    if repo.job_id(part_no, part_rev) != job_name:
        raise GeometryError(f'job {job_name!r} is not a valid job id')
    job = repo.job(part_no, part_rev)
    cli.copy_step(step, job)
    LOG.info('A11 reading part component %r', component)
    loaded = part.load_part(job.model_step, component)
    record = cli.write_part_record(job, step.name, loaded, part_no, part_rev,
                                   PART_IMPORT_LINEAR_DEFLECTION_M, PART_IMPORT_ANGULAR_DEFLECTION_DEG)
    breptools.Clean(loaded.shape)
    view_summary = view.export(loaded, job.model_step, component, cmap['source']['up_axis'],
                               job.cache_dir / 'view', created_by, job.dir)
    verts, tris, face_records = part_collision_mesh(loaded.shape, chord_near_mm, chord_far_mm)
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
                     'mesh': {'file': 'cache/collision.stl', 'sha256': sha256_file(collision), 'face_ids': True},
                     'padding_mm': PART_PADDING_MM}]
    placement = jsonio.make_header(created_by, [job.model_step, collision, faces_json], job.dir)
    placement.update({'schema_version': '2.0', 'source': 'fixture_cad', 'verified': False,
                      'flange_T_part': pose_mm(flange_T_part), 'objects': part_objects,
                      'scene_sha256': canonical_sha256(part_objects)})
    jsonio.write_json(job.dir / 'placement.json', placement, 'v2/placement')

    roles: Dict[str, int] = {}
    for role, _label in direct.values():
        roles[role] = roles.get(role, 0) + 1
    near = [r for r in face_records if r['near_joint']]
    return {
        'cell_model': str(model_dir), 'job': str(job.dir), 'mapped_components_per_role': roles,
        'objects': {o['object_id']: o['mesh']['triangles'] for o in objects},
        'part': {'component': component, 'triangles': int(len(tris)), 'faces': len(face_records),
                 'near_joint_faces': len(near), 'edges': record['edges'], 'joints': len(view_summary['joints'])},
        'world_T_base': pose_mm(fr['world_T_base']),
        'world_T_positioner_axis': dict(pose_mm(fr['world_T_flange']),
                                        direction=[float(v) for v in fr['axis_direction']]),
        'flange_T_part': pose_mm(flange_T_part),
    }
