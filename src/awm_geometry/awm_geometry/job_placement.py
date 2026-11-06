"""CP-60 job import: place the part on the cell positioner from the job STEP and cross-check it against the cell.

Inputs (all mm): the job folder (``model.step``, ``roles.json`` scope job, ``axis_pick.json``) and the cell model
(``cell/cad``: ``roles.json``, ``axis_pick.json``, ``cell_components.json``; ``cell/model``: ``frames.yaml``,
``scene_manifest.json``). Outputs in the job folder: ``cad_check.json`` (always), and when check 1 is not red
``placement.json`` (``source: job_cad``) + ``cache/collision.stl`` / ``collision_faces.json`` (part mesh with the D20
face ids) + hull meshes of the job's own rotating components.

Placement: part frame = job file frame; positioner_flange (θ = 0) = cell axis frame;
``flange_T_part = Rx(φ) · inv(file_T_axis_job)`` with φ from check 1 (the job positioner may stand at another angle).
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from awm_core import axis_frame, cad_roles, frames, jsonio, stl
from awm_core.hashing import sha256_file
from awm_core.repository import Repository
from awm_geometry import assembly, cross_check, hull, meshops, miniyaml, part, scene
from awm_geometry.occ_io import GeometryError

LOG = logging.getLogger('awm_geometry')
SAMPLE_SPACING_MM = 1.0
SLAB_MM = 1.0
OBJECTS_DIR = 'objects'  # jobs/<job>/cache/objects/<id>.stl: hulls of job rotary_fixture components


def _read(path: Path, schema: Optional[str] = None) -> Dict[str, Any]:
    if not path.is_file():
        raise GeometryError(f'missing input {path}')
    doc = json.loads(path.read_text(encoding='utf-8'))
    if schema:
        jsonio.validate(doc, schema)
    return doc


def _rotary_axis_T(tree, eff, file_T_axis: np.ndarray) -> Dict[str, Any]:
    """``{radius_mm (filled by caller), axis_T: {instance name: 4x4}, duplicates}`` of the rotary_fixture nodes."""
    inv = frames.invert(file_T_axis)
    counts: Dict[str, int] = {}
    poses: Dict[str, np.ndarray] = {}
    for n in tree:
        if n['parent'] is not None and eff[n['index']] == 'rotary_fixture' and n['instance_name']:
            counts[n['instance_name']] = counts.get(n['instance_name'], 0) + 1
            poses[n['instance_name']] = inv @ np.asarray(n['placement_mm'], dtype=float)
    dup = sorted(k for k, v in counts.items() if v > 1)
    return {'axis_T': {k: v for k, v in poses.items() if k not in dup}, 'duplicates': dup}


def _samples(verts: np.ndarray, tris: np.ndarray, t: np.ndarray) -> np.ndarray:
    pts, _owner = meshops.sample_surface(verts, tris, SAMPLE_SPACING_MM)
    return pts @ t[:3, :3].T + t[:3, 3]


def _pieces(node, label: str):
    pieces = scene._tessellate_mm(node['_shape'], scene.HULL_LINEAR_DEFLECTION_MM, scene.HULL_ANGULAR_DEFLECTION_DEG,
                                  label, [], hull_mode=True)
    verts, tris, n = [], [], 0
    for v, t in pieces:
        verts.append(v)
        if t is not None and len(t):
            tris.append(np.asarray(t) + n)
        n += len(v)
    return np.vstack(verts), (np.vstack(tris) if tris else np.zeros((0, 3), dtype=int))


def _pose_mm(p: Dict[str, Any]) -> np.ndarray:
    return frames.make_transform(p['position_mm'], frames.quat_to_matrix(p['quat_xyzw']))


def _statics(root: Path, manifest: Dict[str, Any], axis_T_world: np.ndarray, x_lo: float, x_hi: float,
             r_max: float) -> Dict[str, np.ndarray]:
    """Surface samples of the cell's static objects in the cell axis frame (θ = 0), only of the triangles near the
    swept cylinder (x_lo..x_hi, radius r_max; conservative prefilter)."""
    out = {}
    for obj in manifest['objects']:
        if obj['role'] != 'cell_static' or obj['parent_frame'] != 'world':
            continue
        verts, tris = stl.read_stl(root / 'cell' / 'model' / obj['mesh']['file'])
        t = axis_T_world @ _pose_mm(obj['pose_mm'])
        v_axis = np.asarray(verts, dtype=float) @ t[:3, :3].T + t[:3, 3]
        keep = cross_check.triangles_near_cylinder(v_axis, tris, x_lo, x_hi, r_max, 2.0 * SLAB_MM)
        if len(keep):
            out[obj.get('component') or obj['object_id']] = _samples(v_axis, np.asarray(tris)[keep], np.eye(4))
    return out


def run(repo_root: Path, job_name: str, created_by: str, chord_near_mm: float = 0.1, chord_far_mm: float = 0.5,
        padding_mm: float = 3.0) -> Dict[str, Any]:
    """Write ``cad_check.json`` (+ placement when check 1 passes) for job ``job_name``; return a summary."""
    root = Path(repo_root)
    repo = Repository(root)
    part_no, part_rev = scene._job_from_name(repo, job_name)
    job = repo.job(part_no, part_rev)
    cad = root.joinpath(*cad_roles.CELL_CAD_DIR)
    frames_path = root / 'cell' / 'model' / 'frames.yaml'
    manifest_path = root / 'cell' / 'model' / 'scene_manifest.json'
    cell_roles = _read(cad / cad_roles.ROLES_FILE, 'v2/cad_roles')
    cell_pick = _read(cad / cad_roles.AXIS_FILE, 'v2/axis_pick')
    cell_tree = _read(cad / cad_roles.CELL_TREE)['components']
    manifest = _read(manifest_path, 'v2/cell_scene')
    if not frames_path.is_file():
        raise GeometryError(f'cell model not exported: {frames_path} missing (save the cell model first)')
    fr = miniyaml.load(frames_path)
    roles = _read(job.dir / cad_roles.ROLES_FILE, 'v2/cad_roles')
    pick = _read(job.dir / cad_roles.AXIS_FILE, 'v2/axis_pick')
    step = job.model_step
    digest = sha256_file(step)
    for what, doc in (('job roles', roles), ('job axis pick', pick)):
        if doc['source']['sha256'] != digest:
            raise GeometryError(f'{what} were made from another STEP file than {step.name}')
    if roles['scope'] != 'job':
        raise GeometryError('job roles.json must have scope "job"')
    up_job, up_cell = roles['source']['up_axis'], cell_roles['source']['up_axis']

    LOG.info('CP-60 reading job assembly %s', step)
    tree, _unit = assembly.component_tree(step, cylinders=False, shapes=True)
    problems = cad_roles.validate_roles(tree, roles, 'job')
    if problems:
        raise GeometryError('job roles do not fit the job file:\n  ' + '\n  '.join(problems))
    eff = cad_roles.effective_roles(tree, roles['assignments'])
    eff_cell = cad_roles.effective_roles(cell_tree, cell_roles['assignments'])
    rel = cad_roles.rel_paths(tree)
    try:
        f_T_aj = axis_frame.file_T_axis(pick['center_file_mm'], pick['axis_file'], up_job)
        f_T_ac = axis_frame.file_T_axis(cell_pick['center_file_mm'], cell_pick['axis_file'], up_cell)
    except ValueError as exc:
        raise GeometryError(str(exc)) from exc
    tol_mm, tol_deg = cad_roles.CHECK_TOL_MM, cad_roles.CHECK_TOL_DEG

    # --- check 1: positioner match, phi ---------------------------------------------------------------------
    cell_side = dict(_rotary_axis_T(cell_tree, eff_cell, f_T_ac), radius_mm=cell_pick['radius_mm'])
    job_side = dict(_rotary_axis_T(tree, eff, f_T_aj), radius_mm=pick['radius_mm'])
    c1 = cross_check.positioner_match(cell_side, job_side, tol_mm, tol_deg)
    phi = c1['phi_deg'] or 0.0
    checks: List[Dict[str, Any]] = [dict(id=1, name='positioner match', status=c1['status'],
                                         values=dict(c1['values'], phi_deg=c1['phi_deg']),
                                         tolerance=c1['tolerance'], note=c1['note'])]

    # --- checks 2 / 3: robot base and tool against the cell ---------------------------------------------------
    rj, rc = cad_roles.robot_node(tree, eff), cad_roles.robot_node(cell_tree, eff_cell)
    if rj is None or rc is None:
        checks.append(dict(id=2, name='robot base vs cell', status='skipped', note='no robot component in the job file'))
    else:
        c2 = cross_check.pose_check(frames.invert(f_T_ac) @ np.asarray(cell_tree[rc]['placement_mm']),
                                    frames.invert(f_T_aj) @ np.asarray(tree[rj]['placement_mm']), tol_mm, tol_deg)
        checks.append(dict(id=2, name='robot base vs cell', **c2,
                           note='within tolerance' if c2['status'] == 'green' else 'cell robot base is used'))
    tj = cad_roles.nodes_with_role(tree, eff, 'tool')
    tc = cad_roles.nodes_with_role(cell_tree, eff_cell, 'tool')
    if rj is None or rc is None or not tj or not tc:
        checks.append(dict(id=3, name='tool vs cell', status='skipped',
                           note='needs a robot and a tool component in both files'))
    else:
        c3 = cross_check.pose_check(
            frames.invert(np.asarray(cell_tree[rc]['placement_mm'])) @ np.asarray(cell_tree[tc[0]]['placement_mm']),
            frames.invert(np.asarray(tree[rj]['placement_mm'])) @ np.asarray(tree[tj[0]]['placement_mm']),
            tol_mm, tol_deg)
        checks.append(dict(id=3, name='tool vs cell', **c3, note='information only; the cell TCP is used'))

    # --- meshes: part (D20 face ids) and the rotating set -----------------------------------------------------
    loaded = part.load_part(step, components=True)
    lost = [f'B{b}' for b, ci in enumerate(loaded.solid_components or [], 1) if ci is None]
    if lost:
        raise GeometryError(f'solid(s) {", ".join(lost)} lost their component link in check / heal: their role is '
                            'unknown, so they would be missing from the part mesh and the cross-check')
    part_solids = {b for b, ci in enumerate(loaded.solid_components or [], 1)
                   if ci is not None and eff[ci] == 'part'}
    if not part_solids:
        raise GeometryError('no solid of a part component found in the job file')
    verts, tris, face_records = scene.part_collision_mesh(loaded.shape, chord_near_mm, chord_far_mm, part_solids)
    a_T_f = frames.invert(f_T_aj)
    rotating = {'part': _samples(verts, tris, a_T_f)}
    for i in cad_roles.nodes_with_role(tree, eff, 'rotary_fixture'):
        v, t = _pieces(tree[i], rel[i])
        if len(t):
            rotating[rel[i]] = _samples(v, t, a_T_f)
    # job-only components (instance name not in the cell CAD) go into the scene, one hull per leaf: rotary ones on
    # the flange, static ones (also unassigned) in world (review I2 / I3, spec §3: unassigned = static obstacle)
    cell_names = {n['instance_name'] for n in cell_tree if n['parent'] is not None}
    job_only = [(i, eff[i]) for i in range(len(tree)) if tree[i]['parent'] is not None and not tree[i]['is_assembly']
           and tree[i].get('bbox_mm') is not None and eff[i] in ('rotary_fixture', 'static')
           and tree[i]['instance_name'] not in cell_names]
    a_T_w = frames.invert(_pose_mm(fr['world_T_positioner_axis']))
    every = np.vstack(list(rotating.values()))
    statics = _statics(root, manifest, a_T_w, float(every[:, 0].min()), float(every[:, 0].max()),
                       float(np.hypot(every[:, 1], every[:, 2]).max()))
    hits = cross_check.swept_envelope_hits(rotating, statics, SLAB_MM)
    checks.append(dict(id=4, name='swept rotating set vs static', status='red' if hits else 'green',
                       values={'hits': hits[:50], 'slab_mm': SLAB_MM, 'sample_spacing_mm': SAMPLE_SPACING_MM,
                               'turn_deg': 360.0},
                       note=('static object(s) inside the volume swept by the rotating set: ' +
                             ', '.join(sorted({h['static'] for h in hits}))) if hits else
                       'full turn checked (angle limits unknown, OI-27)'))
    checks.append(dict(id=5, name='seam edges on part solids', status='green',
                       note='enforced when seams are picked (awm_seam)'))
    checks.append(dict(id=6, name='inputs recorded', status='green',
                       note='input hashes below; seam_axis.json records them (stale -> re-plan)'))

    # --- write -------------------------------------------------------------------------------------------------
    inputs = [step, job.dir / cad_roles.ROLES_FILE, job.dir / cad_roles.AXIS_FILE, frames_path,
              cad / cad_roles.ROLES_FILE, cad / cad_roles.AXIS_FILE]
    status = cross_check.worst(c['status'] for c in checks)
    own = {a['path'] for a in roles['assignments']}
    check_doc = jsonio.make_header(created_by, inputs, job.dir)
    check_doc.update({'schema_version': '2.0', 'status': status, 'checks': checks, 'phi_deg': c1['phi_deg'],
                      'excluded': [rel[i] for i in cad_roles.uniform_nodes(tree, eff, cad_roles.EXCLUDE)],
                      'unassigned_static': [rel[i] for i in cad_roles.uniform_nodes(tree, eff, 'static')
                                            if rel[i] not in own]})
    placement_path = job.dir / 'placement.json'
    if c1['status'] == 'red':
        if placement_path.is_file():
            placement_path.unlink()  # a placement from another positioner pick would be silently wrong
        jsonio.write_json(job.dir / cad_roles.CHECK_FILE, check_doc, 'v2/cad_check')
        LOG.warning('check 1 red: %s', c1['note'])
        return {'status': status, 'cad_check': str(job.dir / cad_roles.CHECK_FILE), 'placement': None}

    flange_T_file = frames.make_transform([0, 0, 0], frames.axis_angle_to_matrix([1, 0, 0], np.radians(phi))) @ a_T_f
    job.cache_dir.mkdir(parents=True, exist_ok=True)
    collision = job.cache_dir / 'collision.stl'
    hull.write_stl(collision, verts, tris)
    faces_doc = jsonio.make_header(created_by, [step, collision], job.dir)
    faces_doc.update({'schema_version': '2.0', 'unit': 'mm', 'frame': 'part', 'mesh': 'cache/collision.stl',
                      'component': None, 'triangles': int(len(tris)), 'chord_near_mm': float(chord_near_mm),
                      'chord_far_mm': float(chord_far_mm), 'angular_near_deg': scene.PART_NEAR_ANGULAR_DEG,
                      'angular_far_deg': scene.PART_FAR_ANGULAR_DEG, 'faces': face_records})
    faces_json = job.cache_dir / 'collision_faces.json'
    faces_json.write_text(json.dumps(faces_doc, indent=1) + '\n')
    objects = [{'object_id': 'part', 'role': 'part', 'parent_frame': 'positioner_flange',
                'pose_mm': scene.pose_mm(flange_T_file),
                'mesh': {'file': 'cache/collision.stl', 'sha256': sha256_file(collision), 'face_ids': True},
                'padding_mm': scene.PART_PADDING_MM}]
    obj_dir = job.cache_dir / OBJECTS_DIR
    world_T_file = _pose_mm(fr['world_T_positioner_axis']) @ a_T_f  # static: no turn phi (they do not rotate)
    ids = {'part'}
    for i, role in job_only:
        v_file, _t = _pieces(tree[i], rel[i])
        place = np.asarray(tree[i]['placement_mm'], dtype=float)
        local = (v_file - place[:3, 3]) @ place[:3, :3]
        hv, ht = hull.convex_hull(local)
        oid = scene.object_id(tree[i]['instance_name'] or tree[i]['product_name'])
        if oid in ids:
            oid = f'{oid}_{i}'
        ids.add(oid)
        mesh = obj_dir / f'{oid}.stl'
        obj_dir.mkdir(parents=True, exist_ok=True)
        hull.write_stl(mesh, hv, ht)
        rotary = role == 'rotary_fixture'
        objects.append({'object_id': oid, 'role': 'rotary_fixture' if rotary else 'fixture',
                        'parent_frame': 'positioner_flange' if rotary else 'world',
                        'pose_mm': scene.pose_mm((flange_T_file if rotary else world_T_file) @ place),
                        'mesh': {'file': f'cache/{OBJECTS_DIR}/{oid}.stl', 'sha256': sha256_file(mesh),
                                 'face_ids': False}, 'padding_mm': float(padding_mm)})
    placement = jsonio.make_header(created_by, inputs + [collision, faces_json], job.dir)
    placement.update({'schema_version': '2.0', 'source': 'job_cad', 'verified': False,
                      'flange_T_part': scene.pose_mm(flange_T_file), 'axis_pick': cad_roles.AXIS_FILE,
                      'phi_deg': float(phi), 'objects': objects, 'scene_sha256': scene.canonical_sha256(objects)})
    jsonio.write_json(placement_path, placement, 'v2/placement')
    jsonio.write_json(job.dir / cad_roles.CHECK_FILE, check_doc, 'v2/cad_check')
    return {'status': status, 'cad_check': str(job.dir / cad_roles.CHECK_FILE), 'placement': str(placement_path),
            'phi_deg': phi, 'checks': {c['id']: c['status'] for c in checks},
            'flange_T_part': scene.pose_mm(flange_T_file)}
