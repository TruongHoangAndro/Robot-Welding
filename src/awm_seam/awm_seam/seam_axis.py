"""``seam_axis.json``: the D04 seams expressed in the positioner frame (CP-58 decision 2, CP-59).

Derived data, never edited. For every seam point of ``seams.json`` (part frame) this writes the point and its seam
frame (X = travel tangent, Z = bisector of n1 / n2 out of the material, Y = Z x X; beam = -Z) in the
``positioner_flange`` frame at theta = 0, which equals the ``positioner_axis`` frame at theta = 0. The axis is +X
through the origin of that frame (convention of ``cell/model/frames.yaml``, checked here), so a point at angle theta
is the theta = 0 point rotated by theta about +X. Per point: ``r_mm`` (distance from the axis), ``axial_mm`` (x) and
``phi_deg`` = atan2(z, y) about +X, unwrapped along the seam.

The transform is ``placement.json`` ``flange_T_part`` (mm, unit quaternion) and goes only through
``awm_core.frames``; angles leave ``atan2`` through ``awm_core.units``. The header records the SHA-256 of
``seams.json``, ``placement.json`` and ``frames.yaml``; :func:`read_seam_axis` refuses a stale file.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml

from awm_core import cad_roles, frames, jsonio, units
from awm_core.repository import JobPaths
from awm_seam import joint_seams

CREATED_BY = joint_seams.CREATED_BY
SCHEMA = 'v2/seam_axis'
SCHEMA_VERSION = '2.0'
#: Assumption: a seam is a Mode C candidate when its radius about the axis varies by at most this much
#: (lower bound of the +-0.1-0.2 mm laser accuracy band, DES-001 AD5). Parameter of build_seam_axis.
TOL_R_SPREAD_MM = 0.1
#: Assumption: ... and its tangent leaves the plane normal to the axis by at most this angle.
TOL_AXIS_ANGLE_DEG = 0.5
QUAT_NORM_TOL = 1e-6          # contract T0.5 rule 2
AXIS_CONVENTION_TOL = 1e-6    # frames.yaml direction / point vs the pose of world_T_positioner_axis


class SeamAxisError(ValueError):
    """Inputs of seam_axis.json are inconsistent (frame convention, quaternion, geometry)."""


def seam_axis_path(job: JobPaths) -> Path:
    return job.dir / 'seam_axis.json'


def placement_path(job: JobPaths) -> Path:
    return job.dir / 'placement.json'


def frames_path(job: JobPaths) -> Path:
    return job.repo.cell_dir / 'model' / 'frames.yaml'


def _rotation(quat_xyzw, what: str) -> np.ndarray:
    q = np.asarray(quat_xyzw, float)
    if q.shape != (4,) or abs(float(np.linalg.norm(q)) - 1.0) > QUAT_NORM_TOL:
        raise SeamAxisError(f'{what}: quaternion {list(q)} is not a unit quaternion (|q| = 1 +- {QUAT_NORM_TOL})')
    return frames.quat_to_matrix(q)


def pose_mm_to_matrix(pose: Dict, what: str) -> np.ndarray:
    """``{position_mm, quat_xyzw}`` -> 4x4 transform in mm."""
    return frames.make_transform(pose['position_mm'], _rotation(pose['quat_xyzw'], what))


def check_frames(path: Path) -> Dict:
    """Read ``frames.yaml`` and check the positioner axis convention (axis = +X of positioner_axis)."""
    if not path.is_file():
        raise FileNotFoundError(f'cell frames not found: {path}')
    doc = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(doc, dict) or doc.get('unit') != 'mm' or 'world_T_positioner_axis' not in doc:
        raise SeamAxisError(f'{path}: needs unit "mm" and world_T_positioner_axis')
    if not str(doc.get('schema_version', '')).startswith('2.'):
        raise SeamAxisError(f'{path}: schema_version {doc.get("schema_version")} is not 2.x')
    axis = doc['world_T_positioner_axis']
    t = pose_mm_to_matrix(axis, f'{path.name} world_T_positioner_axis')
    if 'direction' in axis:
        d = np.asarray(axis['direction'], float)
        if np.linalg.norm(t[:3, 0] - d / np.linalg.norm(d)) > AXIS_CONVENTION_TOL:
            raise SeamAxisError(f'{path.name}: positioner axis direction {list(d)} is not +X of '
                                'world_T_positioner_axis; seam_axis.json assumes axis = +X')
    if 'point_mm' in axis:
        if np.linalg.norm(t[:3, 3] - np.asarray(axis['point_mm'], float)) > AXIS_CONVENTION_TOL:
            raise SeamAxisError(f'{path.name}: positioner axis point is not the origin of world_T_positioner_axis')
    return doc


def _arcs(seam: Dict) -> List[Dict]:
    """CP-53 arcs: equal lengths from the seam start, each extended by the overlap at its end (not the last
    arc of an open seam); on a closed seam the last arc runs over the start (s_end > length)."""
    n, length, overlap = seam['arcs'], float(seam['length_mm']), float(seam['arc_overlap_mm'])
    angles = seam.get('positioner_angles_deg')
    out = []
    for k in range(n):
        end = length * (k + 1) / n
        if seam['closed'] or k < n - 1:
            end += overlap
        out.append({'index': k + 1, 'of': n, 's_start_mm': length * k / n, 's_end_mm': end,
                    'positioner_angle_deg': None if angles is None else float(angles[k])})
    return out


def seam_frame(point: Dict) -> np.ndarray:
    """Seam frame rotation in the part frame: X = t, Z = bisector(n1, n2), Y = Z x X."""
    z = np.asarray(point['n1'], float) + np.asarray(point['n2'], float)
    if np.linalg.norm(z) < 1e-9:
        raise SeamAxisError(f'opposite face normals at s = {point["s_mm"]:.3f} mm: seam frame Z undefined')
    return frames.frame_from_z_x(z, point['t'])


def seam_to_axis(seam: Dict, flange_t_part: np.ndarray, tol_r_spread_mm: float,
                 tol_axis_angle_deg: float) -> Dict:
    """One seam of seams.json -> its seam_axis.json entry."""
    rot, pos = flange_t_part[:3, :3], flange_t_part[:3, 3]
    points, phis, axis_sin = [], [], 0.0
    for p in seam['geometry']['points']:
        q = rot @ np.asarray(p['p_mm'], float) + pos
        r_seam = rot @ seam_frame(p)
        axis_sin = max(axis_sin, abs(float(r_seam[0, 0])))
        phis.append(units.rad_to_deg(math.atan2(q[2], q[1])))
        points.append({'s_mm': float(p['s_mm']), 'p_mm': [float(v) + 0.0 for v in q],
                       'quat_xyzw': [float(v) for v in frames.matrix_to_quat(r_seam)],
                       'r_mm': float(math.hypot(q[1], q[2])), 'axial_mm': float(q[0])})
    unwrapped = np.unwrap(np.asarray(phis), period=360.0)
    if unwrapped[0] <= -180.0:
        unwrapped += 360.0
    for pt, phi in zip(points, unwrapped):
        pt['phi_deg'] = float(phi)
    rs = [p['r_mm'] for p in points]
    xs = [p['axial_mm'] for p in points]
    spread = max(rs) - min(rs)
    angle = units.rad_to_deg(math.asin(min(1.0, axis_sin)))
    return {
        'seam_id': seam['seam_id'], 'joint_edge_id': seam['joint_edge_id'], 'closed': seam['closed'],
        'reversed': seam['reversed'], 'length_mm': seam['length_mm'],
        'spacing_mm': seam['geometry']['spacing_mm'],
        'positioner_mode': seam.get('positioner_mode', 'sync'),
        'positioner_angles_deg': seam.get('positioner_angles_deg'),
        'arcs': _arcs(seam),
        'coaxial': {'is_coaxial': bool(spread <= tol_r_spread_mm and angle <= tol_axis_angle_deg),
                    'r_spread_mm': float(spread), 'axis_angle_deg': float(angle),
                    'tol_r_spread_mm': float(tol_r_spread_mm), 'tol_axis_angle_deg': float(tol_axis_angle_deg)},
        'summary': {'points': len(points), 'r_min_mm': min(rs), 'r_max_mm': max(rs),
                    'axial_min_mm': min(xs), 'axial_max_mm': max(xs),
                    'phi_start_deg': points[0]['phi_deg'], 'phi_end_deg': points[-1]['phi_deg']},
        'points': points,
    }


def build_seam_axis(job: JobPaths, tol_r_spread_mm: float = TOL_R_SPREAD_MM,
                    tol_axis_angle_deg: float = TOL_AXIS_ANGLE_DEG, frames_file: Optional[Path] = None,
                    created_by: str = CREATED_BY) -> Dict:
    """Write ``seam_axis.json`` from ``seams.json`` + ``placement.json`` + ``frames.yaml`` (all fresh)."""
    frames_file = Path(frames_file) if frames_file else frames_path(job)
    check_frames(frames_file)
    seams = joint_seams.load_seams(job)
    placement = jsonio.read_json(placement_path(job), 'v2/placement', base_dir=job.dir)
    flange_t_part = pose_mm_to_matrix(placement['flange_T_part'], 'placement.json flange_T_part')
    inputs = [job.seams_json, placement_path(job), frames_file]
    if (job.dir / joint_seams.ROLES_FILE).is_file():  # CP-60 job import: a role / axis / STEP edit -> stale
        inputs += [job.model_step, job.dir / joint_seams.ROLES_FILE]
        if (job.dir / cad_roles.AXIS_FILE).is_file():
            inputs.append(job.dir / cad_roles.AXIS_FILE)
    doc = jsonio.make_header(created_by, inputs, job.dir)
    doc.update({
        'schema_version': SCHEMA_VERSION, 'frame': 'positioner_flange', 'theta_deg': 0,
        'axis': {'point_mm': [0.0, 0.0, 0.0], 'direction': [1.0, 0.0, 0.0]},
        'flange_T_part': {'position_mm': [float(v) for v in placement['flange_T_part']['position_mm']],
                          'quat_xyzw': [float(v) for v in placement['flange_T_part']['quat_xyzw']]},
        'seams': [seam_to_axis(s, flange_t_part, tol_r_spread_mm, tol_axis_angle_deg)
                  for s in seams['seams']],
    })
    jsonio.write_json(seam_axis_path(job), doc, SCHEMA)
    return doc


def read_seam_axis(job: JobPaths, deep: bool = True) -> Dict:
    """Read ``seam_axis.json``; refuse it if seams.json, placement.json or frames.yaml changed since it was
    written (StaleInputError). ``deep`` also checks the inputs of seams.json (part.json, topology.json)."""
    doc = jsonio.read_json(seam_axis_path(job), SCHEMA, base_dir=job.dir)
    if not str(doc['schema_version']).startswith('2.'):
        raise SeamAxisError(f'{seam_axis_path(job)}: schema_version {doc["schema_version"]} is not 2.x')
    if deep:
        joint_seams.load_seams(job)
    return doc


def axis_to_part(doc: Dict, point: Dict) -> Dict:
    """Inverse transform of one seam_axis point: position (mm) and seam frame quaternion in the part frame."""
    t = frames.invert(pose_mm_to_matrix(doc['flange_T_part'], 'seam_axis.json flange_T_part'))
    p = t[:3, :3] @ np.asarray(point['p_mm'], float) + t[:3, 3]
    rot = t[:3, :3] @ _rotation(point['quat_xyzw'], 'seam_axis point')
    return {'p_mm': [float(v) for v in p], 'quat_xyzw': [float(v) for v in frames.matrix_to_quat(rot)]}
