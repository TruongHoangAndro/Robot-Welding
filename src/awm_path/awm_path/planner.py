"""M4 path planner: seams + process parameters -> path.json in the part frame (A41-A45)."""

from __future__ import annotations

import logging
from typing import Dict, List, Optional

import numpy as np

from awm_core import frames, jsonio, occ_bridge
from awm_core.calibration import ActiveCalibration, load_active
from awm_core.repository import JobPaths
from awm_core.stl import read_stl
from awm_path import __version__
from awm_path.motion import plan_seam_motion
from awm_path.moveit_client import MoveItClient
from awm_path.params import ChainParams
from awm_path.poses import PathError, seam_poses

CREATED_BY = f'awm_path {__version__}'
LOG = logging.getLogger('awm_path')
TOOL_FOR_PROCESS = {'MIG': 'tcp_mig', 'LASER': 'tcp_laser'}
TCP_POS_TOL_M = 1.0e-4
TCP_ROT_TOL_RAD = 1.0e-3


class PlanningError(RuntimeError):
    """The job cannot be planned at all (missing input, inconsistent cell model, ...)."""


def ensure_seam_geometry(job: JobPaths, params: ChainParams) -> Dict:
    """A41: (re)sample the seams in the OCC env if seam_geometry.json is missing or stale."""
    try:
        doc = jsonio.read_json(job.seam_geometry_json, 'seam_geometry', base_dir=job.dir)
        if abs(doc['spacing_m'] - params.sample_spacing_m) < 1e-12:
            return doc
    except (FileNotFoundError, jsonio.StaleInputError):
        pass
    LOG.info('A41 sampling seams (spacing %.4f m) in the OCC interpreter', params.sample_spacing_m)
    occ_bridge.run_geometry_cli(['sample', '--job', str(job.dir), '--spacing-m',
                                 repr(params.sample_spacing_m)],
                                occ_python_cmd=params.occ_python_cmd)
    return jsonio.read_json(job.seam_geometry_json, 'seam_geometry', base_dir=job.dir)


def check_tcp(client: MoveItClient, cal: ActiveCalibration, params: ChainParams) -> None:
    """The TCP of the cell model (D14) must equal the TCP of the active calibration (D08)."""
    fk = client.compute_fk(params.home_joints_rad, ['tool0', params.tcp_link])
    model = frames.compose(frames.invert(fk['tool0']), fk[params.tcp_link])
    calib = cal.tool0_T_tcp(params.tcp_link)
    dp = float(np.linalg.norm(model[:3, 3] - calib[:3, 3]))
    da = frames.rotation_angle_between(model[:3, :3], calib[:3, :3])
    if dp > TCP_POS_TOL_M or da > TCP_ROT_TOL_RAD:
        raise PlanningError(
            f'TCP {params.tcp_link} of the cell model differs from calibration '
            f'{cal.calibration_id} by {dp * 1000:.2f} mm / {np.degrees(da):.3f} deg: update '
            f'awm_description/config/cell_nominal.yaml or the calibration')


def load_part_into_scene(client: MoveItClient, job: JobPaths, part: Dict,
                         cal: ActiveCalibration) -> None:
    mesh = job.dir / part['mesh']['file']
    vertices, triangles = read_stl(mesh)
    client.set_part(vertices, triangles, cal.base_T_part)
    LOG.info('part mesh in planning scene: %d triangles at base_T_part', len(triangles))


def plan_path(job: JobPaths, params: ChainParams, client: MoveItClient) -> Dict:
    """Plan every seam; write path.json. Seams that fail are listed in ``rejected`` with a reason."""
    part = jsonio.read_json(job.part_json, 'part', base_dir=job.dir)
    seams = jsonio.read_json(job.seams_json, 'seams', base_dir=job.dir)
    process = jsonio.read_json(job.process_json, 'process', base_dir=job.dir)
    cal = load_active(job.repo)
    geometry = ensure_seam_geometry(job, params)
    geo_by_id = {g['seam_id']: g for g in geometry['seams']}
    proc_by_id = {r['seam_id']: r for r in process['seams']}
    unassigned = {r['seam_id']: r['reason'] for r in process['unassigned']}

    check_tcp(client, cal, params)
    load_part_into_scene(client, job, part, cal)
    base_T_part = cal.base_T_part

    accepted: List[Dict] = []
    rejected: List[Dict] = []
    for seam in sorted(seams['seams'], key=lambda s: s['sequence']):
        sid = seam['seam_id']
        row = proc_by_id.get(sid)
        reason: Optional[str] = None
        if row is None:
            reason = f'no process parameters: {unassigned.get(sid, "not in process.json")}'
        elif TOOL_FOR_PROCESS[row['process']] != params.tcp_link:
            reason = (f'process {row["process"]} needs tool {TOOL_FOR_PROCESS[row["process"]]}, '
                      f'the cell model plans with {params.tcp_link} (TODO(OI-2): laser head)')
        elif sid not in geo_by_id:
            reason = 'no seam geometry samples'
        if reason is None:
            try:
                sp = seam_poses(geo_by_id[sid], seam, row, params.approach_distance_m,
                                params.tool_roll)
            except PathError as exc:
                reason = str(exc)
        if reason is not None:
            LOG.warning('%s rejected: %s', sid, reason)
            rejected.append({'seam_id': sid, 'reason': reason})
            continue
        to_base = [frames.compose(base_T_part, p) for p in sp.weld]
        motion = plan_seam_motion(client, sid, frames.compose(base_T_part, sp.approach), to_base,
                                  frames.compose(base_T_part, sp.retract), params)
        if not motion.ok:
            LOG.warning('%s rejected: %s', sid, motion.reason)
            rejected.append({'seam_id': sid, 'reason': motion.reason})
            continue
        LOG.info('%s accepted: %s', sid, motion.reason)
        accepted.append({
            'seam_id': sid, 'process': row['process'], 'sequence': seam['sequence'],
            'travel_speed_m_s': row['motion']['travel_speed_mm_s'] / 1000.0,
            'approach_pose': frames.matrix_to_pose(sp.approach),
            'weld_poses': [frames.matrix_to_pose(p) for p in sp.weld],
            'retract_pose': frames.matrix_to_pose(sp.retract),
            'weld_length_m': sp.length_m,
            'reach': {'ok': True, 'fraction': 1.0, 'detail': motion.reason,
                      'seed_joints': [float(q) for q in motion.ik_start]},
            'collision': {'ok': True, 'detail': 'compute_cartesian_path avoid_collisions=true '
                                                'against cell model + part mesh'},
        })
    inputs = [job.part_json, job.seams_json, job.process_json, job.seam_geometry_json,
              job.repo.active_calibration]
    doc = jsonio.make_header(CREATED_BY, inputs, job.dir)
    doc.update({'frame': 'part', 'tool': params.tcp_link, 'group': params.planning_group,
                'calibration': {'calibration_id': cal.calibration_id, 'sha256': cal.sha256},
                'seams': accepted, 'rejected': rejected})
    jsonio.write_json(job.path_json, doc, 'path')
    return doc
