"""M6: path.json (part frame) -> programs/<rev>/program.json (robot base frame).

A61 base_T_tcp = base_T_fixture · fixture_T_part · part_T_pose with the ACTIVE calibration. The
joint path checked by M4 for that same calibration is reused (pick_ik is stochastic, re-solving
could give a different branch); every joint point is verified by FK against the transformed poses.
A62 trajectory segments (air / approach / weld / retract) with timing, A63 WeldStart / WeldEnd
events at segment boundaries where the robot is at rest, A64 storage per program revision.
"""

from __future__ import annotations

import logging
from typing import Dict, List, Optional, Tuple

import numpy as np

from awm_core import frames, jsonio
from awm_core.calibration import load_active
from awm_core.repository import JobPaths
from awm_path.moveit_client import MoveItClient, TrajectoryPoints
from awm_path.params import ChainParams
from awm_path.planner import check_tcp, load_part_into_scene
from awm_program import __version__
from awm_program.timing import (TimingError, arc_length, constant_speed_times,
                                max_velocity_ratio, trapezoid_times)

CREATED_BY = f'awm_program {__version__}'
LOG = logging.getLogger('awm_program')
VEL_TOL = 1.05        # 5 % margin over the limit for time-parameterisation round-off
POSE_TOL_M = 2.0e-4   # FK of the stored joint path vs. the transformed weld path (A61 check)
ROT_TOL_RAD = 2.0e-3


class ProgramError(RuntimeError):
    """The program cannot be generated (reason in the message); nothing is written."""


def velocity_limits(params: ChainParams, kind: str) -> np.ndarray:
    """Joint velocity limits of a segment kind.

    Weld segments move at the WPS travel speed and are checked against the configured joint
    limits; air / approach / retract moves are additionally scaled by ``velocity_scaling``.
    """
    vmax = np.asarray(params.max_joint_velocity_rad_s, float)
    return vmax if kind == 'weld' else vmax * params.velocity_scaling


def check_limits(name: str, traj: TrajectoryPoints, params: ChainParams, kind: str) -> None:
    ratio, joint = max_velocity_ratio(traj.positions, traj.times, velocity_limits(params, kind))
    if ratio > VEL_TOL:
        raise ProgramError(f'{name}: joint {params.joint_names[joint]} needs {ratio:.2f} x its '
                           f'{"" if kind == "weld" else "scaled "}velocity limit')


def _distance_to_polyline(p: np.ndarray, poly: np.ndarray) -> float:
    a, b = poly[:-1], poly[1:]
    ab = b - a
    denom = np.einsum('ij,ij->i', ab, ab)
    t = np.clip(np.einsum('ij,ij->i', p - a, ab) / np.where(denom > 0, denom, 1.0), 0.0, 1.0)
    return float(np.min(np.linalg.norm(a + ab * t[:, None] - p, axis=1)))


def verify_joint_path(client: MoveItClient, params: ChainParams, sid: str,
                      joints: Dict[str, np.ndarray], approach: np.ndarray,
                      weld: List[np.ndarray], retract: np.ndarray) -> np.ndarray:
    """A61 check: the stored joints put the TCP on the base-frame poses. Returns weld TCP points."""
    def fk(q):
        return client.compute_fk(q, [params.tcp_link])[params.tcp_link]

    def same(a, b, what):
        dp = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
        da = frames.rotation_angle_between(a[:3, :3], b[:3, :3])
        if dp > POSE_TOL_M or da > ROT_TOL_RAD:
            raise ProgramError(f'{sid}: {what} of the stored joint path is {dp * 1000:.2f} mm / '
                               f'{np.degrees(da):.2f} deg off the transformed pose: re-run awm_path')

    same(fk(joints['approach'][0]), approach, 'approach start')
    same(fk(joints['weld'][0]), weld[0], 'weld start')
    same(fk(joints['weld'][-1]), weld[-1], 'weld end')
    same(fk(joints['retract'][-1]), retract, 'retract end')
    poly = np.array([p[:3, 3] for p in weld])
    tcp = np.array([fk(q)[:3, 3] for q in joints['weld']])
    worst = max(_distance_to_polyline(p, poly) for p in tcp)
    if worst > POSE_TOL_M:
        raise ProgramError(f'{sid}: weld joint path leaves the seam by {worst * 1000:.2f} mm')
    return tcp


def _time_trapezoid(q: np.ndarray, params: ChainParams) -> TrajectoryPoints:
    vmax = velocity_limits(params, 'approach')
    amax = np.asarray(params.max_joint_acceleration_rad_s2, float) * params.acceleration_scaling
    times, keep = trapezoid_times(q, vmax, amax)
    return TrajectoryPoints(np.asarray(q, float)[keep], times)


def _time_weld(q: np.ndarray, tcp: np.ndarray, speed_m_s: float,
               params: ChainParams) -> TrajectoryPoints:
    keep = np.concatenate([[True], np.linalg.norm(np.diff(tcp, axis=0), axis=1) > 1e-9])
    try:
        times = constant_speed_times(arc_length(tcp[keep]), speed_m_s, params.weld_accel_m_s2)
    except TimingError as exc:
        raise ProgramError(f'weld timing: {exc}') from exc
    return TrajectoryPoints(np.asarray(q, float)[keep], times)


class _Builder:
    """Collects segments and events; enforces exact continuity between segments."""

    def __init__(self, home: np.ndarray) -> None:
        self.home = home
        self.segments: List[Dict] = []
        self.events: List[Dict] = []

    @property
    def end(self) -> np.ndarray:
        if not self.segments:
            return self.home
        return np.asarray(self.segments[-1]['points'][-1]['q_rad'])

    def add(self, kind: str, seam_id: Optional[str], traj: TrajectoryPoints,
            speed: Optional[float] = None) -> str:
        seg_id = f'G{len(self.segments) + 1:03d}'
        if np.max(np.abs(traj.positions[0] - self.end)) > 1e-6:
            raise ProgramError(f'{seg_id} ({kind} {seam_id}) does not start where the previous '
                               f'segment ends')
        times = traj.times - traj.times[0]
        self.segments.append({
            'segment_id': seg_id, 'kind': kind, 'seam_id': seam_id,
            'duration_s': float(times[-1]), 'tcp_speed_m_s': speed,
            'points': [{'t_s': float(t), 'q_rad': [float(v) for v in q]}
                       for t, q in zip(times, traj.positions)]})
        return seg_id

    def event(self, etype: str, seam_id: str, after: str, row: Dict) -> None:
        self.events.append({'event_id': f'V{len(self.events) + 1:03d}', 'type': etype,
                            'seam_id': seam_id, 'after_segment': after, 'process': row['process'],
                            'wps_id': row['wps_id'], 'parameters': row['parameters'],
                            'timing': row['timing']})


def generate_program(job: JobPaths, params: ChainParams, client: MoveItClient) -> Tuple[str, Dict]:
    """Build and store the next program revision. Returns (revision, document)."""
    path = jsonio.read_json(job.path_json, 'path', base_dir=job.dir)
    part = jsonio.read_json(job.part_json, 'part', base_dir=job.dir)
    process = jsonio.read_json(job.process_json, 'process', base_dir=job.dir)
    cal = load_active(job.repo)
    if path['calibration']['sha256'] != cal.sha256:
        raise ProgramError('path.json was checked with another calibration; re-run awm_path')
    if not path['seams']:
        raise ProgramError('path.json has no accepted seam: nothing to program '
                           f'(rejected: {[r["seam_id"] for r in path["rejected"]]})')
    if path['rejected']:
        raise ProgramError('path.json has rejected seams '
                           f'{[(r["seam_id"], r["reason"]) for r in path["rejected"]]}: '
                           'fix or remove them before generating a program (no partial programs)')
    if path['tool'] != params.tcp_link:
        raise ProgramError(f'path planned for tool {path["tool"]}, cell model uses '
                           f'{params.tcp_link}')
    proc_by_id = {r['seam_id']: r for r in process['seams']}
    check_tcp(client, cal, params)
    load_part_into_scene(client, job, part, cal)
    base_T_part = cal.base_T_part
    home = np.asarray(params.home_joints_rad)
    scale = (params.velocity_scaling, params.acceleration_scaling)
    prog = _Builder(home)

    def air_move(goal: np.ndarray, label: str) -> None:
        if np.max(np.abs(goal - prog.end)) < 1e-6:
            return
        traj = None
        for attempt in range(params.air_planning_retries):
            # OMPL is randomised; move_group validates every returned path (ValidateSolution).
            traj = client.plan_joint(prog.end, goal, params.air_planning_time_s,
                                     params.air_planning_attempts, *scale)
            if traj is not None:
                break
            LOG.warning('air move %s: planning attempt %d failed', label, attempt + 1)
        if traj is None:
            raise ProgramError(f'air move {label}: no collision-free joint path (OMPL, '
                               f'{params.air_planning_retries} tries)')
        traj.positions[-1] = goal  # remove the 1e-4 rad goal tolerance so segments chain exactly
        check_limits(f'air move {label}', traj, params, 'air')
        prog.add('air', None, traj)

    for seam in sorted(path['seams'], key=lambda s: s['sequence']):
        sid = seam['seam_id']
        row = proc_by_id[sid]
        to_base = [frames.compose(base_T_part, frames.pose_to_matrix(p))       # A61
                   for p in seam['weld_poses']]
        approach = frames.compose(base_T_part, frames.pose_to_matrix(seam['approach_pose']))
        retract = frames.compose(base_T_part, frames.pose_to_matrix(seam['retract_pose']))
        joints = {k: np.asarray(v, float) for k, v in seam['joint_path'].items()}
        tcp = verify_joint_path(client, params, sid, joints, approach, to_base, retract)

        air_move(joints['approach'][0], f'to {sid}')                         # A62
        app = _time_trapezoid(joints['approach'], params)
        check_limits(f'{sid} approach', app, params, 'approach')
        app_id = prog.add('approach', sid, app)
        weld = _time_weld(joints['weld'], tcp, seam['travel_speed_m_s'], params)
        check_limits(f'{sid} weld', weld, params, 'weld')
        weld_id = prog.add('weld', sid, weld, seam['travel_speed_m_s'])
        ret = _time_trapezoid(joints['retract'], params)
        check_limits(f'{sid} retract', ret, params, 'retract')
        prog.add('retract', sid, ret)
        prog.event('WeldStart', sid, app_id, row)                              # A63
        prog.event('WeldEnd', sid, weld_id, row)
        LOG.info('%s: approach %.1f s, weld %.1f s at %.1f mm/s, retract %.1f s', sid,
                 app.times[-1], weld.times[-1], seam['travel_speed_m_s'] * 1000, ret.times[-1])
    air_move(home, 'to home')

    motion_time = sum(s['duration_s'] for s in prog.segments)
    dwell = sum(e['timing']['pre_flow_s'] for e in prog.events if e['type'] == 'WeldStart') + \
        sum(e['timing']['crater_fill_s'] + e['timing']['post_flow_s']
            for e in prog.events if e['type'] == 'WeldEnd')
    rev = job.next_program_rev()
    inputs = [job.path_json, job.process_json, job.part_json, job.repo.active_calibration]
    doc = jsonio.make_header(CREATED_BY, inputs, job.dir)
    doc.update({
        'program_rev': rev, 'part_no': part['part_no'], 'part_rev': part['part_rev'],
        'frame': params.base_frame, 'tool': params.tcp_link,
        'calibration': {'calibration_id': cal.calibration_id, 'sha256': cal.sha256,
                        'status': cal.status},
        'joint_names': list(params.joint_names), 'home_joints': [float(q) for q in home],
        'segments': prog.segments, 'events': prog.events,
        'cycle_time_s': motion_time + dwell,
        'limits': {'velocity_scaling': params.velocity_scaling,
                   'acceleration_scaling': params.acceleration_scaling},
    })
    jsonio.write_json(job.program_json(rev), doc, 'program')                  # A64
    LOG.info('program %s: %d segments, %d events, cycle %.1f s (motion %.1f s)', rev,
             len(prog.segments), len(prog.events), doc['cycle_time_s'], motion_time)
    return rev, doc
