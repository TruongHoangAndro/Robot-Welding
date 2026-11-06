"""Generate ``fixtures/s4_synthetic`` for the S4 playback tests (vitest). Test data only — no part or cell data.

Runs inside awm_dev (needs xacro, awm_path, awm_description)::

    scripts/dx "python3 src/awm_web/scripts/make_s4_fixture.py"

A synthetic cell (base and positioner frames rotated about Z and offset, so the URDF chain is exercised) is
rendered with ``awm_path.description.load_planning_description`` (head model ``hbox``: no CAD files). A smooth
7-joint motion (robot q + positioner θ, degrees) gives TCP poses **in the part frame** with the planner's own
forward kinematics (``batch_fk`` · ``tool0_T_tcp``, ``base_link_T_axis`` · Rx(θ) · ``flange_T_part``), exactly as
the G1.3 golden check does. The browser test runs the same q / θ through the URDF with urdf-loader and must
reproduce these poses (Review Focus 5). Outputs: robot_description.json, path.json (D07-shaped), program_2_1.json
(G1.5 contract: one 7-joint segment per controller ``robot_positioner``), program_2_0.json (robot / positioner
segments with ``hold_deg``), placement.json, seams.json.
"""

from __future__ import annotations

import json
import math
import re
import tempfile
from pathlib import Path

import numpy as np

from awm_core import frames
from awm_path.core.batch_kin import batch_fk
from awm_path.core.sync import rot_x
from awm_path.description import load_planning_description

OUT = Path(__file__).resolve().parents[1] / 'fixtures' / 's4_synthetic'


def _qz(deg: float):
    a = math.radians(deg) / 2.0
    return [0.0, 0.0, math.sin(a), math.cos(a)]


def _frames_yaml() -> str:
    base_q, axis_q = _qz(20.0), _qz(30.0)
    d = [math.cos(math.radians(30.0)), math.sin(math.radians(30.0)), 0.0]
    return ('# synthetic test frames (make_s4_fixture.py)\nschema_version: "2.0"\ncreated_by: "test"\n'
            'created_at: "2026-11-06T00:00:00Z"\ninputs: []\nschema: "awm_cell_frames/0.1"\nunit: "mm"\n'
            f'world_T_base:\n  position_mm: [100.0, 50.0, 0.0]\n  quat_xyzw: {base_q}\n'
            f'world_T_positioner_axis:\n  position_mm: [80.0, -420.0, 160.0]\n  quat_xyzw: {axis_q}\n'
            f'  point_mm: [80.0, -420.0, 160.0]\n  direction: {d}\n')


def _pose(t: np.ndarray) -> dict:
    return {'position_mm': [round(float(v), 9) for v in t[:3, 3]],
            'quat_xyzw': [round(float(v), 12) for v in frames.matrix_to_quat(t[:3, :3])]}


def main() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        fy = Path(tmp) / 'frames.yaml'
        fy.write_text(_frames_yaml())
        desc = load_planning_description(fy, 'hbox')
    cell = desc.sync_cell
    ftp_pose = {'position_mm': [12.0, -25.0, 40.0], 'quat_xyzw': [0.2, -0.1, 0.3, 0.92]}
    n = np.linalg.norm(ftp_pose['quat_xyzw'])
    ftp_pose['quat_xyzw'] = [float(v / n) for v in ftp_pose['quat_xyzw']]
    ftp = frames.make_transform(ftp_pose['position_mm'], frames.quat_to_matrix(ftp_pose['quat_xyzw']))

    n_pts, dt = 31, 0.4
    u = np.linspace(0.0, 1.0, n_pts)
    q0 = np.array([95.0, -105.0, 95.0, -80.0, -85.0, 10.0])
    q = q0 + np.outer(np.sin(np.pi * u), [8.0, -6.0, 10.0, 12.0, -9.0, 25.0]) + np.outer(u, [3, 2, -4, 5, 6, -40])
    theta = -30.0 + 95.0 * u
    fk = batch_fk(q) @ cell.tool0_T_tcp
    points, timed = [], []
    for k in range(n_pts):
        part_t_tcp = np.linalg.inv(cell.base_link_T_axis @ rot_x(theta[k]) @ ftp) @ fk[k]
        status = 'ok'
        points.append({'s_mm': round(2.0 * k, 6), 'pose_mm': _pose(part_t_tcp), 'psi_deg': 0.0,
                       'theta_deg': round(float(theta[k]), 9), 'q_deg': [round(float(v), 9) for v in q[k]],
                       'min_clearance_mm': None, 'status': status})
        timed.append({'t_s': round(k * dt, 6), 'q_deg': points[-1]['q_deg'], 'theta_deg': points[-1]['theta_deg']})
    path = {'schema_version': '2.1', 'created_by': 'make_s4_fixture.py (synthetic test data)',
            'created_at': '2026-11-06T00:00:00Z', 'inputs': [], 'frame': 'part', 'tcp': 'tcp_laser',
            'collision_checked': False,
            'seams': [{'seam_id': 'S01', 'arc': {'index': 1, 'of': 1, 'start_s_mm': 0.0, 'end_s_mm': 60.0},
                       'mode': 'sync', 'config_label': 'S+E+W+', 'travel_speed_mm_s': 5.0, 'points': points,
                       'timed': [{'kind': 'weld', 'points': timed}],
                       'planning': {'max_travel_speed_mm_s': 4.2, 'speed_within_limits': False}}],
            'rejected': [{'seam_id': 'S02', 'arc': {'index': 1, 'of': 1, 'start_s_mm': 0.0, 'end_s_mm': 10.0},
                          'reason': 'synthetic rejection', 'coverage_fraction': 0.5, 'mode': 'sync',
                          'failing_points': [{'index': 2, 's_mm': 4.0, 'reasons': {'joint_limit': 3},
                                              'status': 'limit'}],
                          'failing_ranges': [{'s_from_mm': 4.0, 's_to_mm': 4.0, 'status': 'limit', 'points': 1}]}]}
    home = [90.0, -90.0, 90.0, -90.0, -90.0, 0.0]
    first = list(q[0]) + [float(theta[0])]
    last = list(q[-1]) + [float(theta[-1])]

    def pt(t, pos):
        return {'time_from_start_s': t, 'positions_deg': [round(float(v), 9) for v in pos],
                'velocities_deg_s': [0.0] * len(pos), 'accelerations_deg_s2': [0.0] * len(pos)}

    weld = [pt(round(k * dt, 6), list(q[k]) + [float(theta[k])]) for k in range(n_pts)]
    names = {'robot': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                       'wrist_3_joint'], 'positioner': ['positioner_joint']}
    seg = {'controller': 'robot_positioner', 'hold_deg': [], 'tcp_speed_mm_s': None}
    prog21 = {'schema_version': '2.1', 'program_rev': 'P001', 'collision_checked': False, 'joint_names': names,
              'home': {'robot_deg': home, 'positioner_deg': 0.0},
              'segments': [
                  {**seg, 'segment_id': 'G001', 'kind': 'air', 'seam_id': None,
                   'points': [pt(0.0, home + [0.0]), pt(2.0, first)]},
                  {**seg, 'segment_id': 'G002', 'kind': 'weld', 'seam_id': 'S01', 'positioner_mode': 'sync',
                   'tcp_speed_mm_s': 5.0, 'points': weld},
                  {**seg, 'segment_id': 'G003', 'kind': 'retract', 'seam_id': 'S01',
                   'points': [pt(0.0, last), pt(1.5, home + [0.0])]}],
              'events': [{'event_id': 'V001', 'type': 'WeldStart', 'seam_id': 'S01', 'after_segment': 'G001',
                          'wps_id': 'WPS-TEST', 'preset': 1},
                         {'event_id': 'V002', 'type': 'WeldEnd', 'seam_id': 'S01', 'after_segment': 'G002',
                          'wps_id': 'WPS-TEST', 'preset': 1}],
              'cycle_time_s': 15.5}
    prog20 = {'schema_version': '2.0', 'program_rev': 'P001', 'joint_names': names,
              'home': {'robot_deg': home, 'positioner_deg': 0.0},
              'segments': [
                  {'segment_id': 'G001', 'controller': 'positioner', 'kind': 'index', 'seam_id': None,
                   'hold_deg': home, 'tcp_speed_mm_s': None, 'points': [pt(0.0, [0.0]), pt(1.0, [45.0])]},
                  {'segment_id': 'G002', 'controller': 'robot', 'kind': 'weld', 'seam_id': 'S01',
                   'hold_deg': [45.0], 'tcp_speed_mm_s': 5.0,
                   'points': [pt(0.0, home), pt(2.0, list(q[0]))]}],
              'events': [], 'cycle_time_s': 3.0}
    placement = {'schema_version': '2.0', 'source': 'study', 'verified': False, 'flange_T_part': ftp_pose,
                 'objects': []}
    seams = {'schema_version': '2.1', 'seams': [
        {'seam_id': 'S02', 'geometry': {'spacing_mm': 2.0, 'points': [
            {'s_mm': 2.0 * k, 'p_mm': [2.0 * k, 0.0, 0.0]} for k in range(6)]}}]}
    OUT.mkdir(parents=True, exist_ok=True)
    files = {'robot_description.json': {'urdf': re.sub(r'<!--.*?-->', '', desc.urdf, flags=re.S), 'head_model': 'hbox', 'unit': 'm, rad (URDF, REP 103)'},
             'path.json': path, 'program_2_1.json': prog21, 'program_2_0.json': prog20,
             'placement.json': placement, 'seams.json': seams}
    for name, doc in files.items():
        (OUT / name).write_text(json.dumps(doc, indent=1) + '\n')
    print(f'wrote {len(files)} files to {OUT}')


if __name__ == '__main__':
    main()
