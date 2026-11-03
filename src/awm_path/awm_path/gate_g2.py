"""Entry gate G2 runner (WP-S S.4 C4; TST-002 Rev 0.3 draft §5.2 G2 = C1 scene sanity incl. contact-zone poses).

    ros2 run awm_path g2_gate --poses docs/generated/spike/g2_poses.yaml --repo <spike_repo> --job <job> \
        --out-dir docs/generated/spike [--head-model H-BOX]

Starts ``sim_mock.launch.py`` headless (mock hardware only) with the cell frames of ``cell/model/frames.yaml`` and the
cell manifest ``cell/model/scene_manifest.json``, loads the job scene (D19) through :class:`MoveGroupCellScene`,
resolves every pose of the pose file (data, not source) with the analytic IK, and checks F4 validity, the F9 contact
check where listed, and — for the record only — the F5 weld clearance (5 mm). Writes ``g2_log.json`` and
``g2_log.md`` (records of TST-002 §9: commit and dirty flag, input hashes, backends, scene hash, head model).
Exit code 0 only if G2 passes. A failing G2 stops the sweep (S.5).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from awm_core.frames import make_transform, quat_to_matrix
from awm_core.hashing import sha256_file
from awm_path.core.contact import ContactParams, PartMesh, contact_check, point_triangle_distances
from awm_path.core.gate import GatePose, GateResult, gate_summary, load_pose_specs, resolve_pose
from awm_path.core.kinematics import fk_tcp
from awm_path.core.scene_files import JobObject, part_object_files, world_T_object

#: F5 weld clearance recorded with every pose (TST-002 C2, D16 weld_clearance_mm). Information only in G2.
WELD_CLEARANCE_MM = 5.0
#: Radius (mm) of the nearest-part-face annotation around tcp_laser.
_NEAR_PART_MM = 50.0


def _fmt(values) -> str:
    return ' '.join(f'{float(v):.12g}' for v in values)


def launch_args_from_frames(frames_yaml) -> Dict[str, str]:
    """``sim_mock.launch.py`` pose arguments from the export-scene ``frames.yaml`` (mm, quaternion xyzw)."""
    doc = yaml.safe_load(Path(frames_yaml).read_text())
    base, axis = doc['world_T_base'], doc['world_T_positioner_axis']
    rot = quat_to_matrix(axis['quat_xyzw'])
    direction = np.asarray(axis['direction'], dtype=float)
    local = rot.T @ (direction / np.linalg.norm(direction))
    return {'base_xyz_mm': _fmt(base['position_mm']), 'base_quat_xyzw': _fmt(base['quat_xyzw']),
            'positioner_xyz_mm': _fmt(axis['position_mm']), 'positioner_quat_xyzw': _fmt(axis['quat_xyzw']),
            'positioner_axis_xyz': _fmt(local)}


def _nearest_face(part: PartMesh, point: np.ndarray) -> Optional[Tuple[str, float]]:
    idx = part.local(point, _NEAR_PART_MM)
    if not len(idx):
        return None
    d = point_triangle_distances(point[None], part.triangles_mm[idx])[0]
    k = int(np.argmin(d))
    return part.face_ids[int(part.face_index[idx[k]])], float(d[k])


def run_gate(scene, kin, poses: Sequence[GatePose], seams: Mapping[str, Tuple[Tuple[str, ...], Tuple[str, ...]]],
             part: PartMesh, part_object: JobObject, contact_params: ContactParams,
             clearance_mm: float = WELD_CLEARANCE_MM) -> List[GateResult]:
    """Evaluate every pose. ``scene`` provides ``world_T_links``, ``check`` and ``check_clearance`` (mm / deg);
    ``part_object`` is the D19 part object (its parent frame is respected, ``scene_files.world_T_object``)."""
    zeros = [0.0] * 6
    w = scene.world_T_links(zeros, 0.0, ['base_link', 'tool0', 'tcp_laser'])
    world_T_base = w['base_link']
    tool0_T_tcp = np.linalg.inv(w['tool0']) @ w['tcp_laser']
    results: List[GateResult] = []
    for pose in poses:
        t0 = time.perf_counter()
        theta = pose.theta_deg
        world_T_part = world_T_object(part_object, scene.world_T_links(zeros, theta, ['positioner_flange'])[
            'positioner_flange'])
        q, how = resolve_pose(pose, world_T_base, world_T_part, tool0_T_tcp, kin)
        t1 = time.perf_counter()
        (f4,) = scene.check([q], theta)
        t2 = time.perf_counter()
        (clr,) = scene.check_clearance([q], theta, clearance_mm)
        t3 = time.perf_counter()
        world_T_tcp = world_T_base @ fk_tcp(kin, q, tool0_T_tcp)
        part_T_tcp = np.linalg.inv(world_T_part) @ world_T_tcp
        contact = None
        if 'contact' in pose.checks:
            contact = contact_check(part, part_T_tcp, seams[pose.seam], contact_params)
        t4 = time.perf_counter()
        valid = bool(f4.valid) and (contact is None or contact.ok)
        near = _nearest_face(part, part_T_tcp[:3, 3])
        details = {
            'theta_deg': theta, 'q_deg': [float(x) for x in q], 'ik': how,
            'validity': bool(f4.valid), 'clearance_5mm_valid': bool(clr.valid),
            'clearance_contacts': list(clr.contacts),
            'tcp_world_mm': [float(x) for x in world_T_tcp[:3, 3]],
            'tcp_part_mm': [float(x) for x in part_T_tcp[:3, 3]],
            'nearest_part_face': None if near is None else {'face_id': near[0], 'distance_mm': near[1]},
            'time_s': {'resolve': t1 - t0, 'validity': t2 - t1, 'clearance': t3 - t2, 'contact': t4 - t3},
        }
        results.append(GateResult(pose, valid, tuple(f4.contacts), contact, details))
    return results


def _contact_dict(c) -> Optional[Dict[str, Any]]:
    if c is None:
        return None
    d = c.tcp_seam_distance_mm
    return {'ok': c.ok, 'reason': c.reason, 'tcp_seam_distance_mm': d if np.isfinite(d) else None,
            'tcp_nearest_other': list(c.tcp_nearest_other) if c.tcp_nearest_other else None,
            'offending': [list(x) for x in c.offending], 'samples': c.samples, 'triangles': c.triangles}


def _caught_by(r: GateResult) -> str:
    causes = []
    if not r.details['validity']:
        causes.append('validity')
    if r.contact is not None and not r.contact.ok:
        causes.append(f'contact:{r.contact.reason}')
    return ', '.join(causes) or '-'


def write_gate_log(json_path, md_path, record: Mapping[str, Any], results: Sequence[GateResult],
                   notes: Sequence[str] = ()):
    """Write the G2 log (JSON record + Markdown table + notes of the pose file). Returns the two paths."""
    summary = gate_summary(results)
    poses = [{'id': r.pose.pose_id, 'kind': r.pose.kind, 'description': r.pose.description,
              'expect_valid': r.pose.expect_valid, 'expect_objects': list(r.pose.expect_objects),
              'valid': r.valid, 'passed': r.passed, 'expected_object_seen': r.expected_object_seen,
              'contacts': list(r.contacts), 'caught_by': _caught_by(r), 'contact': _contact_dict(r.contact),
              **(r.details or {})} for r in results]
    json_path, md_path = Path(json_path), Path(md_path)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps({'record': dict(record), 'summary': summary, 'notes': list(notes),
                                     'poses': poses}, indent=1,
                                    allow_nan=False) + '\n')
    c = summary['counts']
    lines = [f'# TST-002 G2 (C1 scene sanity) — {"PASS" if summary["pass"] else "FAIL"}', '',
             f'Record: `{json_path.name}` (commit, input hashes, scene hash, backends, head model).', '',
             f'- colliding invalid: {c["colliding"][0]} / {c["colliding"][1]}',
             f'- free valid: {c["free"][0]} / {c["free"][1]}',
             f'- contact-zone invalid: {c["contact_zone"][0]} / {c["contact_zone"][1]}',
             f'- expected object in a contact pair: {summary["expected_objects_seen"][0]} / '
             f'{summary["expected_objects_seen"][1]} (reported, not a criterion)',
             f'- failing: {", ".join(summary["failing"]) or "none"}; too few poses: '
             f'{", ".join(summary["missing"]) or "none"}', '',
             '| Pose | Kind | θ deg | Label | Expect | Result | Caught by | Contact pairs (F4) | 5 mm clear | Note |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for p in poses:
        pairs = '; '.join(p['contacts'][:4]) + (' …' if len(p['contacts']) > 4 else '')
        lines.append(f'| {p["id"]} | {p["kind"]} | {p.get("theta_deg", 0):g} | {p.get("ik", {}).get("label", "")} | '
                     f'{"valid" if p["expect_valid"] else "invalid"} | '
                     f'{"valid" if p["valid"] else "invalid"} {"PASS" if p["passed"] else "FAIL"} | {p["caught_by"]} | '
                     f'{pairs or "-"} | {"yes" if p.get("clearance_5mm_valid") else "no"} | {p["description"]} |')
    if notes:
        lines += ['', '## Notes', ''] + [f'- {n}' for n in notes]
    md_path.write_text('\n'.join(lines) + '\n')
    return json_path, md_path


def _git(args: List[str], cwd: Path) -> str:
    try:
        return subprocess.run(['git', *args], cwd=cwd, capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return ''


def _rel(path: Path) -> str:
    return os.path.relpath(Path(path).resolve(), Path.cwd())


def main(argv: Optional[List[str]] = None) -> int:
    """CLI: launch the mock-hardware scene, run G2, write the log. Exit 0 = G2 pass."""
    ap = argparse.ArgumentParser(prog='g2_gate', description='Spike / simulation tool: starts its own sim_mock launch '
                                 '(mock hardware only).\n\n' + __doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--poses', required=True, help='G2 pose file (data, docs/generated/spike/)')
    ap.add_argument('--repo', required=True, help='repository root with cell/model and jobs/<job>')
    ap.add_argument('--job', required=True)
    ap.add_argument('--out-dir', required=True, help='folder for g2_log.json / g2_log.md')
    ap.add_argument('--head-model', default='H-BOX', help='head model name recorded with the result')
    ap.add_argument('--log-dir', default=None, help='folder for the launch log (default: a temp folder)')
    args = ap.parse_args(argv)

    import rclpy
    from awm_bringup.testing import SimLaunch
    from awm_path.backends.move_group import MoveGroupCellScene
    from awm_path.core.kinematics import UrAnalyticKinematics

    repo, poses_file = Path(args.repo), Path(args.poses)
    job = repo / 'jobs' / args.job
    frames = repo / 'cell' / 'model' / 'frames.yaml'
    cell_manifest = repo / 'cell' / 'model' / 'scene_manifest.json'
    placement = job / 'placement.json'
    pose_doc = yaml.safe_load(poses_file.read_text())
    poses, seams = load_pose_specs(pose_doc)
    launch_args = dict(launch_args_from_frames(frames), cell_manifest=str(cell_manifest.resolve()))
    contact_params = ContactParams()
    kin = UrAnalyticKinematics()
    with SimLaunch(log_dir=args.log_dir, launch_file='sim_mock.launch.py', launch_args=launch_args) as sim:
        rclpy.init()
        node = rclpy.create_node('awm_g2_gate')
        try:
            scene = MoveGroupCellScene(node, cell_manifest, placement, job, head_model=args.head_model)
            scene.load(timeout_s=180.0)
            part_obj, mesh_file, faces_file = part_object_files(scene.job)
            part = PartMesh.from_files(mesh_file, faces_file)
            t0 = time.perf_counter()
            results = run_gate(scene, kin, poses, seams, part, part_obj, contact_params)
            wall = time.perf_counter() - t0
        except Exception:
            print(sim.log_tail(60), file=sys.stderr)
            raise
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    here = poses_file.resolve().parent
    record = {
        'created_by': 'awm_path.gate_g2', 'created_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'commit': _git(['rev-parse', 'HEAD'], here), 'dirty': bool(_git(['status', '--porcelain'], here)),
        'inputs': [{'name': _rel(p), 'sha256': sha256_file(p)} for p in (poses_file, frames, cell_manifest,
                                                                          placement, mesh_file, faces_file)],
        'scene_backend': scene.record, 'kinematics_backend': kin.name, 'head_model': args.head_model,
        'psi_case': 'n/a (scene sanity)', 'contact_params': contact_params.as_record(),
        'weld_clearance_mm': WELD_CLEARANCE_MM, 'launch_args': {k: v for k, v in launch_args.items()
                                                               if k != 'cell_manifest'},
        'wall_time_s': wall, 'assumptions': [
            'part and base poses from CAD component placements, no REF_* markers (OI-23)',
            'H-BOX head, nozzle Ø20 x 40 mm, s_c 0.3 mm, TCP at the nozzle tip (TST-002 §3.1)',
            'contact point tolerance 0.15 mm (F9 gives no value)'],
    }
    out = Path(args.out_dir)
    j, md = write_gate_log(out / 'g2_log.json', out / 'g2_log.md', record, results,
                           notes=[str(n) for n in pose_doc.get('notes') or ()])
    s = gate_summary(results)
    print(f'G2 {"PASS" if s["pass"] else "FAIL"}: {s["counts"]}; failing {s["failing"]}; log {j}, {md}')
    return 0 if s['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
