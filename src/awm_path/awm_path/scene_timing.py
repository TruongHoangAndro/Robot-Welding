"""C5 timing of the scene checks per candidate state (WP-S S.4 C5; TST-002 Rev 0.3 draft §5.3 C5).

    ros2 run awm_path scene_timing --repo <repo> --job <job> --joint <joint_edge_id> --n1 <faces> --n2 <faces> \
        --arc-start-deg <deg> --arc-end-deg <deg> --out-json <timing.json> --out-csv <candidates.csv>

Starts ``sim_mock.launch.py`` headless (mock hardware only) with the export-scene frames and cell manifest, builds
seam frames on the circle through the D20 preview frames of one joint edge (``topology.json``; an approximation of
the seam for timing only — S.5 uses the D04 seam geometry), runs the S.3 sweep (Ψ-FREE or Ψ-WIRE) at the F1 pitch,
scores every candidate (F4, F5 levels, F9) and records the time per state with the backend names.
All ids and faces come from the command line (data), never from source.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import List, Mapping, Optional, Sequence

import numpy as np

from awm_core import units


def _circumcentre(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    ab, ac = b - a, c - a
    n = np.cross(ab, ac)
    nn = float(n @ n)
    if nn < 1e-18:
        raise ValueError('preview points are collinear')
    return a + (np.cross(n, ab) * (ac @ ac) + np.cross(ac, n) * (ab @ ab)) / (2.0 * nn)


def circle_frames(preview: Sequence[Mapping], phi_start_deg: float, phi_end_deg: float, n: int) -> List[np.ndarray]:
    """``n`` seam frames (part frame, mm) on the circle through the preview frames, from ``phi_start`` to ``phi_end``
    (deg, measured from preview frame 0 in its travel direction). X = tangent (travel), Z = radial (out of the
    material, as the preview Z), Y = Z × X."""
    if len(preview) < 3 or n < 1:
        raise ValueError('need at least 3 preview frames and n >= 1')
    pts = [np.asarray(p['point_mm'], dtype=float) for p in preview]
    c = _circumcentre(*pts[:3])
    r = float(np.linalg.norm(pts[0] - c))
    e0 = (pts[0] - c) / r
    if float(e0 @ np.asarray(preview[0]['z'], dtype=float)) < 0:
        raise ValueError('preview Z is not radial outwards')
    x0 = np.asarray(preview[0]['x'], dtype=float)
    e1 = x0 - e0 * float(x0 @ e0)
    e1 /= np.linalg.norm(e1)
    out = []
    for k in range(n):
        phi = phi_start_deg if n == 1 else phi_start_deg + (phi_end_deg - phi_start_deg) * k / (n - 1)
        a = units.deg_to_rad(phi)
        z = math.cos(a) * e0 + math.sin(a) * e1
        x = -math.sin(a) * e0 + math.cos(a) * e1
        t = np.eye(4)
        t[:3, :3] = np.stack([x, np.cross(z, x), z], axis=1)
        t[:3, 3] = c + r * z
        out.append(t)
    return out


def main(argv: Optional[List[str]] = None) -> int:
    """CLI (mock hardware only): sweep + scene scores on one arc, timing record."""
    ap = argparse.ArgumentParser(prog='scene_timing', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--repo', required=True)
    ap.add_argument('--job', required=True)
    ap.add_argument('--joint', required=True, help='joint_edge_id in cache/view/topology.json')
    ap.add_argument('--n1', required=True, help='comma-separated n1 face ids (D04 faces.n1)')
    ap.add_argument('--n2', required=True, help='comma-separated n2 face ids (D04 faces.n2)')
    ap.add_argument('--arc-start-deg', type=float, required=True)
    ap.add_argument('--arc-end-deg', type=float, required=True)
    ap.add_argument('--theta-deg', type=float, default=0.0, help='positioner angle')
    ap.add_argument('--psi-case', choices=['free', 'wire'], default='free')
    ap.add_argument('--head-model', default='H-BOX')
    ap.add_argument('--out-json', required=True)
    ap.add_argument('--out-csv', required=True)
    ap.add_argument('--log-dir', default=None)
    args = ap.parse_args(argv)

    import rclpy
    from awm_bringup.testing import SimLaunch
    from awm_core.hashing import sha256_file
    from awm_path.backends.move_group import MoveGroupCellScene
    from awm_path.core.contact import ContactParams, PartMesh
    from awm_path.core.kinematics import UrAnalyticKinematics
    from awm_path.core.scoring import SceneScoreParams, score_candidates
    from awm_path.core.sweep import PlannerParams, required_points, sweep, write_candidates_csv
    from awm_path.gate_g2 import _git, launch_args_from_frames

    repo = Path(args.repo)
    job = repo / 'jobs' / args.job
    frames_file = repo / 'cell' / 'model' / 'frames.yaml'
    cell_manifest = repo / 'cell' / 'model' / 'scene_manifest.json'
    placement = job / 'placement.json'
    topo_file = job / 'cache' / 'view' / 'topology.json'
    topo = json.loads(topo_file.read_text())
    joint = [j for j in topo['joints'] if j['joint_edge_id'] == args.joint]
    if len(joint) != 1:
        raise SystemExit(f'joint {args.joint} not in {topo_file}')
    preview = joint[0]['preview_frames']
    params = PlannerParams()
    pts = [np.asarray(p['point_mm'], dtype=float) for p in preview]
    r = float(np.linalg.norm(pts[0] - _circumcentre(*pts[:3])))
    length = r * units.deg_to_rad(abs(args.arc_end_deg - args.arc_start_deg))
    n = required_points(length, units.rad_to_deg(1.0 / r), params, closed=False)
    part_frames = circle_frames(preview, args.arc_start_deg, args.arc_end_deg, n)
    psi = params.psi_free if args.psi_case == 'free' else params.psi_wire
    seam_faces = (tuple(args.n1.split(',')), tuple(args.n2.split(',')))
    kin = UrAnalyticKinematics()
    launch_args = dict(launch_args_from_frames(frames_file), cell_manifest=str(cell_manifest.resolve()))
    with SimLaunch(log_dir=args.log_dir, launch_file='sim_mock.launch.py', launch_args=launch_args) as sim:
        rclpy.init()
        node = rclpy.create_node('awm_scene_timing')
        try:
            scene = MoveGroupCellScene(node, cell_manifest, placement, job, head_model=args.head_model)
            scene.load(timeout_s=180.0)
            part_obj = [o for o in scene.job.objects if o.role == 'part' and o.face_ids][0]
            part = PartMesh.from_files(job / 'cache' / 'collision.stl', job / 'cache' / 'collision_faces.json')
            w = scene.world_T_links([0.0] * 6, args.theta_deg, ['base_link', 'tool0', 'tcp_laser',
                                                                 'positioner_flange'])
            tool0_T_tcp = np.linalg.inv(w['tool0']) @ w['tcp_laser']
            base_T_part = np.linalg.inv(w['base_link']) @ w['positioner_flange'] @ part_obj.parent_T_obj_mm
            frames_base = [base_T_part @ f for f in part_frames]
            t0 = time.perf_counter()
            layers = sweep(frames_base, tool0_T_tcp, kin, params, psi)
            t_sweep = time.perf_counter() - t0
            scene.set_theta(args.theta_deg)
            scored = score_candidates(layers, frames_base, base_T_part, args.theta_deg, scene, part, seam_faces,
                                      ContactParams(), SceneScoreParams())
        except Exception:
            print(sim.log_tail(60), file=sys.stderr)
            raise
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    write_candidates_csv(args.out_csv, layers, scored.columns)
    t = dict(scored.timing, sweep_s=t_sweep)
    per_point_valid = [sum(s.valid for s in row) for row in scored.scores]
    here = Path.cwd()
    record = {
        'created_by': 'awm_path.scene_timing', 'created_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'commit': _git(['rev-parse', 'HEAD'], here), 'dirty': bool(_git(['status', '--porcelain'], here)),
        'inputs': [{'name': p.name, 'sha256': sha256_file(p)} for p in (frames_file, cell_manifest, placement,
                                                                         topo_file)],
        'scene_backend': scene.record, 'kinematics_backend': kin.name, 'head_model': args.head_model,
        'psi_case': args.psi_case, 'theta_deg': args.theta_deg, 'joint': args.joint,
        'seam_faces': {'n1': list(seam_faces[0]), 'n2': list(seam_faces[1])},
        'seam_geometry': ('circle through the D20 preview frames of the joint edge (approximation for timing; '
                          'S.5 uses the D04 seam)'),
        'arc_deg': [args.arc_start_deg, args.arc_end_deg], 'points': n, 'pitch_mm': length / max(n - 1, 1),
        'candidates_csv': {'file': Path(args.out_csv).name, 'sha256': sha256_file(args.out_csv)},
        'timing': t,
        'projection_per_seam_s': {
            'note': 'time of this run for its points; C5 budget is 30 s per seam incl. DP (S.5)',
            'scene_and_contact_s': t['total_s'], 'sweep_s': t_sweep},
        'valid_nodes_per_point': per_point_valid,
        'contact_ok_points': [c.ok for c in scored.contact],
    }
    Path(args.out_json).write_text(json.dumps(record, indent=1, allow_nan=False) + '\n')
    print(f'{n} points, {t["states_checked"]} states: validity {t["validity_s"]:.2f} s, clearance '
          f'{t["clearance_s"]:.2f} s, contact {t["contact_s"]:.2f} s, per state {t["per_state_s"]:.5f} s, '
          f'sweep {t_sweep:.2f} s; points with >= 1 valid node: {sum(v > 0 for v in per_point_valid)}/{n}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
