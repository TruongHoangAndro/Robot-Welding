"""WP-S S.6 spike run: J1 Mode C clearance check → TST-002 C9 (reported, does not gate; TST-002 Rev 0.3 §5.3, §9;
DES-001 Rev 0.6 draft §4.3 step 3, §6.3).

    ros2 run awm_path s6_run --input <s6_mode_c_input.yaml> --repo <repo> --job <job> --head-model H-BOX \
        --out-dir <records dir>

Spike / simulation tool: starts its own sim_mock launch (mock hardware only). All ids, faces, the contact-location
choice, ψ cases, overlap and the angular-step margin come from the data file, never from source.

1. Seam from the exact D20 joint edge (S.5 ``build_seams_doc``) → ``<job>/s6_spike_seam.json``.
2. Pose choice at θ = 0: the seam points (data-file step) whose seam Z faces the robot base, S.3 sweep ψ × 8 IK
   branches per ψ case, S.4 scene scores (F4, F5 at 5 / 10 mm, F9), ranked by the S.3 node cost with the F7
   tie-break (+ point index). The rank-1 pose is the result; the next ``alternates`` are checked as diagnostics.
3. θ sweep of each checked pose: 0 … ±(360° + overlap) at the B-03 step (sign: the contact advances along +X of the
   seam frame). Per θ: F4 validity, F5 clearance lower bound at the data-file levels (the pairs blocking the first
   level not passed = closest pair), F9 contact on part_T_tcp(θ) (TCP fixed in world, the part turns). F9 is also
   profiled on a finer θ grid (pure).
4. Records ``s6_results.json`` + ``.md`` in --out-dir.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

import numpy as np

from awm_core import jsonio, units
from awm_path.core.mode_c import failing_ranges, interval_bounds

CREATED_BY = 'awm_path.spike_s6'
SEAM_FILE = 's6_spike_seam.json'


# ---------------------------------------------------------------------------------------------------------------
# pure parts
# ---------------------------------------------------------------------------------------------------------------

def theta_summary(rows: List[Dict[str, Any]], weld_clearance_mm: float, step_deg: float, r_max_mm: float
                  ) -> Dict[str, Any]:
    """C9 summary of one θ sweep. Sets ``row['ok']`` = F4 ∧ clearance ≥ weld ∧ F9 on every row."""
    for r in rows:
        lb = r['clearance_lb_mm']
        r['ok'] = bool(r['f4_valid'] and lb is not None and lb >= weld_clearance_mm and r['contact_ok'])
    th = [r['theta_deg'] for r in rows]
    lbs = [r['clearance_lb_mm'] for r in rows]
    key = [-math.inf if v is None else v for v in lbs]
    m = min(key)
    at_min = [i for i, v in enumerate(key) if v == m]
    bounds = interval_bounds(lbs, step_deg, r_max_mm)
    f4_pairs: Counter = Counter()
    for r in rows:
        if not r['f4_valid']:
            f4_pairs.update(r['f4_contacts'])
    faces: Dict[str, float] = {}
    for r in rows:
        for fid, d in r.get('offending_faces', []):
            faces[fid] = min(faces.get(fid, math.inf), d)
    return {
        'C9_pass': all(r['ok'] for r in rows), 'states': len(rows), 'states_ok': sum(r['ok'] for r in rows),
        'min_clearance_lb_mm': None if m == -math.inf else m,
        'theta_at_min_deg': [th[i] for i in at_min],
        'closest_pairs_at_min': dict(Counter(rows[i]['closest_pair'] for i in at_min if rows[i]['closest_pair'])),
        'between_states_min_mm': None if any(b is None for b in bounds) or not bounds else min(bounds),
        'failing': {'any': failing_ranges(th, [r['ok'] for r in rows]),
                    'f4': failing_ranges(th, [r['f4_valid'] for r in rows]),
                    'clearance': failing_ranges(th, [v is not None and v >= weld_clearance_mm for v in lbs]),
                    'contact': failing_ranges(th, [r['contact_ok'] for r in rows])},
        'f4_pairs': dict(f4_pairs.most_common()),
        'contact_offending_faces_min_mm': dict(sorted(faces.items())),
    }


_RUN_KEYS = ('ok', 'f4_valid', 'clearance_lb_mm', 'closest_pair', 'contact_ok', 'contact_reason')


def compress_rows(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    """Runs of consecutive θ rows with the same verdict (ok, F4, clearance level, closest pair, F9)."""
    out: List[Dict[str, Any]] = []
    for r in rows:
        v = {k: r.get(k) for k in _RUN_KEYS}
        if out and all(out[-1][k] == v[k] for k in _RUN_KEYS):
            out[-1]['theta_to_deg'] = r['theta_deg']
            out[-1]['states'] += 1
            out[-1]['f4_contacts'] |= set(r.get('f4_contacts', []))
        else:
            out.append(dict(v, theta_from_deg=r['theta_deg'], theta_to_deg=r['theta_deg'], states=1,
                            f4_contacts=set(r.get('f4_contacts', []))))
    for o in out:
        o['f4_contacts'] = sorted(o['f4_contacts'])
    return out


def _fmt(v: Any, nd: int = 2) -> str:
    if v is None:
        return '—'
    if isinstance(v, float):
        return f'{v:.{nd}f}'
    return str(v)


def write_md(path: Path, rec: Mapping[str, Any]) -> None:
    """Human-readable summary of ``s6_results.json``."""
    g = rec['geometry']
    L: List[str] = [
        f'# WP-S S.6 — J1 Mode C clearance check ({rec["head_model"]}) — TST-002 C9 (reported, does not gate)', '',
        f'Record `s6_results.json`; commit `{rec["commit"]}`, dirty: {str(rec["dirty"]).lower()}, created '
        f'{rec["created_at"]}. Scene hash `{rec["scene_backend"]["scene_hash"]}`; kinematics '
        f'`{rec["kinematics_backend"]}`; scene backend `{rec["scene_backend"]["content"]["backend"]}`; clearance '
        f'links {", ".join(rec["clearance_links"])} (F5 method: {rec["clearance_method"]}).', '',
        f'Seam {rec["seam"]["seam_id"]} ({rec["seam"]["note"].split("|")[0].strip()}), r {g["seam_radius_mm"]:.3f} mm, '
        f'length {rec["seam"]["length_mm"]:.3f} mm, closing overlap {rec["seam"]["arc_overlap_mm"]} mm = '
        f'{g["overlap_deg"]:.2f}°; span {g["span_deg"]:.2f}°.', '',
        f'- F8 coaxiality (seam circle vs positioner axis, flange frame): centre offset {g["f8"]["offset_mm"]:.4f} mm, '
        f'axis angle {g["f8"]["angle_deg"]:.4f}° → {"PASS" if g["f8"]["pass"] else "FAIL"} '
        f'(tol {g["f8"]["tol_mm"]} mm / {g["f8"]["tol_deg"]}°).',
        f'- Rotating set r_max {g["r_max_mm"]:.2f} mm ({g["r_max_object"]}); B-03 step Δθ_max = 2·({g["clearance_mm"]} − '
        f'{g["margin_mm"]}) / r_max = {g["step_max_deg"]:.3f}° → {g["states"]} angles at {g["step_deg"]:.3f}°.',
        f'- Flange model check (world_T_flange(0)·Rx(θ) vs /compute_fk): max {g["flange_model_check"]["max_pos_mm"]:.2e} '
        f'mm / {g["flange_model_check"]["max_rot_deg"]:.2e}° at θ {g["flange_model_check"]["thetas_deg"]}.',
        f'- F9 per θ: the TCP is fixed in world; the contact check runs on part_T_tcp(θ) = (world_T_flange(θ)·'
        f'flange_T_part)⁻¹·world_T_tcp, i.e. on the seam point that is under the nozzle at that angle '
        f'(s advances r·|θ|). For a seam within F8 and a nozzle that is a body of revolution about the beam axis, '
        f'the set of nozzle poses relative to the part over one full turn is the same for every Mode C robot pose '
        f'on the circle and every ψ (only the θ at which each seam point is reached changes): the F9 verdict over '
        f'the turn does not depend on the chosen pose; F4 / F5 do.', '']
    L += ['## Pose choice at θ = 0', '',
          '| ψ case | Points facing the robot / sampled | Candidates | Kinematically valid | F4 valid | Clearance ≥ 5 mm '
          '| F9 points ok | Valid nodes | Chosen (point, s, ψ, label, cost) |', '|---|---|---|---|---|---|---|---|---|']
    for case, c in rec['cases'].items():
        n = c['counts']
        ch = c['ranking'][0] if c['ranking'] else None
        chs = (f'{ch["point"]}, s {ch["s_mm"]:.2f}, ψ {ch["psi_deg"]:.0f}°, {ch["label"]}, {ch["cost"]:.3f}'
               if ch else 'none')
        L.append(f'| {case} | {n["points_facing"]} / {n["points_sampled"]} | {n["candidates"]} | {n["kin_valid"]} | '
                 f'{n["f4_valid"]} | {n["clear_weld"]} | {n["contact_points_ok"]} / {n["points_facing"]} | '
                 f'{n["node_valid"]} | {chs} |')
    L.append('')
    for case, c in rec['cases'].items():
        for p in c['poses']:
            s = p['summary']
            L += [f'## θ sweep — {case}, rank {p["rank"]} ({"result" if p["rank"] == 1 else "diagnostic"}): point '
                  f'{p["point"]} (s {p["s_mm"]:.2f} mm), ψ {p["psi_deg"]:.0f}°, {p["label"]}, θ sign '
                  f'{p["direction"]:+d}', '',
                  f'C9 **{"PASS" if s["C9_pass"] else "FAIL"}** — {s["states_ok"]} / {s["states"]} angles ok; min '
                  f'clearance lower bound {_fmt(s["min_clearance_lb_mm"], 1)} mm at θ '
                  f'{", ".join(f"{t:.1f}" for t in s["theta_at_min_deg"][:6])}{" …" if len(s["theta_at_min_deg"]) > 6 else ""}'
                  f' (closest pairs {json.dumps(s["closest_pairs_at_min"])}); guaranteed between checked angles '
                  f'{_fmt(s["between_states_min_mm"], 2)} mm.', '',
                  '| θ from → to (°) | States | OK | F4 | F4 contacts | Clearance lb (mm) | Closest pair (first level not '
                  'passed) | F9 |', '|---|---|---|---|---|---|---|---|']
            for r in p['runs']:
                L.append(f'| {r["theta_from_deg"]:.2f} → {r["theta_to_deg"]:.2f} | {r["states"]} | '
                         f'{"yes" if r["ok"] else "no"} | {"valid" if r["f4_valid"] else "invalid"} | '
                         f'{", ".join(r["f4_contacts"][:3])}{" …" if len(r["f4_contacts"]) > 3 else ""} | '
                         f'{_fmt(r["clearance_lb_mm"], 1) if r["clearance_lb_mm"] is not None else "< " + str(rec["clearance_levels_mm"][0])} | '
                         f'{r["closest_pair"] or "—"} | {"ok" if r["contact_ok"] else r["contact_reason"]} |')
            fp = p['contact_profile']
            L += ['', f'F9 fine profile ({fp["step_deg"]:.3f}° step, {fp["states"]} angles): {fp["failing_states"]} '
                  f'fail ({100.0 * fp["failing_states"] / fp["states"]:.1f} %), reasons {json.dumps(fp["reasons"])}; '
                  f'offending faces (min distance mm) {json.dumps({k: round(v, 3) for k, v in fp["offending_faces_min_mm"].items()})}; '
                  f'failing ranges {len(fp["failing_ranges"])}: '
                  + ', '.join(f'{r["theta_from_deg"]:.1f}…{r["theta_to_deg"]:.1f}' for r in fp['failing_ranges'][:12])
                  + (' …' if len(fp['failing_ranges']) > 12 else ''), '']
    L += ['## Not run / open', ''] + [f'- {x}' for x in rec['not_run']] + ['', '## Assumptions', ''] + [
        f'- {x}' for x in rec['assumptions']]
    path.write_text('\n'.join(L) + '\n')


# ---------------------------------------------------------------------------------------------------------------
# runner (ROS, mock hardware only)
# ---------------------------------------------------------------------------------------------------------------

def _rotating_set_r_max(cell_manifest: Path, job, axis_point, axis_dir):
    """r_max of everything on positioner_flange (cell objects + job objects), flange frame, mm."""
    from awm_core.frames import make_transform, quat_to_matrix
    from awm_core.stl import read_stl
    from awm_path.core.mode_c import radial_extent_mm
    doc = json.loads(cell_manifest.read_text())
    per: Dict[str, float] = {}
    for o in doc['objects']:
        if o['parent_frame'] != 'positioner_flange':
            continue
        v, _ = read_stl(cell_manifest.parent / o['mesh']['file'])
        t = make_transform(np.array(o['pose_mm']['position_mm']), quat_to_matrix(o['pose_mm']['quat_xyzw']))
        per[o['object_id']] = radial_extent_mm(v @ t[:3, :3].T + t[:3, 3], axis_point, axis_dir)
    for o in job.objects:
        if o.parent_frame != 'positioner_flange':
            continue
        t = o.parent_T_obj_mm
        per[o.object_id] = radial_extent_mm(o.vertices_mm @ t[:3, :3].T + t[:3, 3], axis_point, axis_dir)
    obj = max(per, key=per.get)
    return per[obj], obj, per


def _theta_sweep(ctx, pose, thetas, levels, weld_mm) -> List[Dict[str, Any]]:
    from awm_path.core.contact import contact_check
    from awm_path.core.mode_c import part_T_tcp_at
    scene, q = ctx['scene'], list(pose['q_deg'])
    rows = []
    for th in thetas:
        th = float(th)
        scene.set_theta(th)
        f4 = scene.check([q], th)[0]
        lb, closest = None, None
        for lv in levels:
            r = scene.check_clearance([q], th, lv)[0]
            if not r.valid:
                closest = ';'.join(r.contacts) or None
                break
            lb = lv
        c = contact_check(ctx['part'], part_T_tcp_at(ctx['flange0'], ctx['flange_T_part'], pose['world_T_tcp'], th),
                          ctx['seam_faces'], ctx['contact_params'])
        rows.append({'theta_deg': th,
                     's_contact_mm': float((pose['s_mm'] + units.deg_to_rad(abs(th)) * ctx['radius']) % ctx['length']),
                     'f4_valid': bool(f4.valid), 'f4_contacts': list(f4.contacts), 'clearance_lb_mm': lb,
                     'closest_pair': closest, 'contact_ok': bool(c.ok), 'contact_reason': c.reason,
                     'tcp_seam_distance_mm': c.tcp_seam_distance_mm if math.isfinite(c.tcp_seam_distance_mm) else None,
                     'tcp_nearest_other': list(c.tcp_nearest_other) if c.tcp_nearest_other else None,
                     'offending_faces': [list(x) for x in c.offending]})
    return rows


def _contact_profile(ctx, pose, span_deg, direction, step_deg) -> Dict[str, Any]:
    from awm_path.core.contact import contact_check
    from awm_path.core.mode_c import part_T_tcp_at, theta_grid
    th = theta_grid(span_deg, step_deg, direction)
    res = [contact_check(ctx['part'], part_T_tcp_at(ctx['flange0'], ctx['flange_T_part'], pose['world_T_tcp'],
                                                    float(t)), ctx['seam_faces'], ctx['contact_params']) for t in th]
    faces: Dict[str, float] = {}
    for c in res:
        for fid, d in c.offending:
            faces[fid] = min(faces.get(fid, math.inf), d)
    return {'step_deg': float(abs(th[1] - th[0])), 'states': len(th),
            'failing_states': sum(not c.ok for c in res),
            'reasons': dict(Counter(c.reason for c in res if c.reason)),
            'offending_faces_min_mm': dict(sorted(faces.items())),
            'failing_ranges': failing_ranges([float(t) for t in th], [c.ok for c in res])}


def _cmd_run(args) -> int:
    import dataclasses
    import yaml
    import rclpy
    from awm_bringup.testing import SimLaunch
    from awm_core.hashing import sha256_file, sha256_text
    from awm_path.backends.move_group import MoveGroupCellScene
    from awm_path.core.contact import ContactParams, PartMesh
    from awm_path.core.kinematics import UrAnalyticKinematics, pose_error
    from awm_path.core.mode_c import (angular_step_max_deg, closing_overlap_deg, coaxiality, facing_points,
                                      pick_poses, select_pose, theta_grid, travel_sign, world_T_flange_at)
    from awm_path.core.scoring import SceneScoreParams, score_candidates
    from awm_path.core.scene_files import CLEARANCE_METHOD, DEFAULT_CLEARANCE_LINKS, part_object_files
    from awm_path.core.seam_input import fit_circle, frames_at
    from awm_path.core.sweep import PlannerParams, sweep, tcp_pose, write_candidates_csv
    from awm_path.gate_g2 import _git, _rel, launch_args_from_frames
    from awm_path.spike_s5 import build_seams_doc

    repo, out_dir, inp = Path(args.repo), Path(args.out_dir), Path(args.input)
    job_dir = repo / 'jobs' / args.job
    frames_file = repo / 'cell' / 'model' / 'frames.yaml'
    cell_manifest = repo / 'cell' / 'model' / 'scene_manifest.json'
    placement = job_dir / 'placement.json'
    topo_file, glb = job_dir / 'cache' / 'view' / 'topology.json', job_dir / 'cache' / 'view' / 'view.glb'
    data = yaml.safe_load(inp.read_text())
    mc = data['mode_c']
    header = {'schema_version': '2.0', 'created_by': CREATED_BY, 'created_at': jsonio.utc_now_iso(),
              'inputs': [{'name': p.name, 'sha256': sha256_file(p)} for p in (inp, topo_file, glb)]}
    seams_doc = build_seams_doc({k: data[k] for k in ('part_no', 'part_rev', 'seams')},
                                json.loads(topo_file.read_text()), glb, header)
    seam_file = job_dir / SEAM_FILE
    seam_file.write_text(json.dumps(seams_doc, indent=1, allow_nan=False) + '\n')
    seam = [s for s in seams_doc['seams'] if s['seam_id'] == mc['seam_id']]
    if len(seam) != 1 or seam[0]['arcs'] != 1:
        raise SystemExit(f'Mode C seam {mc["seam_id"]}: one closed seam with arcs = 1 required')
    seam = seam[0]
    pts, length = seam['geometry']['points'], seam['length_mm']
    fit = fit_circle([p['p_mm'] for p in pts])
    clearance_links = tuple(args.clearance_links.split(',')) if args.clearance_links else DEFAULT_CLEARANCE_LINKS
    params, contact_params, score_params = PlannerParams(), ContactParams(), SceneScoreParams()
    levels = tuple(float(x) for x in mc['clearance_levels_mm'])
    weld_mm = float(mc['angular_step']['clearance_mm'])
    if levels[0] != score_params.weld_clearance_mm or weld_mm != score_params.weld_clearance_mm:
        raise SystemExit('the lowest clearance level and the angular-step clearance must be the weld clearance '
                         f'{score_params.weld_clearance_mm} mm (TST-002 C9)')
    kin = UrAnalyticKinematics()
    head_tag = args.head_model.lower().replace('-', '')
    launch_args = dict(launch_args_from_frames(frames_file), cell_manifest=str(cell_manifest.resolve()))
    cases: Dict[str, Any] = {}
    t_all = time.perf_counter()
    with SimLaunch(log_dir=args.log_dir, launch_file='sim_mock.launch.py', launch_args=launch_args) as sim:
        rclpy.init()
        node = rclpy.create_node('awm_s6_run')
        try:
            t_load = time.perf_counter()
            scene = MoveGroupCellScene(node, cell_manifest, placement, job_dir, head_model=args.head_model,
                                       clearance_links=clearance_links)
            scene.load(timeout_s=180.0)
            part_obj, mesh_file, faces_file = part_object_files(scene.job)
            part = PartMesh.from_files(mesh_file, faces_file)
            scene.set_theta(0.0)
            w = scene.world_T_links([0.0] * 6, 0.0, ['base_link', 'tool0', 'tcp_laser', 'positioner_flange'])
            t_load = time.perf_counter() - t_load
            flange0 = w['positioner_flange']
            if part_obj.parent_frame != 'positioner_flange':
                raise SystemExit('Mode C needs the part on positioner_flange (D19)')
            flange_T_part = part_obj.parent_T_obj_mm
            world_T_part0 = flange0 @ flange_T_part
            base_T_world = np.linalg.inv(w['base_link'])
            tool0_T_tcp = np.linalg.inv(w['tool0']) @ w['tcp_laser']
            # flange model check against /compute_fk
            chk = []
            for th in (90.0, -180.0, -368.0, 270.0):
                fk = scene.world_T_links([0.0] * 6, th, ['positioner_flange'])['positioner_flange']
                e = pose_error(fk, world_T_flange_at(flange0, th))
                chk.append((float(e[0]), float(units.rad_to_deg(e[1]))))
            flange_chk = {'thetas_deg': [90.0, -180.0, -368.0, 270.0], 'max_pos_mm': max(c[0] for c in chk),
                          'max_rot_deg': max(c[1] for c in chk)}
            # F8 and r_max in the flange frame (axis = flange X through its origin)
            ax_p, ax_d = np.zeros(3), np.array([1.0, 0.0, 0.0])
            f8 = coaxiality(flange_T_part[:3, :3] @ fit.centre + flange_T_part[:3, 3], flange_T_part[:3, :3] @ fit.axis,
                            ax_p, ax_d)
            r_max, r_obj, r_per = _rotating_set_r_max(cell_manifest, scene.job, ax_p, ax_d)
            margin = float(mc['angular_step']['margin_mm'])
            step_max = angular_step_max_deg(weld_mm, margin, r_max)
            ov_deg = closing_overlap_deg(float(seam['arc_overlap_mm']), fit.radius)
            span = 360.0 + ov_deg
            # contact-point candidates at θ = 0
            step_s = units.deg_to_rad(float(mc['contact_points']['step_deg'])) * fit.radius
            s_all = np.arange(0.0, length - 1e-9, step_s)
            fr_part_all = frames_at(pts, s_all, True, length)
            fr_world_all = [world_T_part0 @ f for f in fr_part_all]
            sel, cosv = facing_points(fr_world_all, w['base_link'][:3, 3], float(mc['contact_points']['facing_min_cos']))
            if not sel:
                raise SystemExit('no seam point faces the robot base with the data-file cosine')
            s_sel = [float(s_all[i]) for i in sel]
            fr_part = [fr_part_all[i] for i in sel]
            fr_base = [base_T_world @ fr_world_all[i] for i in sel]
            base_T_part = base_T_world @ world_T_part0
            ctx = {'scene': scene, 'part': part, 'flange0': flange0, 'flange_T_part': flange_T_part,
                   'seam_faces': (tuple(seam['faces']['n1']), tuple(seam['faces']['n2'])),
                   'contact_params': contact_params, 'radius': fit.radius, 'length': length}
            done: Dict[tuple, Dict[str, Any]] = {}
            for case in mc['psi_cases']:
                psi = params.psi_free if case == 'free' else params.psi_wire
                t0 = time.perf_counter()
                layers = sweep(fr_base, tool0_T_tcp, kin, params, psi)
                scored = score_candidates(layers, fr_base, base_T_part, 0.0, scene, part, ctx['seam_faces'],
                                          contact_params, score_params)
                t_pick = time.perf_counter() - t0
                csv_name = f's6_candidates_{seam["seam_id"]}_{case}_{head_tag}.csv'
                write_candidates_csv(str(job_dir / csv_name), layers, scored.columns)
                entries = [(li, c, s) for li, (lay, sc) in enumerate(zip(layers, scored.scores))
                           for c, s in zip(lay, sc)]
                ranked = select_pose(entries, params)
                kin_ok = [(li, ci) for li, lay in enumerate(layers) for ci, c in enumerate(lay) if c.reason is None]
                col = scored.columns
                counts = {'points_sampled': len(s_all), 'points_facing': len(sel),
                          'candidates': sum(len(l) for l in layers), 'kin_valid': len(kin_ok),
                          'f4_valid': sum(1 for li, ci in kin_ok if col['scene_valid'][li][ci]),
                          'clear_weld': sum(1 for li, ci in kin_ok if (col['clearance_lb_mm'][li][ci] or 0) >= weld_mm),
                          'contact_points_ok': sum(1 for c in scored.contact if c.ok),
                          'node_valid': len(ranked),
                          'reasons': dict(Counter(c.reason for lay in layers for c in lay if c.reason))}
                per_point = [{'point': li, 's_mm': s_sel[li], 'facing_cos': cosv[sel[li]],
                              'kin_valid': sum(1 for c in layers[li] if c.reason is None),
                              'node_valid': sum(1 for s in scored.scores[li] if s.valid),
                              'contact_ok': scored.contact[li].ok, 'contact_reason': scored.contact[li].reason,
                              'contact_offending': [list(x) for x in scored.contact[li].offending]}
                             for li in range(len(layers))]
                rank_rec = [dict({k: v for k, v in r.items() if k != 'candidate'}, s_mm=s_sel[r['point']])
                            for r in ranked[:10]]
                poses = []
                for r in pick_poses(ranked, int(mc.get('alternates', 0)), bool(mc.get('alternates_distinct_points'))):
                    key = (r['point'], r['psi_index'], r['label'], tuple(round(x, 9) for x in r['q_deg']))
                    if key not in done:
                        wt = world_T_part0 @ tcp_pose(fr_part[r['point']], r['psi_deg'])
                        direction = travel_sign(world_T_part0 @ fr_part[r['point']], flange0[:3, 3], flange0[:3, 0])
                        thetas = theta_grid(span, step_max, direction)
                        pose = dict(r, s_mm=s_sel[r['point']], world_T_tcp=wt, direction=direction)
                        t1 = time.perf_counter()
                        rows = _theta_sweep(ctx, pose, thetas, levels, weld_mm)
                        t_sweep = time.perf_counter() - t1
                        step = float(abs(thetas[1] - thetas[0]))
                        summ = theta_summary(rows, weld_mm, step, r_max)
                        prof = _contact_profile(ctx, pose, span, direction, float(mc['contact_profile_step_deg']))
                        part_T_tcp0 = np.linalg.inv(world_T_part0) @ wt
                        done[key] = {'point': r['point'], 's_mm': s_sel[r['point']], 'psi_deg': r['psi_deg'],
                                     'label': r['label'], 'q_deg': r['q_deg'], 'cost': r['cost'],
                                     'direction': direction, 'step_deg': step,
                                     'tcp_part_mm': [float(v) for v in part_T_tcp0[:3, 3]],
                                     'summary': summ, 'runs': compress_rows(rows), 'rows': rows,
                                     'contact_profile': prof, 'sweep_s': t_sweep}
                        print(f'{case} rank {r["rank"]}: point {r["point"]} psi {r["psi_deg"]} {r["label"]} C9 '
                              f'{summ["C9_pass"]} ({summ["states_ok"]}/{summ["states"]}), min lb '
                              f'{summ["min_clearance_lb_mm"]}, F9 profile fail {prof["failing_states"]}/'
                              f'{prof["states"]}, {t_sweep:.0f} s', flush=True)
                    poses.append(dict(done[key], rank=r['rank']))
                cases[case] = {'psi_deg': list(psi), 'counts': counts, 'per_point': per_point, 'ranking': rank_rec,
                               'pick_s': t_pick, 'candidates_csv': {'file': csv_name,
                                                                    'sha256': sha256_file(job_dir / csv_name)},
                               'poses': poses}
                print(f'{case}: {counts}', flush=True)
        except Exception:
            print(sim.log_tail(60), file=sys.stderr)
            raise
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    out_dir.mkdir(parents=True, exist_ok=True)
    here = out_dir.resolve()
    commit, dirty = _git(['rev-parse', 'HEAD'], here), bool(_git(['status', '--porcelain'], here))
    params_rec = json.loads(json.dumps(dataclasses.asdict(params)))
    inputs = [{'name': _rel(p), 'sha256': sha256_file(p)}
              for p in (inp, frames_file, cell_manifest, placement, mesh_file, faces_file, seam_file, topo_file, glb)]
    rec = {
        'created_by': CREATED_BY, 'created_at': jsonio.utc_now_iso(), 'commit': commit, 'dirty': dirty,
        'task': 'WP-S S.6 (TST-002 Rev 0.3 C9, reported)', 'inputs': inputs,
        'cell_yaml': None, 'cell_yaml_note': 'D16 cell.yaml does not exist yet (T0.1): parameters = PlannerParams '
                                             'defaults (TST-002 §5.1) + the S.6 data file, recorded below',
        'job_dir': _rel(job_dir), 'scene_backend': scene.record, 'kinematics_backend': kin.name,
        'head_model': args.head_model, 'clearance_links': list(clearance_links), 'clearance_method': CLEARANCE_METHOD,
        'mode': 'C', 'scene_load_s': t_load, 'total_s': time.perf_counter() - t_all,
        'params': params_rec, 'params_sha256': sha256_text(json.dumps(params_rec, sort_keys=True)),
        'contact_params': contact_params.as_record(), 'score_params': dataclasses.asdict(score_params),
        'mode_c_input': mc, 'clearance_levels_mm': list(levels),
        'seam': {k: seam[k] for k in ('seam_id', 'joint_edge_id', 'edges', 'solids', 'faces', 'closed', 'length_mm',
                                      'arcs', 'arc_overlap_mm', 'note')},
        'geometry': {'seam_radius_mm': fit.radius, 'seam_centre_part_mm': [float(v) for v in fit.centre],
                     'seam_axis_part': [float(v) for v in fit.axis], 'f8': f8, 'r_max_mm': r_max,
                     'r_max_object': r_obj, 'r_max_per_object_mm': r_per, 'clearance_mm': weld_mm,
                     'margin_mm': margin, 'step_max_deg': step_max, 'overlap_deg': ov_deg, 'span_deg': span,
                     'states': int(math.ceil(span / step_max - 1e-9)) + 1,
                     'step_deg': span / int(math.ceil(span / step_max - 1e-9)),
                     'flange_model_check': flange_chk},
        'cases': cases,
        'not_run': ['H-CAD head model: no head hulls / H-CAD cell model (S.5 open item)',
                    'approach / retract / air move to the Mode C pose (C4): T1.5',
                    'time parameterisation ω = travel speed / r and ramps in the overlap (A47, B-20): WPS travel speed '
                    'not given (I5); the check is geometric (quasi-static, positioner angles only)',
                    'TST-002 §11 not filled (S.7, lead)'],
        'assumptions': [
            'part, robot base and positioner axis poses from CAD component placements, no REF_* markers (OI-23)',
            'H-BOX head (TST-002 §3.1), nozzle Ø20 x 40 mm flat disc, s_c 0.3 mm, TCP at the nozzle tip (I2 / OI-15)',
            'F9 point tolerance 0.15 mm (TST-002 gives no value)',
            f'B-03 angular step: margin {margin} mm (= F4 padding_mm; TST-002 / DES-001 give no margin value), '
            'c = weld clearance verified at every checked angle; between checked angles only c − r_max·Δθ/2 is '
            'guaranteed',
            'r_max over every object on positioner_flange (all holders rotate with the part; I3 / OI-27 open)',
            'positioner flange turns about its own X axis (frames.yaml), checked against /compute_fk',
            f'closing overlap {seam["arc_overlap_mm"]} mm (D04 arc_overlap_mm of the Mode C seam, T0.5 example value; '
            'welding engineer to confirm)',
            'contact location: seam points at the data-file step whose seam Z faces the robot base origin '
            f'(cos ≥ {mc["contact_points"]["facing_min_cos"]}); Ψ-WIRE window centre 0° = seam X (I5)',
            'seam geometry: circle through the D20 edge vertices, n1 = n2 = radial (tube butt joint)'],
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    for c in rec['cases'].values():
        for p in c['poses']:
            p.pop('world_T_tcp', None)
    (out_dir / 's6_results.json').write_text(json.dumps(rec, indent=1, allow_nan=False) + '\n')
    write_md(out_dir / 's6_results.md', rec)
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog='s6_run', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--input', required=True, help='S.6 data file (Engineer data: seam, Mode C choices)')
    ap.add_argument('--repo', required=True)
    ap.add_argument('--job', required=True)
    ap.add_argument('--head-model', default='H-BOX')
    ap.add_argument('--clearance-links', default=None, help='comma-separated; default TST-002 F5 set')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--log-dir', default=None)
    return _cmd_run(ap.parse_args(argv))


if __name__ == '__main__':
    sys.exit(main())
