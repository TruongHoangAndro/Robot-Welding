"""WP-S S.5 spike run: J2 robot-only sweep + ladder DP → TST-002 C2, C3, C5, C8 (TST-002 Rev 0.3 §4, §5.3, §9).

Two sub-commands (all ids, faces, arcs and paths come from data files / the command line, never from source):

    ros2 run awm_path s5_run seam --input <s5_seam_input.yaml> --repo <repo> --job <job>
        pure: D20 joint edge (exact circle, preview frames checked) + Engineer attributes → jobs/<job>/spike_seam.json
        (D04 v2 shape, validated against v2/seams).

    ros2 run awm_path s5_run run --repo <repo> --job <job> --seam-id <id> --psi-cases free,wire \
        --head-model H-BOX --out-dir <records dir>
        Spike / simulation tool: starts its own sim_mock launch (mock hardware only). Positioner fixed (robot-only).
        Per ψ case and arc (CP-53): pitch refined below the F1 ceiling (kinematic step ≤ Δq_max), S.3 sweep ψ × 8 IK
        branches, S.4 scene scores (F4, F5, F9), S.3 ladder DP (open arc; F3, F7, F11) → accepted path or the
        failing points with reason class and colliding pairs. No partial path is ever accepted.
        Writes candidates CSV + spike_path.json into the job folder and s5_results.json / .md into --out-dir.

C5 time per arc = pitch refinement (incl. its sweeps) + scene scores + DP; launch and scene load are recorded but
not counted. C8 = seam length inside accepted arcs / seam length (2-arc case); a closed robot-only path with one arc
is not run (impossible under F3 + F11, CP-53).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

import numpy as np

from awm_core import jsonio, units
from awm_path.core.seam_input import arc_ranges, circle_joint_geometry, frames_at, read_glb_edge_polyline
from awm_path.gate_g2 import add_head_cli, head_assumption

CREATED_BY = 'awm_path.spike_s5'
SPIKE_SEAM_FILE = 'spike_seam.json'


# ---------------------------------------------------------------------------------------------------------------
# pure parts
# ---------------------------------------------------------------------------------------------------------------

def build_seams_doc(seam_input: Mapping[str, Any], topology: Mapping[str, Any], glb_path, header: Mapping[str, Any]
                    ) -> Dict[str, Any]:
    """D04 v2-shaped seams document from the Engineer input and the D20 view model (validated against v2/seams)."""
    edges = {e['edge_id']: e for e in topology['edges']}
    joints = {j['joint_edge_id']: j for j in topology['joints']}
    seams = []
    for s in seam_input['seams']:
        if s.get('reversed'):
            raise ValueError(f'{s["seam_id"]}: reversed seams are not supported by the spike seam input')
        j = joints[s['joint_edge_id']]
        if edges[j['edge_a']]['curve_type'] != 'circle' or not j.get('closed'):
            raise ValueError(f'{s["seam_id"]}: spike seam input supports closed circle joint edges only '
                             f'(D04 v2 geometry from M2, T2.2, otherwise)')
        geo = circle_joint_geometry(read_glb_edge_polyline(glb_path, j['edge_a']), j['preview_frames'],
                                    float(s['geometry_spacing_mm']), float(j['length_mm']))
        seams.append({
            'seam_id': s['seam_id'], 'edges': [j['edge_a'], j['edge_b']], 'joint_edge_id': j['joint_edge_id'],
            'solids': [j['solid_a'], j['solid_b']], 'faces': {'n1': list(s['faces']['n1']), 'n2': list(s['faces']['n2'])},
            'reversed': False, 'range_mm': None, 'closed': True, 'length_mm': geo['length_mm'],
            'joint_type': s['joint_type'], 'sequence': int(s['sequence']), 'leg_mm': None, 'arcs': int(s['arcs']),
            'arc_overlap_mm': float(s['arc_overlap_mm']),
            'note': (s.get('note', '') + ' | geometry: circle through the D20 edge vertices, fit '
                     f'residual {geo["fit"]["max_residual_mm"]:.2e} mm, {geo["fit"]["preview_frames_checked"]} '
                     'B-Rep preview frames checked; n1 = n2 = radial (tube butt joint)').strip(' |'),
            'fingerprint': edges[j['edge_a']]['fingerprint'],
            'geometry': {'spacing_mm': geo['spacing_mm'], 'points': geo['points']},
        })
    doc = dict(header)
    doc.update({'part_no': str(seam_input['part_no']), 'part_rev': str(seam_input['part_rev']), 'seams': seams})
    jsonio.validate(doc, 'v2/seams')
    return doc


def case_summary(arcs: Sequence[Mapping[str, Any]], seam_length_mm: float, c5_budget_s: float = 30.0
                 ) -> Dict[str, Any]:
    """C2 / C3 / C5 / C8 of one ψ case from its arc reports (TST-002 §5.3; C2 / C3 per arc, CP-53)."""
    covered = sum(a['nominal_length_mm'] for a in arcs if a['accepted'])
    seam_s = float(sum(a['timing_s']['c5_s'] for a in arcs))
    return {
        'C2': {'pass': all(a['c2']['pass'] for a in arcs), 'per_arc_pct': [a['c2']['coverage_pct'] for a in arcs]},
        'C3': {'pass': all(a['c3']['pass'] for a in arcs), 'per_arc': [a['c3']['pass'] for a in arcs]},
        'C5': {'pass': seam_s <= c5_budget_s, 'seam_s': seam_s, 'budget_s': c5_budget_s,
               'per_arc_s': [a['timing_s']['c5_s'] for a in arcs]},
        'C8': {'pass': covered >= seam_length_mm - 1e-6, 'coverage_pct': 100.0 * covered / seam_length_mm,
               'arcs': len(arcs), 'accepted_arcs': sum(1 for a in arcs if a['accepted']),
               'one_closed_path': 'not run: impossible under F3 + F11 (CP-53)'},
    }


def _runs(idx: Sequence[int]) -> List[List[int]]:
    out: List[List[int]] = []
    for i in idx:
        if out and i == out[-1][1] + 1:
            out[-1][1] = i
        else:
            out.append([i, i])
    return out


def _fmt_pairs(pairs: Mapping[str, int], n: int = 4) -> str:
    items = list(pairs.items())[:n]
    return ', '.join(f'{k} ×{v}' for k, v in items) + (' …' if len(pairs) > n else '')


def write_md(path: Path, rec: Mapping[str, Any]) -> None:
    """Human-readable summary of ``s5_results.json``."""
    L: List[str] = [f'# WP-S S.5 — J2 robot-only results ({rec["head_model"]})', '',
                    f'Record `s5_results.json`; commit `{rec["commit"]}`, dirty: {str(rec["dirty"]).lower()}, '
                    f'created {rec["created_at"]}. Scene hash `{rec["scene_backend"]["scene_hash"]}`; kinematics '
                    f'`{rec["kinematics_backend"]}`; scene backend `{rec["scene_backend"]["content"]["backend"]}`; '
                    f'θ = {rec["theta_deg"]}° (robot-only); clearance links {", ".join(rec["clearance_links"])} '
                    f'(F5 method: {rec["clearance_method"]}).', '',
                    f'Seam {rec["seam"]["seam_id"]} ({rec["seam"]["note"].split("|")[0].strip()}), length '
                    f'{rec["seam"]["length_mm"]:.3f} mm, {rec["seam"]["arcs"]} arcs, overlap '
                    f'{rec["seam"]["arc_overlap_mm"]} mm. Scene load {rec["scene_load_s"]:.1f} s (not in C5).', '',
                    '| ψ case | Arc | s range (mm) | Points / pitch (mm) | C2 coverage | C3 | C5 arc (s) | Result |',
                    '|---|---|---|---|---|---|---|---|']
    for case, c in rec['cases'].items():
        for a in c['arcs']:
            m = a['c3']['metrics']
            c3 = ('pass: label ' + ', '.join(m['labels']) + f', max |Δq| {m["max_step_deg"]:.2f}°, wrist-3 travel '
                  f'{m["wrist3_travel_deg"]:.1f}°') if a['accepted'] else ('fail: ' + a['c3']['reason'])
            L.append(f'| {case} | {a["index"]}/{a["of"]} | {a["start_s_mm"]:.2f}–{a["end_s_mm"]:.2f} | '
                     f'{a["points"]} / {a["pitch"]["pitch_mm"]:.3f} | {a["c2"]["coverage_pct"]:.1f} % '
                     f'({a["c2"]["points_covered"]}/{a["points"]}) | {c3} | {a["timing_s"]["c5_s"]:.1f} | '
                     f'{"accepted" if a["accepted"] else "rejected"} |')
    L += ['', '| ψ case | C2 | C3 | C5 seam (s) | C8 (2 arcs) |', '|---|---|---|---|---|']
    for case, c in rec['cases'].items():
        s = c['summary']
        L.append(f'| {case} | {"PASS" if s["C2"]["pass"] else "FAIL"} ({", ".join(f"{v:.1f} %" for v in s["C2"]["per_arc_pct"])}) '
                 f'| {"PASS" if s["C3"]["pass"] else "FAIL"} | {s["C5"]["seam_s"]:.1f} '
                 f'{"PASS" if s["C5"]["pass"] else "FAIL"} (≤ {s["C5"]["budget_s"]:.0f}) | '
                 f'{s["C8"]["coverage_pct"]:.1f} % {"PASS" if s["C8"]["pass"] else "FAIL"} |')
    L += ['', '## Failing points (rejected arcs; runs of consecutive points)', '']
    for case, c in rec['cases'].items():
        for a in c['arcs']:
            if a['accepted']:
                continue
            L.append(f'**{case}, arc {a["index"]}** — {a["c3"]["reason"]}; longest continuous window '
                     f'{a["window"]["layers"]} points ({100 * a["window"]["fraction"]:.1f} % of the arc, '
                     f'diagnostic only).')
            L.append('')
            L.append('| Points | s (mm) | Class | Outcomes (first point of run) | Colliding / blocking pairs (run total) |')
            L.append('|---|---|---|---|---|')
            fps = a['failing_points']
            by_class: Dict[str, List[Mapping[str, Any]]] = {}
            for fp in fps:
                by_class.setdefault(fp['class'], []).append(fp)
            for cls, lst in by_class.items():
                idx = {fp['point']: fp for fp in lst}
                for r0, r1 in _runs(sorted(idx)):
                    pairs: Dict[str, int] = {}
                    for i in range(r0, r1 + 1):
                        for k, v in idx[i].get('pairs', {}).items():
                            pairs[k] = pairs.get(k, 0) + v
                    pairs = dict(sorted(pairs.items(), key=lambda kv: (-kv[1], kv[0])))
                    L.append(f'| {r0}–{r1} | {idx[r0]["s_mm"]:.2f}–{idx[r1]["s_mm"]:.2f} | {cls} | '
                             f'{json.dumps(idx[r0]["counts"])} | {_fmt_pairs(pairs)} |')
            L.append('')
    L += ['## Not run / open', ''] + [f'- {x}' for x in rec['not_run']] + ['', '## Assumptions', ''] + [
        f'- {x}' for x in rec['assumptions']]
    path.write_text('\n'.join(L) + '\n')


# ---------------------------------------------------------------------------------------------------------------
# runner (ROS, mock hardware only)
# ---------------------------------------------------------------------------------------------------------------

def _cmd_seam(args) -> int:
    import yaml
    repo = Path(args.repo)
    job = repo / 'jobs' / args.job
    topo_file = job / 'cache' / 'view' / 'topology.json'
    glb = job / 'cache' / 'view' / 'view.glb'
    inp = Path(args.input)
    header = {'schema_version': '2.0', 'created_by': CREATED_BY, 'created_at': jsonio.utc_now_iso(),
              'inputs': [{'name': p.name, 'sha256': jsonio.input_entry(p, p.parent)['sha256']}
                         for p in (inp, topo_file, glb)]}
    doc = build_seams_doc(yaml.safe_load(inp.read_text()), json.loads(topo_file.read_text()), glb, header)
    out = job / SPIKE_SEAM_FILE
    out.write_text(json.dumps(doc, indent=1, allow_nan=False) + '\n')
    print(f'{out}: ' + ', '.join(f'{s["seam_id"]} {len(s["geometry"]["points"])} points, {s["length_mm"]:.3f} mm'
                                  for s in doc['seams']))
    return 0


def _rotation_rate_deg_per_mm(points, closed, length, s0, s1) -> float:
    from awm_path.core.kinematics import pose_error
    s = np.linspace(s0, s1, max(3, int(math.ceil((s1 - s0) / 0.5)) + 1))
    fr = frames_at(points, s, closed, length)
    rates = [units.rad_to_deg(pose_error(a, b)[1]) / (s[i + 1] - s[i]) for i, (a, b) in enumerate(zip(fr[:-1], fr[1:]))]
    return float(max(rates))


def _plan_arc(ctx: Dict[str, Any], seam: Mapping[str, Any], k: int, s0: float, s1: float, psi: Sequence[float],
              case: str) -> Dict[str, Any]:
    from awm_core.frames import matrix_to_quat
    from awm_core.hashing import sha256_file
    from awm_path.core.ladder import solve
    from awm_path.core.s5 import classify_points, longest_window, path_metrics, point_classes, refine_pitch
    from awm_path.core.scoring import score_candidates
    from awm_path.core.sweep import sweep, tcp_pose, write_candidates_csv

    params, length, pts = ctx['params'], seam['length_mm'], seam['geometry']['points']
    cache: Dict[int, Any] = {}

    def sample(n: int):
        s_vals = np.linspace(s0, s1, n)
        fp = frames_at(pts, s_vals, True, length)
        fb = [ctx['base_T_part'] @ f for f in fp]
        cache[n] = (s_vals, fp, fb)
        return sweep(fb, ctx['tool0_T_tcp'], ctx['kin'], params, psi)

    t0 = time.perf_counter()
    rate = _rotation_rate_deg_per_mm(pts, True, length, s0, s1)
    pr = refine_pitch(s1 - s0, rate, sample, params, ctx['pitch_min_mm'])
    t_pitch = time.perf_counter() - t0
    layers = pr.layers
    s_vals, fr_part, fr_base = cache[pr.points]
    t0 = time.perf_counter()
    scored = score_candidates(layers, fr_base, ctx['base_T_part'], ctx['theta_deg'], ctx['scene'], ctx['part'],
                              ctx['seam_faces'], ctx['contact_params'], ctx['score_params'])
    t_score = time.perf_counter() - t0
    t0 = time.perf_counter()
    res = solve(layers, scored.scores, params, closed=False)
    t_dp = time.perf_counter() - t0

    pts_cls = classify_points(layers, scored.columns)
    classes = point_classes(pts_cls)
    covered = sum(1 for c in classes if c is None)
    csv_name = f's5_candidates_{seam["seam_id"]}_arc{k}_{case}_{ctx["head_tag"]}.csv'
    csv_path = ctx['job_dir'] / csv_name
    write_candidates_csv(str(csv_path), layers, scored.columns)

    failing = [dict(p, s_mm=float(s_vals[p['point']]), **{'class': c}) for p, c in zip(pts_cls, classes)
               if c is not None]
    if covered == len(layers) and not res.ok:
        for li in res.failing_layers:
            failing.append(dict(pts_cls[li], s_mm=float(s_vals[li]), **{'class': 'continuity'},
                                dp_reasons=dict(res.reason_counts[li])))
    if res.ok:
        reason = None
    elif covered < len(layers):
        reason = f'no path: C2 fails at {len(layers) - covered} of {len(layers)} points'
    else:
        reason = (f'no continuous path (F3 / F11): DP stops at layer {res.max_reached_layer + 1}, reasons '
                  f'{dict(res.reason_counts[res.failing_layers[0]]) if res.failing_layers else {}}')
    window = {'start': None, 'end': None, 'layers': len(layers), 'fraction': 1.0}
    if not res.ok:
        w = longest_window(layers, scored.scores, params)
        span = (w['layers'] - 1) * pr.pitch_mm if w['layers'] else 0.0
        window = dict(w, fraction=span / (s1 - s0))
    path_out = None
    if res.ok:
        path_out = []
        for c, s, f in zip(res.path, s_vals, fr_part):
            t = tcp_pose(f, c.psi_deg)
            li = int(c.point)
            ci = layers[li].index(next(x for x in layers[li] if x.psi_index == c.psi_index and x.label == c.label))
            path_out.append({'s_mm': float(s), 'psi_deg': c.psi_deg, 'label': c.label, 'q_deg': list(c.q_deg),
                             'pose_mm': {'position': [float(v) for v in t[:3, 3]],
                                         'quat_xyzw': [float(v) for v in matrix_to_quat(t[:3, :3])]},
                             'clearance_lb_mm': scored.scores[li][ci].clearance_mm})
    n_states = scored.timing['states_checked']
    return {
        'index': k, 'of': seam['arcs'], 'start_s_mm': s0, 'end_s_mm': s1, 'nominal_length_mm': length / seam['arcs'],
        'points': len(layers), 'pitch': dict(pr.as_record(), rotation_rate_deg_per_mm=rate),
        'accepted': bool(res.ok),
        'c2': {'pass': covered == len(layers), 'coverage_pct': 100.0 * covered / len(layers),
               'points_covered': covered,
               'failing_classes': {c: classes.count(c) for c in sorted({c for c in classes if c})}},
        'c3': {'pass': bool(res.ok), 'reason': reason, 'metrics': path_metrics(res.path) if res.ok else None,
               'dp': {'max_reached_layer': res.max_reached_layer, 'failing_layers': list(res.failing_layers),
                      'nodes': res.n_nodes, 'edges': res.n_edges, 'cost': res.cost}},
        'timing_s': {'pitch_and_sweep_s': t_pitch, 'scores_s': t_score, 'dp_s': t_dp,
                     'c5_s': t_pitch + t_score + t_dp, 'states_checked': n_states,
                     'per_state_s': scored.timing['per_state_s'], 'contact_s': scored.timing['contact_s']},
        'candidates_csv': {'file': csv_name, 'sha256': sha256_file(csv_path)},
        'window': window, 'failing_points': failing, 'path': path_out,
    }


def _cmd_run(args) -> int:
    import dataclasses
    import rclpy
    from awm_bringup.testing import SimLaunch
    from awm_core.hashing import sha256_file, sha256_text
    from awm_path.backends.move_group import MoveGroupCellScene
    from awm_path.core.contact import ContactParams, PartMesh
    from awm_path.core.kinematics import UrAnalyticKinematics
    from awm_path.core.scene_files import CLEARANCE_METHOD, DEFAULT_CLEARANCE_LINKS, part_object_files, world_T_object
    from awm_path.core.scoring import SceneScoreParams
    from awm_path.core.sweep import PlannerParams
    from awm_path.core.head import contact_params_for
    from awm_path.gate_g2 import _git, _rel, launch_args_from_frames, parse_head_args

    repo, out_dir = Path(args.repo), Path(args.out_dir)
    job = repo / 'jobs' / args.job
    frames_file = repo / 'cell' / 'model' / 'frames.yaml'
    cell_manifest = repo / 'cell' / 'model' / 'scene_manifest.json'
    placement = job / 'placement.json'
    seam_file = job / SPIKE_SEAM_FILE
    seams_doc = json.loads(seam_file.read_text())
    jsonio.validate(seams_doc, 'v2/seams')
    seam = [s for s in seams_doc['seams'] if s['seam_id'] == args.seam_id]
    if len(seam) != 1:
        raise SystemExit(f'seam {args.seam_id} not in {seam_file}')
    seam = seam[0]
    cases = [c.strip() for c in args.psi_cases.split(',') if c.strip()]
    clearance_links = tuple(args.clearance_links.split(',')) if args.clearance_links else DEFAULT_CLEARANCE_LINKS
    params = PlannerParams()
    score_params = SceneScoreParams()
    kin = UrAnalyticKinematics()
    head_tag = args.head_model.lower().replace('-', '')
    launch_args = dict(launch_args_from_frames(frames_file), cell_manifest=str(cell_manifest.resolve()),
                       **parse_head_args(args.head_arg))
    results: Dict[str, Any] = {}
    with SimLaunch(log_dir=args.log_dir, launch_file='sim_mock.launch.py', launch_args=launch_args) as sim:
        rclpy.init()
        node = rclpy.create_node('awm_s5_run')
        try:
            t_load = time.perf_counter()
            scene = MoveGroupCellScene(node, cell_manifest, placement, job, head_model=args.head_model,
                                       clearance_links=clearance_links)
            scene.load(timeout_s=180.0)
            part_obj, mesh_file, faces_file = part_object_files(scene.job)
            part = PartMesh.from_files(mesh_file, faces_file)
            contact_params = contact_params_for(scene.head, ContactParams(rule_b=args.f9_rule))
            scene.set_theta(args.theta_deg)
            w = scene.world_T_links([0.0] * 6, args.theta_deg, ['base_link', 'tool0', 'tcp_laser',
                                                                 'positioner_flange'])
            t_load = time.perf_counter() - t_load
            ctx = {'params': params, 'kin': kin, 'scene': scene, 'part': part, 'theta_deg': args.theta_deg,
                   'tool0_T_tcp': scene.head.tool0_T_tcp_mm,
                   'base_T_part': np.linalg.inv(w['base_link']) @ world_T_object(part_obj, w['positioner_flange']),
                   'seam_faces': (tuple(seam['faces']['n1']), tuple(seam['faces']['n2'])),
                   'contact_params': contact_params, 'score_params': score_params, 'job_dir': job,
                   'head_tag': head_tag, 'pitch_min_mm': args.pitch_min_mm}
            ranges = arc_ranges(seam['length_mm'], seam['arcs'], seam['arc_overlap_mm'], seam['closed'])
            for case in cases:
                psi = params.psi_free if case == 'free' else params.psi_wire
                arcs = []
                for k, (s0, s1) in enumerate(ranges, start=1):
                    a = _plan_arc(ctx, seam, k, s0, s1, psi, case)
                    print(f'{case} arc {k}: {a["points"]} points, pitch {a["pitch"]["pitch_mm"]:.3f} mm, C2 '
                          f'{a["c2"]["coverage_pct"]:.1f} %, accepted {a["accepted"]}, C5 {a["timing_s"]["c5_s"]:.1f} s',
                          flush=True)
                    arcs.append(a)
                results[case] = {'psi_deg': list(psi), 'arcs': arcs,
                                 'summary': case_summary(arcs, seam['length_mm'])}
        except Exception:
            print(sim.log_tail(60), file=sys.stderr)
            raise
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    here = out_dir.resolve()
    commit, dirty = _git(['rev-parse', 'HEAD'], here), bool(_git(['status', '--porcelain'], here))
    params_rec = json.loads(json.dumps(dataclasses.asdict(params)))
    inputs = [{'name': _rel(p), 'sha256': sha256_file(p)}
              for p in (frames_file, cell_manifest, placement, mesh_file, faces_file, seam_file,
                        job / 'cache' / 'view' / 'topology.json', job / 'cache' / 'view' / 'view.glb')]
    # spike_path.json per ψ case (accepted arcs with their path, rejected arcs with failing points only)
    for case, r in results.items():
        doc = {'created_by': CREATED_BY, 'created_at': jsonio.utc_now_iso(), 'commit': commit, 'dirty': dirty,
               'inputs': inputs, 'frame': 'part', 'tcp': 'tcp_laser', 'seam_id': seam['seam_id'],
               'psi_case': case, 'head_model': args.head_model, 'theta_deg': args.theta_deg, 'mode': 'robot-only',
               'arcs': [{'index': a['index'], 'of': a['of'], 'start_s_mm': a['start_s_mm'],
                         'end_s_mm': a['end_s_mm'], 'accepted': a['accepted'], 'config_label':
                         (a['c3']['metrics']['labels'][0] if a['accepted'] else None),
                         'points': a['path'], 'failing_points': None if a['accepted'] else a['failing_points']}
                        for a in r['arcs']]}
        name = f's5_spike_path_{seam["seam_id"]}_{case}_{head_tag}.json'
        (job / name).write_text(json.dumps(doc, indent=1, allow_nan=False) + '\n')
        r['spike_path'] = {'file': name, 'sha256': sha256_file(job / name)}
        for a in r['arcs']:
            a.pop('path', None)
    rec = {
        'created_by': CREATED_BY, 'created_at': jsonio.utc_now_iso(), 'commit': commit, 'dirty': dirty,
        'task': 'WP-S S.5 (TST-002 Rev 0.3 C2, C3, C5, C8)', 'inputs': inputs,
        'cell_yaml': None, 'cell_yaml_note': 'D16 cell.yaml does not exist yet (T0.1): parameters = PlannerParams '
                                             'defaults (TST-002 §5.1), recorded below with their hash',
        'job_dir': _rel(job),
        'scene_backend': scene.record, 'kinematics_backend': kin.name, 'head_model': args.head_model,
        'clearance_links': list(clearance_links), 'clearance_method': CLEARANCE_METHOD,
        'theta_deg': args.theta_deg, 'mode': 'robot-only', 'scene_load_s': t_load,
        'params': params_rec, 'params_sha256': sha256_text(json.dumps(params_rec, sort_keys=True)),
        'pitch_min_mm': args.pitch_min_mm, 'contact_params': contact_params.as_record(),
        'head_args': parse_head_args(args.head_arg), 'f9_rule': args.f9_rule,
        'score_params': dataclasses.asdict(score_params),
        'seam': {k: seam[k] for k in ('seam_id', 'joint_edge_id', 'edges', 'solids', 'faces', 'closed', 'length_mm',
                                      'arcs', 'arc_overlap_mm', 'note')},
        'arc_ranges_mm': [list(r) for r in arc_ranges(seam['length_mm'], seam['arcs'], seam['arc_overlap_mm'],
                                                      seam['closed'])],
        'cases': results,
        'not_run': ['H-CAD head model: no head hulls exported and no H-CAD cell model (needs cad-geometry + cell-sim '
                    'work; decision 2026-11-03: only if available) — open item',
                    'C4 approach / retract / air move: S.6 / T1.5',
                    'C8 one closed robot-only path (arcs = 1): impossible under F3 + F11 (CP-53), not run',
                    'Mode B (positioner angles entered by the Engineer): only after the decision, B-12'],
        'assumptions': [
            'part and robot base poses from CAD component placements, no REF_* markers (OI-23, scene decisions D1–D3)',
            head_assumption(args.head_model, contact_params, scene.head.as_record()) + '; conduit at the head side '
            '(scene decision D7)',
            f'F9 point tolerance {contact_params.point_tolerance_mm:g} mm (TST-002 gives no value); F6 elbow 5° / '
            'shoulder 50 mm',
            'Ψ-WIRE window centre 0° (= seam X, wire-leading direction unknown, I5 / E-05) ± 15° in 5° steps',
            'arc_overlap_mm 2.0 (T0.5 example value; welding engineer to confirm)',
            f'pitch floor {args.pitch_min_mm} mm for the CP-53 refinement',
            'seam geometry: circle through the D20 edge vertices, n1 = n2 = radial (tube butt joint), checked '
            'against the B-Rep preview frames',
            'F11 wrist-3 travel ±180° applied per arc (each arc is one weld segment, emission stops between arcs, '
            'CP-53)',
            'all holders on positioner_flange (I3 / OI-27 open)'],
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / 's5_results.json').write_text(json.dumps(rec, indent=1, allow_nan=False) + '\n')
    write_md(out_dir / 's5_results.md', rec)
    for case, r in results.items():
        s = r['summary']
        print(f'{case}: C2 {s["C2"]} C3 {s["C3"]["pass"]} C5 {s["C5"]["seam_s"]:.1f} s C8 '
              f'{s["C8"]["coverage_pct"]:.1f} %')
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog='s5_run', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('seam', help='build spike_seam.json (pure)')
    p.add_argument('--input', required=True)
    p.add_argument('--repo', required=True)
    p.add_argument('--job', required=True)
    p = sub.add_parser('run', help='spike run (starts sim_mock, mock hardware only)')
    p.add_argument('--repo', required=True)
    p.add_argument('--job', required=True)
    p.add_argument('--seam-id', required=True)
    p.add_argument('--psi-cases', default='free,wire')
    p.add_argument('--head-model', default='H-BOX')
    p.add_argument('--theta-deg', type=float, default=0.0)
    p.add_argument('--clearance-links', default=None, help='comma-separated; default TST-002 F5 set')
    p.add_argument('--pitch-min-mm', type=float, default=0.05)
    p.add_argument('--out-dir', required=True)
    p.add_argument('--log-dir', default=None)
    add_head_cli(p)
    args = ap.parse_args(argv)
    return _cmd_seam(args) if args.cmd == 'seam' else _cmd_run(args)


if __name__ == '__main__':
    sys.exit(main())
