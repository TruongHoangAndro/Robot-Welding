"""WP-S S.5b spike run: J2 Mode B **screening** — positioner angle table for the Engineer (Pivot evidence for the
TST-002 meeting; DES-001 Rev 0.6 draft §4.3 step 3, §6.3 Mode B, B-12; TST-002 Rev 0.3 draft §5.1, §5.3 C2, C3, C8).

    ros2 run awm_path s5b_run --input <s5b_mode_b_input.yaml> --repo <repo> --job <job> --head-model H-BOX \
        --out-dir <records dir>

Spike / simulation tool: starts its own sim_mock launch (mock hardware only; the positioner is the mock joint). Seam,
θ step, ψ cases, pitch rule, overlap, bins and set sizes come from the data file; the θ limits come from the sim cell
config (``awm_description/config/positioner_limits.yaml``, Assumption, OI-27). Nothing is chosen for the Engineer:

1. θ list = multiples of the step inside the limits (0 included). θ and θ ± 360° place the part and holders
   identically (checked by FK of ``positioner_flange``), so one sweep per class.
2. Per θ class and ψ case, on the whole closed seam: CP-53 pitch refinement with the ``scene_valid`` rule (lead
   decision 2026-11-04), S.3 sweep ψ × 8 IK branches, S.4 scores (F4, F5 at the weld clearance, F9; F9 computed once
   per seam point and reused — it does not depend on θ or ψ), failure classes and pairs, s-intervals with ≥ 1 valid
   node, continuous DP windows (``core.mode_b.window_lengths``).
3. Coverage map (bins) and the 2- / 3-angle sets whose windows cover the seam with the join overlap — **suggestions**.
4. Records ``s5b_mode_b_screening.json`` + ``.md`` in --out-dir; candidates (csv.gz) per θ in the job folder.
"""

from __future__ import annotations

import argparse
import gzip
import json
import os
import shutil
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

import numpy as np

from awm_core import jsonio, units
from awm_path.core.mode_b import circular_runs
from awm_path.gate_g2 import add_head_cli, head_assumption
from awm_path.core.s5 import PITCH_RULES

CREATED_BY = 'awm_path.spike_s5b'
RECORD = 's5b_mode_b_screening'
_INPUT_KEYS = ('seam_id', 'seam_file', 'theta_step_deg', 'psi_cases', 'pitch_rule', 'pitch_min_mm', 'coverage_bin_mm',
               'set_sizes', 'join_overlap_mm', 'window_start_stride', 'max_sets_listed', 'time_budget_s',
               'b03_margin_mm')


# ---------------------------------------------------------------------------------------------------------------
# pure parts
# ---------------------------------------------------------------------------------------------------------------

def read_input(path) -> Dict[str, Any]:
    """Engineer / lead data file of the screening (all keys required; checked)."""
    import yaml
    d = yaml.safe_load(Path(path).read_text())
    missing = [k for k in _INPUT_KEYS if k not in d]
    if missing:
        raise ValueError(f'{path}: missing keys {missing}')
    if d['pitch_rule'] not in PITCH_RULES:
        raise ValueError(f'{path}: pitch_rule must be one of {PITCH_RULES}')
    if not set(d['psi_cases']) <= {'free', 'wire'} or not d['psi_cases']:
        raise ValueError(f'{path}: psi_cases must be a non-empty subset of free, wire')
    for k in ('theta_step_deg', 'pitch_min_mm', 'coverage_bin_mm', 'time_budget_s', 'b03_margin_mm'):
        d[k] = float(d[k])
        if d[k] <= 0:
            raise ValueError(f'{path}: {k} must be > 0')
    if d['join_overlap_mm'] is not None:
        d['join_overlap_mm'] = float(d['join_overlap_mm'])
    d['set_sizes'] = [int(x) for x in d['set_sizes']]
    d['window_start_stride'] = int(d['window_start_stride'])
    d['max_sets_listed'] = int(d['max_sets_listed'])
    return d


def valid_intervals(flags: Sequence[bool], pitch_mm: float) -> List[Dict[str, Any]]:
    """Circular runs of points with ≥ 1 valid node, in mm (the end may pass the seam start: s > L)."""
    return [{'s_from_mm': a * pitch_mm, 's_to_mm': b * pitch_mm, 'points': b - a + 1} for a, b in circular_runs(flags)]


def failure_summary(points: Sequence[Mapping[str, Any]], classes: Sequence[Optional[str]], pitch_mm: float,
                    n_pairs: int = 6) -> Dict[str, Any]:
    """Per θ: counts per failing class, the dominant class, the most frequent colliding / blocking pairs, and the
    circular runs of each class with their pairs (``classes`` from ``core.s5.point_classes``)."""
    counts = Counter(c for c in classes if c)
    pairs: Counter = Counter()
    for p, c in zip(points, classes):
        if c:
            pairs.update(p.get('pairs', {}))
    runs = []
    for cls in sorted(counts):
        for a, b in circular_runs([c == cls for c in classes]):
            rp: Counter = Counter()
            for i in range(a, b + 1):
                rp.update(points[i % len(points)].get('pairs', {}))
            runs.append({'class': cls, 'points': b - a + 1, 's_from_mm': a * pitch_mm, 's_to_mm': b * pitch_mm,
                         'pairs': dict(sorted(rp.items(), key=lambda kv: (-kv[1], kv[0]))[:n_pairs])})
    runs.sort(key=lambda r: (r['s_from_mm'], r['class']))
    top = sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))
    return {'class_counts': dict(sorted(counts.items())), 'dominant_class': top[0][0] if top else None,
            'top_pairs': dict(sorted(pairs.items(), key=lambda kv: (-kv[1], kv[0]))[:n_pairs]), 'runs': runs}


def coverage_map(bins_by_theta: Mapping[float, Sequence[bool]], bin_mm: float, length_mm: float
                 ) -> List[Dict[str, Any]]:
    """Per bin the angles whose continuous windows cover it; consecutive bins with the same angle set merged."""
    thetas = sorted(bins_by_theta)
    nb = len(next(iter(bins_by_theta.values()))) if bins_by_theta else 0
    out: List[Dict[str, Any]] = []
    for k in range(nb):
        ts = [t for t in thetas if bins_by_theta[t][k]]
        lo, hi = k * bin_mm, min((k + 1) * bin_mm, length_mm)
        if out and out[-1]['thetas'] == ts:
            out[-1]['s_to_mm'] = hi
        else:
            out.append({'s_from_mm': lo, 's_to_mm': hi, 'thetas': ts})
    return out


def rotating_set_ids(cell_manifest: Mapping[str, Any], placement: Mapping[str, Any]) -> set:
    """Object ids that turn with the positioner (parent frame ``positioner_flange``) in the cell scene manifest and
    the job placement D19 (the part and the holders on the flange; Assumption I3 / OI-27 as in the scene files)."""
    return {str(o['object_id']) for doc in (cell_manifest, placement) for o in doc.get('objects', [])
            if o.get('parent_frame') == 'positioner_flange'}


def theta_invariant_blocking(rows, psi_all: Sequence[float], head_links, rotating_ids, bin_mm: float,
                             length_mm: float) -> Dict[str, Any]:
    """Which seam bins no positioner angle can open (Mode B).

    With work / travel angle fixed, the head pose relative to the part and to every object that turns with it
    depends only on (s, ψ), not on θ or the IK branch. A contact between a head link and the rotating set is
    therefore the same at every θ. ``rows`` = (s_mm, ψ_deg, kinematic reason, F4 contacts) of the candidates of all
    screened θ. A bin is ``blocked`` when **at every sampled point of the bin** (every s of ``rows`` in the bin) every
    ψ of ``psi_all`` was kinematically valid at that point at some θ and every such candidate had a head ↔
    rotating-set contact (F4, 3 mm padding). ψ never kinematically valid at a point of the bin are listed
    (``psi_unseen``, union over the points): for them the screening has no evidence and the bin is not marked blocked
    (review M3: ψ seen at different points of a bin are no evidence for one point).
    """
    nb = max(1, int(np.ceil(length_mm / bin_mm - 1e-9)))
    seen: List[Dict[float, set]] = [{} for _ in range(nb)]
    free: List[Dict[float, set]] = [{} for _ in range(nb)]
    pairs: Counter = Counter()
    head, rot = set(head_links), set(rotating_ids)
    for s_mm, psi, reason, contacts in rows:
        b = min(nb - 1, int(float(s_mm) // bin_mm))
        key = round(float(s_mm), 6)
        seen[b].setdefault(key, set())
        free[b].setdefault(key, set())
        if reason:
            continue
        hp = []
        for pair in (contacts or '').split(';'):
            if not pair:
                continue
            a, c = pair.split('<->')
            if (a in rot and c in head) or (c in rot and a in head):
                hp.append(pair)
        seen[b][key].add(float(psi))
        if hp:
            pairs.update(hp)
        else:
            free[b][key].add(float(psi))
    psi_all = sorted(float(x) for x in psi_all)
    bins = []
    for b in range(nb):
        unseen = sorted({p for pts in seen[b].values() for p in psi_all if p not in pts})
        free_b = sorted(set().union(*free[b].values())) if free[b] else []
        blocked = bool(seen[b]) and not unseen and not free_b
        bins.append({'s_from_mm': b * bin_mm, 's_to_mm': min((b + 1) * bin_mm, length_mm), 'blocked': blocked,
                     'psi_unseen': unseen if seen[b] else psi_all, 'psi_free': free_b, 'points': len(seen[b])})
    runs = []
    for b in bins:
        if b['blocked'] and runs and abs(runs[-1]['s_to_mm'] - b['s_from_mm']) < 1e-9:
            runs[-1]['s_to_mm'] = b['s_to_mm']
        elif b['blocked']:
            runs.append({'s_from_mm': b['s_from_mm'], 's_to_mm': b['s_to_mm']})
    return {'bins': bins, 'blocked_runs': runs, 'pairs': dict(sorted(pairs.items(), key=lambda kv: (-kv[1], kv[0])))}


def _f(v: Optional[float], nd: int = 1) -> str:
    return '—' if v is None else f'{v:.{nd}f}'


def _iv(ivs: Sequence[Mapping[str, Any]], n: int = 4) -> str:
    s = ', '.join(f'{a["s_from_mm"]:.1f}–{a["s_to_mm"]:.1f}' for a in ivs[:n])
    return (s + (' …' if len(ivs) > n else '')) or 'none'


def write_md(path: Path, rec: Mapping[str, Any]) -> None:
    """Human-readable summary of ``s5b_mode_b_screening.json``."""
    lim = rec['positioner_limits']
    L: List[str] = [
        f'# WP-S S.5b — J2 Mode B screening ({rec["head_model"]}) — positioner angle table for the Engineer', '',
        '**Screening only.** The positioner angles of Mode B are Engineer-entered data (DES-001 Rev 0.6 draft §4.3 '
        'step 3, B-12). The sets at the end are *suggestions* for Truong, not a decision and not a planned path. '
        'No partial path is accepted anywhere; the final Mode B run with the chosen angles is a separate run.', '',
        f'Record `{RECORD}.json`; commit `{rec["commit"]}`, dirty: {str(rec["dirty"]).lower()}, created '
        f'{rec["created_at"]}. Scene hash `{rec["scene_backend"]["scene_hash"]}`; kinematics `{rec["kinematics_backend"]}`;'
        f' scene backend `{rec["scene_backend"]["content"]["backend"]}`; head {rec["head_model"]}; clearance links '
        f'{", ".join(rec["clearance_links"])} (F5 method: {rec["clearance_method"]}).', '',
        f'Seam {rec["seam"]["seam_id"]}, closed, length {rec["seam"]["length_mm"]:.3f} mm, whole seam (not split). '
        f'Pitch rule `{rec["pitch_rule"]}` (CP-53, lead decision 2026-11-04), floor {rec["input"]["pitch_min_mm"]} mm. '
        f'Join overlap {rec["join_overlap_mm"]} mm. Window start stride {rec["input"]["window_start_stride"]}.', '',
        f'Positioner limits (Assumption, sim cell config `{lim["source"]}`, OI-27): {lim["lower_deg"]:g}° … '
        f'{lim["upper_deg"]:g}°. θ step {rec["input"]["theta_step_deg"]:g}° → {len(rec["thetas_deg"])} angles; '
        f'{len(rec["theta_classes"])} geometric classes (θ ≡ θ ± 360°, flange FK max difference '
        f'{rec["theta_equivalence"]["max_flange_diff_mm"]:.2e} mm / {rec["theta_equivalence"]["max_flange_diff_deg"]:.2e}°'
        '). One sweep per class; the class is shown by its representative in (−180°, 180°].', '']
    for case, c in rec['cases'].items():
        L += [f'## Ψ-{case.upper()} (ψ = {c["psi_deg"][0]:g} … {c["psi_deg"][-1]:g}°, {len(c["psi_deg"])} values)', '',
              '| θ class (°) | In-limit θ (°) | Points / pitch (mm) | Valid-node coverage | s with ≥ 1 valid node (mm) '
              '| Longest continuous window (mm) | Dominant failure / top pairs | Time (s) |',
              '|---|---|---|---|---|---|---|---|']
        for r in c['thetas']:
            w = r['longest_window']
            win = (f'{w["s_from_mm"]:.1f}–{w["s_to_mm"]:.1f} ({w["length_mm"]:.1f} mm, {w["pct"]:.1f} %, '
                   f'{w["label"]})') if w else 'none'
            fs = r['failures']
            fail = 'none' if not fs['dominant_class'] else (
                f'{fs["dominant_class"]} ({", ".join(f"{k} {v}" for k, v in fs["class_counts"].items())}); '
                + ', '.join(f'{k} ×{v}' for k, v in list(fs['top_pairs'].items())[:3]))
            L.append(f'| {r["theta_deg"]:g} | {", ".join(f"{t:g}" for t in r["equivalent_thetas_deg"])} | '
                     f'{r["pitch"]["points"]} / {r["pitch"]["pitch_mm"]:.3f}{"" if r["pitch"]["ok"] else " (floor)"} | '
                     f'{r["valid_node_pct"]:.1f} % | {_iv(r["valid_intervals"])} | {win} | {fail} | '
                     f'{r["timing_s"]["total_s"]:.0f} |')
        L += ['', f'### Coverage map Ψ-{case.upper()} (θ classes whose continuous windows cover each '
                  f'{rec["input"]["coverage_bin_mm"]:g} mm bin)', '', '| s (mm) | θ classes (°) |', '|---|---|']
        for m in c['coverage_map']:
            L.append(f'| {m["s_from_mm"]:.0f}–{m["s_to_mm"]:.1f} | {", ".join(f"{t:g}" for t in m["thetas"]) or "**none**"} |')
        L += ['', f'Bins covered by at least one θ: {c["bins_covered_any_pct"]:.1f} %.', '',
              f'### Suggestions Ψ-{case.upper()} (for Truong — not a decision)', '']
        for k, sets in c['cover_sets'].items():
            if sets['sets']:
                L.append(f'**{k} angles:** {sets["count"]} covering set(s) with ≥ {rec["join_overlap_mm"]} mm overlap at '
                         f'every join (no redundant angle); top {len(sets["sets"])} by the smallest join overlap:')
                L.append('')
                for s in sets['sets']:
                    L.append('- θ ' + ' / '.join(f'{x:g}°' for x in s['thetas']) + ' — segments ' + '; '.join(
                        f'θ {g["theta_deg"]:g}°: s {g["s_from_mm"]:.1f}–{g["s_to_mm"]:.1f}' for g in s['segments'])
                        + f' — smallest join overlap {s["min_join_overlap_mm"]:.1f} mm')
            else:
                b = sets['best']
                L.append(f'**{k} angles:** no set covers 100 % with the join overlap. Best coverage: '
                         + ('none' if not b else (' / '.join(f'{x:g}°' for x in b['thetas'])
                                                 + f' → {b["covered_pct"]:.1f} % (' + '; '.join(
                                                     f'θ {g["theta_deg"]:g}°: s {g["s_from_mm"]:.1f}–{g["s_to_mm"]:.1f}'
                                                     for g in b['segments']) + ')')))
            L.append('')
        L += [f'### Failing runs Ψ-{case.upper()} (per θ class; runs of consecutive points, pairs = run total)', '']
        for r in c['thetas']:
            L.append(f'- θ {r["theta_deg"]:g}°: ' + ('; '.join(
                f'{x["class"]} s {x["s_from_mm"]:.1f}–{x["s_to_mm"]:.1f}'
                + (f' [{", ".join(f"{k} ×{v}" for k, v in list(x["pairs"].items())[:3])}]' if x['pairs'] else '')
                for x in r['failures']['runs']) or 'none'))
        L.append('')
    L += ['## Not run / open', ''] + [f'- {x}' for x in rec['not_run']] + ['', '## Assumptions', ''] + [
        f'- {x}' for x in rec['assumptions']]
    path.write_text('\n'.join(L) + '\n')


# ---------------------------------------------------------------------------------------------------------------
# runner (ROS, mock hardware only)
# ---------------------------------------------------------------------------------------------------------------

def _gzip_csv(path: Path) -> Path:
    gz = path.with_suffix(path.suffix + '.gz')
    with open(path, 'rb') as src, gzip.GzipFile(gz, 'wb', mtime=0) as dst:
        shutil.copyfileobj(src, dst)
    os.remove(path)
    return gz


def _screen_theta(ctx: Dict[str, Any], theta: float, psi: Sequence[float], case: str) -> Dict[str, Any]:
    import dataclasses
    from awm_core.hashing import sha256_file
    from awm_path.core.contact import contact_check
    from awm_path.core.mode_b import bins_covered, maximal_windows, window_intervals, window_lengths
    from awm_path.core.s5 import classify_points, point_classes, refine_pitch
    from awm_path.core.scoring import score_candidates
    from awm_path.core.seam_input import frames_at
    from awm_path.core.sweep import sweep, tcp_pose, write_candidates_csv

    params, seam, scene = ctx['params'], ctx['seam'], ctx['scene']
    length, pts = seam['length_mm'], seam['geometry']['points']
    t_all = time.perf_counter()
    scene.set_theta(theta)
    w = scene.world_T_links([0.0] * 6, theta, ['base_link', 'positioner_flange'])
    base_T_part = np.linalg.inv(w['base_link']) @ ctx['world_T_object'](ctx['part_obj'], w['positioner_flange'])
    cache: Dict[int, Any] = {}       # key = point index at the finest level (s = key · L / n_max)
    n_max = ctx['n_max']
    t_sweep = t_score = 0.0
    contact_new = 0

    def sample(n: int):
        nonlocal t_sweep, t_score, contact_new
        if n_max % n:
            raise ValueError('refinement level does not divide the finest level')
        keys = [i * (n_max // n) for i in range(n)]
        new = [k for k in keys if k not in cache]
        if new:
            s_vals = np.array([k * length / n_max for k in new])
            fp = frames_at(pts, s_vals, True, length)
            fb = [base_T_part @ f for f in fp]
            t0 = time.perf_counter()
            lay = sweep(fb, ctx['tool0_T_tcp'], ctx['kin'], params, psi)
            t_sweep += time.perf_counter() - t0
            con = []
            for k, f in zip(new, fp):
                if k not in ctx['contact']:
                    ctx['contact'][k] = contact_check(ctx['part'], tcp_pose(f, 0.0), ctx['seam_faces'],
                                                      ctx['contact_params'])
                    contact_new += 1
                con.append(ctx['contact'][k])
            t0 = time.perf_counter()
            sc = score_candidates(lay, fb, base_T_part, theta, scene, ctx['part'], ctx['seam_faces'],
                                  ctx['contact_params'], ctx['score_params'], contact=con)
            t_score += time.perf_counter() - t0
            for j, k in enumerate(new):
                cache[k] = (lay[j], sc.scores[j], {name: col[j] for name, col in sc.columns.items()})
        layers = [[dataclasses.replace(c, point=i) for c in cache[k][0]] for i, k in enumerate(keys)]
        scores = [cache[k][1] for k in keys]
        sample.columns = {name: [cache[k][2][name] for k in keys] for name in cache[keys[0]][2]}
        return (layers, scores) if ctx['pitch_rule'] == 'scene_valid' else layers

    pr = refine_pitch(length, ctx['rate'], sample, params, ctx['pitch_min_mm'], rule=ctx['pitch_rule'], closed=True)
    layers, columns = pr.layers, sample.columns
    scores = pr.scores if pr.scores is not None else [cache[i * (n_max // pr.points)][1] for i in range(pr.points)]
    pitch = pr.pitch_mm
    pts_cls = classify_points(layers, columns)
    classes = point_classes(pts_cls)
    flags = [c is None for c in classes]
    t0 = time.perf_counter()
    lengths = window_lengths(layers, scores, params, stride=ctx['stride'])
    wins = maximal_windows(lengths)
    t_win = time.perf_counter() - t0
    ivs = window_intervals(wins, pitch)
    longest = None
    if wins:
        a, la = max(wins, key=lambda x: (x[1], -x[0]))
        from awm_path.core.ladder import solve
        lay2, sc2 = list(layers) + list(layers), list(scores) + list(scores)
        res = solve(lay2[a:a + la], sc2[a:a + la], params, closed=False)
        lab = res.path[0].label if res.ok else None
        longest = {'start_point': a, 'points': la, 's_from_mm': a * pitch, 's_to_mm': (a + la - 1) * pitch,
                   'length_mm': (la - 1) * pitch, 'pct': 100.0 * (la - 1) * pitch / length, 'label': lab,
                   'metrics': None}
        if res.ok:
            from awm_path.core.s5 import path_metrics
            longest['metrics'] = path_metrics(res.path)
    csv_name = f's5b_candidates_{seam["seam_id"]}_theta{theta:+g}_{case}_{ctx["head_tag"]}.csv'
    csv_path = ctx['job_dir'] / csv_name
    write_candidates_csv(str(csv_path), layers, columns)
    gz = _gzip_csv(csv_path)
    covered = sum(flags)
    return {
        'theta_deg': theta, 'equivalent_thetas_deg': ctx['classes'][theta],
        'pitch': dict(pr.as_record(), rotation_rate_deg_per_mm=ctx['rate']),
        'valid_node_pct': 100.0 * covered / len(flags), 'valid_points': covered,
        'valid_intervals': valid_intervals(flags, pitch),
        'windows': [{'start_point': a, 'points': la, 's_from_mm': iv[0], 's_to_mm': iv[1]}
                    for (a, la), iv in zip(wins, ivs)],
        'window_intervals_mm': [list(iv) for iv in ivs],
        'longest_window': longest,
        'bins': bins_covered(ivs, length, ctx['bin_mm']),
        'failures': failure_summary(pts_cls, classes, pitch),
        'candidates_csv': {'file': gz.name, 'sha256': sha256_file(gz)},
        'timing_s': {'sweep_s': t_sweep, 'scores_s': t_score, 'windows_s': t_win, 'contact_new_points': contact_new,
                     'total_s': time.perf_counter() - t_all},
    }


def _cmd_run(args) -> int:
    import dataclasses
    import rclpy
    from awm_bringup.cell_config import load_positioner_limits, positioner_limits_path
    from awm_bringup.testing import SimLaunch
    from awm_core.hashing import sha256_file, sha256_text
    from awm_path.backends.move_group import MoveGroupCellScene
    from awm_path.core.contact import ContactParams, PartMesh
    from awm_path.core.kinematics import UrAnalyticKinematics, pose_error
    from awm_path.core.mode_b import best_cover, cover_sets, screening_thetas, theta_classes
    from awm_path.core.scene_files import CLEARANCE_METHOD, DEFAULT_CLEARANCE_LINKS, part_object_files, world_T_object
    from awm_path.core.scoring import SceneScoreParams
    from awm_path.core.sweep import PlannerParams, required_points
    from awm_path.core.head import contact_params_for
    from awm_path.gate_g2 import _git, _rel, launch_args_from_frames, parse_head_args, head_launch_args
    from awm_path.spike_s5 import _rotation_rate_deg_per_mm

    t_run = time.perf_counter()
    inp_file = Path(args.input)
    inp = read_input(inp_file)
    repo, out_dir = Path(args.repo), Path(args.out_dir)
    job = repo / 'jobs' / args.job
    frames_file = repo / 'cell' / 'model' / 'frames.yaml'
    cell_manifest = repo / 'cell' / 'model' / 'scene_manifest.json'
    placement = job / 'placement.json'
    seam_file = job / inp['seam_file']
    seams_doc = json.loads(seam_file.read_text())
    jsonio.validate(seams_doc, 'v2/seams')
    seam = [s for s in seams_doc['seams'] if s['seam_id'] == inp['seam_id']]
    if len(seam) != 1 or not seam[0]['closed']:
        raise SystemExit(f'seam {inp["seam_id"]} not in {seam_file} or not closed')
    seam = seam[0]
    lim_file = positioner_limits_path()
    lim = load_positioner_limits(lim_file)
    thetas = screening_thetas(lim['lower_deg'], lim['upper_deg'], inp['theta_step_deg'])
    classes = theta_classes(thetas)
    if args.probe_theta is not None:
        classes = {k: v for k, v in classes.items() if k == float(args.probe_theta)}
        inp['psi_cases'] = inp['psi_cases'][:1]
        if not classes:
            raise SystemExit(f'--probe-theta {args.probe_theta} is not a θ class representative')
    join_ov = seam['arc_overlap_mm'] if inp['join_overlap_mm'] is None else inp['join_overlap_mm']
    clearance_links = tuple(args.clearance_links.split(',')) if args.clearance_links else DEFAULT_CLEARANCE_LINKS
    params = PlannerParams()
    score_params = SceneScoreParams()
    kin = UrAnalyticKinematics()
    head_tag = args.head_model.lower().replace('-', '')
    length, pts = seam['length_mm'], seam['geometry']['points']
    rate = _rotation_rate_deg_per_mm(pts, True, length, 0.0, length)
    n0 = required_points(length, rate, params, closed=True)
    n_max = n0
    while length / (2 * n_max) >= inp['pitch_min_mm']:
        n_max *= 2
    launch_args = dict(launch_args_from_frames(frames_file), cell_manifest=str(cell_manifest.resolve()),
                       **head_launch_args(args.head_model, args.head_dir, repo),
                       **parse_head_args(args.head_arg))
    results: Dict[str, Any] = {}
    not_run: List[str] = []
    equiv = {'max_flange_diff_mm': 0.0, 'max_flange_diff_deg': 0.0}
    with SimLaunch(log_dir=args.log_dir, launch_file='sim_mock.launch.py', launch_args=launch_args) as sim:
        rclpy.init()
        node = rclpy.create_node('awm_s5b_run')
        try:
            t_load = time.perf_counter()
            scene = MoveGroupCellScene(node, cell_manifest, placement, job, head_model=args.head_model,
                                       clearance_links=clearance_links)
            scene.load(timeout_s=180.0)
            part_obj, mesh_file, faces_file = part_object_files(scene.job)
            part = PartMesh.from_files(mesh_file, faces_file)
            contact_params = contact_params_for(scene.head, ContactParams(rule_b=args.f9_rule))
            t_load = time.perf_counter() - t_load
            for rep, members in classes.items():
                ref = scene.world_T_links([0.0] * 6, rep, ['positioner_flange'])['positioner_flange']
                for t in members:
                    fk = scene.world_T_links([0.0] * 6, t, ['positioner_flange'])['positioner_flange']
                    dp, dr = pose_error(ref, fk)
                    equiv['max_flange_diff_mm'] = max(equiv['max_flange_diff_mm'], float(dp))
                    equiv['max_flange_diff_deg'] = max(equiv['max_flange_diff_deg'], float(units.rad_to_deg(dr)))
            if equiv['max_flange_diff_mm'] > 1e-6 or equiv['max_flange_diff_deg'] > 1e-6:
                raise SystemExit(f'θ ≡ θ ± 360° does not hold in the scene: {equiv}')
            ctx = {'params': params, 'kin': kin, 'scene': scene, 'part': part, 'part_obj': part_obj, 'seam': seam,
                   'world_T_object': world_T_object, 'tool0_T_tcp': scene.head.tool0_T_tcp_mm,
                   'seam_faces': (tuple(seam['faces']['n1']), tuple(seam['faces']['n2'])),
                   'contact_params': contact_params, 'score_params': score_params, 'job_dir': job,
                   'head_tag': head_tag, 'pitch_min_mm': inp['pitch_min_mm'], 'pitch_rule': inp['pitch_rule'],
                   'rate': rate, 'n_max': n_max, 'contact': {}, 'stride': inp['window_start_stride'],
                   'bin_mm': inp['coverage_bin_mm'], 'classes': classes}
            case_time: Dict[str, float] = {}
            for case in inp['psi_cases']:
                elapsed = time.perf_counter() - t_run
                if case_time and elapsed + 0.6 * max(case_time.values()) > inp['time_budget_s']:
                    not_run.append(f'Ψ-{case.upper()}: skipped, time budget {inp["time_budget_s"]:.0f} s '
                                   f'(elapsed {elapsed:.0f} s)')
                    continue
                t_case = time.perf_counter()
                psi = params.psi_free if case == 'free' else params.psi_wire
                rows = []
                for rep in classes:
                    r = _screen_theta(ctx, rep, psi, case)
                    w = r['longest_window']
                    print(f'{case} θ {rep:+g}: {r["pitch"]["points"]} pts, valid {r["valid_node_pct"]:.1f} %, longest '
                          f'window {w["length_mm"] if w else 0:.1f} mm, {r["timing_s"]["total_s"]:.0f} s', flush=True)
                    rows.append(r)
                case_time[case] = time.perf_counter() - t_case
                wins = {r['theta_deg']: [tuple(iv) for iv in r['window_intervals_mm']] for r in rows}
                sets: Dict[str, Any] = {}
                for k in inp['set_sizes']:
                    t0 = time.perf_counter()
                    found = cover_sets(wins, length, join_ov, k)
                    sets[str(k)] = {'count': len(found), 'sets': found[:inp['max_sets_listed']],
                                    'best': None if found else best_cover(wins, length, k),
                                    'time_s': time.perf_counter() - t0}
                bins = {r['theta_deg']: r['bins'] for r in rows}
                nb = len(rows[0]['bins'])
                results[case] = {'psi_deg': list(psi), 'thetas': rows, 'time_s': case_time[case],
                                 'coverage_map': coverage_map(bins, inp['coverage_bin_mm'], length),
                                 'bins_covered_any_pct': 100.0 * sum(any(b[i] for b in bins.values())
                                                                     for i in range(nb)) / nb,
                                 'cover_sets': sets}
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
              for p in (inp_file, frames_file, cell_manifest, placement, mesh_file, faces_file, seam_file, lim_file)]
    rec = {
        'created_by': CREATED_BY, 'created_at': jsonio.utc_now_iso(), 'commit': commit, 'dirty': dirty,
        'task': 'WP-S S.5b J2 Mode B screening (Pivot evidence, TST-002 meeting 2026-11-06; PLAN T1.4 step 1)'
                + (' — TIMING PROBE, not the result' if args.probe_theta is not None else ''),
        'inputs': inputs, 'input': inp, 'job_dir': _rel(job),
        'scene_backend': scene.record, 'kinematics_backend': kin.name, 'head_model': args.head_model,
        'clearance_links': list(clearance_links), 'clearance_method': CLEARANCE_METHOD, 'mode': 'B (screening)',
        'positioner_limits': dict(lim, source=_rel(lim_file)), 'thetas_deg': thetas,
        'theta_classes': {f'{k:g}': v for k, v in classes.items()}, 'theta_equivalence': equiv,
        'pitch_rule': inp['pitch_rule'], 'pitch_levels': {'ceiling_points': n0, 'finest_points': n_max},
        'join_overlap_mm': join_ov, 'scene_load_s': t_load, 'total_s': time.perf_counter() - t_run,
        'params': params_rec, 'params_sha256': sha256_text(json.dumps(params_rec, sort_keys=True)),
        'contact_params': contact_params.as_record(), 'score_params': dataclasses.asdict(score_params),
        'head_args': parse_head_args(args.head_arg), 'f9_rule': args.f9_rule,
        'seam': {k: seam[k] for k in ('seam_id', 'joint_edge_id', 'edges', 'solids', 'faces', 'closed', 'length_mm',
                                      'arc_overlap_mm', 'note')},
        'cases': results,
        'not_run': not_run + [
            'final Mode B run with the angles chosen by the Engineer (next run, after Truong picks 2–3 angles)',
            'index moves between angles, approach / retract / air moves (C4): not part of the screening',
            'H-CAD head model: no head hulls / H-CAD cell model (open item, as in S.5)'],
        'assumptions': [
            f'positioner limits {lim["lower_deg"]:g}…{lim["upper_deg"]:g}° from the sim cell config '
            '(awm_description positioner_limits.yaml), not the real drive / cables / belts (OI-14, OI-27)',
            'θ ≡ θ ± 360°: the scene is periodic in θ (checked by flange FK); cable wrap of the real positioner not '
            'modelled (OI-27)',
            'all holders on positioner_flange, i.e. they rotate with the part (I3 / OI-27 open)',
            'part and robot base poses from CAD component placements, no REF_* markers (OI-23, scene decisions D1–D3)',
            head_assumption(args.head_model, contact_params, scene.head.as_record()) + '; conduit at the head side '
            '(scene decision D7)',
            f'F9 point tolerance {contact_params.point_tolerance_mm:g} mm (TST-002 gives no value); F6 elbow 5° / '
            'shoulder 50 mm',
            'Ψ-WIRE window centre 0° (= seam X, wire-leading direction unknown, I5 / E-05) ± 15° in 5° steps',
            f'join overlap {join_ov} mm between Mode B segments = seam arc_overlap_mm (T0.5 example value; welding '
            'engineer to confirm, OI-11 / OI-16)',
            f'pitch floor {inp["pitch_min_mm"]} mm; CP-53 pitch rule on scene-valid nodes (lead decision 2026-11-04)',
            'F11 wrist-3 travel ±180° applied per window (each Mode B segment is one weld segment, emission stops '
            'between segments, DES-001 §6.2)',
            f'B-03 margin {inp["b03_margin_mm"]} mm (S.6 Assumption; = F4 padding) — recorded, not used: no positioner '
            'index move is planned in the screening',
            'covering sets use one continuous window per angle (one weld segment per angle); joins need the overlap',
            'seam geometry: circle through the D20 edge vertices, n1 = n2 = radial (tube butt joint), S.5 record'],
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / f'{RECORD}.json').write_text(json.dumps(rec, indent=1, allow_nan=False) + '\n')
    write_md(out_dir / f'{RECORD}.md', rec)
    for case, c in results.items():
        print(f'{case}: ' + ', '.join(f'{k} angles: {v["count"]} sets' for k, v in c['cover_sets'].items()))
    return 0


#: Head-side links of the cell model (H-BOX / H-CAD link names, not part data): nozzle, head body, mount, conduit.
HEAD_LINKS = ('nozzle_tip', 'nozzle_body', 'head_body', 'head_mount', 'conduit')
CHECK_RECORD = 's5b_mode_b_rotating_set_check'


def main_check(argv: Optional[List[str]] = None) -> int:
    """``s5b_check``: pure post-processing of a screening record (no ROS): the seam bins that no positioner angle
    can open because the head collides with the part or with a holder that turns with it (θ-invariant)."""
    import csv
    from awm_core.hashing import sha256_file
    from awm_path.gate_g2 import _git, _rel

    ap = argparse.ArgumentParser(prog='s5b_check', description=main_check.__doc__)
    ap.add_argument('--record', required=True)
    ap.add_argument('--repo', required=True)
    ap.add_argument('--job', required=True)
    ap.add_argument('--head-links', default=','.join(HEAD_LINKS))
    ap.add_argument('--bin-mm', type=float, default=None, help='default: coverage_bin_mm of the record')
    ap.add_argument('--out-dir', required=True)
    args = ap.parse_args(argv)
    rec_file, repo = Path(args.record), Path(args.repo)
    job = repo / 'jobs' / args.job
    rec = json.loads(rec_file.read_text())
    cell_file, placement_file = repo / 'cell' / 'model' / 'scene_manifest.json', job / 'placement.json'
    for name, path in (('scene_manifest.json', cell_file), ('placement.json', placement_file)):
        want = [i['sha256'] for i in rec['inputs'] if i['name'].endswith(name)]
        if not want or want[0] != sha256_file(path):
            raise SystemExit(f'{path}: hash differs from the screening record')
    rot = rotating_set_ids(json.loads(cell_file.read_text()), json.loads(placement_file.read_text()))
    head = tuple(args.head_links.split(','))
    bin_mm = args.bin_mm or rec['input']['coverage_bin_mm']
    length = rec['seam']['length_mm']
    out: Dict[str, Any] = {}
    for case, c in rec['cases'].items():
        rows = []
        for r in c['thetas']:
            f = job / r['candidates_csv']['file']
            if sha256_file(f) != r['candidates_csv']['sha256']:
                raise SystemExit(f'{f}: hash differs from the screening record')
            pitch = r['pitch']['pitch_mm']
            with gzip.open(f, 'rt', newline='') as fh:
                for x in csv.DictReader(fh):
                    rows.append((int(x['point']) * pitch, float(x['psi_deg']), x['reason'] or None,
                                 x['scene_contacts'] or ''))
        out[case] = theta_invariant_blocking(rows, c['psi_deg'], head, rot, bin_mm, length)
        out[case]['blocked_mm'] = sum(b['s_to_mm'] - b['s_from_mm'] for b in out[case]['bins'] if b['blocked'])
    here = Path(args.out_dir).resolve()
    doc = {'created_by': CREATED_BY + '.check', 'created_at': jsonio.utc_now_iso(),
           'commit': _git(['rev-parse', 'HEAD'], here), 'dirty': bool(_git(['status', '--porcelain'], here)),
           'inputs': [{'name': _rel(p), 'sha256': sha256_file(p)} for p in (rec_file, cell_file, placement_file)],
           'screening_commit': rec['commit'], 'head_links': list(head), 'rotating_set': sorted(rot),
           'bin_mm': bin_mm, 'seam_length_mm': length, 'cases': out,
           'method': theta_invariant_blocking.__doc__.strip()}
    Path(args.out_dir).mkdir(parents=True, exist_ok=True)
    (Path(args.out_dir) / f'{CHECK_RECORD}.json').write_text(json.dumps(doc, indent=1, allow_nan=False) + '\n')
    L = ['# WP-S S.5b — J2 seam bins that no positioner angle can open (head ↔ rotating set)', '',
         f'Record `{CHECK_RECORD}.json`; commit `{doc["commit"]}`, dirty: {str(doc["dirty"]).lower()}; from the '
         f'screening record `{rec_file.name}` (commit `{rec["commit"]}`). Head links {", ".join(head)}; rotating set '
         f'(parent frame positioner_flange, Assumption I3 / OI-27) {", ".join(sorted(rot))}; bins {bin_mm:g} mm.', '',
         'Method: ' + ' '.join(doc['method'].split()), '']
    for case, r in out.items():
        L.append(f'**Ψ-{case.upper()}:** blocked at every θ: '
                 + (', '.join(f's {x["s_from_mm"]:.0f}–{x["s_to_mm"]:.1f} mm' for x in r['blocked_runs']) or 'none')
                 + f' ({r["blocked_mm"]:.1f} mm = {100 * r["blocked_mm"] / length:.1f} % of the seam). Pairs: '
                 + ', '.join(f'{k} ×{v}' for k, v in list(r['pairs'].items())[:6]) + '.')
        unseen = [b for b in r['bins'] if b['psi_unseen'] and not b['psi_free']]
        if unseen:
            L.append('Bins without a free ψ but with ψ never kinematically valid (no evidence, not marked): '
                     + ', '.join(f'{b["s_from_mm"]:.0f}' for b in unseen) + '.')
        L.append('')
    (Path(args.out_dir) / f'{CHECK_RECORD}.md').write_text('\n'.join(L) + '\n')
    for case, r in out.items():
        print(case, r['blocked_runs'], f'{r["blocked_mm"]:.1f} mm')
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog='s5b_run', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--input', required=True)
    ap.add_argument('--repo', required=True)
    ap.add_argument('--job', required=True)
    ap.add_argument('--head-model', default='H-BOX')
    ap.add_argument('--clearance-links', default=None, help='comma-separated; default TST-002 F5 set')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--log-dir', default=None)
    add_head_cli(ap)
    ap.add_argument('--probe-theta', type=float, default=None,
                    help='timing probe: screen only this θ class and the first ψ case (record is a probe, not the result)')
    return _cmd_run(ap.parse_args(argv))


if __name__ == '__main__':
    sys.exit(main())
