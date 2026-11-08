"""``python3 -m awm_path.cli plan`` — kinematic 7-axis weld-path planning (G1.3; CP-58, CP-59, CP-53).

Contract agreed with the web agent (``awm_api`` planner adapter, G1.4)::

    python3 -m awm_path.cli plan --job <dir> --collision off --out <file> [--seams S01 ...]
        [--head-model hcad|hbox] [--head-dir <dir>] [--travel-speed-mm-s <v>]

* stdout: ``progress <0..1> <text>`` lines (never decreasing), last line ``result <ok|rejected> …``;
* exit 0 = every planned arc accepted; exit 2 = D07 written, at least one arc rejected (failing s ranges inside);
  exit 1 = refused, nothing written, one ``error: …`` message on stderr. Refusals: ``--collision on`` (Goal 2),
  stale ``seam_axis.json`` (``awm_seam.seam_axis.read_seam_axis``), unknown seam id, description / limits
  disagreement, H-CAD head folder missing;
* ``--out`` receives D07 v2 MINOR 2.1 with ``collision_checked: false`` (validated before it is written) and
  ``planner.params.record_version`` = ``awm_core.staleness.PLANNER_RECORD_VERSION`` (G1.17);
* inputs (seam_axis.json, seams.json, placement.json, frames.yaml, head.yaml) must be plain files inside the
  repository root: an input reached through a symbolic link (inside or outside the repository, e.g. a linked head
  folder) or outside the root is refused before planning (G1.17, ``awm_core.staleness``), so every reader checks the
  same file and gives the same verdict.

``python3 -m awm_path.cli check --job <dir> [--path <file>]`` prints the problems and ``state <ok|stale|missing>`` of
a D07 by the shared staleness rule (``awm_core.staleness.document_verdict``: input hashes, inputs inside the
repository without links, record version — the rule ``awm_program`` and ``awm_api`` use); exit 0 = ok, 1 = stale or
missing. ``ok`` here does not mean ``awm_program.cli generate`` will accept the D07: generate also compares the
robot description and the NOMINAL calibration with the ones recorded in D07 and refuses an active D08.

The robot description is rendered from xacro with the ``frames.yaml`` pose arguments and ``head_model`` /
``head_dir`` (no running simulation, no rclpy). Defaults (recorded in ``planner.params``): head model ``hcad``
(Goal 1 global constraint) with the head folder ``<repo>/cell/model/head``; travel speed
``DEFAULT_TRAVEL_SPEED_MM_S`` (Assumption until D06 v2 carries it per seam).
"""

from __future__ import annotations

import argparse
import json
import os
import secrets
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from awm_core import jsonio, staleness
from awm_core.hashing import sha256_file, sha256_text
from awm_core.repository import JobPaths

CREATED_BY = 'awm_path.cli plan (G1.3 kinematic 7-axis planner)'
#: Assumption (recorded per run): WPS travel speed until D06 v2 carries one per seam.
DEFAULT_TRAVEL_SPEED_MM_S = 10.0
#: Cell identity for the context record (project-level, not part data); revision = frames.yaml hash prefix.
CELL_ID = 'APLSMN601-CELL'
EXIT_OK, EXIT_REFUSED, EXIT_REJECTED = 0, 1, 2


class Refused(Exception):
    """Planning refused before anything is written (exit 1)."""


class _Progress:
    """Prints monotone ``progress <f> <text>`` lines on stdout."""

    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self, frac: float, text: str) -> None:
        self.value = max(self.value, min(1.0, max(0.0, float(frac))))
        sys.stdout.write(f'progress {self.value:.3f} {text}\n')
        sys.stdout.flush()


def _git_record() -> Dict:
    """Commit and dirty flag of the source checkout this module runs from (symlink-install resolves to src)."""
    here = Path(__file__).resolve().parent
    try:
        commit = subprocess.run(['git', '-c', 'safe.directory=*', '-C', str(here), 'rev-parse', 'HEAD'],
                                capture_output=True, text=True, check=True, timeout=10).stdout.strip()
        dirty = subprocess.run(['git', '-c', 'safe.directory=*', '-C', str(here), 'status', '--porcelain',
                                '--untracked-files=no'], capture_output=True, text=True, check=True,
                               timeout=10).stdout.strip() != ''
        return {'commit': commit, 'dirty': dirty}
    except (OSError, subprocess.SubprocessError):
        env = os.environ.get('AWM_GIT_COMMIT', '').strip()
        if env:
            return {'commit': env, 'dirty': True}
        raise Refused('git commit of the planner source is unknown (no git checkout and AWM_GIT_COMMIT not set); '
                      'every planning record needs it')


def _run_id(created_at: str) -> str:
    stamp = created_at.replace('-', '').replace(':', '')[:15] + 'Z'
    return f'R{stamp}-{secrets.token_hex(3)}'


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='awm_path.cli', description=__doc__.split('\n')[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    pl = sub.add_parser('plan', help='plan the weld paths of a job (D07 v2)')
    pl.add_argument('--job', required=True, help='job folder <repo>/jobs/<part>_<rev>')
    pl.add_argument('--collision', required=True, choices=['on', 'off'],
                    help='off = kinematic only (Goal 1, CP-58); on is refused until Goal 2')
    pl.add_argument('--out', required=True, help='D07 output file')
    pl.add_argument('--seams', nargs='*', default=None, help='seam ids (default: all seams of seam_axis.json)')
    pl.add_argument('--head-model', choices=['hcad', 'hbox'], default='hcad')
    pl.add_argument('--head-dir', default=None, help='H-CAD head folder (default <repo>/cell/model/head)')
    pl.add_argument('--travel-speed-mm-s', type=float, default=DEFAULT_TRAVEL_SPEED_MM_S,
                    help='travel speed relative to the part (Assumption default until D06 v2)')
    ck = sub.add_parser('check', help='staleness verdict of a D07 by the shared rule (awm_core.staleness); '
                        'generate also compares the description, the calibration and the active D08')
    ck.add_argument('--job', required=True, help='job folder <repo>/jobs/<part>_<rev>')
    ck.add_argument('--path', default=None, help='D07 file (default <job>/path.json)')
    return p


def check(args: argparse.Namespace) -> int:
    """Print the shared staleness verdict of a D07; exit 0 = ok, 1 = stale or missing."""
    job = JobPaths(Path(args.job).resolve())
    path_file = Path(args.path).resolve() if args.path else job.path_json
    verdict = staleness.document_verdict(job.dir, str(path_file), job.repo.root)
    for problem in verdict.problems:
        sys.stdout.write(f'problem {problem}\n')
    sys.stdout.write(f'state {verdict.state}\n')
    sys.stdout.flush()
    return EXIT_OK if verdict.ok else EXIT_REFUSED


def _context(desc, frames_yaml: Path, head_hash: str, git: Dict) -> Dict:
    from awm_path.core.d07 import params_sha256
    components = {'kinematics': sha256_file(desc.kinematics_file), 'positioner_axis': sha256_file(frames_yaml),
                  'positioner_limits': sha256_file(desc.positioner_limits_file), f'tcp/{desc.head_model}': head_hash}
    frames_sha = sha256_file(frames_yaml)
    return {'cell_id': CELL_ID, 'cell_rev': f'frames-{frames_sha[:12]}',
            'd14_sha256': desc.as_record()['robot_description_sha256'], 'cell_config_sha256': sha256_file(desc.positioner_limits_file),
            'calibration': {'calibration_id': 'CAL-NOMINAL-FRAMES', 'status': 'NOMINAL',
                            'sha256': params_sha256(components), 'components': components},
            'robot_serial': 'SIM', 'kinematics_sha256': components['kinematics'],
            'controller_calibration_sha256': None, 'ur_safety_checksum': None, 'ur_installation': None,
            'git': git, 'image_digest': None}


def plan(args: argparse.Namespace) -> int:
    from awm_seam import joint_seams, seam_axis
    from awm_path.core.d07 import ArcPlan, PlannerInfo, build_path_doc, params_sha256
    from awm_path.core.kinematics import UrAnalyticKinematics
    from awm_path.core.sync import ArcSpec, SyncParams, plan_arc

    if args.collision == 'on':
        raise Refused('--collision on is not available: collision checking of the 7-axis planner comes with Goal 2 '
                      '(CP-58); plan with --collision off (kinematic only, collision_checked: false)')
    if not args.travel_speed_mm_s > 0.0:
        raise Refused('--travel-speed-mm-s must be > 0')
    t0 = time.monotonic()
    progress = _Progress()
    job = JobPaths(Path(args.job).resolve())
    if not job.dir.is_dir():
        raise Refused(f'job folder not found: {job.dir}')
    progress(0.0, 'reading seam_axis.json')
    axis_verdict = staleness.document_verdict(job.dir, 'seam_axis.json', job.repo.root)
    if axis_verdict.state == staleness.STALE:
        raise Refused('seam_axis.json is stale (seams, placement or cell frames changed since it was built, or an '
                      f'input outside the repository): {"; ".join(axis_verdict.problems)}\n'
                      'rebuild seam_axis.json (save the seams again) and re-plan')
    try:
        axis_doc = seam_axis.read_seam_axis(job)
        seams_doc = joint_seams.load_seams(job)
    except jsonio.StaleInputError as exc:
        raise Refused(f'seam_axis.json is stale (seams, placement or cell frames changed since it was built): {exc}\n'
                      'rebuild seam_axis.json (save the seams again) and re-plan') from exc
    except FileNotFoundError as exc:
        raise Refused(f'{exc}; build seams.json and seam_axis.json first (awm_seam joint_cli build)') from exc
    except ValueError as exc:
        raise Refused(f'seam_axis.json / seams.json refused: {exc}') from exc
    by_id = {s['seam_id']: s for s in axis_doc['seams']}
    wanted = list(args.seams) if args.seams else list(by_id)
    unknown = [s for s in wanted if s not in by_id]
    if unknown:
        raise Refused(f'unknown seam id(s) {unknown}; seam_axis.json has {sorted(by_id)}')
    if not wanted:
        raise Refused('seam_axis.json has no seams')
    overlap = {s['seam_id']: float(s.get('arc_overlap_mm', 0.0)) for s in seams_doc['seams']}

    frames_yaml = seam_axis.frames_path(job)
    head_dir = args.head_dir or (str(job.repo.cell_dir / 'model' / 'head') if args.head_model == 'hcad' else None)
    inputs = [job.dir / 'seam_axis.json', job.seams_json, seam_axis.placement_path(job), frames_yaml]
    if args.head_model == 'hcad':
        inputs.append(Path(head_dir) / 'head.yaml')
    linked = staleness.linked_or_outside(inputs, job.repo.root)
    if linked:
        raise Refused('input(s) reached through a symbolic link or outside the repository '
                      + str(job.repo.root.resolve()) + ': ' + ', '.join(f'{p} -> {p.resolve()}' for p in linked)
                      + '; a D07 records only plain files inside the repository so that every reader (awm_program, '
                        f'awm_api) checks the same file; {staleness.LINK_FIX} and plan again')
    progress(0.02, f'rendering the robot description (head_model {args.head_model})')
    from awm_path.description import load_planning_description
    try:
        desc = load_planning_description(frames_yaml, args.head_model, str(Path(head_dir).resolve())
                                         if head_dir else None)
    except (ValueError, FileNotFoundError) as exc:
        raise Refused(f'robot description refused: {exc}') from exc
    git = _git_record()
    kin = UrAnalyticKinematics()
    params = SyncParams()

    arcs: List = []
    for sid in wanted:
        seam = by_id[sid]
        for a in seam['arcs']:
            arcs.append((seam, ArcSpec(int(a['index']), int(a['of']), float(a['s_start_mm']), float(a['s_end_mm']),
                                       a.get('positioner_angle_deg'))))
    total = sum(a.s_end_mm - a.s_start_mm for _, a in arcs)
    plans = []
    done = 0.0
    for seam, arc in arcs:
        w = (arc.s_end_mm - arc.s_start_mm) / total
        lo = 0.05 + 0.9 * done

        def cb(f, text, lo=lo, w=w):
            progress(lo + 0.9 * w * min(1.0, max(0.0, f)), text)

        cb(0.0, f'{seam["seam_id"]} arc {arc.index}/{arc.of}: planning in mode {seam["positioner_mode"]}')
        res = plan_arc(kin, desc.sync_cell, seam, arc, seam['positioner_mode'], params, cb)
        plans.append(ArcPlan(res, args.travel_speed_mm_s, overlap.get(seam['seam_id'], 0.0)))
        done += w
        cb(1.0, f'{seam["seam_id"]} arc {arc.index}/{arc.of}: '
                + ('accepted' if res.ok else f'rejected ({res.reason})'))

    head_hash = sha256_text(json.dumps(desc.head.as_record(), sort_keys=True, default=str))
    if args.head_model == 'hcad':
        head_hash = sha256_file(Path(head_dir) / 'head.yaml')
    header = jsonio.make_header(CREATED_BY, inputs, job.dir)
    header['run_id'] = _run_id(header['created_at'])
    run_params = {staleness.RECORD_VERSION_KEY: staleness.PLANNER_RECORD_VERSION, 'collision': 'off',
                  'travel_speed_mm_s': args.travel_speed_mm_s,
                  'travel_speed_source': 'cli default (Assumption)'
                  if args.travel_speed_mm_s == DEFAULT_TRAVEL_SPEED_MM_S else 'cli argument',
                  'head_model': args.head_model, 'seams': wanted, 'sync': params.record(),
                  'description': desc.as_record()}
    info = PlannerInfo(kinematics_backend=kin.name, head_model=args.head_model, params=run_params)
    flange_t_part = seam_axis.pose_mm_to_matrix(axis_doc['flange_T_part'], 'seam_axis.json flange_T_part')
    doc = build_path_doc(plans, desc.sync_cell, flange_t_part, info, header,
                         _context(desc, frames_yaml, head_hash, git), params)
    out = Path(args.out)
    jsonio.write_json(out, doc, 'v2/path')
    n_ok, n_bad = len(doc['seams']), len(doc['rejected'])
    progress(1.0, f'done: {n_ok} arc(s) accepted, {n_bad} rejected, kinematic only (collision not checked)')
    status = 'ok' if n_bad == 0 else 'rejected'
    sys.stdout.write(f'result {status} accepted_arcs={n_ok} rejected_arcs={n_bad} collision_checked=false '
                     f'params_sha256={params_sha256(doc["planner"]["params"])} run_time_s={time.monotonic() - t0:.2f} '
                     f'out={out}\n')
    sys.stdout.flush()
    return EXIT_OK if n_bad == 0 else EXIT_REJECTED


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(list(argv) if argv is not None else None)
    try:
        return check(args) if args.cmd == 'check' else plan(args)
    except Refused as exc:
        sys.stderr.write(f'error: {exc}\n')
        sys.stderr.flush()
        return EXIT_REFUSED


def entry() -> None:
    """console_scripts entry (``ros2 run awm_path awm_path_cli plan …``)."""
    sys.exit(main())


if __name__ == '__main__':
    entry()
