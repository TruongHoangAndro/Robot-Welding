"""``python3 -m awm_program.cli generate`` — synchronised 7-joint program D09 2.1 from the G1.3 path D07 (G1.5;
A61–A64; CP-58, CP-59; DES-001 Rev 0.5 §4.3 steps 5–6, §5.3, §6.2).

    python3 -m awm_program.cli generate --job <job_dir> [--path <path.json>] [--head-dir <dir>]

* stdout: ``progress <0..1> <text>`` lines (never decreasing), last line
  ``result ok program_rev=P### out=<file> cycle_time_s=<v> collision_checked=false`` (exit 0) or
  ``result refused <reason>`` (exit 1, nothing written; the full message also on stderr as ``error: …``) — the line
  style of ``awm_path.cli`` (parsed by the ``awm_api`` runner).
* Output: ``<job>/programs/P###/program.json`` (next free revision; an existing revision is never changed).

Refusals: D07 stale by the shared rule ``awm_core.staleness`` (an input outside the repository, missing or with
another hash, or ``planner.params.record_version`` missing / older: "planned with an older planner; re-plan" —
G1.17, the same verdict as ``awm_path.cli check`` and ``awm_api``), a program input reached through a symbolic link
or outside the repository, schema invalid, a rejected arc (no partial programs), an arc with
``speed_within_limits: false`` or a weld motion above a velocity / acceleration limit (arc and the largest feasible
travel speed in the message; never slowed silently, no limit raised), the robot description differs from the one
recorded in D07, the NOMINAL calibration differs from the D07 one, an active D08 exists (the 7-axis chain needs a
D08 with the positioner axis, not available yet: re-plan after calibration is a later task), approach / retract not
reachable on the weld branch.

Calibration: Goal 1 uses the nominal frames (``cell/model/frames.yaml``) recorded as ``calibration: NOMINAL`` with
the hash of its components (DES-001 §5.1 D08 "NOMINAL from the cell CAD until measured").
"""

from __future__ import annotations

import argparse
import json
import os
import secrets
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from awm_core import jsonio, staleness
from awm_core.hashing import sha256_file, sha256_text
from awm_core.repository import JobPaths

CREATED_BY = 'awm_program.cli generate (G1.5 synchronised 7-joint program)'
CELL_ID = 'APLSMN601-CELL'
EXIT_OK, EXIT_REFUSED = 0, 1
#: D07 planner.head_model (contract T0.5) → description head model
HEAD_MODEL = {'cad': 'hcad', 'box': 'hbox'}


class Refused(Exception):
    """Generation refused; nothing written."""


class _Progress:
    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self, frac: float, text: str) -> None:
        self.value = max(self.value, min(1.0, max(0.0, float(frac))))
        sys.stdout.write(f'progress {self.value:.3f} {text}\n')
        sys.stdout.flush()


def _git_record() -> Dict:
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
        raise Refused('git commit of the program generator source is unknown (no git checkout and AWM_GIT_COMMIT '
                      'not set); every program record needs it')


def _run_id(created_at: str) -> str:
    return 'R' + created_at.replace('-', '').replace(':', '')[:15] + 'Z-' + secrets.token_hex(3)


def _norm(v):
    """Float rounding of the D07 writer (9 decimals) so records compare exactly."""
    if isinstance(v, dict):
        return {k: _norm(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_norm(x) for x in v]
    if isinstance(v, bool) or v is None or isinstance(v, (str, int)):
        return v
    return float(round(float(v), 9))


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='awm_program.cli', description=__doc__.split('\n')[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('generate', help='generate the next program revision (D09 2.1) from D07')
    g.add_argument('--job', required=True, help='job folder <repo>/jobs/<part>_<rev>')
    g.add_argument('--path', default=None, help='D07 file (default <job>/path.json)')
    g.add_argument('--head-dir', default=None, help='H-CAD head folder (default <repo>/cell/model/head)')
    return p


def generate(args: argparse.Namespace) -> int:
    from awm_path.core.d07 import params_sha256
    from awm_path.core.kinematics import UrAnalyticKinematics
    from awm_path.description import load_planning_description
    from awm_seam import seam_axis
    from awm_program.core.program7 import (SCHEMA_VERSION, ProgramParams, ProgramRefused, build_program,
                                           pose_matrix)
    from awm_program.description import load_program_cell

    progress = _Progress()
    job = JobPaths(Path(args.job).resolve())
    if not job.dir.is_dir():
        raise Refused(f'job folder not found: {job.dir}')
    path_file = Path(args.path).resolve() if args.path else job.path_json
    progress(0.0, f'reading D07 {path_file.name}')
    try:
        path = jsonio.read_json(path_file, 'v2/path', check=False)
    except (FileNotFoundError, jsonio.SchemaValidationError) as exc:
        raise Refused(f'D07 refused: {exc}') from exc
    verdict = staleness.document_verdict(job.dir, str(path_file), job.repo.root, path)
    if not verdict.ok:
        raise Refused('D07 is stale (seams, placement, cell frames or head changed since planning, an input outside '
                      f'the repository, or an older planner): {"; ".join(verdict.problems)}; re-plan')
    if not str(path['schema_version']).startswith('2.') or 'sync' not in path['planner'].get('params', {}):
        raise Refused('D07 is not a G1.3 kinematic 7-axis path (planner.params.sync missing); re-plan')
    if job.repo.active_calibration.exists():
        raise Refused(f'an active calibration D08 exists ({job.repo.active_calibration.name}): the 7-axis program needs '
                      'a D08 with the positioner axis and a path planned with it, which Goal 1 does not provide; '
                      'remove it for simulation or wait for the D08 v2 task')
    frames_yaml = seam_axis.frames_path(job)
    head_model = HEAD_MODEL.get(path['planner']['head_model'])
    if head_model is None:
        raise Refused(f'unknown D07 head model {path["planner"]["head_model"]!r}')
    head_dir = args.head_dir or (str(job.repo.cell_dir / 'model' / 'head') if head_model == 'hcad' else None)
    progress(0.05, f'rendering the robot description (head_model {head_model})')
    try:
        desc = load_planning_description(frames_yaml, head_model, str(Path(head_dir).resolve()) if head_dir else None)
    except (ValueError, FileNotFoundError) as exc:
        raise Refused(f'robot description refused: {exc}') from exc
    recorded = path['planner']['params'].get('description')
    if _norm(desc.as_record()) != _norm(recorded):
        raise Refused('the robot description differs from the one D07 was planned with (head, cell frames, limits '
                      'or UR kinematics changed); re-plan')
    head_hash = sha256_text(json.dumps(desc.head.as_record(), sort_keys=True, default=str))
    inputs: List[Path] = [path_file, job.dir / 'seam_axis.json', job.seams_json, job.part_json, frames_yaml]
    if head_model == 'hcad':
        head_yaml = Path(head_dir) / 'head.yaml'
        head_hash = sha256_file(head_yaml)
        inputs.append(head_yaml)
    linked = staleness.linked_or_outside(inputs, job.repo.root)
    if linked:
        raise Refused('program input(s) reached through a symbolic link or outside the repository '
                      + str(job.repo.root.resolve()) + ': ' + ', '.join(f'{q} -> {q.resolve()}' for q in linked)
                      + f'; {staleness.LINK_FIX} and re-plan')
    components = {'kinematics': sha256_file(desc.kinematics_file), 'positioner_axis': sha256_file(frames_yaml),
                  'positioner_limits': sha256_file(desc.positioner_limits_file), f'tcp/{head_model}': head_hash}
    calibration = {'calibration_id': 'CAL-NOMINAL-FRAMES', 'status': 'NOMINAL',
                   'sha256': params_sha256(components), 'components': components}
    if calibration['sha256'] != path['context']['calibration']['sha256']:
        raise Refused('the NOMINAL calibration (frames.yaml, positioner limits, UR kinematics, head) differs from the '
                      'one D07 was planned with; re-plan')
    axis = json.loads((job.dir / 'seam_axis.json').read_text(encoding='utf-8'))
    seams = json.loads(job.seams_json.read_text(encoding='utf-8'))
    part = json.loads(job.part_json.read_text(encoding='utf-8'))
    sequence = {s['seam_id']: int(s.get('sequence', 0)) for s in seams['seams']}
    try:
        cell = load_program_cell(desc)
    except (ValueError, KeyError, FileNotFoundError) as exc:
        raise Refused(f'cell limits / home refused: {exc}') from exc
    git = _git_record()
    params = ProgramParams()

    def cb(f, text):
        progress(0.1 + 0.85 * f, text)

    try:
        body = build_program(path, sequence, pose_matrix(axis['flange_T_part']), cell, UrAnalyticKinematics(),
                             params, None, cb)
    except ProgramRefused as exc:
        raise Refused(str(exc)) from exc

    rev = job.next_program_rev()
    header = jsonio.make_header(CREATED_BY, inputs, job.dir)
    gen_params = {'program': params.record(), 'limits': body['limits'],
                  'limit_sources': {'velocity': 'robot description (URDF <limit>, positioner_limits.yaml)',
                                    'acceleration': 'awm_description joint_limits_v2.yaml, positioner_limits.yaml',
                                    'home': 'awm_description initial_positions_v2.yaml'},
                  'assumptions': ['approach / retract 40 mm along the beam axis', 'weld speed ramp on the seam after '
                                  'WeldStart and before WeldEnd at weld_ramp_accel_mm_s2', 'air / approach / retract at '
                                  'motion_*_scaling of the limits', 'air moves joint-interpolated, not planned around '
                                  'obstacles and not collision-checked (Goal 1, CP-58)']}
    speed_source = path['planner']['params'].get('travel_speed_source', 'D07')
    doc = {
        'schema_version': SCHEMA_VERSION, 'created_by': header['created_by'], 'created_at': header['created_at'],
        'inputs': header['inputs'], 'program_rev': rev, 'part_no': str(part['part_no']),
        'part_rev': str(part['part_rev']), 'run_id': _run_id(header['created_at']),
        'context': {'cell_id': CELL_ID, 'cell_rev': path['context']['cell_rev'],
                    'd14_sha256': desc.as_record()['robot_description_sha256'],
                    'cell_config_sha256': sha256_file(desc.positioner_limits_file), 'calibration': calibration,
                    'robot_serial': 'SIM', 'kinematics_sha256': components['kinematics'],
                    'controller_calibration_sha256': None, 'ur_safety_checksum': None, 'ur_installation': None,
                    'git': git, 'image_digest': None},
        'joint_names': {'robot': list(cell.robot_joints), 'positioner': list(cell.positioner_joints)},
        'home': {'robot_deg': list(cell.home_deg[:6]), 'positioner_deg': cell.home_deg[6]},
        'segments': body['segments'], 'events': body['events'], 'cycle_time_s': body['cycle_time_s'],
        'collision_checked': False,
        'generator': {'params': gen_params, 'params_sha256': params_sha256(gen_params),
                      'path': {'file': os.path.relpath(path_file, job.dir), 'sha256': sha256_file(path_file),
                               'run_id': path['run_id']},
                      'planner_params_sha256': path['planner']['params_sha256'],
                      'head_model': path['planner']['head_model'],
                      'travel_speeds': [{'seam_id': w['seam_id'], 'arc_index': w['arc_index'],
                                         'travel_speed_mm_s': w['travel_speed_mm_s'], 'source': speed_source}
                                        for w in body['weld_segments']],
                      'weld_segments': body['weld_segments'], 'limits': body['limits'],
                      'process': 'not assigned (no D06 in Goal 1): events carry wps_id / preset null'},
    }
    out = job.program_json(rev)
    if out.exists():
        raise Refused(f'{out} exists; a program revision is never overwritten')
    jsonio.write_json(out, doc, 'v2/program')
    progress(1.0, f'{rev}: {len(doc["segments"])} segments, {len(doc["events"])} events, kinematic only '
                  '(collision not checked)')
    sys.stdout.write(f'result ok program_rev={rev} out={out} cycle_time_s={doc["cycle_time_s"]:.2f} '
                     'collision_checked=false\n')
    sys.stdout.flush()
    return EXIT_OK


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(list(argv) if argv is not None else None)
    try:
        return generate(args)
    except Refused as exc:
        text = str(exc)
        sys.stdout.write('result refused ' + ' '.join(text.split()) + '\n')
        sys.stdout.flush()
        sys.stderr.write(f'error: {text}\n')
        sys.stderr.flush()
        return EXIT_REFUSED


def entry() -> None:
    """console_scripts entry (``ros2 run awm_program generate_7axis …``)."""
    sys.exit(main())


if __name__ == '__main__':
    entry()
