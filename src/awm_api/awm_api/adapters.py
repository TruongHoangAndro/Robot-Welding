"""Thin adapters to the engineering CLIs. Each external command is built in exactly one function here, so the
exact command line of a CLI that is not merged yet is wired in one place.

* A11–A14 / A13: ``awm_geometry.cli import`` + ``export-view`` in the OCC conda interpreter (ADR-15).
* A21 / A22 (G1.2, branch ``g1-2-seam-axis``, not merged): ``joint_seam build --job J --picks P [--no-axis]``
  (writes ``seams.json`` D04 v2.1 and ``seam_axis.json``). Prefix from ``Settings.seam_cmd``.
* A41–A49 (G1.3): ``plan --job J --collision off --out O --head-dir H [--travel-speed-mm-s V] [--seams S01 …]``.
  The planner writes D07 to ``--out`` (the run folder); awm_api promotes it to ``<job>/path.json`` only when the
  run succeeds. Prefix from ``Settings.plan_cmd``. Without a speed the planner default (Assumption) is used.
* A61–A64 (G1.5): ``generate --job J``: writes ``<job>/programs/P###/program.json`` itself and ends with
  ``result ok program_rev=P### out=<file> …`` (exit 0) or ``result refused <reason>`` (exit 1). Prefix from
  ``Settings.program_cmd``.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from awm_core import jsonio, occ_bridge
from awm_core.hashing import sha256_file

from awm_api.problems import ProblemError
from awm_api.runner import Step, StepsError
from awm_api.settings import Settings

IMPORT_OUTPUTS = ('model.step', 'part.json', 'cache/edges.json', 'cache/view/view.glb', 'cache/view/topology.json',
                  'cache/components.json')


def _occ_prefix(settings: Settings) -> List[str]:
    try:
        return occ_bridge.interpreter_command(settings.occ_python_cmd) + ['-m', 'awm_geometry.cli']
    except occ_bridge.GeometryCliError as exc:
        raise StepsError(str(exc)) from exc


def _occ_env() -> Dict[str, str]:
    try:
        return occ_bridge.geometry_env()
    except occ_bridge.GeometryCliError as exc:
        raise StepsError(str(exc)) from exc


def staging_dir(settings: Settings, run_dir: Path) -> Path:
    """Hidden staging job folder of an import run (``Repository.jobs()`` ignores hidden folders)."""
    return settings.jobs_dir / f'.import-{run_dir.name}'


def import_steps(settings: Settings, run_dir: Path, step_file: Path, part_no: str, part_rev: str,
                 component: Optional[str], up_axis: str, components: bool = False) -> List[Step]:
    """STEP → job: A11–A14 import, then the D20 view model export, both into the staging folder. ``components``
    (CP-60 job assembly: positioner + fixture + part): whole file per component, plus the component tree."""
    prefix, env = _occ_prefix(settings), _occ_env()
    stage = staging_dir(settings, run_dir)
    if components and component:
        raise ProblemError(400, 'bad-request', detail='give either a component or the assembly import, not both')
    comp = ['--components'] if components else ['--component', component] if component else []
    tree = ([Step('inspect-assembly', prefix + ['inspect-assembly', '--step', str(stage / 'model.step'), '--out',
                                                str(stage / 'cache' / 'components.json')], env=env, weight=1.0)]
            if components else [])
    return [
        Step('import', prefix + ['import', '--job', str(stage), '--step', str(step_file), '--part-no', part_no,
                                 '--part-rev', part_rev] + comp, env=env, weight=3.0),
        Step('export-view', prefix + ['export-view', '--step', str(stage / 'model.step'), '--job', str(stage)]
             + comp + ['--up-axis', up_axis, '--out', str(stage / 'cache' / 'view')], env=env, weight=1.0),
    ] + tree


def promote_import(settings: Settings, run_dir: Path, job_id: str) -> Dict[str, str]:
    stage = staging_dir(settings, run_dir)
    target = settings.jobs_dir / job_id
    if target.exists():
        raise RuntimeError(f'job {job_id} was created by someone else during the import')
    stage.rename(target)
    return {n: sha256_file(target / n) for n in IMPORT_OUTPUTS if (target / n).is_file()}


def discard_import(settings: Settings, run_dir: Path) -> None:
    shutil.rmtree(staging_dir(settings, run_dir), ignore_errors=True)


def _error_lines(text: str, keep: int = 6) -> str:
    lines = [ln for ln in text.splitlines() if ln.strip()]
    errors = [ln for ln in lines if re.search(r'error|refus|fail|unknown|not ', ln, re.IGNORECASE)]
    return ' | '.join((errors or lines)[-keep:])


def build_seams(settings: Settings, job: Path, picks: Sequence[Dict[str, Any]], work_dir: Path,
                build_axis: bool) -> Dict[str, Any]:
    """Picked ids + attributes → ``seams.json`` (+ ``seam_axis.json``) through the seam CLI; returns D04."""
    work_dir.mkdir(parents=True, exist_ok=True)
    picks_file = work_dir / 'picks.json'
    # The G1.2 CLI refuses request fields it does not know; ``range_mm`` (contract SeamPick) is refused by the API
    # in Goal 1 before this point, so it is never sent.
    request = [{k: v for k, v in p.items() if k != 'range_mm'} for p in picks]
    picks_file.write_text(json.dumps({'seams': request}, indent=1) + '\n')
    argv = settings.seam_argv() + ['build', '--job', str(job), '--picks', str(picks_file)]
    if not build_axis:
        argv.append('--no-axis')
    try:
        proc = subprocess.run(argv, capture_output=True, text=True, timeout=settings.seam_timeout_s,
                              env=dict(os.environ), check=False)
    except FileNotFoundError as exc:
        raise ProblemError(500, 'seam-cli-missing', detail=f'cannot start the seam CLI {argv[0]!r}: {exc}') from exc
    except subprocess.TimeoutExpired as exc:
        raise ProblemError(500, 'seam-cli-timeout',
                           detail=f'seam CLI timed out after {settings.seam_timeout_s:.0f} s') from exc
    (work_dir / 'log.txt').write_text(f'$ {" ".join(argv)}\n--- stdout\n{proc.stdout}\n--- stderr\n{proc.stderr}')
    if proc.returncode != 0:
        if 'No module named' in proc.stderr and 'awm_seam' in proc.stderr:
            raise ProblemError(500, 'seam-cli-missing', detail='seam CLI awm_seam.joint_cli is not installed '
                               '(G1.2 not merged yet); set AWM_SEAM_CMD')
        raise ProblemError(422, 'seam-build-failed', 'Seam list refused',
                           detail=_error_lines(proc.stderr + '\n' + proc.stdout) or f'exit {proc.returncode}')
    doc = json.loads((job / 'seams.json').read_text())
    return doc


def plan_steps(settings: Settings, run_dir: Path, job: Path, seams: Optional[List[str]],
               travel_speed_mm_s: Optional[float] = None) -> List[Step]:
    argv = settings.plan_argv() + ['plan', '--job', str(job), '--collision', 'off', '--out',
                                   str(run_dir / 'path.json'), '--head-dir', str(settings.head_dir)]
    if travel_speed_mm_s is not None:
        argv += ['--travel-speed-mm-s', repr(float(travel_speed_mm_s))]
    if seams:
        argv += ['--seams', *seams]
    return [Step('plan', argv, env=dict(os.environ))]


OVERRIDE_FIELD = 'cad_check_override'


def _validate_field(value: Dict[str, Any], schema: str) -> None:
    """Validate one top-level property against its sub-schema of ``schema`` (the rest of the document was already
    validated by the CLI that wrote it)."""
    import jsonschema  # dependency of awm_core.jsonio

    sub = jsonio.load_schema(schema)['properties'][OVERRIDE_FIELD]
    errors = list(jsonschema.Draft202012Validator(sub).iter_errors(value))
    if errors:
        raise RuntimeError(f'{OVERRIDE_FIELD} does not match {schema}: {errors[0].message}')


def _write_doc(path: Path, doc: Dict[str, Any]) -> None:
    tmp = path.with_name(f'.{path.name}.tmp')
    tmp.write_text(json.dumps(doc, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


def record_override(doc_path: Path, job: Path, override: Dict[str, Any], schema: str) -> None:
    """CP-62: write the cross-check-4 override reference into a D07 / D09 document written by a CLI that does not know
    the field yet (least invasive; becomes a no-op check when the CLI writes it itself). Refuses when the override or
    the cross-check changed while the run was going (the run slot normally prevents that)."""
    files = (job / override['file'], job / 'cad_check.json')
    if not all(f.is_file() for f in files) or sha256_file(files[0]) != override['sha256'] or \
            sha256_file(files[1]) != override['cad_check_sha256']:
        raise RuntimeError('the CP-62 override or the cross-check changed during the run; nothing promoted')
    _validate_field(override, schema)
    doc = json.loads(doc_path.read_text())
    if doc.get('collision_checked') is not False:
        raise RuntimeError('a CP-62 override is valid for kinematic-only documents (collision_checked false) only')
    if doc.get(OVERRIDE_FIELD) not in (None, override):
        raise RuntimeError(f'{doc_path.name} names another {OVERRIDE_FIELD} than the one this run relied on')
    doc[OVERRIDE_FIELD] = override
    _write_doc(doc_path, doc)


def promote_plan(run_dir: Path, job: Path, override: Optional[Dict[str, Any]] = None) -> Dict[str, str]:
    out = run_dir / 'path.json'
    if not out.is_file():
        raise RuntimeError('the planner finished without writing path.json')
    if override is not None:
        record_override(out, job, override, 'v2/path')
    tmp = job / '.path.json.tmp'
    shutil.copyfile(out, tmp)
    tmp.replace(job / 'path.json')
    return {'path.json': sha256_file(job / 'path.json')}


def _recorded_path_sha(program: Dict[str, Any]) -> Optional[str]:
    gen = (program.get('generator') or {}).get('path') or {}
    if gen.get('sha256'):
        return gen['sha256']
    return next((i.get('sha256') for i in program.get('inputs', []) if i.get('name') == 'path.json'), None)


def carry_override(job: Path, program_file: Path) -> None:
    """CP-62: copy D07 ``cad_check_override`` into the new D09 when the program was generated from that exact
    path.json (hash recorded by the program CLI)."""
    path = jsonio_read(job / 'path.json')
    ref = (path or {}).get(OVERRIDE_FIELD)
    if not ref:
        return
    program = json.loads(program_file.read_text())
    if _recorded_path_sha(program) != sha256_file(job / 'path.json'):
        raise RuntimeError('the program does not record the current path.json; cannot carry the CP-62 override')
    if program.get(OVERRIDE_FIELD) == ref:
        return
    if program.get(OVERRIDE_FIELD) is not None:
        raise RuntimeError(f'program.json names another {OVERRIDE_FIELD} than path.json')
    if program.get('collision_checked') is not False:
        raise RuntimeError('a CP-62 override is valid for kinematic-only programs (collision_checked false) only')
    _validate_field(ref, 'v2/program')
    program[OVERRIDE_FIELD] = ref
    _write_doc(program_file, program)


def jsonio_read(path: Path) -> Optional[Dict[str, Any]]:
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


_RESULT_OK_RE = re.compile(r'^result ok .*\bprogram_rev=(P[0-9]{3})\b')


def program_steps(settings: Settings, job: Path) -> List[Step]:
    return [Step('program', settings.program_argv() + ['generate', '--job', str(job)], env=dict(os.environ))]


def promote_program(run_dir: Path, job: Path) -> Dict[str, str]:
    """The program CLI writes into the job itself; report the revision named by its ``result ok`` line."""
    log = (run_dir / 'log.txt').read_text(errors='replace') if (run_dir / 'log.txt').is_file() else ''
    revs = [m.group(1) for m in (_RESULT_OK_RE.match(ln) for ln in log.splitlines()) if m]
    if not revs:
        raise RuntimeError('the program CLI finished without a "result ok program_rev=P###" line')
    rel = f'programs/{revs[-1]}/program.json'
    if not (job / rel).is_file():
        raise RuntimeError(f'the program CLI reported {revs[-1]} but {rel} does not exist')
    try:
        carry_override(job, job / rel)
    except Exception as exc:
        where = quarantine_program(job, revs[-1], run_dir.name, str(exc))
        raise RuntimeError(f'{exc}; program {revs[-1]} quarantined to {where} (never left without the CP-62 '
                           'override record)') from exc
    return {rel: sha256_file(job / rel)}


def quarantine_program(job: Path, rev: str, run_id: str, reason: str) -> str:
    """Move a program revision that failed its post-step out of ``programs/`` (review minor 4): it lands in
    ``programs/.quarantine/<rev>-<run_id>/`` with ``QUARANTINE.txt``; job listings ignore hidden folders."""
    target = job / 'programs' / '.quarantine' / f'{rev}-{run_id}'
    target.parent.mkdir(parents=True, exist_ok=True)
    (job / 'programs' / rev).rename(target)
    (target / 'QUARANTINE.txt').write_text(f'{rev} from run {run_id} quarantined by awm_api: {reason}\n')
    return target.relative_to(job).as_posix()


# -- CP-60 / CP-61: cell model, positioner-axis pick, TCP teach, job placement + cross-check -----------------------

def save_upload(upload, target: Path, max_upload_mb: float) -> Path:
    """Stream an uploaded file to ``target`` (413 problem above the size limit)."""
    target.parent.mkdir(parents=True, exist_ok=True)
    limit, size = int(max_upload_mb * 1024 * 1024), 0
    with target.open('wb') as fh:
        while chunk := upload.file.read(1024 * 1024):
            size += len(chunk)
            if size > limit:
                raise ProblemError(413, 'upload-too-large', detail=f'STEP larger than {max_upload_mb:.0f} MB')
            fh.write(chunk)
    return target


def cell_import_steps(settings: Settings, step_file: Path, out_dir: Path, up_axis: str) -> List[Step]:
    """S10 import: component tree (A18 input) and the coarse display model of every leaf component."""
    prefix, env = _occ_prefix(settings), _occ_env()
    tree = out_dir / 'cell_components.json'
    return [
        Step('inspect-assembly', prefix + ['inspect-assembly', '--step', str(step_file), '--out', str(tree)],
             env=env, weight=2.0),
        Step('export-cell-view', prefix + ['export-cell-view', '--step', str(step_file), '--components', str(tree),
                                           '--out', str(out_dir / 'view'), '--up-axis', up_axis], env=env, weight=2.0),
    ]


def geometry_info_step(settings: Settings) -> Step:
    """``awm_geometry.cli info`` (OCC version + awm_geometry version; review I2 cache identity)."""
    return Step('geometry-info', _occ_prefix(settings) + ['info'], env=_occ_env())


def component_view_steps(settings: Settings, step_file: Path, component: str, up_axis: str,
                         out_dir: Path) -> List[Step]:
    """D20 edge view of one component (for a circle pick on it)."""
    return [Step('export-view', _occ_prefix(settings) + ['export-view', '--step', str(step_file), '--component',
                                                         component, '--up-axis', up_axis, '--out', str(out_dir)],
                 env=_occ_env())]


def pick_circle_steps(settings: Settings, step_file: Path, edge_id: str, up_axis: str, out: Path,
                      component: Optional[str] = None, components: bool = False, reverse: bool = False) -> List[Step]:
    """Circle pick with the same load arguments as the view the edge id came from."""
    argv = _occ_prefix(settings) + ['pick-circle', '--step', str(step_file)]
    argv += ['--components'] if components else ['--component', component] if component else []
    argv += ['--edge', edge_id, '--up-axis', up_axis, '--out', str(out)] + (['--reverse'] if reverse else [])
    return [Step('pick-circle', argv, env=_occ_env())]


def cell_save_steps(settings: Settings, cad: Path, repo_root: Path) -> List[Step]:
    """Roles + axis pick -> cad_map.yaml -> cell/model (export-scene without a job)."""
    prefix, env = _occ_prefix(settings), _occ_env()
    step, cmap = cad / 'cell.step', cad / 'cad_map.yaml'
    return [
        Step('apply-roles', prefix + ['apply-roles', '--step', str(step), '--tree', str(cad / 'cell_components.json'),
                                      '--roles', str(cad / 'roles.json'), '--axis', str(cad / 'axis_pick.json'),
                                      '--out', str(cmap)], env=env, weight=1.0),
        Step('export-scene', prefix + ['export-scene', '--step', str(step), '--map', str(cmap),
                                       '--repo', str(repo_root)], env=env, weight=4.0),
    ]


def tcp_teach_steps(settings: Settings, head_step: Path, head_map: Path, repo_root: Path, component: str,
                    edge_id: str, reverse: bool, stickout_mm: float, user: str) -> List[Step]:
    """CP-61: export-head with the TCP taught from a nozzle-mouth circle."""
    argv = _occ_prefix(settings) + ['export-head', '--step', str(head_step), '--map', str(head_map), '--repo',
                                    str(repo_root), '--tcp-component', component, '--tcp-edge', edge_id,
                                    '--stickout-mm', repr(float(stickout_mm)), '--taught-by', user]
    return [Step('export-head', argv + (['--tcp-reverse'] if reverse else []), env=_occ_env(), weight=2.0)]


def job_placement_steps(settings: Settings, job_id: str) -> List[Step]:
    """CP-60 job import: placement.json (source job_cad) + cad_check.json."""
    return [Step('job-placement', _occ_prefix(settings) + ['job-placement', '--repo', str(settings.repo_root),
                                                           '--job', job_id], env=_occ_env(), weight=3.0)]
