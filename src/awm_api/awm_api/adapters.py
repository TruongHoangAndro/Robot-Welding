"""Thin adapters to the engineering CLIs. Each external command is built in exactly one function here, so the
exact command line of a CLI that is not merged yet is wired in one place.

* A11–A14 / A13: ``awm_geometry.cli import`` + ``export-view`` in the OCC conda interpreter (ADR-15).
* A21 / A22 (G1.2, branch ``g1-2-seam-axis``, not merged): ``joint_seam build --job J --picks P [--no-axis]``
  (writes ``seams.json`` D04 v2.1 and ``seam_axis.json``). Prefix from ``Settings.seam_cmd``.
* A41–A49 (G1.3, not written): ``plan --job J --collision off --out O [--seams S01 …]``. **Assumption:** the
  planner writes D07 to ``--out`` (the run folder); awm_api promotes it to ``<job>/path.json`` only when the run
  succeeds. Prefix from ``Settings.plan_cmd``.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from awm_core import occ_bridge
from awm_core.hashing import sha256_file

from awm_api.problems import ProblemError
from awm_api.runner import Step, StepsError
from awm_api.settings import Settings

IMPORT_OUTPUTS = ('model.step', 'part.json', 'cache/edges.json', 'cache/view/view.glb', 'cache/view/topology.json')


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
                 component: Optional[str], up_axis: str) -> List[Step]:
    """STEP → job: A11–A14 import, then the D20 view model export, both into the staging folder."""
    prefix, env = _occ_prefix(settings), _occ_env()
    stage = staging_dir(settings, run_dir)
    comp = ['--component', component] if component else []
    return [
        Step('import', prefix + ['import', '--job', str(stage), '--step', str(step_file), '--part-no', part_no,
                                 '--part-rev', part_rev] + comp, env=env, weight=3.0),
        Step('export-view', prefix + ['export-view', '--step', str(stage / 'model.step'), '--job', str(stage)]
             + comp + ['--up-axis', up_axis, '--out', str(stage / 'cache' / 'view')], env=env, weight=1.0),
    ]


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
    # The G1.2 CLI refuses request fields it does not know; ``range_mm`` (contract SeamPick, not used by S2 in
    # Goal 1) is passed only when set, so the CLI reports it clearly instead of every save failing.
    request = [{k: v for k, v in p.items() if not (k == 'range_mm' and v is None)} for p in picks]
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


def plan_steps(settings: Settings, run_dir: Path, job: Path, seams: Optional[List[str]]) -> List[Step]:
    argv = settings.plan_argv() + ['plan', '--job', str(job), '--collision', 'off', '--out',
                                   str(run_dir / 'path.json')]
    if seams:
        argv += ['--seams', *seams]
    return [Step('plan', argv, env=dict(os.environ))]


def promote_plan(run_dir: Path, job: Path) -> Dict[str, str]:
    out = run_dir / 'path.json'
    if not out.is_file():
        raise RuntimeError('the planner finished without writing path.json')
    tmp = job / '.path.json.tmp'
    shutil.copyfile(out, tmp)
    tmp.replace(job / 'path.json')
    return {'path.json': sha256_file(job / 'path.json')}
