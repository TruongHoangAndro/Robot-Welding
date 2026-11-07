"""A74: release and lock a verified program revision (D10).

scope ``simulation``: needs a PASS verification report (D15); marked as not valid for production.
scope ``production``: additionally needs PASS dry-run (A72) and trial-weld (A73) records of the
same, unchanged program.
Every scope (CP-58, CP-62): no kinematic-only program, no program carrying a CP-62 ``cad_check_override``, and no
release while the job's ``cad_check.json`` (CP-60) is red or stale.

    ros2 run awm_verify release --job <dir> --rev P001 --approved-by "N. Name" --scope simulation
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import stat
import sys
from pathlib import Path
from typing import Dict, List, Optional, Union

from awm_core import jsonio
from awm_core.hashing import sha256_file
from awm_core.repository import JobPaths
from awm_verify import __version__
from awm_verify.records import latest_records

LOG = logging.getLogger('awm_verify.release')
SCOPES = ('simulation', 'production')


class ReleaseError(RuntimeError):
    """Release preconditions are not met (reason in the message)."""


def _lock_tree(root: Path) -> None:
    """Make a released program revision read-only (files 0444, directories 0555)."""
    for dirpath, _dirnames, filenames in os.walk(root, topdown=False):
        for name in filenames:
            os.chmod(os.path.join(dirpath, name), stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH)
        os.chmod(dirpath, stat.S_IRUSR | stat.S_IXUSR | stat.S_IRGRP | stat.S_IXGRP |
                 stat.S_IROTH | stat.S_IXOTH)


#: ADR-17 scopes as amended 2026-11-03 (v2 common.schema scope)
GATE_SCOPES = ('simulation', 'trial', 'production')


#: CP-60 cross-check of the job (``awm_core.cad_roles.CHECK_FILE``), read from the job folder of the program
CAD_CHECK_FILE = 'cad_check.json'
#: D07 / D09 field naming the CP-62 override a kinematic-only run relied on
OVERRIDE_FIELD = 'cad_check_override'
#: CP-60 check 4: rotating set swept over the theta range vs static (the only check CP-62 lets an Admin override)
SWEPT_CHECK_ID = 4


#: CP-62 override record in the job folder
OVERRIDE_FILE = 'cad_check_override.json'
#: D19 placement source written by the CP-60 job import (``awm_geometry.job_placement``)
CP60_SOURCE = 'job_cad'


def _cp60_marker(job_dir: Path, scope: str) -> Optional[str]:
    """Why ``job_dir`` is a CP-60 job (which must have a cross-check), or ``None`` for a manual / copied placement.
    An unreadable ``placement.json`` cannot be classified and is refused."""
    if (job_dir / OVERRIDE_FILE).exists():
        return f'{OVERRIDE_FILE} present'
    placement = job_dir / 'placement.json'
    if not placement.is_file():
        return None
    try:
        source = json.loads(placement.read_text(encoding='utf-8')).get('source')
    except (OSError, ValueError, AttributeError) as exc:
        raise ReleaseError(f'placement.json is unreadable ({exc}); cannot tell whether the job needs a '
                           f'{CAD_CHECK_FILE}; no {scope} release') from exc
    return f'placement source {CP60_SOURCE}' if source == CP60_SOURCE else None


def cad_check_gate(job_dir: Union[str, Path], scope: str) -> None:
    """CP-62 item 3 / CP-60: refuse a release while the job's ``cad_check.json`` is red or stale (any scope).

    * no ``cad_check.json``: refused for a CP-60 job (``placement.json`` source ``job_cad`` or a
      ``cad_check_override.json`` in the job, :func:`_cp60_marker`); old behaviour (nothing to check) only for jobs
      with a manual / copied placement or none;
    * unreadable or schema-invalid → refused;
    * stale → refused. Staleness = **hash check** of the recorded ``inputs`` against the job folder
      (``jsonio.check_inputs``, the reader rule of ``awm_verify``); ``awm_core.staleness`` (G1.17) is not on this
      branch's base — switch to it when it is merged;
    * check 4 red → refused whether or not a CP-62 override exists (the override is for kinematic planning only);
    * any other check red → refused as well (checks 1, 5, 6 are never overridden; CP-60 red blocks Plan).

    A yellow check does not block. Raises ReleaseError."""
    path = Path(job_dir) / CAD_CHECK_FILE
    if not path.is_file():
        reason = _cp60_marker(Path(job_dir), scope)
        if reason:
            raise ReleaseError(f'CP-60 job ({reason}) but no {CAD_CHECK_FILE}: run the cross-check; no {scope} '
                               'release (design rule 4, CP-60)')
        return
    try:
        doc = jsonio.read_json(path, 'v2/cad_check', base_dir=Path(job_dir))
    except jsonio.StaleInputError as exc:
        raise ReleaseError(f'{CAD_CHECK_FILE} is stale (job STEP, roles, axis pick or cell frames changed since the '
                           f'cross-check): {exc}; re-run the cross-check; no {scope} release (CP-60, CP-62)') from exc
    except jsonio.SchemaValidationError as exc:
        raise ReleaseError(f'{CAD_CHECK_FILE} is unreadable: {exc}; no {scope} release (CP-60)') from exc
    red = [c['id'] for c in doc['checks'] if c['status'] == 'red']
    if SWEPT_CHECK_ID in red:
        raise ReleaseError(f'{CAD_CHECK_FILE} check 4 (swept rotating set vs static) is red: every release scope '
                           f'refuses while check 4 is red, with or without a CP-62 override; no {scope} release')
    if red or doc['status'] == 'red':
        raise ReleaseError(f'{CAD_CHECK_FILE} is red (checks {red}); no {scope} release (CP-60)')


def release_gate(program: Dict, scope: str, job_dir: Optional[Union[str, Path]] = None) -> None:
    """Refuse a program that may not be released in ``scope`` — for **every** scope (CP-58, G1.5, CP-62):

    * any program carrying ``cad_check_override`` (kinematic-only planning under a CP-62 override, never releasable);
    * with ``job_dir`` (the job folder of the program): its ``cad_check.json`` red or stale (:func:`cad_check_gate`);
    * v2 (D09 2.x) only: ``collision_checked`` not ``true`` (``false`` = kinematic only, Goal 1; absent = never
      attested); any segment with ``collision_checked: false`` (e.g. a joint-interpolated air move); any weld event
      without a process assignment (``wps_id`` / ``preset`` null: no D06 yet).

    The v2 field checks do not apply to v1 programs (schema 1.x): their collision check is the A71 report.
    Raises ReleaseError."""
    if scope not in GATE_SCOPES:
        raise ReleaseError(f'scope must be one of {GATE_SCOPES}, got {scope!r}')
    rev = program.get('program_rev', '?')
    if OVERRIDE_FIELD in program:  # the key is the marker: a null value is refused too
        ref = program[OVERRIDE_FIELD]
        who = ref.get('by', '?') if isinstance(ref, dict) else '?'
        raise ReleaseError(f'program {rev} carries {OVERRIDE_FIELD} (by {who}): kinematic-only planning under a '
                           f'CP-62 override, never releasable; no {scope} release')
    if job_dir is not None:
        cad_check_gate(job_dir, scope)
    if not str(program.get('schema_version', '')).startswith('2.'):
        return
    if program.get('collision_checked') is not True:
        raise ReleaseError(f'program {rev} has collision_checked: {program.get("collision_checked", "absent")} — '
                           f'kinematic only, collision not checked; no {scope} release (DES-001 rule 4, CP-58)')
    unchecked = [s['segment_id'] for s in program.get('segments', []) if s.get('collision_checked') is False]
    if unchecked:
        raise ReleaseError(f'program {rev}: segments {unchecked} are not collision-checked; no {scope} release')
    missing = [e['event_id'] for e in program.get('events', []) if e.get('wps_id') is None or e.get('preset') is None]
    if missing:
        raise ReleaseError(f'program {rev}: events {missing} have no process assignment (wps_id / preset); '
                           f'no {scope} release')


def is_locked(job: JobPaths, rev: str) -> bool:
    return not os.access(job.program_json(rev), os.W_OK)


def release(job: JobPaths, rev: str, approved_by: str, scope: str, note: str = '') -> Dict:
    if scope not in SCOPES:
        raise ReleaseError(f'scope must be one of {SCOPES}')
    if not approved_by.strip():
        raise ReleaseError('approved_by is required')
    program = job.program_json(rev)
    if not program.is_file():
        raise ReleaseError(f'program {rev} not found')
    if is_locked(job, rev):
        raise ReleaseError(f'program {rev} is already released (read-only)')
    try:
        raw = json.loads(program.read_text(encoding='utf-8'))
    except json.JSONDecodeError as exc:
        raise ReleaseError(f'program {rev} is not valid JSON: {exc}') from exc
    release_gate(raw, scope, job.dir)
    jsonio.read_json(program, 'program', base_dir=job.dir)  # inputs unchanged
    program_sha = sha256_file(program)
    report_path = job.report_json(rev)
    if not report_path.is_file():
        raise ReleaseError(f'no verification report for {rev}: run awm_verify verify first')
    report = jsonio.read_json(report_path, 'verify_report', base_dir=job.dir)
    if report['program_sha256'] != program_sha:
        raise ReleaseError('verification report belongs to another version of the program')
    if report['result'] != 'PASS':
        failed = [k for k, c in report['checks'].items() if not c['ok']]
        raise ReleaseError(f'verification report is FAIL ({", ".join(failed)})')
    records: List[Dict] = []
    if scope == 'production':
        found = latest_records(job, rev)
        for rtype in ('dry_run', 'trial_weld'):
            rec = found.get(rtype)
            if rec is None:
                raise ReleaseError(f'production release needs a {rtype} record (A72 / A73)')
            if rec['doc']['program_sha256'] != program_sha or rec['doc']['result'] != 'PASS':
                raise ReleaseError(f'{rtype} record is not PASS for this program version')
            records.append({'record_type': rtype,
                            'file': rec['path'].relative_to(job.dir).as_posix(),
                            'sha256': sha256_file(rec['path'])})
    inputs = [program, report_path] + [job.dir / r['file'] for r in records]
    doc = jsonio.make_header(f'awm_verify {__version__}', inputs, job.dir)
    doc.update({'program_rev': rev, 'program_sha256': program_sha,
                'report_sha256': sha256_file(report_path), 'scope': scope,
                'records': records, 'approved_by': approved_by,
                'approved_at': jsonio.utc_now_iso(), 'locked': True})
    if scope == 'simulation':
        doc['note'] = ('SIMULATION release: verified on mock hardware only; not valid for '
                       'production (A72 dry run and A73 trial weld missing). ' + note).strip()
    elif note:
        doc['note'] = note
    jsonio.write_json(job.release_json, doc, 'release')
    _lock_tree(job.program_dir(rev))
    LOG.info('A74 %s released (%s) by %s; %s is now read-only', rev, scope, approved_by,
             job.program_dir(rev))
    return doc


def main(argv: Optional[List[str]] = None) -> int:
    logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
    parser = argparse.ArgumentParser(prog='release', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--job', required=True)
    parser.add_argument('--rev', required=True)
    parser.add_argument('--approved-by', required=True)
    parser.add_argument('--scope', choices=SCOPES, required=True)
    parser.add_argument('--note', default='')
    args = parser.parse_args(argv)
    try:
        release(JobPaths(args.job), args.rev, args.approved_by, args.scope, args.note)
    except (ReleaseError, jsonio.SchemaValidationError, jsonio.StaleInputError) as exc:
        LOG.error('%s', exc)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
