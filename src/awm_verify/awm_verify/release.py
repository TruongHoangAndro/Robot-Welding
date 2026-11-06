"""A74: release and lock a verified program revision (D10).

scope ``simulation``: needs a PASS verification report (D15); marked as not valid for production.
scope ``production``: additionally needs PASS dry-run (A72) and trial-weld (A73) records of the
same, unchanged program.

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
from typing import Dict, List, Optional

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


def release_gate(program: Dict, scope: str) -> None:
    """Refuse a v2 program (D09 2.x) that may not be released in ``scope`` — for **every** scope (CP-58, G1.5):

    * ``collision_checked`` not ``true`` (``false`` = kinematic only, Goal 1; absent = never attested);
    * any segment with ``collision_checked: false`` (e.g. a joint-interpolated air move);
    * any weld event without a process assignment (``wps_id`` / ``preset`` null: no D06 yet).

    v1 programs (schema 1.x) are not affected: their collision check is the A71 report. Raises ReleaseError."""
    if scope not in GATE_SCOPES:
        raise ReleaseError(f'scope must be one of {GATE_SCOPES}, got {scope!r}')
    if not str(program.get('schema_version', '')).startswith('2.'):
        return
    rev = program.get('program_rev', '?')
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
    release_gate(raw, scope)
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
