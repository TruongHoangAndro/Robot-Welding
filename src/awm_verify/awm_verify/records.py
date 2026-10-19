"""A72 dry run / A73 trial weld records (entered by a person; no robot is driven here).

    ros2 run awm_verify record --job <dir> --rev P001 --type dry_run --operator "N. Name" \\
        --result PASS --speed-override-pct 25 --seam S01=PASS --seam S02=PASS --notes "..."
    ros2 run awm_verify record --template dry_run      # print an empty record form
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import logging
import sys
from pathlib import Path
from typing import Dict, List, Optional

from awm_core import jsonio
from awm_core.hashing import sha256_file
from awm_core.repository import JobPaths
from awm_verify import __version__

LOG = logging.getLogger('awm_verify.record')
TYPES = ('dry_run', 'trial_weld')


class RecordError(ValueError):
    """The record does not refer to an existing, unchanged program revision."""


def template(record_type: str) -> Dict:
    """Empty form (A72 / A73) as it is filled in on the shop floor."""
    if record_type not in TYPES:
        raise RecordError(f'record type must be one of {TYPES}')
    return {'record_type': record_type, 'program_rev': 'P###', 'operator': '',
            'date': 'YYYY-MM-DD', 'result': 'PASS | FAIL',
            'speed_override_pct': 25 if record_type == 'dry_run' else 100,
            'seams': [{'seam_id': 'S01', 'result': 'PASS | FAIL', 'note': ''}],
            'notes': ('Reduced speed, arc / laser OFF (REQ-002).' if record_type == 'dry_run'
                      else 'Coupon / first part; visual + leg size check per WPS.')}


def write_record(job: JobPaths, rev: str, record_type: str, operator: str, result: str,
                 seams: List[Dict], speed_override_pct: Optional[float], notes: str,
                 date: Optional[str] = None) -> Path:
    if record_type not in TYPES:
        raise RecordError(f'record type must be one of {TYPES}')
    program = job.program_json(rev)
    if not program.is_file():
        raise RecordError(f'program {rev} not found: {program}')
    program_doc = jsonio.read_json(program, 'program', base_dir=job.dir)
    welded = sorted({s['seam_id'] for s in program_doc['segments'] if s['kind'] == 'weld'})
    unknown = sorted({s['seam_id'] for s in seams} - set(welded))
    if unknown:
        raise RecordError(f'seams {unknown} are not in program {rev} ({welded})')
    if result == 'PASS' and any(s['result'] != 'PASS' for s in seams):
        raise RecordError('overall result PASS with a failed seam')
    doc = jsonio.make_header(f'awm_verify {__version__} (record entry)', [program], job.dir)
    doc.update({'record_type': record_type, 'program_rev': rev,
                'program_sha256': sha256_file(program), 'operator': operator,
                'date': date or dt.date.today().isoformat(), 'result': result,
                'speed_override_pct': speed_override_pct, 'seams': seams, 'notes': notes})
    stamp = dt.datetime.now().strftime('%Y%m%dT%H%M%S')
    out = job.records_dir(rev) / f'{record_type}_{stamp}.json'
    jsonio.write_json(out, doc, 'record')
    return out


def latest_records(job: JobPaths, rev: str) -> Dict[str, Dict]:
    """Newest record of each type for a revision: {type: {'path', 'doc'}}."""
    out: Dict[str, Dict] = {}
    if not job.records_dir(rev).is_dir():
        return out
    for path in sorted(job.records_dir(rev).glob('*.json')):
        doc = jsonio.read_json(path, 'record')
        out[doc['record_type']] = {'path': path, 'doc': doc}
    return out


def _seam_arg(text: str) -> Dict:
    sid, _, res = text.partition('=')
    if res not in ('PASS', 'FAIL'):
        raise argparse.ArgumentTypeError('use S01=PASS or S01=FAIL')
    return {'seam_id': sid, 'result': res, 'note': ''}


def main(argv: Optional[List[str]] = None) -> int:
    logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
    parser = argparse.ArgumentParser(prog='record', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--template', choices=TYPES, help='print an empty form and exit')
    parser.add_argument('--job')
    parser.add_argument('--rev')
    parser.add_argument('--type', choices=TYPES)
    parser.add_argument('--operator')
    parser.add_argument('--result', choices=['PASS', 'FAIL'])
    parser.add_argument('--speed-override-pct', type=float, default=None)
    parser.add_argument('--seam', action='append', type=_seam_arg, default=[])
    parser.add_argument('--notes', default='')
    parser.add_argument('--date', default=None)
    args = parser.parse_args(argv)
    if args.template:
        sys.stdout.write(json.dumps(template(args.template), indent=2) + '\n')
        return 0
    missing = [n for n in ('job', 'rev', 'type', 'operator', 'result') if not getattr(args, n)]
    if missing:
        parser.error(f'missing: {", ".join("--" + m for m in missing)}')
    try:
        out = write_record(JobPaths(args.job), args.rev, args.type, args.operator, args.result,
                           args.seam, args.speed_override_pct, args.notes, args.date)
    except (RecordError, jsonio.SchemaValidationError, jsonio.StaleInputError) as exc:
        LOG.error('%s', exc)
        return 1
    LOG.info('record written: %s', out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
