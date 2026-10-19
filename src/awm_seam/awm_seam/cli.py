"""``ros2 run awm_seam seam``: define and export weld seams (A21, A22, A24).

Examples::

    seam define --job <job_dir> --seam-id S01 --pick 0,0.005,0.010 --joint-type fillet \\
        --leg-mm 4 --sequence 1 --material S235JR --thickness-mm 10 --process-hint MIG
    seam define --job <job_dir> --seam-id S02 --edges E11,E12 ...
    seam from-picks --job <job_dir> --picks picks.json [--replace]
    seam list --job <job_dir>
    seam remove --job <job_dir> --seam-id S02
    seam rematch --job <new_job_dir> --from-job <old_job_dir>
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from typing import List, Optional

from awm_core.jsonio import SchemaValidationError, StaleInputError
from awm_core.repository import JobPaths
from awm_seam.seams import (EdgeCatalogue, SeamDefinitionError, SeamRequest, define_seams,
                            export_seams, load_seams, rematch, remove_seam)

LOG = logging.getLogger('awm_seam')


def _point(text: str) -> List[float]:
    values = [float(v) for v in text.split(',')]
    if len(values) != 3:
        raise argparse.ArgumentTypeError('a pick is x,y,z in metres (part frame)')
    return values


def main(argv: Optional[List[str]] = None) -> int:
    logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
    parser = argparse.ArgumentParser(prog='seam', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    d = sub.add_parser('define', help='add or replace one seam')
    d.add_argument('--job', required=True)
    d.add_argument('--seam-id', required=True)
    d.add_argument('--edges', default='', help='comma separated edge ids, e.g. E11,E12')
    d.add_argument('--pick', action='append', type=_point, default=[],
                   help='x,y,z (m, part frame) near the seam edge; repeat for several edges')
    d.add_argument('--joint-type', required=True)
    d.add_argument('--leg-mm', type=float, required=True)
    d.add_argument('--sequence', type=int, required=True)
    d.add_argument('--material', required=True)
    d.add_argument('--thickness-mm', type=float, required=True)
    d.add_argument('--weld-side', default='bisector')
    d.add_argument('--process-hint', choices=['MIG', 'LASER'], default=None)
    d.add_argument('--reversed', action='store_true')
    d.add_argument('--note', default='')
    f = sub.add_parser('from-picks', help='define all seams of a pick request file (JSON)')
    f.add_argument('--job', required=True)
    f.add_argument('--picks', required=True)
    f.add_argument('--replace', action='store_true', help='drop seams not in the file')
    ls = sub.add_parser('list')
    ls.add_argument('--job', required=True)
    rm = sub.add_parser('remove')
    rm.add_argument('--job', required=True)
    rm.add_argument('--seam-id', required=True)
    rem = sub.add_parser('rematch', help='carry seams over from the previous part revision')
    rem.add_argument('--job', required=True)
    rem.add_argument('--from-job', required=True)
    args = parser.parse_args(argv)
    job = JobPaths(args.job)
    try:
        if args.cmd == 'define':
            req = SeamRequest(
                seam_id=args.seam_id, joint_type=args.joint_type, leg_mm=args.leg_mm,
                sequence=args.sequence, material=args.material, thickness_mm=args.thickness_mm,
                edges=[e for e in args.edges.split(',') if e], picks_m=args.pick,
                weld_side=args.weld_side, process_hint=args.process_hint,
                reversed=args.reversed, note=args.note)
            doc = define_seams(job, [req])
        elif args.cmd == 'from-picks':
            data = json.loads(open(args.picks, encoding='utf-8').read())
            doc = define_seams(job, [SeamRequest.from_dict(s) for s in data['seams']],
                               replace=args.replace)
        elif args.cmd == 'remove':
            doc = remove_seam(job, args.seam_id)
        elif args.cmd == 'rematch':
            old_job = JobPaths(args.from_job)
            matched, missing = rematch(load_seams(old_job)['seams'], EdgeCatalogue.load(old_job),
                                       EdgeCatalogue.load(job))
            if missing:
                LOG.error('could not re-match %s: pick these seams again', missing)
                if not matched:
                    return 1
            doc = export_seams(job, matched)
        else:
            doc = load_seams(job)
    except (SeamDefinitionError, SchemaValidationError, StaleInputError,
            FileNotFoundError) as exc:
        LOG.error('%s', exc)
        return 1
    for s in doc['seams']:
        LOG.info('%s seq %d %s %s leg %.1f mm, %.1f mm, edges %s%s', s['seam_id'], s['sequence'],
                 s['joint_type'], s['process_hint'] or '-', s['leg_mm'], s['length_m'] * 1000,
                 ','.join(s['edges']), ' (closed)' if s['closed'] else '')
    return 0


if __name__ == '__main__':
    sys.exit(main())
