"""``ros2 run awm_process assign``: process + WPS per seam → process.json (A31-A33).

Example::

    ros2 run awm_process assign --job <job_dir> [--default-process MIG] [--allow-sample-wps]

Exit code 2 when some seams have no WPS (process.json is still written and lists them).
"""

from __future__ import annotations

import argparse
import logging
import sys
from typing import List, Optional

from awm_core.jsonio import SchemaValidationError, StaleInputError
from awm_core.repository import JobPaths
from awm_process import ProcessError, assign

LOG = logging.getLogger('awm_process')


def main(argv: Optional[List[str]] = None) -> int:
    logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
    parser = argparse.ArgumentParser(prog='assign', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--job', required=True)
    parser.add_argument('--default-process', choices=['MIG', 'LASER'], default='MIG')
    parser.add_argument('--allow-sample-wps', action='store_true',
                        help='accept WPS files with status SAMPLE (simulation / development only)')
    args = parser.parse_args(argv)
    try:
        doc = assign(JobPaths(args.job), args.default_process, args.allow_sample_wps)
    except (ProcessError, SchemaValidationError, StaleInputError, FileNotFoundError) as exc:
        LOG.error('%s', exc)
        return 1
    for row in doc['seams']:
        LOG.info('%s: %s %s, %.1f mm/s', row['seam_id'], row['process'], row['wps_id'],
                 row['motion']['travel_speed_mm_s'])
    for row in doc['unassigned']:
        LOG.error('%s: %s', row['seam_id'], row['reason'])
    return 2 if doc['unassigned'] else 0


if __name__ == '__main__':
    sys.exit(main())
