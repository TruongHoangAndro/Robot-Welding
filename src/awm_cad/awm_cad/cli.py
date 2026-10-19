"""``ros2 run awm_cad import_step``: STEP → job package (A11-A14).

Example::

    ros2 run awm_cad import_step --repo <repo_root> --step part.step --part-no 1234 --part-rev B
"""

from __future__ import annotations

import argparse
import logging
import sys
from typing import List, Optional

from awm_cad import import_step
from awm_core.occ_bridge import GeometryCliError

LOG = logging.getLogger('awm_cad')


def main(argv: Optional[List[str]] = None) -> int:
    logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
    parser = argparse.ArgumentParser(prog='import_step', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--repo', required=True, help='job repository root')
    parser.add_argument('--step', required=True)
    parser.add_argument('--part-no', required=True)
    parser.add_argument('--part-rev', required=True)
    parser.add_argument('--occ-python-cmd', default='',
                        help='OCC interpreter command (default: env AWM_OCC_PYTHON)')
    parser.add_argument('--linear-deflection-m', type=float, default=1.0e-4)
    args = parser.parse_args(argv)
    try:
        result = import_step(args.repo, args.step, args.part_no, args.part_rev,
                             occ_python_cmd=args.occ_python_cmd,
                             linear_deflection_m=args.linear_deflection_m)
    except (GeometryCliError, ValueError, RuntimeError) as exc:
        LOG.error('%s', exc)
        return 1
    LOG.info('part record: %s (%s)', result.job.part_json, result.summary)
    return 0


if __name__ == '__main__':
    sys.exit(main())
