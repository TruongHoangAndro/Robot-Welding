"""``awm_repo``: create a job repository from a template.

Usage: ``awm_repo init --repo <root> --template <dir>``
"""

from __future__ import annotations

import argparse
import logging
import sys
from typing import List, Optional

from awm_core.repository import Repository

LOG = logging.getLogger('awm_repo')


def main(argv: Optional[List[str]] = None) -> int:
    logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
    parser = argparse.ArgumentParser(prog='awm_repo', description=__doc__)
    sub = parser.add_subparsers(dest='cmd', required=True)
    p_init = sub.add_parser('init', help='create cell/, jobs/, logs/ from a template')
    p_init.add_argument('--repo', required=True)
    p_init.add_argument('--template', required=True, help='directory that contains cell/')
    p_init.add_argument('--overwrite', action='store_true')
    args = parser.parse_args(argv)
    try:
        Repository(args.repo).init_from_template(args.template, overwrite=args.overwrite)
    except (FileExistsError, FileNotFoundError) as exc:
        LOG.error('%s', exc)
        return 1
    LOG.info('repository ready: %s', args.repo)
    return 0


if __name__ == '__main__':
    sys.exit(main())
