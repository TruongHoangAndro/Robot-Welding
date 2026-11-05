"""Start the backend: ``python3 -m awm_api --repo <dir> [--web-dist <dir>] [--host 127.0.0.1] [--port 8000]``."""

from __future__ import annotations

import argparse
import logging
import os
import sys
from typing import List, Optional

from awm_api.auth import UsersFileError
from awm_api.settings import DEFAULT_HOST, DEFAULT_PORT, Settings


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog='awm_api', description=__doc__)
    ap.add_argument('--repo', help='job repository root (else AWM_REPO_ROOT)')
    ap.add_argument('--secrets', help='users file (else AWM_API_SECRETS, default <repo>/secrets/awm_api_users.json)')
    ap.add_argument('--web-dist', help='built awm_web dist/ folder to serve at / (else AWM_WEB_DIST)')
    ap.add_argument('--host', default=os.environ.get('AWM_API_HOST', DEFAULT_HOST),
                    help='bind address (default 127.0.0.1: this PC only)')
    ap.add_argument('--port', type=int, default=int(os.environ.get('AWM_API_PORT', DEFAULT_PORT)))
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, stream=sys.stderr, format='[awm_api] %(levelname)s: %(message)s')
    try:
        settings = Settings.from_env(args.repo, args.secrets, args.web_dist)
        from awm_api.app import create_app
        app = create_app(settings)
    except (ValueError, UsersFileError, FileNotFoundError) as exc:
        logging.getLogger('awm_api').error('%s', exc)
        return 2
    import uvicorn
    logging.getLogger('awm_api').info('repository %s; serving http://%s:%d/', settings.repo_root, args.host,
                                      args.port)
    uvicorn.run(app, host=args.host, port=args.port, log_level='info')
    return 0


if __name__ == '__main__':
    sys.exit(main())
