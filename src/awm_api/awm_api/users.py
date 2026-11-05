"""Manage the awm_api users file: ``python3 -m awm_api.users add|list --file <users.json>``.

The password is read from the terminal (or one line on stdin with ``--password-stdin``) and stored as a PBKDF2 hash.
"""

from __future__ import annotations

import argparse
import getpass
import json
import os
import sys
from pathlib import Path
from typing import List, Optional

from awm_api.auth import ROLES, hash_password


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog='awm_api.users', description=__doc__)
    sub = ap.add_subparsers(dest='cmd', required=True)
    a = sub.add_parser('add', help='add or replace a user')
    a.add_argument('--file', required=True)
    a.add_argument('--user', required=True)
    a.add_argument('--role', required=True, choices=ROLES)
    a.add_argument('--password-stdin', action='store_true')
    a = sub.add_parser('list')
    a.add_argument('--file', required=True)
    args = ap.parse_args(argv)
    path = Path(args.file)
    data = json.loads(path.read_text()) if path.is_file() else {'users': []}
    if args.cmd == 'list':
        for u in data['users']:
            sys.stdout.write(f"{u['username']}\t{u['role']}\n")
        return 0
    if args.password_stdin:
        pw = sys.stdin.readline().rstrip('\n')
    else:
        pw = getpass.getpass(f'password for {args.user}: ')
        if pw != getpass.getpass('repeat: '):
            sys.stderr.write('passwords differ\n')
            return 1
    if len(pw) < 4:
        sys.stderr.write('password too short (min 4 characters)\n')
        return 1
    data['users'] = [u for u in data['users'] if u['username'] != args.user]
    data['users'].append({'username': args.user, 'role': args.role, 'password': hash_password(pw)})
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, indent=2) + '\n')
    os.chmod(tmp, 0o600)
    tmp.replace(path)
    sys.stdout.write(f'{args.user} ({args.role}) written to {path}\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
