"""Login stub with roles Operator / Engineer / Admin (DES-001 §7.3, A88 subset).

Credentials live in a JSON users file outside git (``Settings.secrets_file``, created with
``python3 -m awm_api.users add``); passwords are stored as PBKDF2-SHA256 hashes, never in plain text. Tokens are
random bearer tokens kept in memory (a restart logs everybody out). Assumption: this stub is enough for Sim-1 on a
single engineering PC; real user management (S9) comes with A88.
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import hmac
import json
import secrets
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

from awm_api.problems import ProblemError

ROLES = ('Operator', 'Engineer', 'Admin')
DEFAULT_ITERATIONS = 200_000


class UsersFileError(RuntimeError):
    """The users file is missing or malformed."""


def hash_password(password: str, iterations: int = DEFAULT_ITERATIONS, salt: Optional[bytes] = None) -> str:
    salt = salt or secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt, iterations)
    return f'pbkdf2_sha256${iterations}${salt.hex()}${digest.hex()}'


def verify_password(password: str, stored: str) -> bool:
    try:
        algo, iters, salt_hex, digest_hex = stored.split('$')
        if algo != 'pbkdf2_sha256':
            return False
        digest = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), bytes.fromhex(salt_hex), int(iters))
    except (ValueError, TypeError):
        return False
    return hmac.compare_digest(digest.hex(), digest_hex)


@dataclass(frozen=True)
class User:
    username: str
    role: str
    password: str


def load_users(path: Path) -> Dict[str, User]:
    path = Path(path)
    if not path.is_file():
        raise UsersFileError(
            f'users file {path} not found; create it with '
            f'"python3 -m awm_api.users add --file {path} --user <name> --role Engineer" '
            '(keep it out of git)')
    try:
        data = json.loads(path.read_text())
        entries = data['users']
    except (ValueError, KeyError, TypeError) as exc:
        raise UsersFileError(f'users file {path} is not {{"users": [...]}} JSON: {exc}') from exc
    users: Dict[str, User] = {}
    for e in entries:
        name, role, pw = e.get('username'), e.get('role'), e.get('password', '')
        if not name or role not in ROLES:
            raise UsersFileError(f'users file {path}: bad user entry {name!r} (role must be one of {ROLES})')
        if not str(pw).startswith('pbkdf2_sha256$'):
            raise UsersFileError(f'users file {path}: password of {name!r} is not a pbkdf2_sha256 hash')
        users[name] = User(name, role, pw)
    return users


class TokenStore:
    """In-memory bearer tokens with expiry."""

    def __init__(self, ttl_s: float) -> None:
        self.ttl_s = ttl_s
        self._tokens: Dict[str, Tuple[User, float]] = {}
        self._lock = threading.Lock()

    def issue(self, user: User) -> Tuple[str, str]:
        token = secrets.token_urlsafe(32)
        expires = time.time() + self.ttl_s
        with self._lock:
            self._tokens[token] = (user, expires)
        iso = _dt.datetime.fromtimestamp(expires, _dt.timezone.utc).isoformat().replace('+00:00', 'Z')
        return token, iso

    def user(self, token: Optional[str]) -> Optional[User]:
        if not token:
            return None
        with self._lock:
            entry = self._tokens.get(token)
            if entry is None:
                return None
            if entry[1] <= time.time():
                del self._tokens[token]
                return None
            return entry[0]


def unauthorized(detail: str) -> ProblemError:
    return ProblemError(401, 'unauthorized', detail=detail, headers={'WWW-Authenticate': 'Bearer'})


def forbidden(user: User, roles) -> ProblemError:
    return ProblemError(403, 'forbidden', detail=f'role {user.role} may not do this (allowed: {", ".join(roles)})')
