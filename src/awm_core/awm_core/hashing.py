"""SHA-256 helpers used for input tracking (DES-001 §5.2)."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Union

PathLike = Union[str, Path]

_CHUNK = 1 << 20


def sha256_file(path: PathLike) -> str:
    """Return the hex SHA-256 of a file's content."""
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(_CHUNK), b''):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_bytes(data: bytes) -> str:
    """Return the hex SHA-256 of a byte string."""
    return hashlib.sha256(data).hexdigest()


def sha256_text(text: str) -> str:
    """Return the hex SHA-256 of a UTF-8 string."""
    return sha256_bytes(text.encode('utf-8'))
