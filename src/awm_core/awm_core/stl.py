"""Minimal STL reader (binary and ASCII) for planning-scene meshes. No extra dependencies."""

from __future__ import annotations

import struct
from pathlib import Path
from typing import Tuple, Union

import numpy as np

PathLike = Union[str, Path]


def _read_binary(data: bytes) -> np.ndarray:
    (count,) = struct.unpack_from('<I', data, 80)
    expected = 84 + 50 * count
    if len(data) < expected:
        raise ValueError(f'binary STL truncated: {len(data)} bytes, expected {expected}')
    dtype = np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    records = np.frombuffer(data, dtype=dtype, count=count, offset=84)
    return records['v'].astype(float)


def _read_ascii(text: str) -> np.ndarray:
    verts = [[float(x) for x in line.split()[1:4]]
             for line in text.splitlines() if line.strip().startswith('vertex')]
    if len(verts) % 3:
        raise ValueError('ASCII STL vertex count is not a multiple of 3')
    return np.asarray(verts, dtype=float).reshape(-1, 3, 3)


def read_stl(path: PathLike) -> Tuple[np.ndarray, np.ndarray]:
    """Return ``(vertices[N,3], triangles[M,3])`` with shared vertices merged."""
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError(f'STL file too short: {path}')
    is_ascii = data[:5].lower() == b'solid' and b'facet' in data[:1024]
    if is_ascii:
        tris = _read_ascii(data.decode('ascii', errors='replace'))
    else:
        (count,) = struct.unpack_from('<I', data, 80)
        if 84 + 50 * count != len(data) and data[:5].lower() == b'solid':
            tris = _read_ascii(data.decode('ascii', errors='replace'))
        else:
            tris = _read_binary(data)
    if len(tris) == 0:
        raise ValueError(f'STL has no triangles: {path}')
    flat = tris.reshape(-1, 3)
    verts, inverse = np.unique(np.round(flat, 9), axis=0, return_inverse=True)
    return verts, inverse.reshape(-1, 3)
