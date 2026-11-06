"""Test helper: a small synthetic H-CAD head folder with the keys of the A18 / export-head ``head.yaml`` (mm, tool0).

Test data only (no part or cell data): two boxes as STL meshes and hulls, and a beam axis that is deliberately tilted
and offset so the tests do not depend on the real CAD values. Two variants: the automatic beam estimate
(``write_synthetic_head_dir``) and a map TCP override (``write_map_override_head_dir``: TCP given, beam exit derived). ``real_head_dir()`` returns the folder named by the
environment variable ``AWM_HEAD_DIR`` (the real export, e.g. ``<repo_root>/cell/model/head``) or None.
"""

from __future__ import annotations

import hashlib
import math
import os
import struct
from pathlib import Path
from typing import List, Optional, Sequence

import yaml

HEAD_ENV = 'AWM_HEAD_DIR'


def real_head_dir() -> Optional[Path]:
    """Folder of the real head export from AWM_HEAD_DIR, or None when the variable is not set."""
    value = os.environ.get(HEAD_ENV, '').strip()
    return Path(value) if value else None


def _box_triangles(lo: Sequence[float], hi: Sequence[float]) -> List[List[Sequence[float]]]:
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return [[v[a], v[b], v[c]] for a, b, c in faces]


def write_stl(path: Path, triangles) -> str:
    """Binary STL (normals 0); returns the SHA-256 of the file."""
    data = bytearray(b'awm synthetic test head'.ljust(80, b' '))
    data += struct.pack('<I', len(triangles))
    for tri in triangles:
        data += struct.pack('<3f', 0.0, 0.0, 0.0)
        for p in tri:
            data += struct.pack('<3f', *p)
        data += b'\0\0'
    path.write_bytes(bytes(data))
    return hashlib.sha256(bytes(data)).hexdigest()


def write_synthetic_head_dir(folder: Path, tilt_deg: float = 20.0, nozzle_len_mm: float = 40.0) -> Path:
    """Write head.yaml + 4 STL files into ``folder``; beam = tool0 -Z tilted by tilt_deg about tool0 X."""
    t = math.radians(tilt_deg)
    beam = [0.0, math.sin(t), -math.cos(t)]
    exit_mm = [-50.0, 12.5, -300.0]
    tcp = [e + nozzle_len_mm * b for e, b in zip(exit_mm, beam)]
    # tcp rotation = Rx(180 deg + tilt): its Z column is the beam direction.
    a = (math.pi + t) / 2.0
    quat = [math.sin(a), 0.0, 0.0, math.cos(a)]
    return _write_head_dir(folder, tcp, quat, beam, exit_mm, nozzle_len_mm, {'method': 'beam_axis'})


def write_map_override_head_dir(folder: Path, tilt_deg: float = 55.0, nozzle_len_mm: float = 40.0) -> Path:
    """head.yaml as export-head writes it for a map TCP override (frames.tcp point_file_mm / axis_file).

    The TCP is in front of the flange and the beam leaves the nozzle forward and sideways (tool0 +X / +Z, tilted
    tilt_deg from the flange axis about tool0 Y); beam_exit = tcp - nozzle_len_mm * beam (export-head rule)."""
    t = math.radians(tilt_deg)
    beam = [math.sin(t), 0.0, math.cos(t)]
    tcp = [35.0, -4.0, 260.0]
    exit_mm = [p - nozzle_len_mm * b for p, b in zip(tcp, beam)]
    quat = [0.0, math.sin(t / 2.0), 0.0, math.cos(t / 2.0)]  # Ry(tilt): Z column = beam
    return _write_head_dir(folder, tcp, quat, beam, exit_mm, nozzle_len_mm, {'method': 'map', 'behind_flange': False})


def _write_head_dir(folder: Path, tcp, quat, beam, exit_mm, nozzle_len_mm, tcp_estimate) -> Path:
    folder.mkdir(parents=True, exist_ok=True)
    links = {}
    for name, lo, hi in (('head_body', (-120.0, -20.0, -300.0), (40.0, 20.0, 200.0)),
                         ('head_mount', (-40.0, -40.0, 0.0), (40.0, 40.0, 60.0))):
        sha = write_stl(folder / f'{name}.stl', _box_triangles(lo, hi))
        hull_sha = write_stl(folder / f'{name}_hull.stl', _box_triangles(lo, hi))
        links[name] = {'components': ['synthetic'],
                       'mesh': {'file': f'{name}.stl', 'sha256': sha, 'triangles': 12},
                       'hull': {'file': f'{name}_hull.stl', 'sha256': hull_sha, 'triangles': 12}}
    doc = {'schema_version': '2.0', 'created_by': 'test', 'created_at': '2026-11-05T00:00:00Z', 'inputs': [],
           'schema': 'awm_head/0.1', 'head_model': 'H-CAD', 'unit': 'mm', 'frame': 'tool0',
           'tool0_T_tcp_mm': {'xyz': tcp, 'quat_xyzw': quat},
           'beam_axis_tool0': beam, 'beam_exit_tool0_mm': exit_mm, 'nozzle_len_mm': nozzle_len_mm,
           'links': links, 'estimates': {'tcp': tcp_estimate}, 'assumptions': ['synthetic test head']}
    (folder / 'head.yaml').write_text(yaml.safe_dump(doc, sort_keys=False))
    return folder
