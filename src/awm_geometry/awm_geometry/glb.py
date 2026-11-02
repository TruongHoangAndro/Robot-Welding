"""Minimal glTF 2.0 binary (GLB) writer and reader for the D20 view model (numpy + struct only).

Contract: ``docs/generated/contracts/D20_view_model.md``. No OCC here, so the module is importable from
any interpreter. Positions are float32 and written unchanged in the caller's unit (D20: mm, stated in
``asset.extras.unit``); no extensions, no compression.

Layout written by :class:`GlbBuilder`: one buffer, one bufferView per accessor (4-byte aligned),
POSITION accessors with ``min`` / ``max``, uint32 indices.
"""

from __future__ import annotations

import json
import struct
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

GLB_MAGIC = 0x46546C67  # 'glTF'
GLB_VERSION = 2
CHUNK_JSON = 0x4E4F534A  # 'JSON'
CHUNK_BIN = 0x004E4942  # 'BIN\0'

MODE_LINE_STRIP = 3
MODE_TRIANGLES = 4
FLOAT = 5126
UNSIGNED_INT = 5125
ARRAY_BUFFER = 34962
ELEMENT_ARRAY_BUFFER = 34963


class GlbError(ValueError):
    """Malformed GLB data."""


def _pad4(data: bytes, fill: bytes) -> bytes:
    return data + fill * ((4 - len(data) % 4) % 4)


class GlbBuilder:
    """Collects nodes, meshes and binary data, then serialises a GLB file."""

    def __init__(self, asset_extras: Dict[str, Any], generator: str) -> None:
        self.gltf: Dict[str, Any] = {
            'asset': {'version': '2.0', 'generator': generator, 'extras': dict(asset_extras)},
            'scene': 0, 'scenes': [{'nodes': []}], 'nodes': [], 'meshes': [],
            'accessors': [], 'bufferViews': [], 'buffers': [],
        }
        self._bin = bytearray()

    def _view(self, data: bytes, target: int) -> int:
        offset = len(self._bin)
        self._bin += _pad4(data, b'\x00')
        self.gltf['bufferViews'].append({'buffer': 0, 'byteOffset': offset,
                                         'byteLength': len(data), 'target': target})
        return len(self.gltf['bufferViews']) - 1

    def _accessor(self, view: int, component: int, count: int, kind: str,
                  bounds: Optional[Tuple[List[float], List[float]]] = None) -> int:
        acc: Dict[str, Any] = {'bufferView': view, 'byteOffset': 0, 'componentType': component,
                               'count': count, 'type': kind}
        if bounds is not None:
            acc['min'], acc['max'] = bounds
        self.gltf['accessors'].append(acc)
        return len(self.gltf['accessors']) - 1

    def _vec3(self, values: np.ndarray, with_bounds: bool) -> int:
        arr = np.ascontiguousarray(values, dtype='<f4').reshape(-1, 3)
        if len(arr) == 0:
            raise GlbError('empty vertex array')
        bounds = None
        if with_bounds:
            bounds = ([float(v) for v in arr.min(axis=0)], [float(v) for v in arr.max(axis=0)])
        view = self._view(arr.tobytes(), ARRAY_BUFFER)
        return self._accessor(view, FLOAT, len(arr), 'VEC3', bounds)

    def add_node(self, name: str, parent: Optional[int] = None, mesh: Optional[int] = None,
                 extras: Optional[Dict[str, Any]] = None) -> int:
        """Add a node; ``parent`` None makes it a scene root. Return its index."""
        node: Dict[str, Any] = {'name': name}
        if mesh is not None:
            node['mesh'] = mesh
        if extras is not None:
            node['extras'] = extras
        self.gltf['nodes'].append(node)
        index = len(self.gltf['nodes']) - 1
        if parent is None:
            self.gltf['scenes'][0]['nodes'].append(index)
        else:
            self.gltf['nodes'][parent].setdefault('children', []).append(index)
        return index

    def add_triangles(self, name: str, positions: np.ndarray, normals: np.ndarray,
                      indices: np.ndarray) -> int:
        """Add a TRIANGLES mesh (POSITION, NORMAL, uint32 indices). Return the mesh index."""
        pos = self._vec3(positions, True)
        nor = self._vec3(normals, False)
        idx = np.ascontiguousarray(indices, dtype='<u4').reshape(-1)
        if len(idx) == 0 or len(idx) % 3:
            raise GlbError(f'mesh {name}: index count {len(idx)} is not a positive multiple of 3')
        view = self._view(idx.tobytes(), ELEMENT_ARRAY_BUFFER)
        ind = self._accessor(view, UNSIGNED_INT, len(idx), 'SCALAR',
                             ([float(idx.min())], [float(idx.max())]))
        self.gltf['meshes'].append({'name': name, 'primitives': [
            {'attributes': {'POSITION': pos, 'NORMAL': nor}, 'indices': ind,
             'mode': MODE_TRIANGLES}]})
        return len(self.gltf['meshes']) - 1

    def add_line_strip(self, name: str, positions: np.ndarray) -> int:
        """Add a LINE_STRIP mesh (POSITION only). Return the mesh index."""
        pos = self._vec3(positions, True)
        self.gltf['meshes'].append({'name': name, 'primitives': [
            {'attributes': {'POSITION': pos}, 'mode': MODE_LINE_STRIP}]})
        return len(self.gltf['meshes']) - 1

    def to_bytes(self) -> bytes:
        """Serialise as GLB: 12-byte header, JSON chunk (space padded), BIN chunk (zero padded)."""
        gltf = dict(self.gltf)
        gltf['buffers'] = [{'byteLength': len(self._bin)}]
        text = json.dumps(gltf, separators=(',', ':'), allow_nan=False).encode('utf-8')
        json_chunk = _pad4(text, b' ')
        bin_chunk = bytes(self._bin)
        total = 12 + 8 + len(json_chunk) + 8 + len(bin_chunk)
        return b''.join([
            struct.pack('<III', GLB_MAGIC, GLB_VERSION, total),
            struct.pack('<II', len(json_chunk), CHUNK_JSON), json_chunk,
            struct.pack('<II', len(bin_chunk), CHUNK_BIN), bin_chunk,
        ])


def read_glb(data: bytes) -> Tuple[Dict[str, Any], bytes]:
    """Parse a GLB file; return ``(gltf json, bin chunk)``. Checks header, lengths and alignment."""
    if len(data) < 20:
        raise GlbError('file too short')
    magic, version, length = struct.unpack_from('<III', data, 0)
    if magic != GLB_MAGIC or version != GLB_VERSION:
        raise GlbError(f'bad header magic {magic:#x} / version {version}')
    if length != len(data) or length % 4:
        raise GlbError(f'header length {length} vs file {len(data)}')
    json_len, json_type = struct.unpack_from('<II', data, 12)
    if json_type != CHUNK_JSON or json_len % 4:
        raise GlbError('first chunk is not an aligned JSON chunk')
    gltf = json.loads(data[20:20 + json_len].decode('utf-8'))
    offset = 20 + json_len
    binary = b''
    if offset < length:
        bin_len, bin_type = struct.unpack_from('<II', data, offset)
        if bin_type != CHUNK_BIN or bin_len % 4 or offset + 8 + bin_len != length:
            raise GlbError('second chunk is not an aligned BIN chunk ending the file')
        binary = data[offset + 8:offset + 8 + bin_len]
    return gltf, binary


_NUMPY = {FLOAT: '<f4', UNSIGNED_INT: '<u4'}
_WIDTH = {'SCALAR': 1, 'VEC3': 3}


def accessor_array(gltf: Dict[str, Any], binary: bytes, index: int) -> np.ndarray:
    """Data of accessor ``index`` (tightly packed views only), shape (count,) or (count, 3)."""
    acc = gltf['accessors'][index]
    view = gltf['bufferViews'][acc['bufferView']]
    width = _WIDTH[acc['type']]
    dtype = np.dtype(_NUMPY[acc['componentType']])
    start = view.get('byteOffset', 0) + acc.get('byteOffset', 0)
    nbytes = acc['count'] * width * dtype.itemsize
    if start % 4 or start + nbytes > view.get('byteOffset', 0) + view['byteLength']:
        raise GlbError(f'accessor {index} outside its bufferView or misaligned')
    arr = np.frombuffer(binary, dtype=dtype, count=acc['count'] * width, offset=start)
    return arr.reshape(-1, 3) if width == 3 else arr
