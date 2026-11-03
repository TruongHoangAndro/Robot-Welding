"""Scene files for the M4 scene backend (WP-S S.4 C1 / C2; DES-001 Rev 0.6 draft §5.2 D19, §5.4; TST-002 Rev 0.3
draft §5.1 F4, F5, §9).

* :func:`load_job_scene` — job-owned scene objects of D19 ``placement.json`` v2 (part with per-triangle face ids,
  fixtures, clamps), schema-validated, input hashes and every mesh hash checked;
* :func:`load_cell_manifest` — identity of the cell scene manifest (``cell/model/scene_manifest.json``, export-scene)
  that the ``cell_scene_loader`` of ``awm_bringup`` puts into the planning scene;
* :func:`scene_hash` — content hash of a scene record (canonical JSON; headers such as ``created_at`` excluded by
  the caller, scene decision D10);
* :func:`clearance_padding_mm` — link padding of the F5 clearance check.

Pure Python, mm (B-06). No rclpy.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional, Tuple, Union

import numpy as np

from awm_core import jsonio
from awm_core.frames import make_transform, quat_to_matrix
from awm_core.hashing import sha256_file, sha256_text
from awm_core.stl import read_stl

PathLike = Union[str, Path]

#: F5 / DES-001 Rev 0.6 draft §5.4: weld and air clearances apply to the head body, mount and conduit. The nozzle is
#: not in the list: it ends at the contact point (its standoff is F9, its padding F4).
DEFAULT_CLEARANCE_LINKS: Tuple[str, ...] = ('head_body', 'head_mount', 'conduit')

#: F5 method record (TST-002 §9: "clearance method (F5)").
CLEARANCE_METHOD = ('validity check (move_group check_state_validity) on unpadded geometry with the clearance links '
                    'inflated by exactly the clearance (planning-scene padding diff); every other link padding 0; '
                    'self-collision unpadded')


class SceneFileError(ValueError):
    """A scene file is malformed, fails its schema, or a referenced file does not match its hash."""


@dataclass(frozen=True)
class JobObject:
    """One job-owned scene object (mm). ``parent_frame``: ``positioner_flange`` or ``world``."""

    object_id: str
    role: str
    parent_frame: str
    parent_T_obj_mm: np.ndarray
    vertices_mm: np.ndarray
    triangles: np.ndarray
    mesh_sha256: str
    face_ids: bool
    mesh_file: Optional[Path] = None


@dataclass(frozen=True)
class JobScene:
    """D19 v2 objects plus the identity of the file."""

    path: Path
    file_sha256: str
    scene_sha256: str
    source: str
    verified: bool
    objects: Tuple[JobObject, ...]


@dataclass(frozen=True)
class CellManifest:
    """Identity of the cell scene manifest (content id ``scene_sha256`` and file hash)."""

    path: Path
    file_sha256: str
    scene_sha256: str
    object_ids: Tuple[str, ...]


def _pose_mm(p: Mapping) -> np.ndarray:
    return make_transform(np.asarray(p['position_mm'], dtype=float), quat_to_matrix(p['quat_xyzw']))


def load_job_scene(placement: PathLike, job_dir: PathLike) -> JobScene:
    """Read D19 ``placement.json`` (schema ``v2/placement``) of a job; check its inputs and every mesh hash."""
    placement, job_dir = Path(placement), Path(job_dir)
    try:
        doc = jsonio.read_json(placement, 'v2/placement', base_dir=job_dir)
    except (jsonio.SchemaValidationError, jsonio.StaleInputError, FileNotFoundError) as exc:
        raise SceneFileError(str(exc)) from exc
    objects = []
    for o in doc['objects']:
        mesh = job_dir / o['mesh']['file']
        if not mesh.is_file():
            raise SceneFileError(f'{placement}: mesh {o["mesh"]["file"]} of {o["object_id"]} not found')
        digest = sha256_file(mesh)
        if digest != o['mesh']['sha256']:
            raise SceneFileError(f'{placement}: {o["mesh"]["file"]} sha256 {digest} != {o["mesh"]["sha256"]}')
        verts, tris = read_stl(mesh)
        objects.append(JobObject(o['object_id'], o['role'], o['parent_frame'], _pose_mm(o['pose_mm']), verts, tris,
                                 digest, bool(o['mesh']['face_ids']), mesh))
    return JobScene(placement, sha256_file(placement), doc['scene_sha256'], doc['source'], bool(doc['verified']),
                    tuple(objects))


#: Face map written next to the part collision mesh by export-scene (S.4 A3).
COLLISION_FACES_FILE = 'collision_faces.json'


def part_object_files(job: JobScene) -> Tuple[JobObject, Path, Path]:
    """The single part object with per-triangle face ids, its (hash-checked) mesh file and the face map next to it."""
    parts = [o for o in job.objects if o.role == 'part' and o.face_ids]
    if len(parts) != 1 or parts[0].mesh_file is None:
        raise SceneFileError(f'{job.path}: expected one part object with face ids, got {len(parts)}')
    return parts[0], parts[0].mesh_file, parts[0].mesh_file.parent / COLLISION_FACES_FILE


def load_cell_manifest(path: PathLike) -> CellManifest:
    """Identity of a cell scene manifest. Content checks (schema, mesh hashes) are done by its loader node."""
    path = Path(path)
    try:
        doc = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise SceneFileError(f'{path}: {exc}') from exc
    if not doc.get('scene_sha256'):
        raise SceneFileError(f'{path}: no scene_sha256 (content id of the cell scene)')
    return CellManifest(path, sha256_file(path), str(doc['scene_sha256']),
                        tuple(str(o.get('object_id')) for o in doc.get('objects', [])))


def world_T_object(obj: JobObject, world_T_flange_mm: np.ndarray) -> np.ndarray:
    """world_T_object (mm) for the positioner flange pose of the current angle."""
    return obj.parent_T_obj_mm if obj.parent_frame == 'world' else world_T_flange_mm @ obj.parent_T_obj_mm


def scene_hash(record: Mapping) -> str:
    """SHA-256 of the canonical JSON (sorted keys, no spaces) of a scene record."""
    return sha256_text(json.dumps(record, sort_keys=True, separators=(',', ':'), allow_nan=False))


def clearance_padding_mm(f4_padding_mm: Mapping[str, float], links: Iterable[str], clearance_mm: float
                         ) -> Dict[str, float]:
    """Padding per link for the F5 clearance check: ``clearance_mm`` on ``links``, 0 on every other link."""
    if clearance_mm < 0:
        raise ValueError('clearance_mm must be >= 0')
    links = tuple(links)
    unknown = sorted(set(links) - set(f4_padding_mm))
    if unknown:
        raise ValueError(f'clearance links without collision geometry: {unknown}')
    return {k: (float(clearance_mm) if k in links else 0.0) for k in f4_padding_mm}


__all__ = ['COLLISION_FACES_FILE', 'part_object_files', 'DEFAULT_CLEARANCE_LINKS', 'CLEARANCE_METHOD', 'SceneFileError', 'JobObject', 'JobScene', 'CellManifest',
           'load_job_scene', 'load_cell_manifest', 'world_T_object', 'scene_hash', 'clearance_padding_mm']
