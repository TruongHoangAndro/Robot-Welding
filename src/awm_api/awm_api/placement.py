"""Placement for a job made in the browser (G1.6, Goal 1 acceptance gap): D19 ``v2/placement`` from a manual pose.

A job created from a STEP upload has no ``placement.json`` (the golden part has one from the cell CAD,
``source: fixture_cad``). Until the placement study (A17) exists, the Engineer enters the pose of the part in
``positioner_flange`` (mm, degrees) or copies the placement of another job. Both give ``source: study``,
``verified: false`` and an **Assumption** mark in ``created_by`` (the schema has no note field). The only scene
object is the part, with the D20 display model as its mesh (mm, faces as nodes; not a collision mesh — collision is
Goal 2, CP-58). Changing the placement makes ``seam_axis.json`` and ``path.json`` stale by their input hashes.

Written by the backend because no CLI owns a manual placement yet (A15 belongs to M1); the document is validated
with the frozen ``v2/placement`` schema before it is written.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

import numpy as np

from awm_core import frames, jsonio, units
from awm_core.hashing import sha256_bytes, sha256_file

PLACEMENT_SCHEMA_VERSION = '2.0'
VIEW_MESH = 'cache/view/view.glb'
CREATED_BY = 'awm_api placement PUT'
#: Largest accepted distance of the part origin from the flange (mm). A plausibility bound, not a cell limit.
MAX_OFFSET_MM = 5000.0


def quat_from_input(rpy_deg: Optional[Sequence[float]], quat_xyzw: Optional[Sequence[float]]) -> list:
    """Unit quaternion (xyzw, w ≥ 0) from roll-pitch-yaw in degrees (URDF convention) or a quaternion."""
    if rpy_deg is not None:
        r, p, y = (float(units.deg_to_rad(v)) for v in rpy_deg)
        q = frames.matrix_to_quat(frames.rpy_to_matrix(r, p, y))
    else:
        q = np.asarray(quat_xyzw, dtype=float)
        n = float(np.linalg.norm(q))
        if not n > 1e-9:
            raise ValueError('quaternion has zero length')
        q = q / n
        if q[3] < 0:
            q = -q
    return [float(v) + 0.0 for v in q]


def scene_sha256(objects: Sequence[Dict[str, Any]]) -> str:
    """Same rule as ``awm_geometry.scene.canonical_sha256``: canonical JSON of the objects (mesh hashes in)."""
    return sha256_bytes(json.dumps(list(objects), sort_keys=True, separators=(',', ':')).encode())


def build(job: Path, flange_t_part: Dict[str, list], how: str, extra_inputs: Sequence[Path] = ()) -> Dict[str, Any]:
    """Placement document for ``job`` with ``flange_T_part`` (``position_mm`` + ``quat_xyzw``)."""
    view = job / VIEW_MESH
    pose = {'position_mm': [float(v) for v in flange_t_part['position_mm']],
            'quat_xyzw': [float(v) for v in flange_t_part['quat_xyzw']]}
    objects = [{'object_id': 'part', 'role': 'part', 'parent_frame': 'positioner_flange', 'pose_mm': pose,
                'mesh': {'file': VIEW_MESH, 'sha256': sha256_file(view), 'face_ids': False}, 'padding_mm': 0.0}]
    doc = jsonio.make_header(f'{CREATED_BY} ({how}; Assumption: pose entered by the Engineer, not measured, '
                             'simulation only)', [view, *extra_inputs], job)
    doc['schema_version'] = PLACEMENT_SCHEMA_VERSION
    doc.update({'source': 'study', 'verified': False, 'flange_T_part': pose, 'objects': objects,
                'scene_sha256': scene_sha256(objects)})
    jsonio.validate(doc, 'v2/placement')
    return doc


def write(job: Path, doc: Dict[str, Any]) -> None:
    jsonio.write_json(job / 'placement.json', doc, 'v2/placement')
