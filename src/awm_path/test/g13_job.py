"""Synthetic job for the G1.3 CLI tests (test data, not part data in source).

A ring of radius ``RING_R_MM`` about the part Z axis, placed coaxial with the positioner axis (part +Z = flange +X).
seams.json and seam_axis.json are written by the real awm_seam code (``joint_seams.build_seams`` with an exact ring
sampler instead of the OCC process, ``seam_axis.build_seam_axis``).
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List

from awm_core import jsonio
from awm_core.repository import Repository

RING_R_MM = 13.335
#: Test cell: base_link = world; positioner axis +X through (0, -450, 150) mm (reachable with the H-BOX head).
FRAMES_YAML = (
    '# test frames\nschema_version: "2.0"\ncreated_by: "test"\ncreated_at: "2026-11-06T00:00:00Z"\ninputs: []\n'
    'schema: "awm_cell_frames/0.1"\nunit: "mm"\n'
    'world_T_base:\n  position_mm: [0.0, 0.0, 0.0]\n  quat_xyzw: [0.0, 0.0, 0.0, 1.0]\n'
    'world_T_positioner_axis:\n  position_mm: [0.0, -450.0, 150.0]\n  quat_xyzw: [0.0, 0.0, 0.0, 1.0]\n'
    '  point_mm: [0.0, -450.0, 150.0]\n  direction: [1.0, 0.0, 0.0]\n')


def _topology(job_dir: Path) -> Dict:
    doc = jsonio.make_header('test', [job_dir / 'model.step'], job_dir)
    doc['schema_version'] = '2.0'
    length = 2.0 * math.pi * RING_R_MM
    edge = {'curve_type': 'circle', 'length_mm': length, 'closed': True, 'tangent_neighbours': []}
    doc.update({
        'unit': 'mm', 'frame': 'part', 'source': {'file': 'model.step', 'component': None, 'up_axis': '+Z'},
        'tessellation': {'face_linear_deflection_mm': 0.05, 'face_angular_deflection_deg': 5.0,
                         'edge_chord_mm': 0.05},
        'solids': [{'solid_id': f'B{i}', 'name': n, 'volume_mm3': 1.0, 'bbox_min_mm': [0, 0, 0],
                    'bbox_max_mm': [1, 1, 1]} for i, n in ((1, 'a'), (2, 'b'))],
        'faces': [{'face_id': f'F{i}', 'solid_id': 'B1' if i < 4 else 'B2', 'surface_type': 'cylinder',
                   'area_mm2': 1.0} for i in range(1, 7)],
        'edges': [dict(edge, edge_id='E1', solid_id='B1', faces=['F1', 'F2'], fingerprint='fa', joint_edge_id='JE1'),
                  dict(edge, edge_id='E5', solid_id='B2', faces=['F4', 'F5'], fingerprint='fc',
                       joint_edge_id='JE1')],
        'joints': [{'joint_edge_id': 'JE1', 'solid_a': 'B1', 'solid_b': 'B2', 'edge_a': 'E1', 'edge_b': 'E5',
                    'length_mm': length, 'closed': True, 'preview_frames': []}],
    })
    return doc


def ring_sampler(job, request: Dict) -> Dict:
    """Exact ring samples (mm, part frame) in the format of ``awm_geometry.cli sample-joints``."""
    length = 2.0 * math.pi * RING_R_MM
    out = []
    for req in request['seams']:
        n = int(math.ceil(length / request['pitch_mm'])) + 1
        spacing = length / (n - 1)
        sign = -1.0 if req['reversed'] else 1.0
        pts: List[Dict] = []
        for k in range(n):
            a = sign * k * spacing / RING_R_MM
            c, d = math.cos(a), math.sin(a)
            pts.append({'s_mm': k * spacing, 'p_mm': [RING_R_MM * c, RING_R_MM * d, 0.0],
                        't': [-sign * d, sign * c, 0.0], 'n1': [c, d, 0.0], 'n2': [c, d, 0.0]})
        out.append({'seam_id': req['seam_id'], 'joint_edge_id': 'JE1', 'solid_a': 'B1', 'solid_b': 'B2',
                    'edge_a': 'E1', 'edge_b': 'E5', 'closed': True, 'length_mm': length, 'start_offset_mm': 0.0,
                    'spacing_mm': spacing, 'faces_n1': ['F1'], 'faces_n2': ['F4'], 'points': pts})
    return {'unit': 'mm', 'frame': 'part', 'seams': out}


def make_ring_job(root: Path, mode: str = 'sync', arcs: int = 1, angles=None):
    """Repository with job TEST-RING_A: seams.json (one seam S01 on JE1) and seam_axis.json."""
    from awm_seam import joint_seams, seam_axis

    repo = Repository(root)
    job = repo.job('TEST-RING', 'A')
    (job.dir / 'cache' / 'view').mkdir(parents=True)
    (job.dir / 'model.step').write_text('dummy step for tests\n')
    part = jsonio.make_header('test', [job.dir / 'model.step'], job.dir)
    part.update({'schema_version': '2.0', 'part_no': 'TEST-RING', 'part_rev': 'A'})
    job.part_json.write_text(json.dumps(part))
    (job.dir / 'cache' / 'view' / 'topology.json').write_text(json.dumps(_topology(job.dir)))
    s = math.sqrt(0.5)
    pose = {'position_mm': [0.0, 0.0, 0.0], 'quat_xyzw': [0.0, s, 0.0, s]}   # part +Z along flange +X
    pl = jsonio.make_header('test', [], job.dir)
    pl.update({'schema_version': '2.0', 'source': 'fixture_cad', 'verified': False, 'flange_T_part': pose,
               'objects': [{'object_id': 'part', 'role': 'part', 'parent_frame': 'positioner_flange',
                            'pose_mm': pose, 'mesh': {'file': 'cache/collision.stl', 'sha256': '1' * 64,
                                                      'face_ids': True}, 'padding_mm': 0.0}],
               'scene_sha256': '0' * 64})
    jsonio.write_json(job.dir / 'placement.json', pl, 'v2/placement')
    frames = root / 'cell' / 'model' / 'frames.yaml'
    frames.parent.mkdir(parents=True, exist_ok=True)
    frames.write_text(FRAMES_YAML)
    req = joint_seams.JointSeamRequest.from_dict({'seam_id': 'S01', 'edges': ['E1'], 'joint_type': 'butt',
                                                  'sequence': 1, 'positioner_mode': mode, 'arcs': arcs,
                                                  'arc_overlap_mm': 2.0, 'positioner_angles_deg': angles})
    joint_seams.build_seams(job, [req], 1.0, ring_sampler)
    seam_axis.build_seam_axis(job)
    return job
