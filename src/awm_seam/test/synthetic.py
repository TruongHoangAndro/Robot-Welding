"""Synthetic job data for the G1.2 tests (no OCC): a two-solid ring joint and a fake joint sampler.

The ring is a circle of radius ``RING_R_MM`` about the part Z axis at z = 0 (test data, not part data in source).
Forward weld direction = counter-clockwise about +Z (the curve direction of the joint edge of solid_a).
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List, Optional

from awm_core import jsonio
from awm_core.repository import Repository

RING_R_MM = 13.335


def topology_doc(job_dir: Path) -> Dict:
    """D20 topology.json with one closed joint JE1 (E1 of B1 / E5 of B2) and one non-joint edge E2."""
    doc = jsonio.make_header('test', [job_dir / 'model.step'], job_dir)
    doc['schema_version'] = '2.0'
    length = 2.0 * math.pi * RING_R_MM
    edge = {'curve_type': 'circle', 'length_mm': length, 'closed': True, 'tangent_neighbours': []}
    doc.update({
        'unit': 'mm', 'frame': 'part',
        'source': {'file': 'model.step', 'component': None, 'up_axis': '+Z'},
        'tessellation': {'face_linear_deflection_mm': 0.05, 'face_angular_deflection_deg': 5.0,
                         'edge_chord_mm': 0.05},
        'solids': [{'solid_id': 'B1', 'name': 'a', 'volume_mm3': 1.0, 'bbox_min_mm': [0, 0, 0],
                    'bbox_max_mm': [1, 1, 1]},
                   {'solid_id': 'B2', 'name': 'b', 'volume_mm3': 1.0, 'bbox_min_mm': [0, 0, 0],
                    'bbox_max_mm': [1, 1, 1]}],
        'faces': [{'face_id': f'F{i}', 'solid_id': 'B1' if i < 4 else 'B2', 'surface_type': 'cylinder',
                   'area_mm2': 1.0} for i in range(1, 7)],
        'edges': [
            dict(edge, edge_id='E1', solid_id='B1', faces=['F1', 'F2'], fingerprint='fa', joint_edge_id='JE1'),
            dict(edge, edge_id='E2', solid_id='B1', faces=['F2', 'F3'], fingerprint='fb', joint_edge_id=None),
            dict(edge, edge_id='E5', solid_id='B2', faces=['F4', 'F5'], fingerprint='fc', joint_edge_id='JE1'),
        ],
        'joints': [{'joint_edge_id': 'JE1', 'solid_a': 'B1', 'solid_b': 'B2', 'edge_a': 'E1', 'edge_b': 'E5',
                    'length_mm': length, 'closed': True, 'preview_frames': []}],
    })
    return doc


def make_job(root: Path, part_no: str = 'TEST-RING', with_cell: bool = True,
             flange_T_part: Optional[Dict] = None):
    """Repository with one job: model.step (dummy), part.json, cache/view/topology.json,
    placement.json and cell/model/frames.yaml."""
    repo = Repository(root)
    job = repo.job(part_no, 'A')
    (job.dir / 'cache' / 'view').mkdir(parents=True)
    (job.dir / 'model.step').write_text('dummy step for tests\n')
    part = jsonio.make_header('test', [job.dir / 'model.step'], job.dir)
    part.update({'schema_version': '2.0', 'part_no': part_no, 'part_rev': 'A'})
    job.part_json.write_text(json.dumps(part))
    (job.dir / 'cache' / 'view' / 'topology.json').write_text(json.dumps(topology_doc(job.dir)))
    if with_cell:
        write_placement(job, flange_T_part or coaxial_pose())
        write_frames(root / 'cell' / 'model' / 'frames.yaml')
    return job


def coaxial_pose(offset_mm=(0.0, 0.0, 0.0)) -> Dict:
    """flange_T_part with part +Z along the positioner axis (flange +X): rotation +90 deg about Y."""
    s = math.sqrt(0.5)
    return {'position_mm': list(offset_mm), 'quat_xyzw': [0.0, s, 0.0, s]}


def write_placement(job, pose: Dict) -> None:
    doc = jsonio.make_header('test', [], job.dir)
    doc.update({'schema_version': '2.0', 'source': 'fixture_cad', 'verified': False, 'flange_T_part': pose,
                'objects': [{'object_id': 'part', 'role': 'part', 'parent_frame': 'positioner_flange',
                             'pose_mm': pose, 'mesh': {'file': 'cache/collision.stl', 'sha256': '1' * 64,
                                                        'face_ids': True}, 'padding_mm': 0.0}],
                'scene_sha256': '0' * 64})
    jsonio.write_json(job.dir / 'placement.json', doc, 'v2/placement')


def write_frames(path: Path, direction=(1.0, 0.0, 0.0)) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        '# test frames\nschema_version: "2.0"\ncreated_by: "test"\ncreated_at: "2026-11-05T00:00:00Z"\n'
        'inputs: []\nschema: "awm_cell_frames/0.1"\nunit: "mm"\n'
        'world_T_positioner_axis:\n  position_mm: [10.0, -400.0, 75.0]\n  quat_xyzw: [0.0, 0.0, 0.0, 1.0]\n'
        f'  point_mm: [10.0, -400.0, 75.0]\n  direction: [{direction[0]}, {direction[1]}, {direction[2]}]\n')


def fake_sampler(job, request: Dict) -> Dict:
    """Stand-in for ``awm_geometry.cli sample-joints``: exact ring samples (mm) in the part frame."""
    length = 2.0 * math.pi * RING_R_MM
    out = []
    for req in request['seams']:
        assert req['joint_edge_id'] == 'JE1'
        n = int(math.ceil(length / request['pitch_mm'])) + 1
        spacing = length / (n - 1)
        start = float(req.get('start_offset_mm') or 0.0)
        sign = -1.0 if req['reversed'] else 1.0
        pts: List[Dict] = []
        for k in range(n):
            s = k * spacing
            a = (start + sign * s) / RING_R_MM
            c, d = math.cos(a), math.sin(a)
            pts.append({'s_mm': s, 'p_mm': [RING_R_MM * c, RING_R_MM * d, 0.0],
                        't': [-sign * d, sign * c, 0.0], 'n1': [c, d, 0.0], 'n2': [c, d, 0.0]})
        out.append({'seam_id': req['seam_id'], 'joint_edge_id': 'JE1', 'solid_a': 'B1', 'solid_b': 'B2',
                    'edge_a': 'E1', 'edge_b': 'E5', 'closed': True, 'length_mm': length,
                    'start_offset_mm': start, 'spacing_mm': spacing, 'faces_n1': ['F1'], 'faces_n2': ['F4'],
                    'points': pts})
    return {'unit': 'mm', 'frame': 'part', 'seams': out}


def request(**kw) -> Dict:
    base = {'seam_id': 'S01', 'edges': ['E1'], 'joint_type': 'butt', 'sequence': 1}
    base.update(kw)
    return base


