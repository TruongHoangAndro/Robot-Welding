"""WP-S S.5 Task 3: pure parts of the runner — spike seam file (D04 v2 shape) and the C2 / C3 / C5 / C8 summary."""

import math

import numpy as np
import pytest

from awm_path.spike_s5 import build_seams_doc, case_summary
from test_core_seam_input import _circle_pts, _glb, _preview, R


def _topology():
    return {'edges': [{'edge_id': 'E7', 'curve_type': 'circle', 'fingerprint': 'fp-e7'},
                      {'edge_id': 'E8', 'curve_type': 'circle', 'fingerprint': 'fp-e8'}],
            'joints': [{'joint_edge_id': 'JE1', 'solid_a': 'B1', 'solid_b': 'B2', 'edge_a': 'E7', 'edge_b': 'E8',
                        'length_mm': 2 * math.pi * R, 'closed': True, 'preview_frames': _preview()}]}


def _input(**kw):
    s = {'seam_id': 'S01', 'joint_edge_id': 'JE1', 'faces': {'n1': ['F1'], 'n2': ['F2']}, 'joint_type': 'butt',
         'reversed': False, 'sequence': 1, 'arcs': 2, 'arc_overlap_mm': 2.0, 'geometry_spacing_mm': 0.5}
    s.update(kw)
    return {'part_no': 'P1', 'part_rev': 'A', 'seams': [s]}


def test_build_seams_doc_validates_against_seams_v2(tmp_path):
    glb = tmp_path / 'view.glb'
    glb.write_bytes(_glb([('E7', _circle_pts(36).astype(np.float32), 3)]))
    header = {'schema_version': '2.0', 'created_by': 'test', 'created_at': '2026-11-04T00:00:00Z', 'inputs': []}
    doc = build_seams_doc(_input(), _topology(), glb, header)
    s = doc['seams'][0]
    assert s['edges'] == ['E7', 'E8'] and s['solids'] == ['B1', 'B2'] and s['closed'] is True
    assert s['arcs'] == 2 and s['arc_overlap_mm'] == 2.0 and s['fingerprint'] == 'fp-e7'
    assert s['length_mm'] == pytest.approx(2 * math.pi * R, abs=1e-6)
    assert len(s['geometry']['points']) == math.ceil(2 * math.pi * R / 0.5)
    with pytest.raises(ValueError, match='reversed'):
        build_seams_doc(_input(reversed=True), _topology(), glb, header)
    topo = _topology()
    topo['edges'][0]['curve_type'] = 'bspline'
    with pytest.raises(ValueError, match='circle'):
        build_seams_doc(_input(), topo, glb, header)


def _arc(ok, coverage, t, length=40.0, metrics=None):
    return {'accepted': ok, 'c2': {'coverage_pct': coverage, 'pass': coverage == 100.0},
            'c3': {'pass': ok, 'metrics': metrics}, 'timing_s': {'c5_s': t}, 'nominal_length_mm': length,
            'window': {'fraction': 0.5}}


def test_case_summary_gates():
    s = case_summary([_arc(True, 100.0, 10.0), _arc(True, 100.0, 12.0)], seam_length_mm=80.0)
    assert s['C2']['pass'] and s['C3']['pass'] and s['C5']['pass'] and s['C8']['coverage_pct'] == 100.0
    assert s['C5']['seam_s'] == pytest.approx(22.0)
    s = case_summary([_arc(True, 100.0, 20.0), _arc(False, 80.0, 20.0)], seam_length_mm=80.0)
    assert not s['C2']['pass'] and not s['C3']['pass'] and not s['C5']['pass']
    assert s['C8']['coverage_pct'] == pytest.approx(50.0) and not s['C8']['pass']
