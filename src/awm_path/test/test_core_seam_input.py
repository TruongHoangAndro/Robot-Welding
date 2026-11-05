"""WP-S S.5 Task 1: seam input from the exact D20 joint edge, seam frames along s, arc split (CP-53)."""

import json
import math
import struct

import numpy as np
import pytest

from awm_path.core.seam_input import (SeamInputError, arc_ranges, circle_joint_geometry, fit_circle, frames_at,
                                      read_glb_edge_polyline)

C = np.array([10.0, -5.0, 3.0])
R = 13.335
AXIS = np.array([0.0, 0.1, 1.0]) / np.linalg.norm([0.0, 0.1, 1.0])


def _basis():
    e0 = np.cross(AXIS, [1.0, 0.0, 0.0])
    e0 /= np.linalg.norm(e0)
    return e0, np.cross(AXIS, e0)


def _circle_pts(n, closed_repeat=True):
    e0, e1 = _basis()
    a = np.linspace(0.0, 2 * math.pi, n + 1)[: (n + 1 if closed_repeat else n)]
    return C + R * (np.cos(a)[:, None] * e0 + np.sin(a)[:, None] * e1)


def _preview(sign=1.0):
    e0, e1 = _basis()
    out = []
    for k in range(4):
        a = k * math.pi / 2
        er = math.cos(a) * e0 + math.sin(a) * e1
        out.append({'point_mm': list(C + R * er), 'z': list(sign * er), 'x': list(np.cross(AXIS, er))})
    return out


def test_fit_circle_is_exact_on_curve_points():
    fit = fit_circle(_circle_pts(37))
    assert np.allclose(fit.centre, C, atol=1e-9)
    assert fit.radius == pytest.approx(R, abs=1e-9)
    assert abs(abs(float(fit.axis @ AXIS)) - 1.0) < 1e-12
    assert fit.max_residual_mm < 1e-9


def test_circle_joint_geometry_frames_are_radial_and_follow_the_edge():
    geo = circle_joint_geometry(_circle_pts(37), _preview(), spacing_max_mm=0.5, length_mm=2 * math.pi * R)
    pts = geo['points']
    assert geo['closed'] is True
    assert pts[0]['s_mm'] == 0.0
    assert np.allclose(pts[0]['p_mm'], _preview()[0]['point_mm'], atol=1e-9)
    assert geo['spacing_mm'] <= 0.5
    for p in pts:
        er = (np.asarray(p['p_mm']) - C) / R
        assert np.linalg.norm(np.asarray(p['p_mm']) - C) == pytest.approx(R, abs=1e-9)
        assert np.allclose(p['n1'], er, atol=1e-9) and np.allclose(p['n2'], er, atol=1e-9)
        assert np.allclose(p['t'], np.cross(AXIS, er), atol=1e-9)
    frames = frames_at(pts, [0.0, 10.0, geo['length_mm'] + 1.0], closed=True, length_mm=geo['length_mm'])
    for f in frames:
        assert np.allclose(f[:3, :3].T @ f[:3, :3], np.eye(3), atol=1e-9)
        er = (f[:3, 3] - C) / np.linalg.norm(f[:3, 3] - C)
        assert np.allclose(f[:3, 2], er, atol=1e-6)
        sag = geo['spacing_mm'] ** 2 / (8 * R)          # chord sag of the linear interpolation
        assert R - sag - 1e-9 <= np.linalg.norm(f[:3, 3] - C) <= R + 1e-9
    # s beyond the length wraps on a closed seam: s = L + 1 equals s = 1
    assert np.allclose(frames[2], frames_at(pts, [1.0], True, geo['length_mm'])[0], atol=1e-9)


def test_inward_preview_normal_gives_inward_z_and_mismatches_are_refused():
    geo = circle_joint_geometry(_circle_pts(37), _preview(sign=-1.0), spacing_max_mm=1.0,
                                length_mm=2 * math.pi * R)
    p = geo['points'][3]
    assert np.allclose(p['n1'], -(np.asarray(p['p_mm']) - C) / R, atol=1e-9)
    bad = _preview()
    bad[2] = dict(bad[2], z=list(np.asarray(bad[2]['z']) * math.cos(0.01) + AXIS * math.sin(0.01)))
    with pytest.raises(SeamInputError, match='preview frame 2'):
        circle_joint_geometry(_circle_pts(37), bad, spacing_max_mm=1.0, length_mm=2 * math.pi * R)
    with pytest.raises(SeamInputError, match='length'):
        circle_joint_geometry(_circle_pts(37), _preview(), spacing_max_mm=1.0, length_mm=2 * math.pi * R + 0.5)
    wobbly = _circle_pts(37) + np.random.default_rng(0).normal(scale=0.05, size=(38, 3))
    with pytest.raises(SeamInputError, match='not a circle'):
        circle_joint_geometry(wobbly, _preview(), spacing_max_mm=1.0, length_mm=2 * math.pi * R)


def test_arc_ranges_equal_lengths_with_overlap():
    assert arc_ranges(100.0, 2, 2.0, closed=True) == [(0.0, 52.0), (50.0, 102.0)]
    assert arc_ranges(90.0, 3, 1.0, closed=False) == [(0.0, 31.0), (30.0, 61.0), (60.0, 90.0)]
    assert arc_ranges(90.0, 1, 1.0, closed=False) == [(0.0, 90.0)]
    with pytest.raises(ValueError):
        arc_ranges(90.0, 0, 1.0, closed=False)
    with pytest.raises(ValueError):
        arc_ranges(90.0, 2, -1.0, closed=False)


def _glb(nodes):
    """Hand-built GLB: each node = (name, positions (n, 3) float32, mode)."""
    bin_ = b''
    acc, views, meshes, gnodes = [], [], [], []
    for i, (name, pos, mode) in enumerate(nodes):
        data = np.asarray(pos, dtype='<f4').tobytes()
        views.append({'buffer': 0, 'byteOffset': len(bin_), 'byteLength': len(data)})
        acc.append({'bufferView': i, 'componentType': 5126, 'count': len(pos), 'type': 'VEC3'})
        meshes.append({'name': name, 'primitives': [{'attributes': {'POSITION': i}, 'mode': mode}]})
        gnodes.append({'name': name, 'mesh': i, 'extras': {'edge_id': name}})
        bin_ += data
    doc = {'asset': {'version': '2.0', 'extras': {'unit': 'mm'}}, 'nodes': gnodes, 'meshes': meshes,
           'accessors': acc, 'bufferViews': views, 'buffers': [{'byteLength': len(bin_)}]}
    js = json.dumps(doc).encode()
    js += b' ' * ((4 - len(js) % 4) % 4)
    body = struct.pack('<II', len(js), 0x4E4F534A) + js + struct.pack('<II', len(bin_), 0x004E4942) + bin_
    return struct.pack('<III', 0x46546C67, 2, 12 + len(body)) + body


def test_read_glb_edge_polyline(tmp_path):
    pts = _circle_pts(8).astype(np.float32)
    f = tmp_path / 'view.glb'
    f.write_bytes(_glb([('F1', pts[:3], 4), ('E7', pts, 3)]))
    got = read_glb_edge_polyline(f, 'E7')
    assert got.dtype == np.float64 and np.allclose(got, pts)
    with pytest.raises(SeamInputError):
        read_glb_edge_polyline(f, 'E99')
    with pytest.raises(SeamInputError):
        read_glb_edge_polyline(f, 'F1')      # a face node is not a polyline
