"""WP-S S.4 C3: nozzle contact check F9 (TST-002 Rev 0.3 §5.1 F9; DES-001 Rev 0.6 draft §5.4, B-04).

Synthetic test geometry only (no part data): flat faces triangulated on a grid, lengths in mm.
"""

import json
import math
import struct

import numpy as np
import pytest

from awm_core import units
from awm_path.core.contact import (ContactParams, PartMesh, contact_check, disc_samples,
                                   point_triangle_distances)

SEAM = (('A',), ('B',))   # (n1 faces, n2 faces)


def _rect(origin, u, v, nu, nv):
    """Grid of 2·nu·nv triangles spanning origin + s·u + t·v, s, t in [0, 1]."""
    o, u, v = (np.asarray(x, dtype=float) for x in (origin, u, v))
    tris = []
    for i in range(nu):
        for j in range(nv):
            p00 = o + u * i / nu + v * j / nv
            p10 = o + u * (i + 1) / nu + v * j / nv
            p01 = o + u * i / nu + v * (j + 1) / nv
            p11 = o + u * (i + 1) / nu + v * (j + 1) / nv
            tris += [[p00, p10, p11], [p00, p11, p01]]
    return np.asarray(tris)


def _mesh(faces):
    """faces: list of (face_id, triangles) -> PartMesh."""
    ids = tuple(f for f, _ in faces)
    tris = np.concatenate([t for _, t in faces])
    index = np.concatenate([np.full(len(t), k) for k, (_, t) in enumerate(faces)])
    return PartMesh(tris, index, ids)


def _plate(wall_x=None, internal=False, far=False):
    """Seam along the Y axis at x = 0 on the plane z = 0: face A (x < 0), face B (x > 0).

    wall_x: optional wall face W (plane x = wall_x, z from 0 to 20) — a corner wall next to face A.
    internal: the end (contact) face E of solid A below the seam (plane x = 0, z from -3 to 0), touching the seam.
    far: a non-seam face F 50 mm away (must not change any result).
    """
    faces = [('A', _rect([-30, -30, 0], [30, 0, 0], [0, 60, 0], 60, 60)),
             ('B', _rect([0, -30, 0], [30, 0, 0], [0, 60, 0], 60, 60))]
    if wall_x is not None:
        faces.append(('W', _rect([wall_x, -30, 0], [0, 60, 0], [0, 0, 20], 60, 20)))
    if internal:
        faces.append(('E', _rect([0, -30, -3], [0, 60, 0], [0, 0, 3], 60, 6)))
    if far:
        faces.append(('F', _rect([-10, -10, 50], [20, 0, 0], [0, 20, 0], 4, 4)))
    return _mesh(faces)


def _tcp(x=0.0, y=0.0, z=0.0, psi_deg=0.0, tilt_deg=0.0):
    """part_T_tcp with the beam (tool Z) pointing down into the plate (-Z part), rotated psi about the beam and
    tilted about the part Y axis."""
    rx180 = np.diag([1.0, -1.0, -1.0])
    c, s = math.cos(units.deg_to_rad(psi_deg)), math.sin(units.deg_to_rad(psi_deg))
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    c, s = math.cos(units.deg_to_rad(tilt_deg)), math.sin(units.deg_to_rad(tilt_deg))
    ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    t = np.eye(4)
    t[:3, :3] = ry @ rx180 @ rz
    t[:3, 3] = [x, y, z]
    return t


# ---------------------------------------------------------------------------------------------- distances

def test_point_triangle_distances_match_dense_sampling() -> None:
    rng = np.random.default_rng(20261103)
    tris = rng.uniform(-5.0, 5.0, size=(30, 3, 3))
    pts = rng.uniform(-8.0, 8.0, size=(25, 3))
    d = point_triangle_distances(pts, tris)
    assert d.shape == (25, 30)
    # barycentric grid on every triangle: the true distance is <= the sampled one and close to it
    k = 60
    w = np.array([(i / k, j / k) for i in range(k + 1) for j in range(k + 1 - i)])
    for t in range(30):
        a, b, c = tris[t]
        samples = a + w[:, :1] * (b - a) + w[:, 1:] * (c - a)
        brute = np.linalg.norm(pts[:, None, :] - samples[None], axis=2).min(axis=1)
        edge = max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b))
        assert np.all(d[:, t] <= brute + 1e-9)
        assert np.all(brute - d[:, t] <= edge / k + 1e-9)


def test_point_triangle_distances_degenerate_triangle_is_finite() -> None:
    tris = np.array([[[0, 0, 0], [1, 0, 0], [2, 0, 0]]], dtype=float)  # zero area
    d = point_triangle_distances(np.array([[1.0, 1.0, 0.0], [3.0, 0.0, 0.0]]), tris)
    assert np.all(np.isfinite(d))
    assert d[0, 0] == pytest.approx(1.0)
    assert d[1, 0] == pytest.approx(1.0)


# ---------------------------------------------------------------------------------------------- samples

def test_disc_samples_cover_centre_rings_and_rim() -> None:
    p = ContactParams()
    s = disc_samples(p)
    r = np.linalg.norm(s[:, :2], axis=1)
    assert np.allclose(s[:, 2], 0.0)
    assert np.allclose(s[0], 0.0)
    assert np.isclose(r.max(), p.nozzle_radius_mm)
    assert np.count_nonzero(np.isclose(r, p.nozzle_radius_mm)) == p.rim_samples
    rim = s[np.isclose(r, p.nozzle_radius_mm)]
    gaps = np.linalg.norm(rim - np.roll(rim, 1, axis=0), axis=1)
    assert gaps.max() < 2.0 * p.standoff_mm  # a wall crossing the rim is never missed


def test_disc_sample_set_is_invariant_under_5_degree_rotations() -> None:
    s = disc_samples(ContactParams())
    c, si = math.cos(units.deg_to_rad(5.0)), math.sin(units.deg_to_rad(5.0))
    rot = s @ np.array([[c, -si, 0], [si, c, 0], [0, 0, 1]]).T
    d = np.linalg.norm(rot[:, None, :] - s[None], axis=2).min(axis=1)
    assert d.max() < 1e-9


# ---------------------------------------------------------------------------------------------- contact rule

def test_tcp_on_seam_line_passes() -> None:
    res = contact_check(_plate(), _tcp(), SEAM, ContactParams())
    assert res.ok and res.reason is None
    assert res.tcp_seam_distance_mm == pytest.approx(0.0, abs=1e-9)
    assert res.offending == ()


def test_tcp_on_seam_face_off_the_line_passes() -> None:
    assert contact_check(_plate(), _tcp(x=-3.0), SEAM, ContactParams()).ok


def test_internal_contact_face_touching_the_seam_is_a_tie_not_a_failure() -> None:
    """Butt joint: the end face of solid A touches the seam edge; the seam faces are as close there."""
    res = contact_check(_plate(internal=True), _tcp(), SEAM, ContactParams())
    assert res.ok, res


def test_internal_face_marginally_closer_at_the_seam_edge_is_not_off_seam() -> None:
    """Rule (a) uses the seam-edge exception too: on the real mesh the end face of a butt joint can be a few
    1e-6 mm closer to tcp_laser than the seam faces (tessellation of the shared edge)."""
    plate = _plate(internal=True)
    tris = plate.triangles_mm.copy()
    e = plate.face_ids.index('E')
    tris[plate.face_index == e, :, 0] -= 0.0      # E stays at x = 0
    tris[plate.face_index != e, :, 2] -= 1e-5     # seam faces 1e-5 mm lower than the top edge of E
    mesh = type(plate)(tris, plate.face_index, plate.face_ids)
    res = contact_check(mesh, _tcp(), SEAM, ContactParams())
    assert res.ok, res


def test_tcp_inside_a_non_seam_face_fails_off_seam() -> None:
    """TST-002 G2 contact-zone pose 1 (synthetic): tip 0.5 mm into a non-seam face."""
    mesh = _mesh([('A', _rect([-30, -30, 0], [30, 0, 0], [0, 60, 0], 30, 30)),
                  ('N', _rect([0, -30, 0], [30, 0, 0], [0, 60, 0], 30, 30)),
                  ('B', _rect([-40, -10, 50], [10, 0, 0], [0, 10, 0], 2, 2))])   # far n2 face
    res = contact_check(mesh, _tcp(x=15.0, z=-0.5), (('A',), ('B',)), ContactParams())
    assert not res.ok and res.reason == 'contact_off_seam'
    assert res.tcp_nearest_other[0] == 'N'
    assert res.tcp_nearest_other[1] == pytest.approx(0.5, abs=1e-9)


def test_tcp_above_the_seam_within_and_beyond_tolerance() -> None:
    p = ContactParams(point_tolerance_mm=0.15)
    assert contact_check(_plate(), _tcp(z=0.1), SEAM, p).ok
    res = contact_check(_plate(), _tcp(z=0.2), SEAM, p)
    assert not res.ok and res.reason == 'contact_off_seam'


def test_rim_into_corner_wall_fails() -> None:
    """TST-002 G2 contact-zone pose 2 (synthetic): rim (r = 10 mm) crosses the wall at x = -8."""
    res = contact_check(_plate(wall_x=-8.0), _tcp(), SEAM, ContactParams())
    assert not res.ok and res.reason == 'other_face_within_standoff'
    assert [f for f, _ in res.offending] == ['W']


@pytest.mark.parametrize('gap_mm, ok', [(0.2, False), (0.29, False), (0.31, True), (0.5, True)])
def test_wall_gap_against_standoff(gap_mm, ok) -> None:
    """No other face within s_c = 0.3 mm of the nozzle end disc / rim."""
    res = contact_check(_plate(wall_x=-(10.0 + gap_mm)), _tcp(), SEAM, ContactParams(standoff_mm=0.3))
    assert res.ok == ok, res


def test_wall_meeting_only_one_seam_face_is_not_excused_by_the_seam_edge_rule() -> None:
    """The corner wall touches face A along its base line; only faces whose nearest point lies on the seam edge
    (on an n1 face and on an n2 face) are excused."""
    res = contact_check(_plate(wall_x=-9.9, internal=True), _tcp(), SEAM, ContactParams())
    assert res.reason == 'other_face_within_standoff'
    assert [f for f, _ in res.offending] == ['W']


def test_far_faces_do_not_change_the_result() -> None:
    p = ContactParams()
    for kw in ({}, {'wall_x': -8.0}):
        a = contact_check(_plate(**kw), _tcp(), SEAM, p)
        b = contact_check(_plate(far=True, **kw), _tcp(), SEAM, p)
        assert (a.ok, a.reason, a.offending) == (b.ok, b.reason, b.offending)


def test_result_is_invariant_under_psi_steps_of_5_degrees() -> None:
    p = ContactParams()
    mesh = _plate(wall_x=-10.15)
    base = contact_check(mesh, _tcp(), SEAM, p)
    for k in range(1, 72):
        r = contact_check(mesh, _tcp(psi_deg=5.0 * k), SEAM, p)
        assert (r.ok, r.reason) == (base.ok, base.reason)
        assert [f for f, _ in r.offending] == [f for f, _ in base.offending]
        assert [d for _, d in r.offending] == pytest.approx([d for _, d in base.offending], abs=1e-9)


def test_flat_tip_tilted_on_a_seam_face_is_left_to_the_validity_check() -> None:
    """A tilted tip penetrates its own seam face: the contact rule does not see penetration (only faces other
    than the seam faces); the F4 validity check of the unpadded nozzle_tip does (TST-002 G2 pose 3)."""
    res = contact_check(_plate(), _tcp(tilt_deg=8.0), SEAM, ContactParams())
    assert res.ok


def test_unknown_seam_face_is_an_error() -> None:
    with pytest.raises(ValueError, match='Z9'):
        contact_check(_plate(), _tcp(), (('A',), ('Z9',)), ContactParams())


def test_overlapping_n1_n2_is_an_error() -> None:
    with pytest.raises(ValueError, match='n1 and n2'):
        contact_check(_plate(), _tcp(), (('A',), ('A', 'B')), ContactParams())


def test_non_rigid_pose_is_an_error() -> None:
    t = _tcp()
    t[0, 0] = 2.0
    with pytest.raises(ValueError):
        contact_check(_plate(), t, SEAM, ContactParams())


# ---------------------------------------------------------------------------------------------- files

def _write_stl(path, tris):
    with open(path, 'wb') as fh:
        fh.write(b'\0' * 80 + struct.pack('<I', len(tris)))
        for t in tris:
            fh.write(struct.pack('<3f', 0, 0, 0) + struct.pack('<9f', *t.reshape(-1)) + b'\0\0')


def test_part_mesh_from_files(tmp_path) -> None:
    from awm_core.hashing import sha256_file
    tris_a = _rect([0, 0, 0], [1, 0, 0], [0, 1, 0], 2, 1)
    tris_b = _rect([0, 0, 1], [1, 0, 0], [0, 1, 0], 1, 1)
    stl = tmp_path / 'collision.stl'
    _write_stl(stl, np.concatenate([tris_a, tris_b]))
    doc = {'inputs': [{'name': 'cache/collision.stl', 'sha256': sha256_file(stl)}], 'unit': 'mm',
           'frame': 'part', 'triangles': 6,
           'faces': [{'face_id': 'F1', 'first_triangle': 0, 'triangle_count': 4},
                     {'face_id': 'F2', 'first_triangle': 4, 'triangle_count': 2}]}
    faces = tmp_path / 'collision_faces.json'
    faces.write_text(json.dumps(doc))
    mesh = PartMesh.from_files(stl, faces)
    assert mesh.face_ids == ('F1', 'F2')
    assert list(mesh.face_index) == [0, 0, 0, 0, 1, 1]
    assert np.allclose(mesh.triangles_mm[4], tris_b[0], atol=1e-6)

    doc['faces'][1]['triangle_count'] = 1
    faces.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='cover'):
        PartMesh.from_files(stl, faces)
    doc['faces'][1]['triangle_count'] = 2
    doc['inputs'][0]['sha256'] = '0' * 64
    faces.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='sha256'):
        PartMesh.from_files(stl, faces)


# ---------------------------------------------------------------------------------------------- rule (b), CP-56

def _chamfer_plate():
    """Seam faces A (x < 0) and B (0 < x < 5) on z = 0; chamfer C shares the edge x = 5 with B and slopes away from
    the nozzle (down to z = -5 at x = 15) — the J1 chamfer F11 case of CP-56 (synthetic)."""
    return _mesh([('A', _rect([-30, -30, 0], [30, 0, 0], [0, 60, 0], 60, 60)),
                  ('B', _rect([0, -30, 0], [5, 0, 0], [0, 60, 0], 10, 60)),
                  ('C', _rect([5, -30, 0], [10, 0, -5], [0, 60, 0], 20, 60))])


def _ridge_plate(height_mm):
    """Seam faces A / B on z = 0 and a ridge R (thread crest) under the disc: two faces rising from x = 7 to
    z = height at x = 8 and falling back at x = 9 (rising towards the nozzle)."""
    return _mesh([('A', _rect([-30, -30, 0], [30, 0, 0], [0, 60, 0], 60, 60)),
                  ('B', _rect([0, -30, 0], [7, 0, 0], [0, 60, 0], 14, 60)),
                  ('R', np.concatenate([_rect([7, -30, 0], [1, 0, height_mm], [0, 60, 0], 4, 60),
                                        _rect([8, -30, height_mm], [1, 0, -height_mm], [0, 60, 0], 4, 60)])),
                  ('G', _rect([9, -30, 0], [21, 0, 0], [0, 60, 0], 21, 60))])


def test_rule_b_face_sloping_away_from_the_nozzle_passes() -> None:
    """CP-56 decision 1: a non-seam face within s_c that lies only on the far side of the contact plane (plane
    through tcp_laser, normal to the beam axis) does not count."""
    res = contact_check(_chamfer_plate(), _tcp(), SEAM, ContactParams())
    assert res.ok and res.reason is None, res
    assert res.offending == ()
    assert [f for f, _ in res.excused] == ['C']   # within s_c, excused by rule (b) — recorded


def test_rule_b_unsigned_reproduces_the_s4_rule() -> None:
    """The S.4 rule (unsigned distance, records before CP-56) stays available for reproducing S.5 / S.5b records."""
    res = contact_check(_chamfer_plate(), _tcp(), SEAM, ContactParams(rule_b='unsigned'))
    assert res.reason == 'other_face_within_standoff'
    assert [f for f, _ in res.offending] == ['C']


def test_rule_b_crest_rising_towards_the_nozzle_fails() -> None:
    res = contact_check(_ridge_plate(0.25), _tcp(), SEAM, ContactParams())
    assert res.reason == 'other_face_within_standoff'
    assert [f for f, _ in res.offending] == ['R']


def test_rule_b_crest_rising_less_than_the_contact_tolerance_is_excused_and_recorded() -> None:
    p = ContactParams()
    res = contact_check(_ridge_plate(0.1), _tcp(), SEAM, p)
    assert 0.1 <= p.point_tolerance_mm
    assert res.ok, res
    assert 'R' in [f for f, _ in res.excused]


def test_rule_b_wall_just_inside_the_standoff_still_fails() -> None:
    """A wall rising towards the nozzle at a lateral gap g < s_c fails although sqrt(g² + tol²) > s_c: the nozzle
    side is sampled at the tolerance height, not only the end disc."""
    p = ContactParams(standoff_mm=0.3, point_tolerance_mm=0.15)
    res = contact_check(_plate(wall_x=-(10.0 + 0.29)), _tcp(), SEAM, p)
    assert res.reason == 'other_face_within_standoff'
    assert [f for f, _ in res.offending] == ['W']


def test_rule_b_unknown_rule_is_an_error() -> None:
    with pytest.raises(ValueError, match='rule_b'):
        ContactParams(rule_b='signed?')


def test_contact_params_record_names_the_rule() -> None:
    assert ContactParams().as_record()['rule_b'] == 'rising'


def test_clip_triangles_above_keeps_exactly_the_part_above_the_plane() -> None:
    from awm_path.core.contact import clip_triangles_above
    rng = np.random.default_rng(20261105)
    tris = rng.uniform(-5.0, 5.0, size=(200, 3, 3))
    origin, up, h_min = np.array([0.3, -0.2, 0.1]), np.array([0.2, 0.3, 1.0]), 0.15
    pieces, owner = clip_triangles_above(tris, origin, up, h_min)
    u = up / np.linalg.norm(up)
    assert np.all((pieces - origin) @ u >= h_min - 1e-9)
    # area above the plane per source triangle = area of its pieces (Monte Carlo on barycentric samples)
    area = lambda t: 0.5 * np.linalg.norm(np.cross(t[..., 1, :] - t[..., 0, :], t[..., 2, :] - t[..., 0, :]), axis=-1)
    w = rng.dirichlet([1, 1, 1], size=20000)
    for i in range(0, 200, 7):
        pts = w @ tris[i]
        frac = np.mean((pts - origin) @ u > h_min)
        got = area(pieces[owner == i]).sum() if np.any(owner == i) else 0.0
        assert got == pytest.approx(frac * area(tris[i]), abs=0.02 * area(tris[i]) + 1e-9)
