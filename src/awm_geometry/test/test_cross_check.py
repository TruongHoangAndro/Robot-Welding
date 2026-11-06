"""CP-60 cross-check math (pure numpy): positioner match with the angle phi, pose offsets, swept envelope."""
import math

import numpy as np

from awm_core import frames
from awm_geometry.cross_check import pose_check, positioner_match, swept_envelope_hits, worst

TOL_MM, TOL_DEG = 0.05, 0.1


def rx(deg):
    return frames.make_transform([0, 0, 0], frames.axis_angle_to_matrix([1, 0, 0], math.radians(deg)))


def pose(xyz, rpy_deg=(0, 0, 0)):
    return frames.xyz_rpy_to_matrix(xyz, [math.radians(v) for v in rpy_deg])


CELL = {'radius_mm': 60.0, 'axis_T': {'Disc:1': pose([-10, 0, 0]), 'Holder:1': pose([15, 30, 0], (0, 0, 20))},
        'duplicates': []}


def job_from(cell, t=np.eye(4), radius=None):
    return {'radius_mm': cell['radius_mm'] if radius is None else radius,
            'axis_T': {k: t @ v for k, v in cell['axis_T'].items()}, 'duplicates': []}


def test_identical_is_green():
    r = positioner_match(CELL, job_from(CELL), TOL_MM, TOL_DEG)
    assert r['status'] == 'green' and abs(r['phi_deg']) < 1e-9
    assert sorted(r['values']['matched']) == ['Disc:1', 'Holder:1']


def test_phi_recovered_from_rotated_job():
    """Job positioner turned by +30 deg about the axis: cell = Rx(phi) job -> phi = -30, still green."""
    r = positioner_match(CELL, job_from(CELL, rx(30.0)), TOL_MM, TOL_DEG)
    assert r['status'] == 'green' and abs(r['phi_deg'] + 30.0) < 1e-9


def test_shifted_along_axis_is_red():
    r = positioner_match(CELL, job_from(CELL, pose([120, 0, 0])), TOL_MM, TOL_DEG)
    assert r['status'] == 'red' and r['values']['max_offset_mm'] > 100


def test_one_component_moved_is_red():
    job = job_from(CELL)
    job['axis_T']['Holder:1'] = pose([0.06, 0, 0]) @ job['axis_T']['Holder:1']
    assert positioner_match(CELL, job, TOL_MM, TOL_DEG)['status'] == 'red'


def test_within_tolerance_is_green():
    job = job_from(CELL)
    job['axis_T']['Holder:1'] = pose([0.04, 0, 0], (0.05, 0, 0)) @ job['axis_T']['Holder:1']
    assert positioner_match(CELL, job, TOL_MM, TOL_DEG)['status'] == 'green'


def test_radius_off_by_0p06_is_red():
    r = positioner_match(CELL, job_from(CELL, radius=60.06), TOL_MM, TOL_DEG)
    assert r['status'] == 'red' and 'radius' in r['note']


def test_no_common_component_is_red():
    job = {'radius_mm': 60.0, 'axis_T': {'Other:1': np.eye(4)}, 'duplicates': []}
    r = positioner_match(CELL, job, TOL_MM, TOL_DEG)
    assert r['status'] == 'red' and 'cannot be confirmed' in r['note']


def test_duplicate_instance_name_is_red():
    job = job_from(CELL)
    job['duplicates'] = ['Holder:1']
    r = positioner_match(CELL, job, TOL_MM, TOL_DEG)
    assert r['status'] == 'red' and 'Holder:1' in r['note']


def test_pose_check_offsets():
    a = pose([0, 0, 0])
    assert pose_check(a, pose([0.04, 0, 0]), TOL_MM, TOL_DEG)['status'] == 'green'
    out = pose_check(a, pose([0, 0.8, 0]), TOL_MM, TOL_DEG)
    assert out['status'] == 'yellow' and abs(out['values']['offset_mm'] - 0.8) < 1e-9
    assert pose_check(a, pose([0, 0, 0], (0, 0, 0.2)), TOL_MM, TOL_DEG)['status'] == 'yellow'


def _ring(r0, r1, a0, a1, n=24):
    pts = []
    for a in np.linspace(a0, a1, 5):
        for r in (r0, r1):
            for t in np.linspace(0, 2 * np.pi, n, endpoint=False):
                pts.append([a, r * np.cos(t), r * np.sin(t)])
    return np.array(pts)


def test_static_inside_swept_ring_hits():
    rotating = {'holder': _ring(40.0, 60.0, 0.0, 10.0)}
    statics = {'block': np.array([[5.0, 0.0, 50.0], [5.0, 0.0, 70.0]])}
    hits = swept_envelope_hits(rotating, statics, slab_mm=1.0)
    assert len(hits) == 1 and hits[0]['static'] == 'block' and hits[0]['rotating'] == 'holder'
    assert abs(hits[0]['depth_mm'] - 10.0) < 1e-9


def test_static_in_bore_does_not_hit():
    rotating = {'holder': _ring(40.0, 60.0, 0.0, 10.0)}
    statics = {'shaft': np.array([[5.0, 0.0, 10.0], [5.0, 10.0, 0.0]])}
    assert swept_envelope_hits(rotating, statics, slab_mm=1.0) == []


def test_static_beyond_axial_extent_does_not_hit():
    rotating = {'holder': _ring(40.0, 60.0, 0.0, 10.0)}
    statics = {'block': np.array([[12.0, 0.0, 50.0]])}
    assert swept_envelope_hits(rotating, statics, slab_mm=1.0) == []


def test_worst_status_ignores_skipped():
    assert worst(['green', 'skipped', 'yellow']) == 'yellow'
    assert worst(['green', 'red', 'yellow']) == 'red'
    assert worst(['skipped']) == 'green'


def test_triangles_near_cylinder_keeps_only_the_swept_region():
    from awm_geometry.cross_check import triangles_near_cylinder

    verts = np.array([[0, 50, 0], [5, 50, 0], [0, 55, 0],             # inside (r 50..55, x 0..5)
                      [500, 900, 0], [510, 900, 0], [500, 910, 0],    # far away
                      [2, -500, -500], [2, 500, -500], [2, 0, 900]],  # big triangle spanning the region
                     dtype=float)
    tris = np.array([[0, 1, 2], [3, 4, 5], [6, 7, 8]])
    keep = triangles_near_cylinder(verts, tris, x_lo=-1.0, x_hi=10.0, r_max=60.0, margin=1.0)
    assert sorted(keep.tolist()) == [0, 2]
