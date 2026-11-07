"""G1.3 kinematic 7-axis planner core (CP-58, CP-59; DES-001 §4.3; TST-002 Rev 0.3 draft F1–F3, F6, F7, F11).

Synthetic cell (test data, not part data): UR5e base_link = world, positioner axis = +X through (0, -450, 150) mm,
TCP 150 mm along tool0 Z with the beam along tool0 Z (simple head). A ring of radius 15 mm coaxial with the axis
(seam Z radial out of the material) and an off-axis ring whose axis is perpendicular to the positioner axis.
"""
import math

import numpy as np
import pytest

from awm_core import frames, units
from awm_path.core.batch_kin import batch_fk
from awm_path.core.kinematics import UrAnalyticKinematics, pose_error
from awm_path.core.sync import (ArcSpec, SyncCell, SyncParams, arc_geometry, flange_point_at, plan_arc, rot_x,
                                tcp_target)

R_RING = 15.0
N_RING = 48


@pytest.fixture(scope='module')
def kin():
    return UrAnalyticKinematics()


def _cell(theta_limits=(-400.0, 400.0)):
    axis = frames.make_transform([0.0, -450.0, 150.0], np.eye(3))
    tcp = frames.make_transform([0.0, 0.0, 150.0], np.eye(3))   # beam = tool0 +Z, TCP 150 mm out
    lims = ((-360.0, 360.0), (-360.0, 360.0), (-180.0, 180.0), (-360.0, 360.0), (-360.0, 360.0), (-360.0, 360.0))
    return SyncCell(base_link_T_axis=axis, world_z_base=np.array([0.0, 0.0, 1.0]), tool0_T_tcp=tcp,
                    joint_limits_deg=lims, theta_limits_deg=theta_limits, joint_velocity_deg_s=(180.0,) * 6,
                    theta_velocity_deg_s=30.0, joint_acceleration_deg_s2=(286.0,) * 6, theta_acceleration_deg_s2=50.0)


def _ring_seam(closed=True, offset=(0.0, 0.0, 0.0), axis='x', n=N_RING, r=R_RING):
    """seam_axis.json-like seam entry: ring about the positioner X axis ('x') or about a Z axis through ``offset``
    ('z'); seam Z radial out of the material, X = travel tangent."""
    pts = []
    length = 2.0 * math.pi * r
    for k in range(n + 1):
        a = 2.0 * math.pi * k / n
        if axis == 'x':
            p = np.array([0.0, r * math.cos(a), r * math.sin(a)])
            t = np.array([0.0, -math.sin(a), math.cos(a)])
            z = np.array([0.0, math.cos(a), math.sin(a)])
        else:
            p = np.array([r * math.cos(a), r * math.sin(a), 0.0])
            t = np.array([-math.sin(a), math.cos(a), 0.0])
            z = np.array([math.cos(a), math.sin(a), 0.0])
        p = p + np.asarray(offset)
        rot = np.column_stack([t, np.cross(z, t), z])
        pts.append({'s_mm': length * k / n, 'p_mm': list(p), 'quat_xyzw': list(frames.matrix_to_quat(rot)),
                    'r_mm': float(math.hypot(p[1], p[2])), 'axial_mm': float(p[0]),
                    'phi_deg': float(units.rad_to_deg(math.atan2(p[2], p[1])))})
    phis = np.unwrap([p['phi_deg'] for p in pts], period=360.0)
    for p, f in zip(pts, phis):
        p['phi_deg'] = float(f)
    coax = axis == 'x' and not any(offset)
    return {'seam_id': 'S01', 'closed': closed, 'length_mm': length, 'spacing_mm': length / n,
            'coaxial': {'is_coaxial': coax}, 'points': pts if closed else pts[: n // 2 + 1]}


def _check_path(res, cell, kin, params):
    q = np.array([p.q_deg for p in res.points])
    th = np.array([p.theta_deg for p in res.points])
    q7 = np.column_stack([q, th])
    steps = np.abs(np.diff(q7, axis=0))
    assert steps.max() <= params.dq_max_deg + 1e-9
    assert np.all(np.abs(q[:, 5] - q[0, 5]) <= params.wrist3_travel_deg + 1e-9)
    assert np.all(np.abs(q[:, 5]) <= params.wrist3_abs_deg + 1e-9)
    assert cell.theta_limits_deg[0] - 1e-9 <= th.min() and th.max() <= cell.theta_limits_deg[1] + 1e-9
    fk = batch_fk(q) @ cell.tool0_T_tcp
    for k, p in enumerate(res.points):
        want = tcp_target(cell, p.frame_flange, p.theta_deg, p.psi_deg)
        dp, dr = pose_error(fk[k], want)
        assert dp <= 0.01 and units.rad_to_deg(dr) <= 0.01
    assert all(p.status == 'ok' for p in res.points)
    assert len({p.label for p in res.points}) == 1


def test_rotation_model_point_at_theta_is_rotated_point():
    cell = _cell()
    f0 = frames.make_transform([3.0, 10.0, -4.0], frames.axis_angle_to_matrix([0, 1, 0], 0.3))
    for th in (0.0, 37.0, -200.0, 390.0):
        got = flange_point_at(cell, f0, th)
        want = cell.base_link_T_axis @ rot_x(th) @ f0
        assert np.allclose(got, want, atol=1e-12)
        # rotation about +X keeps x and the radius, adds theta to phi
        p = (frames.invert(cell.base_link_T_axis) @ got)[:3, 3]
        assert p[0] == pytest.approx(3.0)
        assert math.hypot(p[1], p[2]) == pytest.approx(math.hypot(10.0, -4.0))
        dphi = units.rad_to_deg(math.atan2(p[2], p[1]) - math.atan2(-4.0, 10.0))
        assert (dphi - th + 180.0) % 360.0 - 180.0 == pytest.approx(0.0, abs=1e-9)


def test_arc_geometry_closed_with_overlap_and_refinement():
    seam = _ring_seam()
    L = seam['length_mm']
    g = arc_geometry(seam, ArcSpec(1, 1, 0.0, L + 2.0, None), max_pitch_mm=seam['spacing_mm'])
    assert g.s[0] == pytest.approx(0.0) and g.s[-1] == pytest.approx(L + 2.0)
    assert np.all(np.diff(g.s) <= seam['spacing_mm'] + 1e-9)
    assert g.phi[-1] - g.phi[0] == pytest.approx(360.0 + units.rad_to_deg(2.0 / R_RING), abs=1e-6)
    # points on the ring (circle interpolation, frames orthonormal)
    for f in g.frames:
        assert math.hypot(f[1, 3], f[2, 3]) == pytest.approx(R_RING, abs=1e-9)
    g2 = arc_geometry(seam, ArcSpec(1, 1, 0.0, L + 2.0, None), max_pitch_mm=seam['spacing_mm'] / 4)
    assert len(g2.s) > 3 * len(g.s) and g2.interpolated.sum() > 0
    for f in g2.frames:
        assert math.hypot(f[1, 3], f[2, 3]) == pytest.approx(R_RING, abs=1e-9)
        z = f[:3, 2]
        assert np.allclose(z, [0.0, f[1, 3] / R_RING, f[2, 3] / R_RING], atol=1e-9)


@pytest.fixture(scope='module')
def ring_sync(kin):
    seam = _ring_seam()
    return plan_arc(kin, _cell(), seam, ArcSpec(1, 1, 0.0, seam['length_mm'] + 2.0, None), 'sync', SyncParams())


@pytest.mark.parametrize('mode', ['sync', 'C'])
def test_coaxial_ring_sync_and_c_are_100_percent(kin, mode, ring_sync):
    cell, params = _cell(), SyncParams()
    seam = _ring_seam()
    L = seam['length_mm']
    res = ring_sync if mode == 'sync' else plan_arc(kin, cell, seam, ArcSpec(1, 1, 0.0, L + 2.0, None), mode, params)
    assert res.ok, (res.reason, res.failing_ranges)
    assert res.points[-1].s_mm == pytest.approx(L + 2.0)
    _check_path(res, cell, kin, params)
    q = np.array([p.q_deg for p in res.points])
    th = np.array([p.theta_deg for p in res.points])
    if mode == 'C':
        assert np.ptp(q, axis=0).max() == 0.0                       # robot at rest
        assert abs(th[-1] - th[0]) == pytest.approx(360.0 + units.rad_to_deg(2.0 / R_RING), abs=1e-6)
    # CP-53 finding with theta free: the closed ring closes without a net 360 deg robot joint turn
    assert res.planning['closure'] is not None
    assert res.planning['closure']['dq_robot_max_deg'] < 360.0 - 1e-6
    assert abs(res.planning['closure']['dtheta_deg']) == pytest.approx(360.0, abs=5.0 + 1e-6)
    assert res.attempts[-1]['result'] == 'ok' and res.pitch_mm <= seam['spacing_mm'] + 1e-9


def test_pitch_refined_on_step_break(ring_sync):
    """CP-53 F1: at the 1.96 mm sample spacing the ring needs 7.5 deg per step; the pitch is halved and recorded."""
    att = ring_sync.attempts
    assert att[0]['result'] == 'rejected' and att[0]['status_counts'].get('step', 0) > 0
    assert ring_sync.planning['pitch_mm'] == pytest.approx(att[0]['pitch_mm'] / 2.0)
    assert ring_sync.planning['interpolated_points'] > 0


def test_sync_weld_point_kept_up(ring_sync):
    """Cost CP-59: on the coaxial ring the positioner keeps the seam Z close to world +Z."""
    params, res = SyncParams(), ring_sync
    assert res.ok
    assert res.planning['seam_z_up_max_deg'] <= params.theta_step_deg + 1e-6


def test_mode_b_uses_only_the_entered_angle(kin):
    cell, params = _cell(), SyncParams()
    seam = _ring_seam(closed=False)
    L = seam['points'][-1]['s_mm']
    res = plan_arc(kin, cell, seam, ArcSpec(1, 1, 0.0, L, 35.0), 'B', params)
    assert res.ok, (res.reason, res.failing_ranges)
    assert {p.theta_deg for p in res.points} == {35.0}
    _check_path(res, cell, kin, params)
    out = plan_arc(kin, _cell((-30.0, 30.0)), seam, ArcSpec(1, 1, 0.0, L, 35.0), 'B', params)
    assert not out.ok and out.failing_ranges[0]['status'] == 'limit'
    with pytest.raises(ValueError, match='positioner_angle'):
        plan_arc(kin, cell, seam, ArcSpec(1, 1, 0.0, L, None), 'B', params)


def test_mode_c_refused_for_a_non_coaxial_seam(kin):
    cell, params = _cell(), SyncParams()
    seam = _ring_seam(offset=(0.0, 40.0, 0.0), axis='z')
    res = plan_arc(kin, cell, seam, ArcSpec(1, 1, 0.0, seam['length_mm'] + 2.0, None), 'C', params)
    assert not res.ok and 'coaxial' in res.reason


def test_unreachable_arc_rejected_with_failing_ranges(kin):
    """No partial path: a ring 3 m away is rejected with its s range and status ik."""
    cell, params = _cell((-10.0, 10.0)), SyncParams()
    seam = _ring_seam(closed=False, offset=(3000.0, 0.0, 0.0))
    L = seam['points'][-1]['s_mm']
    res = plan_arc(kin, cell, seam, ArcSpec(1, 1, 0.0, L, None), 'sync', params)
    assert not res.ok and res.points == []
    assert res.failing_ranges[0]['status'] == 'ik'
    assert res.failing_ranges[0]['s_from_mm'] == pytest.approx(0.0)
    assert res.failing_ranges[-1]['s_to_mm'] == pytest.approx(L)
    assert res.coverage_fraction == 0.0


def test_sync_off_axis_ring_and_determinism(kin):
    """J2-like ring (axis perpendicular to the positioner axis, off-axis): planned or rejected with reasons; the same
    inputs give the identical result."""
    cell, params = _cell(), SyncParams()
    seam = _ring_seam(offset=(0.0, 0.0, 60.0), axis='z', n=36)
    arc = ArcSpec(1, 2, 0.0, seam['length_mm'] / 2 + 2.0, None)
    a = plan_arc(kin, cell, seam, arc, 'sync', params)
    b = plan_arc(kin, cell, seam, arc, 'sync', params)
    assert a.ok == b.ok and a.reason == b.reason
    assert [(p.theta_deg, p.psi_deg, tuple(p.q_deg)) for p in a.points] == \
           [(p.theta_deg, p.psi_deg, tuple(p.q_deg)) for p in b.points]
    if a.ok:
        _check_path(a, cell, kin, params)
    else:
        assert a.failing_ranges
