"""CP-64 /cell/fk and /cell/ik: analytic kinematics in mm / deg, no ROS graph."""

import numpy as np
import pytest
from conftest import auth

from awm_api import jog
from awm_path.core.kinematics import UrAnalyticKinematics

FK, IK = '/api/v1/cell/fk', '/api/v1/cell/ik'
HEAD_YAML = '''schema: "awm_head/0.1"
unit: "mm"
frame: "tool0"
tool0_T_tcp_mm:
  xyz: [43.8, 0.0, 279.6]
  quat_xyzw: [0.0, 0.5, 0.0, 0.8660254037844386]
beam_axis_tool0: [0.8660254037844386, 0.0, 0.5]
'''
Q = [10.0, -80.0, 100.0, -110.0, 80.0, 20.0]


def put_head(repo):
    hd = repo / 'cell' / 'model' / 'head'
    hd.mkdir(parents=True, exist_ok=True)
    (hd / 'head.yaml').write_text(HEAD_YAML)


def tcp_pose(settings, q):
    t0 = UrAnalyticKinematics().fk(q)
    return jog.matrix_to_pose(t0 @ jog.tool0_T_tcp(settings))


def test_fk_round_trips_ik():
    kin = UrAnalyticKinematics()
    sols = kin.ik(kin.fk(Q))
    assert any(np.abs(np.asarray(s) - Q).max() < 1e-3 for s in sols)


def test_pose_matrix_round_trip():
    T = UrAnalyticKinematics().fk(Q)
    assert np.allclose(jog.pose_to_matrix(jog.matrix_to_pose(T)), T, atol=1e-6)


def test_fk_endpoint_returns_both_frames(client, repo):
    h = auth(client, 'op')
    r = client.post(FK, headers=h, json={'joints_deg': Q})
    assert r.status_code == 200, r.text
    assert r.json()['tcp_laser'] is None
    assert np.all(np.isfinite(r.json()['tool0']['xyz_mm']))
    put_head(repo)
    r = client.post(FK, headers=h, json={'joints_deg': Q})
    body = r.json()
    assert np.all(np.isfinite(body['tool0']['xyz_mm'])) and np.all(np.isfinite(body['tcp_laser']['xyz_mm']))
    off = np.linalg.norm(np.asarray(body['tcp_laser']['xyz_mm']) - np.asarray(body['tool0']['xyz_mm']))
    assert abs(off - np.linalg.norm([43.8, 0.0, 279.6])) < 1e-6  # taught offset magnitude (rigid transform)
    assert len(body['tcp_laser']['quat_xyzw']) == 4


def wrapped_dist(q, cur):
    return float(np.abs(jog_wrap(np.asarray(q) - np.asarray(cur))).sum())


def jog_wrap(x):
    from awm_path.core.kinematics import wrap_deg
    return wrap_deg(x)


def test_ik_picks_nearest_to_current(client, repo, settings):
    put_head(repo)
    h = auth(client, 'adm')
    target = tcp_pose(settings, Q)
    kin = UrAnalyticKinematics()
    ok = [np.asarray(s) for s in kin.ik(kin.fk(Q)) if jog.wrist_singular(s) is None]
    assert len(ok) >= 2
    cases = [ok[0] + 0.6 * jog_wrap(ok[1] - ok[0]), ok[-1] + 3.0, ok[1] - 2.0]  # midway between two branches, and near others
    picked = set()
    for cur in cases:
        r = client.post(IK, headers=h, json={'target_tcp_laser': target, 'current_joints_deg': cur.tolist()})
        assert r.status_code == 200, r.text
        got = np.asarray(r.json()['joints_deg'])
        best = min(ok, key=lambda s: wrapped_dist(s, cur))
        assert wrapped_dist(got, best) < 1e-3
        picked.add(int(np.argmin([wrapped_dist(got, s) for s in ok])))
    assert len(picked) >= 2 and picked != {0}  # the nearest is not always the first-listed solution


def test_ik_wrap_returns_current_side_value(client, repo, settings):
    put_head(repo)
    q = [10.0, -80.0, 100.0, -110.0, 80.0, -10.0]
    target = tcp_pose(settings, q)
    cur = [10.0, -80.0, 100.0, -110.0, 80.0, 350.0]
    r = client.post(IK, headers=auth(client, 'adm'), json={'target_tcp_laser': target, 'current_joints_deg': cur})
    assert r.status_code == 200, r.text
    got = r.json()['joints_deg']
    assert abs(got[5] - 350.0) < 1e-3, got  # not a 360 deg spin to -10
    assert np.abs(np.asarray(got) - cur).max() < 1e-3


def test_nearest_solution_respects_joint_limits():
    # wrist 3 at -300 (current 340): unwrapped value 340 + wrap(-300-340)= 60 stays in range, elbow limit +-180 keeps q
    out = jog.nearest_solution([[0, 0, 170.0, 0, 0, 0]], [0, 0, -175.0, 0, 0, 0])
    assert out[2] == 170.0  # -175 + wrap(345) = -190 is past the elbow limit: keep the in-range value


def test_ik_out_of_reach(client, repo):
    put_head(repo)
    r = client.post(IK, headers=auth(client, 'adm'), json={
        'target_tcp_laser': {'xyz_mm': [5000.0, 0.0, 0.0], 'quat_xyzw': [0, 0, 0, 1]}, 'current_joints_deg': Q})
    assert r.status_code == 422 and r.json()['type'] == '/problems/ik-unreachable', r.text


def test_ik_reports_singular(client, repo, settings):
    put_head(repo)
    target = tcp_pose(settings, [10.0, -80.0, 0.5, -110.0, 80.0, 20.0])  # arm stretched: both IK solutions inside the F6 margin
    r = client.post(IK, headers=auth(client, 'adm'), json={'target_tcp_laser': target, 'current_joints_deg': Q})
    assert r.status_code == 422 and r.json()['type'] == '/problems/ik-singular', r.text


def test_ik_no_tcp_409(client):
    r = client.post(IK, headers=auth(client, 'adm'), json={
        'target_tcp_laser': {'xyz_mm': [400.0, 0.0, 400.0], 'quat_xyzw': [0, 0, 0, 1]}, 'current_joints_deg': Q})
    assert r.status_code == 409 and r.json()['type'] == '/problems/ik-no-tcp', r.text


def test_ik_requires_admin(client, repo, settings):
    put_head(repo)
    body = {'target_tcp_laser': tcp_pose(settings, Q), 'current_joints_deg': Q}
    for user in ('op', 'eng'):
        assert client.post(IK, headers=auth(client, user), json=body).status_code == 403
    assert client.post(IK, headers=auth(client, 'adm'), json=body).status_code != 403
