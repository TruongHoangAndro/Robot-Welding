"""CP-64 /cell/home: taught cell home (D22), stale when cell/model/frames.yaml changes."""

import pytest
from conftest import auth

HOME = '/api/v1/cell/home'
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
          'wrist_3_joint']
BODY = {'robot_joints_deg': dict(zip(JOINTS, [0.0, -90.0, 90.0, -90.0, -90.0, 0.0])), 'positioner_joint_deg': 12.5}


@pytest.fixture
def frames(repo):
    p = repo / 'cell' / 'model' / 'frames.yaml'
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text('frames: v1\n')
    return p


def test_put_then_get_home(client, frames):
    h = auth(client, 'adm')
    r = client.put(HOME, headers=h, json=BODY)
    assert r.status_code in (200, 201), r.text
    g = client.get(HOME, headers=h)
    assert g.status_code == 200, g.text
    body = g.json()
    assert body['state'] == 'ok'
    assert body['home']['created_by'] == 'adm' and body['home']['source'] == 'taught'
    assert body['home']['robot_joints_deg'] == BODY['robot_joints_deg']
    assert body['home']['positioner_joint_deg'] == 12.5
    assert body['home']['inputs'][0]['name'] == 'cell/model/frames.yaml'


def test_get_home_404_when_absent(client, frames):
    r = client.get(HOME, headers=auth(client, 'adm'))
    assert r.status_code == 404
    assert r.headers['content-type'].startswith('application/problem+json')
    assert r.json()['type'].endswith('/problems/no-home')


def test_put_home_validates_joints(client, frames):
    h = auth(client, 'adm')
    bad = {'robot_joints_deg': {'shoulder_pan_joint': 0.0}, 'positioner_joint_deg': 0.0}
    assert client.put(HOME, headers=h, json=bad).status_code == 422
    assert client.put(HOME, headers=h, json={'robot_joints_deg': BODY['robot_joints_deg']}).status_code == 422
    assert client.get(HOME, headers=h).status_code == 404


def test_home_stale_when_frames_change(client, frames):
    h = auth(client, 'adm')
    assert client.put(HOME, headers=h, json=BODY).status_code in (200, 201)
    frames.write_text('frames: v2\n')
    assert client.get(HOME, headers=h).json()['state'] == 'stale'


def test_home_requires_admin(client, frames):
    for user in ('op', 'eng'):
        assert client.put(HOME, headers=auth(client, user), json=BODY).status_code == 403
    assert client.put(HOME, headers=auth(client, 'adm'), json=BODY).status_code != 403


def test_put_home_rejects_nan(client, frames):
    h = auth(client, 'adm')
    for bad in ('NaN', 'Infinity'):
        raw = ('{"robot_joints_deg": {' + ', '.join(f'"{j}": 0.0' for j in JOINTS[:5]) + f', "wrist_3_joint": {bad}}}, '
               '"positioner_joint_deg": 0.0}')
        r = client.put(HOME, headers={**h, 'Content-Type': 'application/json'}, content=raw)
        assert r.status_code == 422, r.text
    assert client.get(HOME, headers=h).status_code == 404


def test_put_home_no_frames_409(client):
    r = client.put(HOME, headers=auth(client, 'adm'), json=BODY)
    assert r.status_code == 409
    assert r.json()['type'].endswith('/problems/no-frames')


def test_get_home_corrupt_is_problem(client, frames, repo):
    h = auth(client, 'adm')
    (repo / 'cell' / 'model' / 'home.json').write_text('{not json')
    r = client.get(HOME, headers=h)
    assert r.status_code == 422 and r.json()['type'].endswith('/problems/home-invalid')
    (repo / 'cell' / 'model' / 'home.json').write_text('{"inputs": "x"}')
    assert client.get(HOME, headers=h).status_code == 422
