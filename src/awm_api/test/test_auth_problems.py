"""Login stub, roles (DES-001 §7.3, CP-21) and RFC 9457 problem details."""

import json

import pytest
from fastapi.testclient import TestClient

from awm_api.app import create_app
from awm_api.auth import UsersFileError, hash_password, load_users, verify_password
from conftest import auth, make_job


def test_password_hash_round_trip():
    h = hash_password('secret', iterations=1000)
    assert h.startswith('pbkdf2_sha256$1000$')
    assert verify_password('secret', h)
    assert not verify_password('Secret', h)


def test_users_file_refuses_plain_text_and_unknown_role(tmp_path):
    f = tmp_path / 'u.json'
    f.write_text(json.dumps({'users': [{'username': 'a', 'role': 'Engineer', 'password': 'plain'}]}))
    with pytest.raises(UsersFileError, match='pbkdf2'):
        load_users(f)
    f.write_text(json.dumps({'users': [{'username': 'a', 'role': 'Boss',
                                        'password': hash_password('x', iterations=1000)}]}))
    with pytest.raises(UsersFileError, match='role'):
        load_users(f)


def test_missing_users_file_is_a_clear_error(settings, tmp_path):
    settings.secrets_file = tmp_path / 'nope.json'
    with pytest.raises(UsersFileError, match='awm_api.users add'):
        create_app(settings)


def test_login_returns_token_and_role(client):
    r = client.post('/api/v1/auth/login', json={'username': 'eng', 'password': 'eng-pass'})
    assert r.status_code == 200
    body = r.json()
    assert body['role'] == 'Engineer' and body['token'] and body['expires_at']


def test_login_wrong_password_is_problem_401(client):
    r = client.post('/api/v1/auth/login', json={'username': 'eng', 'password': 'nope'})
    assert r.status_code == 401
    assert r.headers['content-type'].startswith('application/problem+json')
    p = r.json()
    assert p['status'] == 401 and p['type'].startswith('/problems/') and p['title']


def test_no_token_is_401_problem(client):
    r = client.get('/api/v1/jobs')
    assert r.status_code == 401
    assert r.headers['content-type'].startswith('application/problem+json')
    assert r.headers.get('www-authenticate') == 'Bearer'


def test_bad_token_is_401(client):
    r = client.get('/api/v1/jobs', headers={'Authorization': 'Bearer forged'})
    assert r.status_code == 401


def test_every_role_can_read_the_job_list(client):
    for user in ('op', 'eng', 'adm'):
        assert client.get('/api/v1/jobs', headers=auth(client, user)).status_code == 200


def test_seam_editing_is_engineer_only(client, repo):
    make_job(repo)
    body = {'seams': [{'seam_id': 'S01', 'edges': ['E1'], 'reversed': False, 'joint_type': 'butt',
                       'sequence': 1}]}
    for user in ('op', 'adm'):
        r = client.put('/api/v1/jobs/P1_A/seams', json=body, headers=auth(client, user))
        assert r.status_code == 403, user
        assert r.headers['content-type'].startswith('application/problem+json')


def test_operator_cannot_create_jobs_or_plan(client, repo):
    make_job(repo, placement=True)
    h = auth(client, 'op')
    r = client.post('/api/v1/jobs', headers=h, data={'part_no': 'X', 'part_rev': 'A'},
                    files={'step': ('x.step', b'ISO', 'application/octet-stream')})
    assert r.status_code == 403
    assert client.post('/api/v1/jobs/P1_A/plan', headers=h, json={}).status_code == 403


def test_unknown_api_path_is_problem_404(client):
    r = client.get('/api/v1/nothing', headers=auth(client, 'eng'))
    assert r.status_code == 404
    assert r.headers['content-type'].startswith('application/problem+json')


def test_request_validation_is_problem_422_with_pointers(client):
    r = client.post('/api/v1/auth/login', json={'username': 'eng'})
    assert r.status_code == 422
    p = r.json()
    assert p['status'] == 422 and any(e['pointer'] == '/password' for e in p['errors'])


def test_token_expiry(settings):
    settings.token_ttl_s = -1.0
    with TestClient(create_app(settings)) as c:
        r = c.post('/api/v1/auth/login', json={'username': 'eng', 'password': 'eng-pass'})
        token = r.json()['token']
        assert c.get('/api/v1/jobs', headers={'Authorization': f'Bearer {token}'}).status_code == 401
