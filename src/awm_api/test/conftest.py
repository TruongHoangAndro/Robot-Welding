"""Shared fixtures: a temporary job repository, a users file and a FastAPI test client wired to fake CLIs."""

import json
import shlex
import sys
import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from awm_api.app import create_app
from awm_api.auth import hash_password
from awm_api.settings import Settings

HERE = Path(__file__).resolve().parent
FAKE = HERE / 'fake_cli.py'
PASSWORDS = {'op': 'op-pass', 'eng': 'eng-pass', 'adm': 'adm-pass'}
ROLES = {'op': 'Operator', 'eng': 'Engineer', 'adm': 'Admin'}


def fake_cmd() -> str:
    return f'{shlex.quote(sys.executable)} {shlex.quote(str(FAKE))}'


@pytest.fixture
def users_file(tmp_path: Path) -> Path:
    path = tmp_path / 'secrets' / 'users.secrets.json'
    path.parent.mkdir()
    path.write_text(json.dumps({'users': [
        {'username': u, 'role': ROLES[u], 'password': hash_password(p, iterations=1000)}
        for u, p in PASSWORDS.items()]}))
    return path


@pytest.fixture
def repo(tmp_path: Path) -> Path:
    root = tmp_path / 'repo'
    (root / 'jobs').mkdir(parents=True)
    (root / 'cell').mkdir()
    return root


@pytest.fixture
def settings(repo: Path, users_file: Path) -> Settings:
    return Settings(repo_root=repo, secrets_file=users_file, occ_python_cmd=fake_cmd(),
                    seam_cmd=fake_cmd(), plan_cmd=fake_cmd(), program_cmd=fake_cmd(), describe_cmd=fake_cmd(),
                    kill_grace_s=2.0, package_shares={'ur_description': repo.parent / 'share' / 'ur_description'})


@pytest.fixture
def client(settings: Settings):
    with TestClient(create_app(settings)) as c:
        yield c


def auth(client: TestClient, user: str) -> dict:
    r = client.post('/api/v1/auth/login', json={'username': user, 'password': PASSWORDS[user]})
    assert r.status_code == 200, r.text
    return {'Authorization': f'Bearer {r.json()["token"]}'}


def wait_run(client: TestClient, headers: dict, run_id: str, timeout_s: float = 60.0) -> dict:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        r = client.get(f'/api/v1/runs/{run_id}', headers=headers)
        assert r.status_code == 200, r.text
        run = r.json()
        if run['state'] in ('succeeded', 'failed', 'cancelled'):
            return run
        time.sleep(0.05)
    raise AssertionError(f'run {run_id} did not finish in {timeout_s} s')


def make_job(repo: Path, job_id: str = 'P1_A', placement: bool = False, view: bool = True) -> Path:
    """A job folder as left by import + export-view (fake contents)."""
    job = repo / 'jobs' / job_id
    (job / 'cache' / 'view').mkdir(parents=True)
    part_no, part_rev = job_id.rsplit('_', 1)
    (job / 'part.json').write_text(json.dumps({'schema_version': '1.0', 'part_no': part_no,
                                               'part_rev': part_rev}))
    (job / 'model.step').write_text('ISO-10303-21; fake')
    if view:
        (job / 'cache' / 'view' / 'view.glb').write_bytes(b'glTF-fake')
        (job / 'cache' / 'view' / 'topology.json').write_text('{"schema_version": "2.0"}')
    if placement:
        (job / 'placement.json').write_text(json.dumps({'schema_version': '2.0', 'source': 'fixture_cad',
                                                        'verified': False}))
    return job
