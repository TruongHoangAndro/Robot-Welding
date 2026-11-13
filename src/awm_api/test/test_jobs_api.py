"""Jobs: STEP upload as an import run (fake OCC CLI), list / detail, D20 view files, placement, files, simulation."""

import json
import os

from conftest import auth, make_job, wait_run

STEP = ('part.step', b'ISO-10303-21; fake step', 'application/octet-stream')


def upload(client, h, part_no='P9', part_rev='A', **extra):
    data = {'part_no': part_no, 'part_rev': part_rev, **extra}
    return client.post('/api/v1/jobs', headers=h, data=data, files={'step': STEP})


def test_create_job_runs_import_and_export_view(client, repo, tmp_path, monkeypatch):
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    h = auth(client, 'eng')
    r = upload(client, h, component='Assy-1', up_axis='+Y')
    assert r.status_code == 202, r.text
    run_id = r.json()['run_id']
    assert r.headers['location'] == f'/api/v1/runs/{run_id}'
    run = wait_run(client, h, run_id)
    assert run['state'] == 'succeeded', run
    assert run['kind'] == 'import' and run['job_id'] == 'P9_A'
    job = repo / 'jobs' / 'P9_A'
    assert (job / 'part.json').is_file() and (job / 'cache/view/view.glb').is_file()
    assert set(run['outputs']) >= {'part.json', 'cache/view/view.glb', 'cache/view/topology.json'}
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    assert [c[0] for c in calls] == ['import', 'export-view']
    assert calls[0][calls[0].index('--component') + 1] == 'Assy-1'
    assert '--up-axis=+Y' in calls[1]
    # no staging leftovers are listed as jobs
    assert [p.name for p in (repo / 'jobs').iterdir() if not p.name.startswith('.')] == ['P9_A']


def test_create_job_reports_no_placement(client, repo):
    h = auth(client, 'eng')
    run = wait_run(client, h, upload(client, h).json()['run_id'])
    assert run['state'] == 'succeeded'
    d = client.get('/api/v1/jobs/P9_A', headers=h).json()
    assert d['placement'] is None
    assert 'no placement' in d['placement_note'].lower()
    r = client.get('/api/v1/jobs/P9_A/placement', headers=h)
    assert r.status_code == 404 and r.json()['type'] == '/problems/no-placement'


def test_failed_import_leaves_no_job(client, repo, monkeypatch):
    monkeypatch.setenv('FAKE_FAIL', 'STEP has no solid')
    h = auth(client, 'eng')
    run = wait_run(client, h, upload(client, h).json()['run_id'])
    assert run['state'] == 'failed' and 'STEP has no solid' in run['problem']['detail']
    assert list((repo / 'jobs').iterdir()) == []


def test_create_job_bad_ids_400(client):
    h = auth(client, 'eng')
    r = upload(client, h, part_no='../evil')
    assert r.status_code == 400 and r.json()['errors'][0]['pointer'] == '/part_no'
    r = upload(client, h, up_axis='up')
    assert r.status_code == 400


def test_create_job_existing_409(client, repo):
    make_job(repo, 'P9_A')
    r = upload(client, auth(client, 'eng'))
    assert r.status_code == 409 and r.json()['type'] == '/problems/job-exists'


def test_admin_may_create_jobs(client):
    assert upload(client, auth(client, 'adm')).status_code == 202


def test_second_upload_while_busy_409_then_cancel(client, repo, monkeypatch):
    monkeypatch.setenv('FAKE_SLEEP_S', '20')
    h = auth(client, 'eng')
    run_id = upload(client, h).json()['run_id']
    r = upload(client, h, part_no='P8')
    assert r.status_code == 409 and r.json()['type'] == '/problems/run-busy'
    jobs = client.get('/api/v1/jobs', headers=h).json()
    assert [(j['job_id'], j['status']) for j in jobs] == [('P9_A', 'importing')]
    assert client.delete(f'/api/v1/runs/{run_id}', headers=h).status_code == 202
    run = wait_run(client, h, run_id)
    assert run['state'] == 'cancelled'
    assert list((repo / 'jobs').iterdir()) == []
    assert client.delete(f'/api/v1/runs/{run_id}', headers=h).status_code == 409


def test_job_list_and_detail(client, repo):
    make_job(repo, 'P1_A', placement=True)
    (repo / 'jobs' / 'P1_A' / 'path.json').write_text('{}')
    make_job(repo, 'P2_B')
    h = auth(client, 'op')
    jobs = client.get('/api/v1/jobs', headers=h).json()
    assert [(j['job_id'], j['part_no'], j['part_rev'], j['status']) for j in jobs] == [
        ('P1_A', 'P1', 'A', 'planned'), ('P2_B', 'P2', 'B', 'draft')]
    d = client.get('/api/v1/jobs/P1_A', headers=h).json()
    assert set(d['documents']) >= {'part.json', 'placement.json', 'path.json', 'cache/view/view.glb'}
    assert all(len(v) == 64 for v in d['documents'].values())
    assert d['placement'] == {'source': 'fixture_cad', 'verified': False}
    assert d['programs'] == [] and d['active'] == {'simulation': None, 'trial': None, 'production': None}
    assert d['active_run'] is None and d['seam_axis'] == 'missing'


def test_unknown_job_404(client):
    h = auth(client, 'eng')
    for url in ('/api/v1/jobs/NO_A', '/api/v1/jobs/NO_A/seams', '/api/v1/jobs/NO_A/view/view.glb'):
        r = client.get(url, headers=h)
        assert r.status_code == 404 and r.json()['type'] == '/problems/job-not-found', url


def test_bad_job_id_rejected(client):
    r = client.get('/api/v1/jobs/..%2F..%2Fetc_A', headers=auth(client, 'eng'))
    assert r.status_code in (400, 404)


def test_view_files(client, repo):
    make_job(repo)
    h = auth(client, 'op')
    r = client.get('/api/v1/jobs/P1_A/view/view.glb', headers=h)
    assert r.status_code == 200 and r.content == b'glTF-fake'
    assert r.headers['content-type'] == 'model/gltf-binary'
    r = client.get('/api/v1/jobs/P1_A/view/topology.json', headers=h)
    assert r.status_code == 200 and r.json() == {'schema_version': '2.0'}
    assert client.get('/api/v1/jobs/P1_A/view/model.brep', headers=h).status_code in (404, 422)


def test_view_missing_404(client, repo):
    make_job(repo, view=False)
    r = client.get('/api/v1/jobs/P1_A/view/view.glb', headers=auth(client, 'eng'))
    assert r.status_code == 404 and r.json()['type'] == '/problems/view-not-found'


def test_files_endpoint_refuses_traversal(client, repo):
    job = make_job(repo)
    (job / 'cache' / 'collision.stl').write_bytes(b'solid x')
    (repo / 'secret.txt').write_text('no')
    h = auth(client, 'eng')
    assert client.get('/api/v1/jobs/P1_A/files/cache/collision.stl', headers=h).content == b'solid x'
    for bad in ('../../secret.txt', '..%2F..%2Fsecret.txt', '%2E%2E/%2E%2E/secret.txt'):
        r = client.get(f'/api/v1/jobs/P1_A/files/{bad}', headers=h)
        assert r.status_code in (400, 403, 404), bad  # httpx may normalise ../ away: then the job id is bad
        assert r.content != b'no' and r.headers['content-type'].startswith('application/problem+json')
    os.symlink(repo / 'secret.txt', job / 'cache' / 'link.txt')
    assert client.get('/api/v1/jobs/P1_A/files/cache/link.txt', headers=h).status_code == 403


def test_simulation_404_until_path_v2(client, repo):
    make_job(repo, placement=True)
    h = auth(client, 'eng')
    r = client.get('/api/v1/jobs/P1_A/simulation', headers=h)
    assert r.status_code == 404 and r.json()['type'] == '/problems/no-simulation'
    (repo / 'jobs/P1_A/path.json').write_text(json.dumps({'schema_version': '2.0', 'collision_checked': False}))
    r = client.get('/api/v1/jobs/P1_A/simulation', headers=h)
    assert r.status_code == 200
    assert r.json()['path']['collision_checked'] is False and r.json()['placement']['source'] == 'fixture_cad'


def test_simulation_refuses_v1_path(client, repo):
    make_job(repo, placement=True)
    (repo / 'jobs/P1_A/path.json').write_text(json.dumps({'schema_version': '1.0'}))
    assert client.get('/api/v1/jobs/P1_A/simulation', headers=auth(client, 'eng')).status_code == 404


def test_released_status(client, repo):
    job = make_job(repo, 'P3_A')
    (job / 'active_simulation.json').write_text(json.dumps({'program_rev': 'P001'}))
    (job / 'programs' / 'P001').mkdir(parents=True)
    d = client.get('/api/v1/jobs/P3_A', headers=auth(client, 'eng')).json()
    assert d['status'] == 'released_simulation' and d['programs'] == ['P001']
    assert d['active']['simulation'] == 'P001'


def test_delete_job_removes_it(client, repo):
    make_job(repo, 'P5_A')
    h = auth(client, 'eng')
    assert [j['job_id'] for j in client.get('/api/v1/jobs', headers=h).json()] == ['P5_A']
    r = client.delete('/api/v1/jobs/P5_A', headers=h)
    assert r.status_code == 204, r.text
    assert not (repo / 'jobs' / 'P5_A').exists()
    assert client.get('/api/v1/jobs', headers=h).json() == []


def test_delete_job_operator_forbidden(client, repo):
    make_job(repo, 'P5_A')
    r = client.delete('/api/v1/jobs/P5_A', headers=auth(client, 'op'))
    assert r.status_code == 403
    assert (repo / 'jobs' / 'P5_A').exists()


def test_delete_job_not_found_404(client, repo):
    r = client.delete('/api/v1/jobs/P5_A', headers=auth(client, 'eng'))
    assert r.status_code == 404 and r.json()['type'] == '/problems/job-not-found'


def test_delete_job_busy_409(client, repo):
    make_job(repo, 'P5_A')
    h = auth(client, 'eng')
    with client.app.state.runs.slot('plan', 'P5_A'):
        r = client.delete('/api/v1/jobs/P5_A', headers=h)
    assert r.status_code == 409 and r.json()['type'] == '/problems/run-busy'
    assert (repo / 'jobs' / 'P5_A').exists()


def test_static_front_end_served_with_spa_fallback(settings, tmp_path):
    from fastapi.testclient import TestClient
    from awm_api.app import create_app
    dist = tmp_path / 'dist'
    (dist / 'assets').mkdir(parents=True)
    (dist / 'index.html').write_text('<html>awm</html>')
    (dist / 'assets' / 'a.js').write_text('js')
    settings.web_dist = dist
    with TestClient(create_app(settings)) as c:
        assert c.get('/').text == '<html>awm</html>'
        assert c.get('/assets/a.js').text == 'js'
        assert c.get('/jobs/whatever').text == '<html>awm</html>'
        r = c.get('/api/v1/nothing')
        assert r.status_code in (401, 404) and r.headers['content-type'].startswith('application/problem+json')
