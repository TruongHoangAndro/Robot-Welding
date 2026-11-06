"""G1.6 API: plan travel speed, program runs (G1.5 CLI, fake), simulation payload, placement PUT / copy, robot
description with mesh URL rewrite and read-only mesh serving (traversal refused), roles."""

import json
import os
from pathlib import Path

import pytest

from conftest import auth, make_job, wait_run
from awm_core.hashing import sha256_file


def pick(seam_id='S01', **kw):
    p = {'seam_id': seam_id, 'edges': ['E1'], 'reversed': False, 'joint_type': 'butt', 'sequence': 1}
    p.update(kw)
    return p


def manual(position=(10.0, 20.0, -30.0), rpy=(90.0, 0.0, 0.0)):
    return {'flange_T_part': {'position_mm': list(position), 'rpy_deg': list(rpy)}}


def placed_job(client, repo, h, job_id='P1_A'):
    """Job with a placement made through the API, seams and seam_axis built."""
    job = make_job(repo, job_id)
    r = client.put(f'/api/v1/jobs/{job_id}/placement', headers=h, json=manual())
    assert r.status_code == 200, r.text
    r = client.put(f'/api/v1/jobs/{job_id}/seams', headers=h, json={'seams': [pick()]})
    assert r.status_code == 200, r.text
    return job


def planned_job(client, repo, h, job_id='P1_A'):
    job = placed_job(client, repo, h, job_id)
    run = wait_run(client, h, client.post(f'/api/v1/jobs/{job_id}/plan', headers=h, json={}).json()['run_id'])
    assert run['state'] == 'succeeded', run
    # the fake planner writes no inputs; give the path the inputs a real D07 records
    doc = json.loads((job / 'path.json').read_text())
    doc['inputs'] = [{'name': n, 'sha256': sha256_file(job / n)} for n in ('seam_axis.json', 'placement.json')]
    (job / 'path.json').write_text(json.dumps(doc))
    return job


def cell_model(repo: Path) -> Path:
    model = repo / 'cell' / 'model'
    (model / 'head').mkdir(parents=True)
    (model / 'frames.yaml').write_text('unit: "mm"\n')
    (model / 'head' / 'head.yaml').write_text('schema: "awm_head/0.1"\n')
    (model / 'head' / 'head_body.stl').write_bytes(b'solid head\nendsolid head\n')
    share = repo.parent / 'share' / 'ur_description' / 'meshes' / 'ur5e' / 'visual'
    share.mkdir(parents=True)
    (share / 'base.dae').write_text('<COLLADA/>')
    (repo.parent / 'share' / 'ur_description' / 'package.xml').write_text('<package/>')
    return model


# -- 1. plan travel speed ---------------------------------------------------------------------------------------

def test_plan_passes_travel_speed_and_head_dir(client, repo, tmp_path, monkeypatch):
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    h = auth(client, 'eng')
    placed_job(client, repo, h)
    r = client.post('/api/v1/jobs/P1_A/plan', headers=h, json={'travel_speed_mm_s': 3.5})
    assert r.status_code == 202, r.text
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    call = json.loads(log.read_text().splitlines()[-1])
    assert call[call.index('--travel-speed-mm-s') + 1] == '3.5'
    assert call[call.index('--head-dir') + 1] == str(repo / 'cell' / 'model' / 'head')


def test_plan_without_speed_uses_planner_default(client, repo, tmp_path, monkeypatch):
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    h = auth(client, 'eng')
    placed_job(client, repo, h)
    wait_run(client, h, client.post('/api/v1/jobs/P1_A/plan', headers=h, json={}).json()['run_id'])
    assert '--travel-speed-mm-s' not in json.loads(log.read_text().splitlines()[-1])


@pytest.mark.parametrize('speed', [0, -1.0, 'fast'])
def test_plan_bad_speed_422(client, repo, speed):
    h = auth(client, 'eng')
    placed_job(client, repo, h)
    r = client.post('/api/v1/jobs/P1_A/plan', headers=h, json={'travel_speed_mm_s': speed})
    assert r.status_code == 422
    assert '/travel_speed_mm_s' in [e['pointer'] for e in r.json()['errors']]


# -- 2. program runs ----------------------------------------------------------------------------------------------

def test_program_needs_path(client, repo):
    h = auth(client, 'eng')
    placed_job(client, repo, h)
    r = client.post('/api/v1/jobs/P1_A/program', headers=h, json={})
    assert r.status_code == 409 and r.json()['type'] == '/problems/no-path'


def test_program_run_succeeds(client, repo, tmp_path, monkeypatch):
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    h = auth(client, 'eng')
    job = planned_job(client, repo, h)
    assert client.post('/api/v1/jobs/P1_A/program', headers=auth(client, 'op'), json={}).status_code == 403
    r = client.post('/api/v1/jobs/P1_A/program', headers=h, json={})
    assert r.status_code == 202, r.text
    run = wait_run(client, h, r.json()['run_id'])
    assert run['state'] == 'succeeded' and run['kind'] == 'program'
    assert run['outputs'] == {'programs/P001/program.json': sha256_file(job / 'programs/P001/program.json')}
    assert json.loads(log.read_text().splitlines()[-1]) == ['generate', '--job', str(job)]
    assert client.get('/api/v1/jobs/P1_A', headers=h).json()['programs'] == ['P001']


def test_program_refusal_reaches_run_problem(client, repo, monkeypatch):
    h = auth(client, 'eng')
    job = planned_job(client, repo, h)
    monkeypatch.setenv('FAKE_REFUSE', 'travel speed 10 mm/s above the feasible maximum 6.98 mm/s (S01 arc 1)')
    run = wait_run(client, h, client.post('/api/v1/jobs/P1_A/program', headers=h, json={}).json()['run_id'])
    assert run['state'] == 'failed'
    assert 'feasible maximum 6.98 mm/s' in run['problem']['detail']
    assert not (job / 'programs').exists()


def test_program_refused_when_path_stale(client, repo):
    h = auth(client, 'eng')
    job = planned_job(client, repo, h)
    assert client.put('/api/v1/jobs/P1_A/placement', headers=h, json=manual(position=(1, 2, 3))).status_code == 200
    r = client.post('/api/v1/jobs/P1_A/program', headers=h, json={})
    assert r.status_code == 409 and r.json()['type'] == '/problems/path-stale'
    assert client.get('/api/v1/jobs/P1_A', headers=h).json()['path'] == 'stale'
    assert job.is_dir()


# -- 3. simulation payload ----------------------------------------------------------------------------------------

def test_simulation_404_without_path(client, repo):
    h = auth(client, 'eng')
    placed_job(client, repo, h)
    r = client.get('/api/v1/jobs/P1_A/simulation', headers=h)
    assert r.status_code == 404 and r.json()['type'] == '/problems/no-simulation'


def test_simulation_path_variant(client, repo):
    h = auth(client, 'eng')
    planned_job(client, repo, h)
    r = client.get('/api/v1/jobs/P1_A/simulation', headers=auth(client, 'op'))
    assert r.status_code == 200, r.text
    d = r.json()
    assert d['source'] == 'path' and d['program'] is None and d['program_rev'] is None
    assert d['path']['collision_checked'] is False and d['collision_checked'] is False
    assert d['path_state'] == 'ok' and d['placement']['source'] == 'study'
    assert d['seams']['seams'][0]['seam_id'] == 'S01'
    assert d['view_url'] == '/api/v1/jobs/P1_A/view/view.glb'
    assert d['robot_description_url'] == '/api/v1/cell/robot_description'


def test_simulation_program_variant_and_stale_fallback(client, repo):
    h = auth(client, 'eng')
    job = planned_job(client, repo, h)
    wait_run(client, h, client.post('/api/v1/jobs/P1_A/program', headers=h, json={}).json()['run_id'])
    d = client.get('/api/v1/jobs/P1_A/simulation', headers=h).json()
    assert d['source'] == 'program' and d['program_rev'] == 'P001' and d['program_state'] == 'ok'
    assert d['program']['cycle_time_s'] == 12.5 and d['path'] is not None
    # a re-planned path makes P001 stale: playback falls back to the path and says why
    doc = json.loads((job / 'path.json').read_text())
    doc['note'] = 're-planned'
    (job / 'path.json').write_text(json.dumps(doc))
    d = client.get('/api/v1/jobs/P1_A/simulation', headers=h).json()
    assert d['source'] == 'path' and d['program'] is None
    assert d['program_rev'] == 'P001' and d['program_state'] == 'stale'
    assert any('P001' in n for n in d['notes'])


def test_simulation_latest_current_program(client, repo):
    h = auth(client, 'eng')
    planned_job(client, repo, h)
    for _ in range(2):
        wait_run(client, h, client.post('/api/v1/jobs/P1_A/program', headers=h, json={}).json()['run_id'])
    assert client.get('/api/v1/jobs/P1_A/simulation', headers=h).json()['program_rev'] == 'P002'


# -- 5. placement -------------------------------------------------------------------------------------------------

def test_placement_manual_valid_v2(client, repo):
    from awm_core import jsonio
    make_job(repo)
    h = auth(client, 'eng')
    r = client.put('/api/v1/jobs/P1_A/placement', headers=h, json=manual())
    assert r.status_code == 200, r.text
    doc = r.json()
    jsonio.validate(doc, 'v2/placement')
    assert doc['source'] == 'study' and doc['verified'] is False
    assert 'Assumption' in doc['created_by']
    assert doc['flange_T_part']['position_mm'] == [10.0, 20.0, -30.0]
    q = doc['flange_T_part']['quat_xyzw']
    assert q == pytest.approx([2 ** -0.5, 0.0, 0.0, 2 ** -0.5])
    assert doc['objects'][0]['mesh']['file'] == 'cache/view/view.glb'
    assert client.get('/api/v1/jobs/P1_A/placement', headers=h).json() == doc
    d = client.get('/api/v1/jobs/P1_A', headers=h).json()
    assert d['placement'] == {'source': 'study', 'verified': False}


def test_placement_quaternion_input_and_refusals(client, repo):
    make_job(repo)
    h = auth(client, 'eng')
    ok = {'flange_T_part': {'position_mm': [0, 0, 0], 'quat_xyzw': [0, 0, 0, 2]}}
    r = client.put('/api/v1/jobs/P1_A/placement', headers=h, json=ok)
    assert r.status_code == 200 and r.json()['flange_T_part']['quat_xyzw'] == [0.0, 0.0, 0.0, 1.0]
    bad = [
        {},
        {'flange_T_part': {'position_mm': [0, 0], 'rpy_deg': [0, 0, 0]}},
        {'flange_T_part': {'position_mm': [0, 0, 0]}},
        {'flange_T_part': {'position_mm': [0, 0, 0], 'rpy_deg': [0, 0, 0], 'quat_xyzw': [0, 0, 0, 1]}},
        {'flange_T_part': {'position_mm': [0, 0, 0], 'quat_xyzw': [0, 0, 0, 0]}},
        {'flange_T_part': {'position_mm': [0, 0, 1e9], 'rpy_deg': [0, 0, 0]}},
        {**manual(), 'copy_from': 'P2_A'},
    ]
    for body in bad:
        r = client.put('/api/v1/jobs/P1_A/placement', headers=h, json=body)
        assert r.status_code == 422, (body, r.text)


def test_placement_roles_and_view_needed(client, repo):
    make_job(repo)
    make_job(repo, 'P3_A', view=False)
    assert client.put('/api/v1/jobs/P1_A/placement', headers=auth(client, 'op'), json=manual()).status_code == 403
    assert client.put('/api/v1/jobs/P1_A/placement', headers=auth(client, 'adm'), json=manual()).status_code == 403
    r = client.put('/api/v1/jobs/P3_A/placement', headers=auth(client, 'eng'), json=manual())
    assert r.status_code == 409 and r.json()['type'] == '/problems/no-view'


def test_placement_copy_from_other_job(client, repo):
    make_job(repo)
    make_job(repo, 'P2_A')
    h = auth(client, 'eng')
    r = client.put('/api/v1/jobs/P2_A/placement', headers=h, json={'copy_from': 'P1_A'})
    assert r.status_code == 409 and r.json()['type'] == '/problems/no-placement'
    src = client.put('/api/v1/jobs/P1_A/placement', headers=h, json=manual(position=(5, 6, 7))).json()
    r = client.put('/api/v1/jobs/P2_A/placement', headers=h, json={'copy_from': 'P1_A'})
    assert r.status_code == 200, r.text
    doc = r.json()
    assert doc['flange_T_part'] == src['flange_T_part'] and doc['source'] == 'study'
    assert 'P1_A' in doc['created_by']
    assert {'name': '../P1_A/placement.json', 'sha256': sha256_file(repo / 'jobs/P1_A/placement.json')} in doc['inputs']
    assert doc['objects'][0]['mesh']['sha256'] == sha256_file(repo / 'jobs/P2_A/cache/view/view.glb')
    for bad in ('P2_A', 'NOPE_A', '../x'):
        r = client.put('/api/v1/jobs/P2_A/placement', headers=h, json={'copy_from': bad})
        assert r.status_code in (400, 404, 422), (bad, r.text)


def test_placement_change_makes_seam_axis_stale(client, repo):
    h = auth(client, 'eng')
    placed_job(client, repo, h)
    assert client.get('/api/v1/jobs/P1_A', headers=h).json()['seam_axis'] == 'ok'
    r = client.put('/api/v1/jobs/P1_A/placement', headers=h, json=manual(rpy=(0, 0, 45)))
    assert r.status_code == 200 and r.headers['x-awm-seam-axis'] == 'stale'
    assert client.get('/api/v1/jobs/P1_A', headers=h).json()['seam_axis'] == 'stale'
    r = client.post('/api/v1/jobs/P1_A/plan', headers=h, json={})
    assert r.status_code == 409 and r.json()['type'] == '/problems/seam-axis-stale'


def test_placement_while_run_active_409(client, repo, monkeypatch):
    h = auth(client, 'eng')
    placed_job(client, repo, h)
    monkeypatch.setenv('FAKE_SLEEP_S', '20')
    run_id = client.post('/api/v1/jobs/P1_A/plan', headers=h, json={}).json()['run_id']
    r = client.put('/api/v1/jobs/P1_A/placement', headers=h, json=manual())
    assert r.status_code == 409 and r.json()['type'] == '/problems/run-busy'
    client.delete(f'/api/v1/runs/{run_id}', headers=h)
    wait_run(client, h, run_id)


# -- 4. robot description + meshes --------------------------------------------------------------------------------

def test_robot_description_rewrites_mesh_urls(client, repo, tmp_path, monkeypatch):
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    cell_model(repo)
    r = client.get('/api/v1/cell/robot_description', headers=auth(client, 'op'))
    assert r.status_code == 200, r.text
    d = r.json()
    urdf = d['urdf']
    assert 'filename="/api/v1/cell/meshes/head/head_body.stl"' in urdf
    assert 'filename="/api/v1/cell/meshes/package/ur_description/meshes/ur5e/visual/base.dae"' in urdf
    assert 'file://' not in urdf and 'package://' not in urdf and str(repo) not in urdf
    assert d['head_model'] == 'hcad' and d['unit'] == 'm, rad (URDF, REP 103)'
    call = json.loads(log.read_text().splitlines()[-1])
    assert call[:3] == ['describe', '--frames', str(repo / 'cell/model/frames.yaml')]
    assert call[call.index('--head-dir') + 1] == str((repo / 'cell/model/head').resolve())
    # cached: a second request does not render again
    client.get('/api/v1/cell/robot_description', headers=auth(client, 'op'))
    assert len(log.read_text().splitlines()) == 1


def test_robot_description_needs_cell_model(client, repo):
    r = client.get('/api/v1/cell/robot_description', headers=auth(client, 'eng'))
    assert r.status_code == 409 and r.json()['type'] == '/problems/no-cell-model'


def test_robot_description_render_failure(client, repo, monkeypatch):
    cell_model(repo)
    monkeypatch.setenv('FAKE_FAIL', 'head.yaml tool0_T_tcp disagrees')
    r = client.get('/api/v1/cell/robot_description', headers=auth(client, 'eng'))
    assert r.status_code == 500 and r.json()['type'] == '/problems/description-failed'
    assert 'tool0_T_tcp' in r.json()['detail']


def test_meshes_served_read_only(client, repo):
    cell_model(repo)
    h = auth(client, 'op')
    client.get('/api/v1/cell/robot_description', headers=h)
    r = client.get('/api/v1/cell/meshes/head/head_body.stl', headers=h)
    assert r.status_code == 200 and r.content.startswith(b'solid head')
    r = client.get('/api/v1/cell/meshes/package/ur_description/meshes/ur5e/visual/base.dae', headers=h)
    assert r.status_code == 200 and r.content == b'<COLLADA/>'
    assert client.get('/api/v1/cell/meshes/head/head_body.stl').status_code == 401
    assert client.put('/api/v1/cell/meshes/head/head_body.stl', headers=h, content=b'x').status_code == 405


def test_mesh_traversal_refused(client, repo, tmp_path):
    model = cell_model(repo)
    h = auth(client, 'op')
    client.get('/api/v1/cell/robot_description', headers=h)
    outside = tmp_path / 'secret.stl'
    outside.write_text('secret')
    os.symlink(outside, model / 'head' / 'link.stl')
    (repo.parent / 'share' / 'other_pkg').mkdir(parents=True)
    (repo.parent / 'share' / 'other_pkg' / 'x.stl').write_text('x')
    cases = [
        '/api/v1/cell/meshes/head/%2e%2e/frames.yaml',
        '/api/v1/cell/meshes/head/%2E%2E%2Fframes.yaml',
        '/api/v1/cell/meshes/head/..%2F..%2Fsecrets%2Fusers.secrets.json',
        '/api/v1/cell/meshes/head/%2Fetc%2Fpasswd',
        '/api/v1/cell/meshes/head/head.yaml',                 # not a mesh file type
        '/api/v1/cell/meshes/head/link.stl',                  # symlink leaving the head folder
        '/api/v1/cell/meshes/package/other_pkg/x.stl',        # package not referenced by the description
        '/api/v1/cell/meshes/package/ur_description/%2e%2e/%2e%2e/secret.stl',
        '/api/v1/cell/meshes/package/ur_description/package.xml',
    ]
    for url in cases:
        r = client.get(url, headers=h)
        assert r.status_code in (403, 404), (url, r.status_code)
        assert r.headers['content-type'].startswith('application/problem+json'), url  # a problem, never a file
        assert r.content != b'secret' and b'pbkdf2' not in r.content
    assert client.get('/api/v1/cell/meshes/head/missing.stl', headers=h).status_code == 404
