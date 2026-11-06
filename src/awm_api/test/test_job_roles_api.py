"""CP-60 job API (S1 / S2): assembly import, roles (template pre-fill), positioner-axis pick, cross-check and its
gates on seam picking, Plan and the manual placement. Fake CLIs (no OCC)."""

import json

from conftest import auth, make_job, wait_run

from test_cell_model_api import argv_lines, tree


def pick(seam_id='S01'):
    return {'seam_id': seam_id, 'edges': ['E1'], 'reversed': False, 'joint_type': 'butt', 'sequence': 1}


def cad_job(repo, job_id='J1_A'):
    job = make_job(repo, job_id)
    (job / 'cache' / 'components.json').write_text(json.dumps(tree('Pos:1', 'Part:1', 'Base:1')))
    (job / 'cache' / 'view' / 'topology.json').write_text(json.dumps({'schema_version': '2.0',
                                                                     'source': {'up_axis': '+Y'}}))
    return job


ROLES = {'assignments': [{'path': 'Pos:1', 'role': 'rotary_fixture'}, {'path': 'Part:1', 'role': 'part'}]}


def checked_job(client, repo, monkeypatch, status='green', job_id='J1_A'):
    job = cad_job(repo, job_id)
    h = auth(client, 'eng')
    assert client.put(f'/api/v1/jobs/{job_id}/roles', headers=h, json=ROLES).status_code == 200
    r = client.put(f'/api/v1/jobs/{job_id}/axis', headers=h, json={'edge_id': 'E3'})
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    monkeypatch.setenv('FAKE_CHECK_STATUS', status)
    r = client.post(f'/api/v1/jobs/{job_id}/check', headers=h, json={})
    assert r.status_code == 202, r.text
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    return job, h


def test_create_job_with_assembly_flag_runs_components(client, repo, tmp_path, monkeypatch):
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    h = auth(client, 'eng')
    r = client.post('/api/v1/jobs', headers=h, files={'step': ('job.step', b'ISO-10303-21; job', 'application/step')},
                    data={'part_no': 'J2', 'part_rev': 'A', 'up_axis': '+Y', 'assembly': 'true'})
    assert r.status_code == 202, r.text
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    by_cmd = {a[0]: a for a in argv_lines(log)}
    assert '--components' in by_cmd['import'] and '--components' in by_cmd['export-view']
    assert by_cmd['inspect-assembly'][by_cmd['inspect-assembly'].index('--out') + 1].endswith('cache/components.json')
    assert (repo / 'jobs' / 'J2_A' / 'cache' / 'components.json').is_file()


def test_roles_prefilled_from_template(client, repo):
    cad_job(repo)
    (repo / 'cell' / 'roles_template.json').write_text(json.dumps({'version': 1, 'roles': {'Pos:1': 'rotary_fixture'}}))
    got = client.get('/api/v1/jobs/J1_A/roles', headers=auth(client, 'op')).json()
    assert got == {'assignments': [{'path': 'Pos:1', 'role': 'rotary_fixture'}], 'prefilled': True}


def test_roles_engineer_only_and_validated(client, repo):
    cad_job(repo)
    for user in ('op', 'adm'):
        assert client.put('/api/v1/jobs/J1_A/roles', headers=auth(client, user), json=ROLES).status_code == 403
    h = auth(client, 'eng')
    r = client.put('/api/v1/jobs/J1_A/roles', headers=h,
                   json={'assignments': [{'path': 'Pos:1', 'role': 'rotary_fixture'}]})
    assert r.status_code == 422 and 'part' in r.text
    r = client.put('/api/v1/jobs/J1_A/roles', headers=h, json=ROLES)
    assert r.status_code == 200 and r.json()['scope'] == 'job' and r.json()['source']['up_axis'] == '+Y'


def test_axis_put_runs_pick_with_components(client, repo, tmp_path, monkeypatch):
    cad_job(repo)
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    h = auth(client, 'eng')
    r = client.put('/api/v1/jobs/J1_A/axis', headers=h, json={'edge_id': 'E3', 'reverse': True})
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    (argv,) = argv_lines(log)
    assert argv[0] == 'pick-circle' and '--components' in argv and '--reverse' in argv
    assert argv[argv.index('--up-axis') + 1] == '+Y'


def test_check_needs_roles_and_axis(client, repo):
    cad_job(repo)
    r = client.post('/api/v1/jobs/J1_A/check', headers=auth(client, 'eng'), json={})
    assert r.status_code == 409 and 'roles.json' in r.text


def test_plan_blocked_when_check_red(client, repo, monkeypatch):
    _job, h = checked_job(client, repo, monkeypatch, 'red')
    r = client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]})
    assert r.status_code == 409 and r.json()['type'].endswith('cad-check-red')
    r = client.post('/api/v1/jobs/J1_A/plan', headers=h, json={})
    assert r.status_code == 409 and r.json()['type'].endswith('cad-check-red')
    assert client.get('/api/v1/jobs/J1_A/check', headers=h).json()['check']['status'] == 'red'


def test_plan_blocked_when_check_stale(client, repo, monkeypatch):
    job, h = checked_job(client, repo, monkeypatch, 'green')
    assert client.get('/api/v1/jobs/J1_A/check', headers=h).json()['state'] == 'ok'
    roles = dict(ROLES, assignments=ROLES['assignments'] + [{'path': 'Base:1', 'role': 'exclude'}])
    assert client.put('/api/v1/jobs/J1_A/roles', headers=h, json=roles).status_code == 200
    assert client.get('/api/v1/jobs/J1_A/check', headers=h).json()['state'] == 'stale'
    r = client.post('/api/v1/jobs/J1_A/plan', headers=h, json={})
    assert r.status_code == 409 and r.json()['type'].endswith('cad-check-stale')


def test_yellow_check_allows_seams(client, repo, monkeypatch):
    _job, h = checked_job(client, repo, monkeypatch, 'yellow')
    r = client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]})
    assert r.status_code == 200, r.text


def test_manual_placement_refused_for_job_cad(client, repo, monkeypatch):
    _job, h = checked_job(client, repo, monkeypatch, 'green')
    r = client.put('/api/v1/jobs/J1_A/placement', headers=h,
                   json={'flange_T_part': {'position_mm': [0, 0, 0], 'rpy_deg': [0, 0, 0]}})
    assert r.status_code == 409 and 'job_cad' in r.text


def test_job_without_roles_keeps_g16_flow(client, repo):
    make_job(repo, 'OLD_A')
    h = auth(client, 'eng')
    r = client.put('/api/v1/jobs/OLD_A/placement', headers=h,
                   json={'flange_T_part': {'position_mm': [0, 0, 0], 'rpy_deg': [0, 0, 0]}})
    assert r.status_code == 200, r.text
    assert client.put('/api/v1/jobs/OLD_A/seams', headers=h, json={'seams': [pick()]}).status_code == 200
