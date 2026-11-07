"""CP-62: recorded Admin override of CP-60 cross-check 4 for kinematic-only planning (POST / GET / DELETE
/jobs/{id}/check/override), its effect on the seam / Plan gate and its record in D07 / D09. Fake CLIs (no OCC)."""

import hashlib
import json

from awm_core import jsonio

from conftest import auth, wait_run

from test_job_roles_api import ROLES, checked_job, pick

URL = '/api/v1/jobs/J1_A/check/override'
REASON = {'reason': 'OI-23: jig bearing seats and belts not separated in the cell CAD; kinematic planning only'}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def overridden_job(client, repo, monkeypatch):
    job, h = checked_job(client, repo, monkeypatch, 'red4')
    r = client.post(URL, headers=auth(client, 'adm'), json=REASON)
    assert r.status_code == 201, r.text
    return job, h


def plan(client, h, **body):
    return client.post('/api/v1/jobs/J1_A/plan', headers=h, json=body)


def test_red4_without_override_blocks_with_cad_cause(client, repo, monkeypatch):
    _job, h = checked_job(client, repo, monkeypatch, 'red4')
    r = client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]})
    assert r.status_code == 409 and r.json()['type'].endswith('cad-check-red')
    assert 'Jig:1' in r.json()['detail'] and 'CP-62' in r.json()['detail']


def test_post_override_writes_valid_record(client, repo, monkeypatch):
    job, _h = overridden_job(client, repo, monkeypatch)
    doc = json.loads((job / 'cad_check_override.json').read_text())
    jsonio.validate(doc, 'v2/cad_check_override')
    assert doc['by'] == 'adm' and doc['role'] == 'admin' and doc['check_id'] == 4
    assert doc['scope'] == 'kinematic_planning' and doc['reason'] == REASON['reason']
    assert doc['cad_check_sha256'] == sha(job / 'cad_check.json')
    assert doc['inputs'] == [{'name': 'cad_check.json', 'sha256': sha(job / 'cad_check.json')}]
    assert doc['findings'] == ['Jig:1', 'Timing Belts:1']
    got = client.get(URL, headers=auth(client, 'op')).json()
    assert got['valid'] is True and got['override']['by'] == 'adm' and got['problem'] is None
    # the cross-check itself stays red (CP-62 point 2)
    assert json.loads((job / 'cad_check.json').read_text())['status'] == 'red'


def test_override_allows_seams_and_kinematic_plan_and_is_recorded(client, repo, monkeypatch):
    job, h = overridden_job(client, repo, monkeypatch)
    r = client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]})
    assert r.status_code == 200, r.text
    r = plan(client, h, collision='off')
    assert r.status_code == 202, r.text
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    path = json.loads((job / 'path.json').read_text())
    assert path['collision_checked'] is False
    rec = json.loads((job / 'cad_check_override.json').read_text())
    assert path['cad_check_override'] == {'file': 'cad_check_override.json',
                                          'sha256': sha(job / 'cad_check_override.json'),
                                          'cad_check_sha256': sha(job / 'cad_check.json'), 'check_id': 4,
                                          'by': 'adm', 'created_at': rec['created_at'], 'reason': REASON['reason']}
    r = client.post('/api/v1/jobs/J1_A/program', headers=h, json={})
    assert r.status_code == 202, r.text
    run = wait_run(client, h, r.json()['run_id'])
    assert run['state'] == 'succeeded', run
    program = json.loads((job / 'programs' / 'P001' / 'program.json').read_text())
    assert program['cad_check_override'] == path['cad_check_override']
    assert run['outputs']['programs/P001/program.json'] == sha(job / 'programs' / 'P001' / 'program.json')


def test_plan_default_collision_is_off(client, repo, monkeypatch):
    _job, h = overridden_job(client, repo, monkeypatch)
    assert client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]}).status_code == 200
    r = plan(client, h)
    assert r.status_code == 202, r.text
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'


def test_collision_on_plan_refused_with_override(client, repo, monkeypatch):
    _job, h = overridden_job(client, repo, monkeypatch)
    assert client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]}).status_code == 200
    r = plan(client, h, collision='on')
    assert r.status_code == 409 and r.json()['type'].endswith('cad-check-override-kinematic-only')


def test_collision_on_refused_without_override(client, repo, monkeypatch):
    _job, h = checked_job(client, repo, monkeypatch, 'green')
    assert client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]}).status_code == 200
    r = plan(client, h, collision='on')
    assert r.status_code == 409 and r.json()['type'].endswith('collision-unavailable')


def test_override_void_after_check_change(client, repo, monkeypatch):
    job, h = overridden_job(client, repo, monkeypatch)
    monkeypatch.setenv('FAKE_CHECK_HITS', 'Jig:1')  # a re-run with other findings changes cad_check.json
    r = client.post('/api/v1/jobs/J1_A/check', headers=h, json={})
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    got = client.get(URL, headers=h).json()
    assert got['valid'] is False and 'changed' in got['problem']
    r = client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]})
    assert r.status_code == 409 and r.json()['type'].endswith('cad-check-override-void')
    r = plan(client, h)
    assert r.status_code == 409 and r.json()['type'].endswith('cad-check-override-void')
    # the Admin records it again for the new check
    assert client.post(URL, headers=auth(client, 'adm'), json=REASON).status_code == 201
    assert client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]}).status_code == 200


def test_stale_check_refuses_post_and_gate(client, repo, monkeypatch):
    _job, h = overridden_job(client, repo, monkeypatch)
    roles = dict(ROLES, assignments=ROLES['assignments'] + [{'path': 'Base:1', 'role': 'exclude'}])
    assert client.put('/api/v1/jobs/J1_A/roles', headers=h, json=roles).status_code == 200
    r = client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]})
    assert r.status_code == 409 and r.json()['type'].endswith('cad-check-stale')
    r = client.post(URL, headers=auth(client, 'adm'), json=REASON)
    assert r.status_code == 409 and r.json()['type'].endswith('cad-check-stale')


def test_other_red_check_refuses_post_and_gate(client, repo, monkeypatch):
    job, h = checked_job(client, repo, monkeypatch, 'red14')
    r = client.post(URL, headers=auth(client, 'adm'), json=REASON)
    assert r.status_code == 409 and r.json()['type'].endswith('override-not-applicable')
    assert 'check 1' in r.json()['detail']
    # a hand-made override on such a check is still refused by the gate
    doc = jsonio.make_header('test', [job / 'cad_check.json'], job)
    doc.update({'schema_version': '2.0', 'by': 'adm', 'role': 'admin', 'reason': 'x', 'check_id': 4, 'scope': 'kinematic_planning',
                'cad_check_sha256': sha(job / 'cad_check.json'), 'findings': ['Jig:1']})
    (job / 'cad_check_override.json').write_text(json.dumps(doc))
    r = client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]})
    assert r.status_code == 409 and r.json()['type'].endswith('cad-check-red')


def test_post_refused_when_check_4_not_red(client, repo, monkeypatch):
    for status in ('green', 'red'):
        checked_job(client, repo, monkeypatch, status, job_id=f'J{status}_A')
        r = client.post(f'/api/v1/jobs/J{status}_A/check/override', headers=auth(client, 'adm'), json=REASON)
        assert r.status_code == 409 and r.json()['type'].endswith('override-not-applicable'), (status, r.text)


def test_non_admin_refused(client, repo, monkeypatch):
    checked_job(client, repo, monkeypatch, 'red4')
    for user in ('eng', 'op'):
        assert client.post(URL, headers=auth(client, user), json=REASON).status_code == 403
    assert client.post(URL, headers=auth(client, 'adm'), json=REASON).status_code == 201
    for user in ('eng', 'op'):
        assert client.delete(URL, headers=auth(client, user)).status_code == 403


def test_reason_required(client, repo, monkeypatch):
    checked_job(client, repo, monkeypatch, 'red4')
    for body in ({}, {'reason': ''}, {'reason': '   '}):
        assert client.post(URL, headers=auth(client, 'adm'), json=body).status_code == 422, body


def test_delete_override(client, repo, monkeypatch):
    job, h = overridden_job(client, repo, monkeypatch)
    adm = auth(client, 'adm')
    assert client.delete(URL, headers=adm).status_code == 204
    assert not (job / 'cad_check_override.json').exists()
    assert client.delete(URL, headers=adm).status_code == 404
    assert client.get(URL, headers=h).status_code == 404
    r = client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]})
    assert r.status_code == 409 and r.json()['type'].endswith('cad-check-red')


def test_invalid_override_file_refused(client, repo, monkeypatch):
    job, h = overridden_job(client, repo, monkeypatch)
    doc = json.loads((job / 'cad_check_override.json').read_text())
    doc['scope'] = 'release'
    (job / 'cad_check_override.json').write_text(json.dumps(doc))
    assert client.get(URL, headers=h).json()['valid'] is False
    r = client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]})
    assert r.status_code == 409 and r.json()['type'].endswith('cad-check-override-invalid')


def test_override_busy_while_run_active(client, repo, monkeypatch):
    _job, h = checked_job(client, repo, monkeypatch, 'red4')
    monkeypatch.setenv('FAKE_SLEEP_S', '30')
    r = client.post('/api/v1/jobs/J1_A/check', headers=h, json={})
    assert r.status_code == 202
    try:
        r2 = client.post(URL, headers=auth(client, 'adm'), json=REASON)
        assert r2.status_code == 409 and r2.json()['type'].endswith('run-busy')
    finally:
        client.delete(f'/api/v1/runs/{r.json()["run_id"]}', headers=h)
        wait_run(client, h, r.json()['run_id'])


def test_green_check_has_no_override_record_in_path(client, repo, monkeypatch):
    job, h = checked_job(client, repo, monkeypatch, 'green')
    assert client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]}).status_code == 200
    r = plan(client, h)
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    assert 'cad_check_override' not in json.loads((job / 'path.json').read_text())


def test_override_change_during_plan_run_is_not_promoted(client, repo, monkeypatch):
    job, h = overridden_job(client, repo, monkeypatch)
    assert client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]}).status_code == 200
    monkeypatch.setenv('FAKE_SLEEP_S', '1.5')
    r = plan(client, h)
    assert r.status_code == 202, r.text
    # DELETE is refused while the run holds the slot; a change on disk is caught at promotion
    assert client.delete(URL, headers=auth(client, 'adm')).status_code == 409
    (job / 'cad_check_override.json').unlink()
    run = wait_run(client, h, r.json()['run_id'])
    assert run['state'] == 'failed' and 'override' in json.dumps(run['problem'])
    assert not (job / 'path.json').exists()


def test_reference_survives_delete_of_the_override(client, repo, monkeypatch):
    """Review I1: D07 / D09 keep who / when / reason after the override record is removed."""
    job, h = overridden_job(client, repo, monkeypatch)
    assert client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]}).status_code == 200
    assert wait_run(client, h, plan(client, h).json()['run_id'])['state'] == 'succeeded'
    assert client.delete(URL, headers=auth(client, 'adm')).status_code == 204
    ref = json.loads((job / 'path.json').read_text())['cad_check_override']
    assert ref['by'] == 'adm' and ref['reason'] == REASON['reason'] and ref['created_at']


def test_seam_gate_runs_inside_the_slot(client, repo, monkeypatch):
    """Minor 3: the gate is evaluated again while the seam save holds the run slot."""
    from awm_api import cad_api
    _job, h = overridden_job(client, repo, monkeypatch)
    seen, real = [], cad_api.cad_gate

    def spy(*a, **kw):
        active = client.app.state.runs.active()
        seen.append(active['kind'] if active else None)
        return real(*a, **kw)

    monkeypatch.setattr(cad_api, 'cad_gate', spy)
    assert client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]}).status_code == 200
    assert 'seams' in seen


def _plan_running(client, repo, monkeypatch):
    job, h = overridden_job(client, repo, monkeypatch)
    assert client.put('/api/v1/jobs/J1_A/seams', headers=h, json={'seams': [pick()]}).status_code == 200
    monkeypatch.setenv('FAKE_SLEEP_S', '1.5')
    r = plan(client, h)
    assert r.status_code == 202, r.text
    return job, h, r.json()['run_id']


def test_cad_check_change_during_plan_run_is_not_promoted(client, repo, monkeypatch):
    """Minor 3 / 8: cad_check.json rewritten during the run (same inputs, other findings) voids the override."""
    job, h, run_id = _plan_running(client, repo, monkeypatch)
    doc = json.loads((job / 'cad_check.json').read_text())
    doc['checks'][1]['values']['hits'] = doc['checks'][1]['values']['hits'][:1]
    (job / 'cad_check.json').write_text(json.dumps(doc))
    run = wait_run(client, h, run_id)
    assert run['state'] == 'failed' and 'void' in json.dumps(run['problem']), run
    assert not (job / 'path.json').exists()


def test_stale_check_during_plan_run_is_not_promoted(client, repo, monkeypatch):
    """Minor 3: an input of cad_check.json changing during the run (stale check) refuses the promotion."""
    job, h, run_id = _plan_running(client, repo, monkeypatch)
    (job / 'model.step').write_text('ISO-10303-21; changed')
    run = wait_run(client, h, run_id)
    assert run['state'] == 'failed' and 'run the cross-check again' in run['problem']['detail'], run
    assert not (job / 'path.json').exists()
