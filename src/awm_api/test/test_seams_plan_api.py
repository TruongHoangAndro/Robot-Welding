"""Seams GET / PUT through the seam CLI adapter (G1.2 not merged: fake), plan runs (G1.3 not written: fake)."""

import json
import os
import time

from conftest import auth, make_job, wait_run


def pick(seam_id='S01', **kw):
    p = {'seam_id': seam_id, 'edges': ['E1'], 'reversed': False, 'joint_type': 'butt', 'sequence': 1,
         'arcs': 1, 'arc_overlap_mm': 2.0, 'positioner_mode': 'sync', 'positioner_angles_deg': None,
         'start_offset_mm': 0.0}
    p.update(kw)
    return p


def put(client, h, *picks, job='P1_A'):
    return client.put(f'/api/v1/jobs/{job}/seams', headers=h, json={'seams': list(picks)})


def test_get_seams_404_before_saving(client, repo):
    make_job(repo)
    r = client.get('/api/v1/jobs/P1_A/seams', headers=auth(client, 'eng'))
    assert r.status_code == 404 and r.json()['type'] == '/problems/no-seams'


def test_put_seams_calls_cli_and_returns_document(client, repo, tmp_path, monkeypatch):
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    make_job(repo, placement=True)
    h = auth(client, 'eng')
    r = put(client, h, pick(), pick('S02', edges=['E4'], sequence=2, arcs=2, positioner_mode='B',
                                    positioner_angles_deg=[0.0, 180.0], reversed=True, start_offset_mm=5.0))
    assert r.status_code == 200, r.text
    doc = r.json()
    assert [s['seam_id'] for s in doc['seams']] == ['S01', 'S02']
    assert doc['seams'][1]['positioner_angles_deg'] == [0.0, 180.0]
    call = json.loads(log.read_text().splitlines()[-1])
    assert call[0] == 'build' and '--no-axis' not in call
    picks = json.loads(open(call[call.index('--picks') + 1]).read())
    assert picks['seams'][1]['reversed'] is True and picks['seams'][1]['start_offset_mm'] == 5.0
    # the G1.2 CLI refuses unknown request fields; range_mm (contract SeamPick) is sent only when set
    assert all('range_mm' not in s for s in picks['seams'])
    assert client.get('/api/v1/jobs/P1_A/seams', headers=h).json() == doc
    d = client.get('/api/v1/jobs/P1_A', headers=h).json()
    assert d['seam_axis'] == 'ok'


def test_put_seams_without_placement_skips_axis(client, repo, tmp_path, monkeypatch):
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    make_job(repo)
    h = auth(client, 'eng')
    r = put(client, h, pick())
    assert r.status_code == 200
    assert '--no-axis' in json.loads(log.read_text().splitlines()[-1])
    assert r.headers['x-awm-seam-axis'].startswith('not built')
    assert client.get('/api/v1/jobs/P1_A', headers=h).json()['seam_axis'] == 'missing'


def test_seam_axis_stale_after_seams_change(client, repo):
    job = make_job(repo, placement=True)
    h = auth(client, 'eng')
    assert put(client, h, pick()).status_code == 200
    seams = json.loads((job / 'seams.json').read_text())
    seams['seams'][0]['note'] = 'edited by hand'
    (job / 'seams.json').write_text(json.dumps(seams))
    assert client.get('/api/v1/jobs/P1_A', headers=h).json()['seam_axis'] == 'stale'
    r = client.post('/api/v1/jobs/P1_A/plan', headers=h, json={})
    assert r.status_code == 409 and r.json()['type'] == '/problems/seam-axis-stale'


def test_put_seams_validation_422(client, repo):
    make_job(repo, placement=True)
    h = auth(client, 'eng')
    cases = [
        (pick(seam_id='X1'), '/seams/0/seam_id'),
        (pick(edges=['F1']), '/seams/0/edges/0'),
        (pick(arcs=13), '/seams/0/arcs'),
        (pick(arc_overlap_mm=-1), '/seams/0/arc_overlap_mm'),
        (pick(positioner_mode='A'), '/seams/0/positioner_mode'),
        (pick(positioner_mode='B', arcs=2, positioner_angles_deg=[0.0]), '/seams/0/positioner_angles_deg'),
        (pick(positioner_mode='sync', positioner_angles_deg=[0.0]), '/seams/0/positioner_angles_deg'),
        (pick(start_offset_mm=-2), '/seams/0/start_offset_mm'),
        (pick(range_mm=[0.0, 10.0]), '/seams/0/range_mm'),  # contract field, not in D04 2.1: arcs + start offset
    ]
    for body, pointer in cases:
        r = put(client, h, body)
        assert r.status_code == 422, (body, r.text)
        assert pointer in [e['pointer'] for e in r.json()['errors']], (pointer, r.json())
    r = put(client, h, pick(), pick())
    assert r.status_code == 422 and 'duplicate' in r.text


def test_put_seams_cli_error_is_422_with_detail(client, repo):
    make_job(repo, placement=True)
    r = put(client, auth(client, 'eng'), pick(edges=['E999']))
    assert r.status_code == 422 and 'E999' in r.json()['detail']
    assert r.json()['type'] == '/problems/seam-build-failed'


def test_put_seams_while_run_active_409(client, repo, monkeypatch):
    make_job(repo, placement=True)
    h = auth(client, 'eng')
    assert put(client, h, pick()).status_code == 200
    monkeypatch.setenv('FAKE_SLEEP_S', '20')
    run_id = client.post('/api/v1/jobs/P1_A/plan', headers=h, json={}).json()['run_id']
    r = put(client, h, pick())
    assert r.status_code == 409 and r.json()['type'] == '/problems/run-busy'
    client.delete(f'/api/v1/runs/{run_id}', headers=h)
    wait_run(client, h, run_id)


def test_plan_preconditions(client, repo):
    make_job(repo)
    h = auth(client, 'eng')
    r = client.post('/api/v1/jobs/P1_A/plan', headers=h, json={})
    assert r.status_code == 409 and r.json()['type'] == '/problems/no-seams'
    assert put(client, h, pick()).status_code == 200
    r = client.post('/api/v1/jobs/P1_A/plan', headers=h, json={})
    assert r.status_code == 409 and r.json()['type'] == '/problems/no-placement'
    assert client.post('/api/v1/jobs/NO_A/plan', headers=h, json={}).status_code == 404


def test_plan_run_promotes_path_json(client, repo, tmp_path, monkeypatch):
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    job = make_job(repo, placement=True)
    h = auth(client, 'eng')
    put(client, h, pick(), pick('S02', sequence=2))
    r = client.post('/api/v1/jobs/P1_A/plan', headers=h, json={'seams': ['S02']})
    assert r.status_code == 202
    run = wait_run(client, h, r.json()['run_id'])
    assert run['state'] == 'succeeded' and run['kind'] == 'plan'
    assert list(run['outputs']) == ['path.json']
    assert json.loads((job / 'path.json').read_text())['collision_checked'] is False
    call = json.loads(log.read_text().splitlines()[-1])
    assert call[:5] == ['plan', '--job', str(job), '--collision', 'off']
    assert call[call.index('--seams') + 1:] == ['S02']


def test_plan_unknown_seam_422(client, repo):
    make_job(repo, placement=True)
    h = auth(client, 'eng')
    put(client, h, pick())
    r = client.post('/api/v1/jobs/P1_A/plan', headers=h, json={'seams': ['S07']})
    assert r.status_code == 422


def test_plan_busy_409_and_cancel_keeps_old_path(client, repo, monkeypatch):
    job = make_job(repo, placement=True)
    (job / 'path.json').write_text('{"old": true}')
    h = auth(client, 'eng')
    put(client, h, pick())
    monkeypatch.setenv('FAKE_SLEEP_S', '20')
    r1 = client.post('/api/v1/jobs/P1_A/plan', headers=h, json={})
    assert r1.status_code == 202
    r2 = client.post('/api/v1/jobs/P1_A/plan', headers=h, json={})
    assert r2.status_code == 409 and r2.json()['type'] == '/problems/run-busy'
    assert client.get('/api/v1/jobs/P1_A', headers=h).json()['active_run'] == r1.json()['run_id']
    assert client.delete(f'/api/v1/runs/{r1.json()["run_id"]}', headers=auth(client, 'op')).status_code == 403
    assert client.delete(f'/api/v1/runs/{r1.json()["run_id"]}', headers=h).status_code == 202
    assert wait_run(client, h, r1.json()['run_id'])['state'] == 'cancelled'
    assert json.loads((job / 'path.json').read_text()) == {'old': True}


def test_plan_progress_over_websocket(client, repo, monkeypatch):
    make_job(repo, placement=True)
    h = auth(client, 'eng')
    put(client, h, pick())
    monkeypatch.setenv('FAKE_SLEEP_S', '1')
    run_id = client.post('/api/v1/jobs/P1_A/plan', headers=h, json={}).json()['run_id']
    token = h['Authorization'].split()[1]
    states = []
    with client.websocket_connect(f'/api/v1/runs/{run_id}/ws?token={token}') as ws:
        while True:
            msg = ws.receive_json()
            states.append((msg['state'], msg['progress']))
            if msg['state'] in ('succeeded', 'failed', 'cancelled'):
                break
    assert states[-1] == ('succeeded', 1.0)
    progresses = [p for _, p in states]
    assert progresses == sorted(progresses) and len(states) >= 2


def test_websocket_refuses_bad_token(client, repo):
    import pytest
    from starlette.websockets import WebSocketDisconnect
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect('/api/v1/runs/R20261106T000000Z-abcdef/ws?token=bad') as ws:
            ws.receive_json()


def test_run_404(client):
    r = client.get('/api/v1/runs/R20261106T000000Z-abcdef', headers=auth(client, 'eng'))
    assert r.status_code == 404 and r.json()['type'] == '/problems/run-not-found'
