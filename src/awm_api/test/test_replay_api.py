"""G1.20 (close-out B1): S4 "Replay in RViz" — awm_api runs ``replay_program --program <D09>`` as a run against the
user-launched sim_mock. Fake runner (fake_cli ``replay``); refusals mapped to clear messages; stale programs refused
before any process starts (awm_core.staleness); one slot per cell; cancel = SIGTERM."""

import json
import os

import pytest

from conftest import auth, wait_run
from test_g16_api import planned_job
from awm_api import replay as replay_mod
from awm_api.settings import Settings

LAUNCH_TAIL = 'trajectory_controllers:=robot_positioner'


def programmed_job(client, repo, h, job_id='P1_A'):
    job = planned_job(client, repo, h, job_id)
    run = wait_run(client, h, client.post(f'/api/v1/jobs/{job_id}/program', headers=h, json={}).json()['run_id'])
    assert run['state'] == 'succeeded', run
    return job


def start(client, h, job_id='P1_A', body=None):
    return client.post(f'/api/v1/jobs/{job_id}/replay', headers=h, json=body or {})


# -- launch line ---------------------------------------------------------------------------------------------------

def test_launch_line_names_repo_root_and_7_joint_controller(tmp_path):
    s = Settings(repo_root=tmp_path / 'my repo', secrets_file=tmp_path / 'u.json')
    line = s.replay_launch_line()
    assert line.startswith('ros2 launch awm_bringup sim_mock.launch.py repo:=')
    assert f"repo:='{(tmp_path / 'my repo').resolve()}'" in line and line.endswith(LAUNCH_TAIL)


def test_replay_cmd_from_env(monkeypatch, tmp_path):
    monkeypatch.setenv('AWM_REPLAY_CMD', 'my_replay --x')
    s = Settings.from_env(repo=str(tmp_path))
    assert s.replay_argv() == ['my_replay', '--x']
    monkeypatch.delenv('AWM_REPLAY_CMD')
    assert Settings.from_env(repo=str(tmp_path)).replay_argv() == ['ros2', 'run', 'awm_bringup', 'replay_program']


# -- refusal messages (pure) -----------------------------------------------------------------------------------------

LAUNCH = 'ros2 launch awm_bringup sim_mock.launch.py repo:=/r trajectory_controllers:=robot_positioner'


@pytest.mark.parametrize('rc, status, reason, cause, words', [
    (2, 'refused', '/controller_manager/list_hardware_components not available: cannot prove mock hardware',
     'sim-not-running', ['sim_mock is not running', LAUNCH]),
    (1, 'error', '/robot_positioner_trajectory_controller/follow_joint_trajectory not available (launch sim_mock with '
                 'trajectory_controllers:=robot_positioner)', 'controller-absent',
     ['trajectory_controllers:=robot_positioner', '7-joint']),
    (2, 'refused', 'replay runs on mock hardware only (ADR-13); not mock: ur (x/Y)', 'not-mock',
     ['not mock hardware', 'nothing was sent']),
    (2, 'refused', 'G001: current state is not the segment start (tolerance 0.05 deg): a 1 deg', 'start-state',
     ['not at the start of segment', 'nothing was sent']),
    (2, 'partial', 'G002: current state is not the segment start (tolerance 0.05 deg): a 1 deg', 'start-state',
     ['1 segment(s) were replayed', 'G002']),
    (1, 'error', "G001: controller error -4 'path tolerance violated'", 'error', ['path tolerance violated']),
])
def test_explain_maps_refusals(rc, status, reason, cause, words):
    result = {'status': status, 'reason': reason, 'segments': [{}] if status == 'partial' else []}
    p = replay_mod.explain(rc, result, [], LAUNCH, '48')
    assert p['cause'] == cause and p['replay_status'] == status
    assert p['status'] == (409 if rc == 2 else 422)
    for w in words:
        assert w in p['detail'], (w, p['detail'])


def test_explain_without_result_line():
    p = replay_mod.explain(1, None, ['Package awm_bringup not found'], LAUNCH, None)
    assert p['cause'] == 'no-result' and 'Package awm_bringup not found' in p['detail']


# -- endpoint ----------------------------------------------------------------------------------------------------------

def test_replay_ok_runs_cli_with_program_only(client, repo, tmp_path, monkeypatch):
    log = tmp_path / 'argv.log'
    h = auth(client, 'eng')
    job = programmed_job(client, repo, h)
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    r = start(client, h)
    assert r.status_code == 202, r.text
    assert r.headers['Location'] == f'/api/v1/runs/{r.json()["run_id"]}'
    run = wait_run(client, h, r.json()['run_id'])
    assert run['state'] == 'succeeded' and run['kind'] == 'replay'
    assert run['result']['status'] == 'ok' and run['result']['final_deviation_deg'] == 0.0
    assert run['result']['max_tracking_error_deg'] == 0.058
    assert any('weld events are not executed' in ln for ln in run['log_tail'])
    assert json.loads(log.read_text().splitlines()[-1]) == [
        'replay', '--program', str(job / 'programs' / 'P001' / 'program.json')]


@pytest.mark.parametrize('case, cause', [('nosim', 'sim-not-running'), ('noctl', 'controller-absent'),
                                         ('nomock', 'not-mock'), ('start', 'start-state'),
                                         ('partial', 'start-state')])
def test_replay_refusals_reach_the_run_problem(client, repo, monkeypatch, case, cause):
    h = auth(client, 'eng')
    programmed_job(client, repo, h)
    monkeypatch.setenv('FAKE_REPLAY', case)
    run = wait_run(client, h, start(client, h).json()['run_id'])
    assert run['state'] == 'failed'
    assert run['problem']['cause'] == cause and run['result']['status'] in ('refused', 'partial', 'error')
    assert run['problem']['type'].startswith('/problems/replay-')


def test_replay_roles(client, repo):
    h = auth(client, 'eng')
    programmed_job(client, repo, h)
    # Operator may not start a replay; Admin may (CP-63: Admin ⊇ Engineer).
    assert start(client, auth(client, 'op')).status_code == 403
    assert start(client, auth(client, 'adm')).status_code != 403


def test_replay_needs_a_program(client, repo):
    h = auth(client, 'eng')
    planned_job(client, repo, h)
    r = start(client, h)
    assert r.status_code == 409 and r.json()['type'] == '/problems/no-program'


def test_replay_unknown_rev_404_and_bad_rev_422(client, repo):
    h = auth(client, 'eng')
    programmed_job(client, repo, h)
    assert start(client, h, body={'program_rev': 'P009'}).status_code == 404
    assert start(client, h, body={'program_rev': '../x'}).status_code == 422


def test_replay_refuses_stale_program_before_any_process(client, repo, tmp_path, monkeypatch):
    log = tmp_path / 'argv.log'
    h = auth(client, 'eng')
    job = programmed_job(client, repo, h)
    (job / 'path.json').write_text((job / 'path.json').read_text() + ' ')  # D07 changed after the program
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    r = start(client, h)
    assert r.status_code == 409 and r.json()['type'] == '/problems/program-stale'
    assert any('path.json' in p for p in r.json()['problems'])
    assert not log.exists()


def test_replay_one_slot_and_cancel_sends_sigterm_to_the_cli(client, repo, monkeypatch):
    """Cancel = SIGTERM to the replay CLI process group: the run ends ``cancelled`` with no result. This does not
    mean the robot stopped: a segment already accepted by the sim_mock controller may still run (review I1); only a
    CLI result with status ``cancelled`` would confirm that the goal was cancelled."""
    h = auth(client, 'eng')
    programmed_job(client, repo, h)
    monkeypatch.setenv('FAKE_SLEEP_S', '20')
    rid = start(client, h).json()['run_id']
    busy = start(client, h)
    assert busy.status_code == 409 and busy.json()['type'] == '/problems/run-busy'
    assert client.delete(f'/api/v1/runs/{rid}', headers=h).status_code == 202
    run = wait_run(client, h, rid, timeout_s=15)
    assert run['state'] == 'cancelled'
    assert run['result'] is None  # the killed CLI confirmed nothing: not "stopped"


def test_simulation_payload_carries_replay_info(client, repo, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '48')
    h = auth(client, 'eng')
    job = programmed_job(client, repo, h)
    d = client.get('/api/v1/jobs/P1_A/simulation', headers=h).json()
    rp = d['replay']
    assert rp['launch_command'].endswith(LAUNCH_TAIL) and str(repo.resolve()) in rp['launch_command']
    assert rp['ros_domain_id'] == '48' and rp['program_rev'] == 'P001' and rp['program_state'] == 'ok'
    assert rp['problems'] == [] and d['program_problems'] == [] and d['path_problems'] == []
    (job / 'path.json').write_text((job / 'path.json').read_text() + ' ')
    d = client.get('/api/v1/jobs/P1_A/simulation', headers=h).json()
    assert d['replay']['program_state'] == 'stale' and d['replay']['problems']


# -- B2 M3: one staleness rule in the job detail --------------------------------------------------------------------

def test_job_detail_uses_core_staleness_rule(client, repo, tmp_path):
    from awm_api import jobs
    from awm_core import staleness
    assert jobs.document_state is staleness.document_state
    h = auth(client, 'eng')
    job = programmed_job(client, repo, h)
    d = client.get('/api/v1/jobs/P1_A', headers=h).json()
    assert d['path'] == 'ok' and d['path_problems'] == []
    assert d['program'] == {'rev': 'P001', 'state': 'ok', 'problems': []}
    # an input outside the repository (e.g. AWM_HEAD_DIR outside repo_root) is stale, not skipped
    outside = tmp_path / 'outside.yaml'
    outside.write_text('x')
    doc = json.loads((job / 'path.json').read_text())
    doc['inputs'].append({'name': '../../../outside.yaml', 'sha256': '0' * 64})
    (job / 'path.json').write_text(json.dumps(doc))
    d = client.get('/api/v1/jobs/P1_A', headers=h).json()
    assert d['path'] == 'stale' and any('outside the repository' in p for p in d['path_problems'])
    assert d['program']['state'] == 'stale'
    r = client.post('/api/v1/jobs/P1_A/program', headers=h, json={})
    assert r.status_code == 409 and any('outside the repository' in p for p in r.json()['problems'])


def test_replay_exit_0_without_result_line_fails(client, repo, monkeypatch):
    """Review m1: exit 0 but no ``replay {json}`` line is not a success."""
    h = auth(client, 'eng')
    programmed_job(client, repo, h)
    monkeypatch.setenv('FAKE_REPLAY', 'noresult')
    run = wait_run(client, h, start(client, h).json()['run_id'])
    assert run['state'] == 'failed' and run['result'] is None
    assert run['problem']['type'] == '/problems/replay-no-result' and 'exit 0' in run['problem']['detail']


@pytest.mark.parametrize('cmd', ['ros2 run awm_bringup replay_program --config /x.yaml',
                                 'replay_program --segments G002-G004', 'replay_program --config=/x.yaml',
                                 'replay_program --segments=G1'])
def test_replay_cmd_with_config_or_segments_refused_at_startup(tmp_path, monkeypatch, cmd):
    """Review m2: the API passes --program only; a prefix that changes the config or the segments is refused."""
    with pytest.raises(ValueError, match='AWM_REPLAY_CMD'):
        Settings(repo_root=tmp_path, secrets_file=tmp_path / 'u.json', replay_cmd=cmd)
    monkeypatch.setenv('AWM_REPLAY_CMD', cmd)
    with pytest.raises(ValueError, match='AWM_REPLAY_CMD'):
        Settings.from_env(repo=str(tmp_path))


# -- review m3: a program whose CP-62 override is void is refused like a stale one -----------------------------------

def _with_override(job):
    from awm_core.hashing import sha256_file
    (job / 'cad_check.json').write_text('{"status": "red"}')
    (job / 'cad_check_override.json').write_text('{"check_id": 4}')
    prog = job / 'programs' / 'P001' / 'program.json'
    doc = json.loads(prog.read_text())
    doc['cad_check_override'] = {'file': 'cad_check_override.json', 'sha256': sha256_file(job / 'cad_check_override.json'),
                                 'cad_check_sha256': sha256_file(job / 'cad_check.json'), 'check_id': 4}
    prog.write_text(json.dumps(doc))


@pytest.mark.parametrize('changed', ['cad_check.json', 'cad_check_override.json'])
def test_replay_refuses_void_cp62_override(client, repo, tmp_path, monkeypatch, changed):
    log = tmp_path / 'argv.log'
    h = auth(client, 'eng')
    job = programmed_job(client, repo, h)
    _with_override(job)
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    r = start(client, h)
    assert r.status_code == 202, r.text                         # override valid -> replay allowed
    wait_run(client, h, r.json()['run_id'])
    (job / changed).write_text('{"changed": true}')
    r = start(client, h)
    assert r.status_code == 409 and r.json()['type'] == '/problems/program-stale'
    assert any('CP-62 override' in p and changed in p for p in r.json()['problems'])
    d = client.get('/api/v1/jobs/P1_A/simulation', headers=h).json()
    assert d['replay']['program_state'] == 'stale' and d['replay']['ready'] is False
    assert any('CP-62 override' in p for p in d['program_problems'])
    assert client.get('/api/v1/jobs/P1_A', headers=h).json()['program']['state'] == 'stale'
    assert len(log.read_text().splitlines()) == 1               # only the first replay started a process


# -- review m7: named older revision, body fields, authentication ---------------------------------------------------

def test_replay_named_revision_is_checked_too(client, repo, tmp_path, monkeypatch):
    h = auth(client, 'eng')
    job = programmed_job(client, repo, h)                                  # P001
    (job / 'path.json').write_text((job / 'path.json').read_text() + ' ')   # P001 now stale
    run = wait_run(client, h, client.post('/api/v1/jobs/P1_A/program', headers=h, json={}).json()['run_id'])
    assert run['state'] == 'succeeded'                                      # P002 current
    r = start(client, h, body={'program_rev': 'P001'})
    assert r.status_code == 409 and r.json()['type'] == '/problems/program-stale'
    assert r.json()['program_rev'] == 'P001' and r.json()['problems']
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    r = start(client, h, body={'program_rev': 'P002'})
    assert r.status_code == 202, r.text
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    assert json.loads(log.read_text().splitlines()[-1])[-1].endswith('programs/P002/program.json')


@pytest.mark.parametrize('body', [{'segments': 'G002-G004'}, {'config': '/x.yaml'}, {'program_rev': 'P001', 'x': 1}])
def test_replay_body_accepts_program_rev_only(client, repo, body):
    h = auth(client, 'eng')
    programmed_job(client, repo, h)
    r = start(client, h, body=body)
    assert r.status_code == 422


def test_replay_needs_a_token(client, repo):
    h = auth(client, 'eng')
    programmed_job(client, repo, h)
    assert client.post('/api/v1/jobs/P1_A/replay', json={}).status_code == 401
    assert client.post('/api/v1/jobs/P1_A/replay', json={}, headers={'Authorization': 'Bearer nope'}).status_code == 401


def test_replay_bad_job_id_400_and_unknown_job_404(client):
    h = auth(client, 'eng')
    assert client.post('/api/v1/jobs/bad/replay', headers=h, json={}).status_code == 400
    assert client.post('/api/v1/jobs/X9_Z/replay', headers=h, json={}).status_code == 404
