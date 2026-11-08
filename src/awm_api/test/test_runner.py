"""T2.7 planning-run runner: run folder, one slot per cell (409), cancel = SIGTERM, progress parsing."""

import os
import sys
import time
from pathlib import Path

import pytest

from awm_api.runner import RunBusyError, RunManager, Step, parse_progress

FAKE = str(Path(__file__).resolve().parent / 'fake_cli.py')


def wait(mgr, run_id, timeout_s=30.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        run = mgr.get(run_id)
        if run['state'] in ('succeeded', 'failed', 'cancelled'):
            return run
        time.sleep(0.02)
    raise AssertionError('run did not finish')


def step(*args, env=None, weight=1.0, label='fake'):
    return Step(label=label, argv=[sys.executable, FAKE, *args], env=env, weight=weight)


def test_parse_progress():
    assert parse_progress('[awm_path] progress 0.42 S01 point 3/7') == (0.42, 'S01 point 3/7')
    assert parse_progress('progress: 1 done') == (1.0, 'done')
    assert parse_progress('INFO something else') is None
    assert parse_progress('progress 7 bad') is None


def test_run_succeeds_writes_run_folder_and_outputs(tmp_path):
    mgr = RunManager(tmp_path / 'runs')
    seen = {}

    def on_success(run_dir: Path):
        seen['dir'] = run_dir
        return {'x.json': '0' * 64}

    run = mgr.start('P1_A', 'plan', [step('sleep')], on_success=on_success)
    assert run['run_id'].startswith('R') and run['state'] in ('queued', 'running')
    done = wait(mgr, run['run_id'])
    assert done['state'] == 'succeeded' and done['progress'] == 1.0
    assert done['outputs'] == {'x.json': '0' * 64}
    assert done['finished_at'] and done['problem'] is None
    run_dir = tmp_path / 'runs' / run['run_id']
    assert seen['dir'] == run_dir
    assert (run_dir / 'run.json').is_file() and (run_dir / 'log.txt').is_file()
    assert any('progress' in line for line in done['log_tail'])


def test_one_slot_per_cell_409(tmp_path):
    mgr = RunManager(tmp_path / 'runs')
    env = dict(os.environ, FAKE_SLEEP_S='5')
    run = mgr.start('P1_A', 'plan', [step('sleep', env=env)])
    with pytest.raises(RunBusyError) as ei:
        mgr.start('P2_A', 'import', [step('sleep')])
    assert ei.value.active_run == run['run_id']
    with pytest.raises(RunBusyError):
        with mgr.slot('seams', 'P1_A'):
            pass
    mgr.cancel(run['run_id'])
    wait(mgr, run['run_id'])
    with mgr.slot('seams', 'P1_A'):
        assert mgr.active_run() is not None
    assert mgr.active_run() is None


def test_cancel_sends_sigterm_and_never_promotes(tmp_path):
    mgr = RunManager(tmp_path / 'runs', kill_grace_s=2.0)
    called = []
    env = dict(os.environ, FAKE_SLEEP_S='30')
    run = mgr.start('P1_A', 'plan', [step('sleep', env=env)], on_success=lambda d: called.append(d) or {},
                    on_failure=lambda d: called.append(('fail', d)))
    time.sleep(0.5)
    t0 = time.monotonic()
    mgr.cancel(run['run_id'])
    done = wait(mgr, run['run_id'])
    assert time.monotonic() - t0 < 2.0, 'SIGTERM must stop the process without waiting for the grace period'
    assert done['state'] == 'cancelled'
    assert called == [('fail', tmp_path / 'runs' / run['run_id'])]
    assert done['outputs'] == {}


def test_failed_step_gives_problem_and_stops_chain(tmp_path):
    mgr = RunManager(tmp_path / 'runs')
    env = dict(os.environ, FAKE_FAIL='boom')
    run = mgr.start('P1_A', 'import', [step('sleep', env=env, label='first'), step('sleep', label='second')])
    done = wait(mgr, run['run_id'])
    assert done['state'] == 'failed'
    assert done['problem']['status'] == 422 or done['problem']['status'] == 500
    assert 'first' in done['problem']['detail'] and 'boom' in done['problem']['detail']
    assert done['step'].startswith('first')


def test_missing_program_fails_cleanly(tmp_path):
    mgr = RunManager(tmp_path / 'runs')
    run = mgr.start('P1_A', 'plan', [Step(label='planner', argv=['/nonexistent/awm_path_cli'])])
    done = wait(mgr, run['run_id'])
    assert done['state'] == 'failed' and 'cannot start' in done['problem']['detail']


def test_weighted_progress(tmp_path):
    mgr = RunManager(tmp_path / 'runs')
    run = mgr.start('P1_A', 'import', [step('sleep', weight=3.0, label='a'), step('sleep', weight=1.0, label='b')])
    done = wait(mgr, run['run_id'])
    assert done['progress'] == 1.0


def test_cancel_finished_run_is_conflict(tmp_path):
    mgr = RunManager(tmp_path / 'runs')
    run = mgr.start('P1_A', 'plan', [step('sleep')])
    wait(mgr, run['run_id'])
    from awm_api.runner import RunFinishedError
    with pytest.raises(RunFinishedError):
        mgr.cancel(run['run_id'])


def test_restart_marks_orphaned_runs_failed(tmp_path):
    mgr = RunManager(tmp_path / 'runs')
    env = dict(os.environ, FAKE_SLEEP_S='3')
    run = mgr.start('P1_A', 'plan', [step('sleep', env=env)])
    time.sleep(0.2)
    mgr2 = RunManager(tmp_path / 'runs')  # a new API process reading the same run folders
    got = mgr2.get(run['run_id'])
    assert got['state'] == 'failed' and 'restarted' in got['problem']['detail']
    mgr.cancel(run['run_id'])
    wait(mgr, run['run_id'])


# -- G1.20: result line and failure explanation (replay run) ------------------------------------------------------

def test_result_line_is_parsed_into_run_result(tmp_path):
    mgr = RunManager(tmp_path / 'runs')
    st = Step(label='replay', argv=[sys.executable, FAKE, 'replay', '--program', 'x.json'],
              env={**os.environ, 'FAKE_REPLAY': 'ok'}, result_prefix='replay')
    done = wait(mgr, mgr.start('P1_A', 'replay', [st])['run_id'])
    assert done['state'] == 'succeeded'
    assert done['result']['status'] == 'ok' and done['result']['final_deviation_deg'] == 0.0


def test_run_without_result_line_has_null_result(tmp_path):
    mgr = RunManager(tmp_path / 'runs')
    done = wait(mgr, mgr.start('P1_A', 'plan', [step('sleep')])['run_id'])
    assert done['result'] is None


def test_explain_builds_the_problem_of_a_failed_step(tmp_path):
    mgr = RunManager(tmp_path / 'runs')
    seen = {}

    def explain(rc, result, tail):
        seen.update(rc=rc, result=result, tail=list(tail))
        return {'type': '/problems/replay-refused', 'title': 'Replay refused', 'status': 409,
                'detail': f'explained {result["status"]}'}

    st = Step(label='replay', argv=[sys.executable, FAKE, 'replay', '--program', 'x.json'],
              env={**os.environ, 'FAKE_REPLAY': 'nomock'}, result_prefix='replay', explain=explain)
    done = wait(mgr, mgr.start('P1_A', 'replay', [st])['run_id'])
    assert done['state'] == 'failed' and seen['rc'] == 2
    assert done['problem'] == {'type': '/problems/replay-refused', 'title': 'Replay refused', 'status': 409,
                               'detail': 'explained refused'}
    assert done['result']['status'] == 'refused'
