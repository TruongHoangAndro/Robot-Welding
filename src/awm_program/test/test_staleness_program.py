"""G1.17 (close-out A3): ``awm_program.cli generate`` uses the shared staleness rule (``awm_core.staleness``), and every
checker — ``awm_path.cli check``, ``awm_core.staleness.document_state`` (what ``awm_api`` switches to) and
``awm_program.cli generate`` — gives the same verdict. Synthetic ring job (``awm_path/test/g13_job.py``).
Needs the robot description (xacro, ament share) and ur-analytic-ik: runs inside awm_dev (``scripts/dx``)."""
import contextlib
import io
import json
import os
import shutil
import sys
from pathlib import Path

import pytest

pytest.importorskip('xacro', reason='xacro not available (run inside awm_dev)')

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'awm_path' / 'test'))
sys.path.insert(0, str(ROOT / 'awm_description' / 'test'))

from awm_core import staleness  # noqa: E402
from g13_job import make_ring_job  # noqa: E402
from head_fixture import write_synthetic_head_dir  # noqa: E402


def _call(module, args):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        rc = module.main(args)
    return rc, out.getvalue(), err.getvalue()


def _plan(job, *extra):
    from awm_path import cli
    return _call(cli, ['plan', '--job', str(job.dir), '--collision', 'off', '--out', str(job.path_json),
                       '--travel-speed-mm-s', '3.0', *extra])


def _verdicts(job, path_file=None):
    """(awm_path check state, shared document_state, awm_program generate rc + text)."""
    from awm_path import cli as path_cli
    from awm_program import cli as program_cli
    extra = ['--path', str(path_file)] if path_file else []
    rc_c, out_c, _ = _call(path_cli, ['check', '--job', str(job.dir), *extra])
    shared = staleness.document_state(job.dir, str(path_file or job.path_json), job.repo.root)
    rc_g, out_g, err_g = _call(program_cli, ['generate', '--job', str(job.dir), *extra])
    return out_c.strip().splitlines()[-1].split()[1], shared, rc_g, out_g + err_g


def test_unchanged_inputs_ok_everywhere(tmp_path):
    job = make_ring_job(tmp_path / 'repo')
    rc, out, err = _plan(job, '--head-model', 'hbox')
    assert rc == 0, out + err
    check, shared, rc_g, text = _verdicts(job)
    assert (check, shared, rc_g) == ('ok', 'ok', 0), text
    assert staleness.document_state(job.dir, 'programs/P001/program.json', job.repo.root) == 'ok'


def test_missing_or_older_record_version_is_refused_everywhere(tmp_path):
    job = make_ring_job(tmp_path / 'repo')
    assert _plan(job, '--head-model', 'hbox')[0] == 0
    doc = json.loads(job.path_json.read_text())
    for value in (None, staleness.PLANNER_RECORD_VERSION - 1):
        if value is None:
            doc['planner']['params'].pop('record_version', None)
        else:
            doc['planner']['params']['record_version'] = value
        old = job.dir / 'path_old.json'
        old.write_text(json.dumps(doc))
        check, shared, rc_g, text = _verdicts(job, old)
        assert (check, shared, rc_g) == ('stale', 'stale', 1), text
        assert 'planned with an older planner; re-plan' in text
    assert not job.programs_dir.exists()


def test_program_from_an_older_d07_is_stale(tmp_path):
    job = make_ring_job(tmp_path / 'repo')
    assert _plan(job, '--head-model', 'hbox')[0] == 0
    from awm_program import cli as program_cli
    assert _call(program_cli, ['generate', '--job', str(job.dir)])[0] == 0
    # simulate a D07 written by an older planner whose program was generated before G1.17
    doc = json.loads(job.path_json.read_text())
    del doc['planner']['params']['record_version']
    job.path_json.write_text(json.dumps(doc))
    prog = json.loads(job.program_json('P001').read_text())
    from awm_core.hashing import sha256_file
    prog['inputs'][0]['sha256'] = sha256_file(job.path_json)
    job.program_json('P001').write_text(json.dumps(prog))
    v = staleness.document_verdict(job.dir, 'programs/P001/program.json', job.repo.root)
    assert v.state == 'stale' and any('older planner' in p for p in v.problems), v.problems


def test_symlinked_head_folder_same_verdict_everywhere(tmp_path):
    """Audit M3: a head folder that leaves the repository through a link → stale for every checker."""
    job = make_ring_job(tmp_path / 'repo')
    head = job.repo.cell_dir / 'model' / 'head'
    write_synthetic_head_dir(head)
    rc, out, err = _plan(job, '--head-model', 'hcad')
    assert rc in (0, 2), out + err
    assert staleness.document_state(job.dir, 'path.json', job.repo.root) == 'ok'
    # the head folder now lives elsewhere and is linked back (golden_repo_g1 layout), contents unchanged
    outside = tmp_path / 'elsewhere' / 'head'
    outside.parent.mkdir(parents=True)
    shutil.move(str(head), str(outside))
    os.symlink(outside, head)
    check, shared, rc_g, text = _verdicts(job)
    assert (check, shared, rc_g) == ('stale', 'stale', 1), text
    assert 'outside the repository' in text and 're-plan' in text
    assert not job.programs_dir.exists()
