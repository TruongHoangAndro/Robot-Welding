"""awm_core.staleness (G1.17, close-out A3): one staleness rule for the planner CLI, awm_program and awm_api."""

import inspect
import json
import os
from pathlib import Path

import pytest

from awm_core import staleness
from awm_core.hashing import sha256_file
from awm_core.staleness import PLANNER_RECORD_VERSION, document_state, document_verdict


def _repo(tmp_path: Path):
    root = tmp_path / 'repo'
    job = root / 'jobs' / 'P1_A'
    head = root / 'cell' / 'model' / 'head'
    job.mkdir(parents=True)
    head.mkdir(parents=True)
    (job / 'seam_axis.json').write_text('{"a": 1}\n')
    (root / 'cell' / 'model' / 'frames.yaml').write_text('frames: 1\n')
    (head / 'head.yaml').write_text('head: 1\n')
    return root, job


def _entry(job: Path, rel: str):
    return {'name': rel, 'sha256': sha256_file(job / rel)}


def _d07(job: Path, version=PLANNER_RECORD_VERSION, head_rel='../../cell/model/head/head.yaml'):
    params = {'collision': 'off'}
    if version is not None:
        params['record_version'] = version
    doc = {'schema_version': '2.1', 'inputs': [_entry(job, 'seam_axis.json'),
                                                 _entry(job, '../../cell/model/frames.yaml'),
                                                 _entry(job, head_rel)],
           'planner': {'kinematics_backend': 'k', 'scene_backend': 's', 'params_sha256': '0' * 64,
                       'psi_case': 'free', 'head_model': 'cad', 'params': params},
           'seams': [], 'rejected': []}
    (job / 'path.json').write_text(json.dumps(doc))
    return doc


def test_unchanged_inputs_ok(tmp_path):
    root, job = _repo(tmp_path)
    _d07(job)
    v = document_verdict(job, 'path.json', root)
    assert v.state == 'ok' and v.problems == () and document_state(job, 'path.json', root) == 'ok'


def test_changed_and_missing_input_stale(tmp_path):
    root, job = _repo(tmp_path)
    _d07(job)
    (job / 'seam_axis.json').write_text('{"a": 2}\n')
    v = document_verdict(job, 'path.json', root)
    assert v.state == 'stale' and 'seam_axis.json: changed since this file was created' in v.problems
    (root / 'cell' / 'model' / 'frames.yaml').unlink()
    v = document_verdict(job, 'path.json', root)
    assert '../../cell/model/frames.yaml: missing' in v.problems
    assert 're-plan' in v.message()


def test_missing_and_unreadable_document(tmp_path):
    root, job = _repo(tmp_path)
    assert document_state(job, 'path.json', root) == 'missing'
    (job / 'path.json').write_text('{not json')
    assert document_state(job, 'path.json', root) == 'missing'


def test_symlink_leaving_repository_is_stale(tmp_path):
    root, job = _repo(tmp_path)
    other = tmp_path / 'other_repo' / 'head'
    other.mkdir(parents=True)
    (other / 'head.yaml').write_text('head: other\n')
    head = root / 'cell' / 'model' / 'head'
    for f in head.iterdir():
        f.unlink()
    head.rmdir()
    os.symlink(other, head)
    # the name a planner would have recorded before G1.17 (resolved, relative to the job folder)
    rel = os.path.relpath(other / 'head.yaml', job)
    _d07(job, head_rel=rel)
    v = document_verdict(job, 'path.json', root)
    assert v.state == 'stale'
    assert any('outside the repository' in p for p in v.problems), v.problems
    # the same answer for the logical (unresolved) name: the rule resolves symlinks
    _d07(job, head_rel='../../cell/model/head/head.yaml')
    v2 = document_verdict(job, 'path.json', root)
    assert v2.state == 'stale' and any('outside the repository' in p for p in v2.problems)
    assert staleness.linked_or_outside([head / 'head.yaml', job / 'seam_axis.json'], root) == [head / 'head.yaml']


def test_symlink_inside_repository_is_refused_and_stale(tmp_path):
    """Review I1: a link inside the repository can be retargeted later without any recorded hash changing."""
    root, job = _repo(tmp_path)
    real = root / 'cell' / 'model' / 'head_v1'
    head = root / 'cell' / 'model' / 'head'
    head.rename(real)
    os.symlink(real, head)
    assert staleness.linked_or_outside([head / 'head.yaml', job / 'seam_axis.json'], root) == [head / 'head.yaml']
    _d07(job)  # logical name through the link
    v = document_verdict(job, 'path.json', root)
    assert v.state == 'stale' and any('symbolic link' in p for p in v.problems), v.problems


def test_folder_replaced_by_a_link_after_planning_is_stale(tmp_path):
    root, job = _repo(tmp_path)
    _d07(job)
    head = root / 'cell' / 'model' / 'head'
    real = root / 'cell' / 'model' / 'head_v2'
    head.rename(real)
    os.symlink(real, head)
    assert document_state(job, 'path.json', root) == 'stale'


def test_repository_root_given_through_a_symlink(tmp_path):
    root, job = _repo(tmp_path)
    _d07(job)
    link = tmp_path / 'repo_link'
    os.symlink(root, link)
    assert document_state(link / 'jobs' / 'P1_A', 'path.json', link) == 'ok'


@pytest.mark.parametrize('version', [None, PLANNER_RECORD_VERSION - 1])
def test_missing_or_older_record_version_is_stale(tmp_path, version):
    root, job = _repo(tmp_path)
    _d07(job, version=version)
    v = document_verdict(job, 'path.json', root)
    assert v.state == 'stale'
    assert any('planned with an older planner; re-plan' in p for p in v.problems), v.problems


def test_newer_record_version_is_stale(tmp_path):
    root, job = _repo(tmp_path)
    _d07(job, version=PLANNER_RECORD_VERSION + 1)
    v = document_verdict(job, 'path.json', root)
    assert v.state == 'stale' and any('newer planner' in p for p in v.problems)


def test_non_d07_needs_no_record_version(tmp_path):
    root, job = _repo(tmp_path)
    doc = {'inputs': [_entry(job, 'seam_axis.json')]}
    assert document_state(job, 'other.json', root, doc) == 'ok'


def test_program_inherits_d07_staleness(tmp_path):
    root, job = _repo(tmp_path)
    _d07(job, version=None)
    prog = job / 'programs' / 'P001'
    prog.mkdir(parents=True)
    doc = {'program_rev': 'P001', 'inputs': [_entry(job, 'path.json'), _entry(job, 'seam_axis.json')]}
    (prog / 'program.json').write_text(json.dumps(doc))
    v = document_verdict(job, 'programs/P001/program.json', root)
    assert v.state == 'stale'
    assert any(p.startswith('path.json: ') and 'older planner' in p for p in v.problems), v.problems
    _d07(job)  # re-planned → the program input hash no longer matches
    v = document_verdict(job, 'programs/P001/program.json', root)
    assert v.state == 'stale' and 'path.json: changed since this file was created' in v.problems


def test_program_with_current_d07_ok(tmp_path):
    root, job = _repo(tmp_path)
    _d07(job)
    doc = {'program_rev': 'P001', 'inputs': [_entry(job, 'path.json')]}
    assert document_state(job, 'programs/P001/program.json', root, doc) == 'ok'


def test_absolute_document_name(tmp_path):
    root, job = _repo(tmp_path)
    _d07(job)
    assert document_state(job, str(job / 'path.json'), root) == 'ok'


def test_document_state_has_the_awm_api_signature():
    """The wave-B switch of awm_api.jobs.document_state is one import line."""
    params = list(inspect.signature(document_state).parameters)
    assert params == ['job', 'name', 'repo_root', 'doc']
    assert inspect.signature(document_state).parameters['doc'].default is None


def test_v2_path_document_without_planner_block_is_d07(tmp_path):
    """Review minor 2: a schema 2.x path document whose planner block is missing gets the version check."""
    root, job = _repo(tmp_path)
    doc = {'schema_version': '2.1', 'inputs': [_entry(job, 'seam_axis.json')], 'seams': [], 'rejected': []}
    v = document_verdict(job, 'path.json', root, doc)
    assert v.state == 'stale' and any('planned with an older planner; re-plan' in p for p in v.problems)


@pytest.mark.parametrize('entry', ['seam_axis.json', {'sha256': '0' * 64}, {'name': 'seam_axis.json'}, None])
def test_malformed_input_entry_is_a_problem(tmp_path, entry):
    """Review minor 3: never an AttributeError."""
    root, job = _repo(tmp_path)
    v = document_verdict(job, 'x.json', root, {'inputs': [entry]})
    assert v.state == 'stale' and v.problems == ('input #0: malformed input entry',)


def test_malformed_inputs_list_is_a_problem(tmp_path):
    root, job = _repo(tmp_path)
    v = document_verdict(job, 'x.json', root, {'inputs': 'seam_axis.json'})
    assert v.state == 'stale' and v.problems == ('inputs: malformed input list',)
