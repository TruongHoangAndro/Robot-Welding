"""G1.17 (close-out A3): the planner CLI and the shared staleness rule (``awm_core.staleness``).

* inputs that resolve outside the repository (a head folder linked out of it, ``--head-dir`` outside) are refused
  at plan time, nothing written;
* a D07 records ``planner.params.record_version`` and is ``ok`` by the shared rule right after planning;
* ``awm_path.cli check`` prints the shared verdict;
* guard: the description record of two synthetic heads (H-BOX and the ``write_synthetic_head_dir`` H-CAD head) is
  pinned to ``PLANNER_RECORD_VERSION`` by its key sets and by value (SHA-256 of the canonical record, location-free).
Needs the robot description (xacro, ament share): runs inside awm_dev (``scripts/dx``)."""
import contextlib
import io
import json
import os
import re
import shutil
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / 'awm_description' / 'test'))

from awm_core import staleness  # noqa: E402
from awm_core.staleness import PLANNER_RECORD_VERSION  # noqa: E402
from awm_path import cli  # noqa: E402
from g13_job import make_ring_job  # noqa: E402
from head_fixture import write_synthetic_head_dir  # noqa: E402

pytest.importorskip('xacro', reason='xacro not available (run inside awm_dev)')

#: Description records of the synthetic heads per PLANNER_RECORD_VERSION: key sets and SHA-256 of the canonical record
#: (``_record_sha256``). Any change of the rendered record (xacro, head.yaml reading, limits files, UR kinematics,
#: head geometry) fails the guard. The guard cannot tell whether the change alters what an existing D07 means: the
#: developer decides, and the only way to make the test pass again is a new PLANNER_RECORD_VERSION with a new entry
#: here (never edit an old entry). It covers the two synthetic heads only, not every possible head.yaml.
RECORD_FORMATS = {
    1: {
        'head': ['chain', 'conduit_source', 'conduit_source_assumption', 'contact_standoff_mm', 'nozzle_geometry',
                 'nozzle_link', 'nozzle_radius_mm', 'nozzle_tip_length_mm', 'robot_description_sha256', 'source',
                 'tool0_T_tcp_mm'],
        'description': ['acceleration_deg_s2', 'cell', 'frames_yaml_check', 'head', 'head_model',
                        'joint_limits_sha256', 'positioner_limits_sha256', 'robot_description_sha256',
                        'ur_kinematics_sha256', 'xacro_args'],
        'sha256': {'hbox': 'a49776c6631ce5b41b1de53cb60d889f5b006374148b9a611ad4d8877446a3f1',
                   'hcad': '443d1372b5c5e0b724c73dc6ae91caee86c7fd8f431e516c9845a1291f4d53da'},
    },
}


def _canonical(value):
    if isinstance(value, float):
        return round(value, 6) + 0.0
    if isinstance(value, dict):
        return {k: _canonical(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_canonical(v) for v in value]
    return value


def _record_sha256(desc, head_dir) -> str:
    """SHA-256 of the description record, floats rounded to 1e-6, independent of the checkout: the URDF hash is
    recomputed without XML comments (xacro writes the absolute path of the source file into one) and with the head
    folder replaced by its placeholder."""
    from awm_core.hashing import sha256_text
    from awm_path.description import location_free_urdf_sha256
    record = json.loads(json.dumps(desc.as_record(), default=str))
    urdf_sha = location_free_urdf_sha256(re.sub(r'<!--.*?-->', '', desc.urdf, flags=re.S), head_dir)
    record['robot_description_sha256'] = urdf_sha
    record['head']['robot_description_sha256'] = urdf_sha
    return sha256_text(json.dumps(_canonical(record), sort_keys=True, separators=(',', ':')))


def _run(args):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        rc = cli.main(args)
    return rc, out.getvalue(), err.getvalue()


def _plan(job_dir: Path, out: Path, *extra):
    return _run(['plan', '--job', str(job_dir), '--collision', 'off', '--out', str(out), *extra])


@pytest.fixture(scope='module')
def planned(tmp_path_factory):
    job = make_ring_job(tmp_path_factory.mktemp('repo'))
    rc, out, err = _plan(job.dir, job.path_json, '--head-model', 'hbox')
    assert rc == 0, out + err
    return job


def test_d07_records_the_planner_record_version(planned):
    doc = json.loads(planned.path_json.read_text())
    assert doc['planner']['params']['record_version'] == PLANNER_RECORD_VERSION
    v = staleness.document_verdict(planned.dir, 'path.json', planned.repo.root)
    assert v.state == 'ok', v.problems


def test_check_subcommand(planned, tmp_path):
    rc, out, err = _run(['check', '--job', str(planned.dir)])
    assert rc == 0 and out.strip().splitlines()[-1] == 'state ok', out + err
    doc = json.loads(planned.path_json.read_text())
    del doc['planner']['params']['record_version']
    old = tmp_path / 'old.json'
    old.write_text(json.dumps(doc))
    rc, out, err = _run(['check', '--job', str(planned.dir), '--path', str(old)])
    assert rc == 1 and out.strip().splitlines()[-1] == 'state stale'
    assert 'planned with an older planner; re-plan' in out
    rc, out, err = _run(['check', '--job', str(planned.dir), '--path', str(tmp_path / 'none.json')])
    assert rc == 1 and out.strip().splitlines()[-1] == 'state missing'


def _link_head_out(tmp_path: Path, job) -> Path:
    outside = write_synthetic_head_dir(tmp_path / 'elsewhere' / 'head')
    model = job.repo.cell_dir / 'model'
    model.mkdir(parents=True, exist_ok=True)
    os.symlink(outside, model / 'head')
    return outside


def test_head_folder_linked_out_of_the_repository_is_refused(tmp_path):
    job = make_ring_job(tmp_path / 'repo')
    _link_head_out(tmp_path, job)
    out = tmp_path / 'p.json'
    rc, stdout, err = _plan(job.dir, out, '--head-model', 'hcad')
    assert rc == 1 and 'outside the repository' in err and 'head.yaml' in err and 'cp -rL' in err, err
    assert not out.exists()


def test_head_dir_outside_the_repository_is_refused(tmp_path):
    job = make_ring_job(tmp_path / 'repo')
    head = write_synthetic_head_dir(tmp_path / 'elsewhere' / 'head')
    out = tmp_path / 'p.json'
    rc, stdout, err = _plan(job.dir, out, '--head-model', 'hcad', '--head-dir', str(head))
    assert rc == 1 and 'outside the repository' in err
    assert not out.exists()


def test_head_folder_linked_inside_the_repository_is_refused(tmp_path):
    """Review I1: the link could be retargeted later while the recorded target stays unchanged."""
    job = make_ring_job(tmp_path / 'repo')
    model = job.repo.cell_dir / 'model'
    real = write_synthetic_head_dir(model / 'heads' / 'v1')
    os.symlink(real, model / 'head')
    rc, stdout, err = _plan(job.dir, job.path_json, '--head-model', 'hcad')
    assert rc == 1 and 'symbolic link' in err and 'cp -rL' in err, err
    assert not job.path_json.exists()


def test_seam_axis_with_an_input_outside_the_repository_is_refused(tmp_path):
    job = make_ring_job(tmp_path / 'repo')
    pl = job.dir / 'placement.json'
    moved = tmp_path / 'elsewhere' / 'placement.json'
    moved.parent.mkdir(parents=True)
    shutil.move(pl, moved)
    os.symlink(moved, pl)
    rc, stdout, err = _plan(job.dir, tmp_path / 'p.json', '--head-model', 'hbox')
    assert rc == 1 and 'outside the repository' in err and 're-plan' in err
    assert not (tmp_path / 'p.json').exists()


def test_record_formats_are_pinned_to_the_record_version(planned):
    """Fails when the synthetic heads' description record changes without a PLANNER_RECORD_VERSION bump."""
    from awm_path.description import load_planning_description
    frames = planned.repo.cell_dir / 'model' / 'frames.yaml'
    head = write_synthetic_head_dir(planned.repo.cell_dir / 'model' / 'head_guard')
    expected = RECORD_FORMATS.get(PLANNER_RECORD_VERSION)
    assert expected is not None, 'add the record key sets of the new PLANNER_RECORD_VERSION to RECORD_FORMATS'
    for model, head_dir in (('hbox', None), ('hcad', str(head))):
        desc = load_planning_description(frames, model, head_dir)
        record = desc.as_record()
        got = {'head': sorted(desc.head.as_record()), 'description': sorted(record)}
        assert got == {k: expected[k] for k in ('head', 'description')}, (
            f'{model}: head / description record format changed: bump awm_core.staleness.PLANNER_RECORD_VERSION and '
            'add the new key sets')
        sha = _record_sha256(desc, head_dir)
        assert sha == expected['sha256'][model], (
            f'{model}: description record value changed ({sha}): if an older D07 is now wrong, bump '
            'awm_core.staleness.PLANNER_RECORD_VERSION and add a new RECORD_FORMATS entry with this value')
