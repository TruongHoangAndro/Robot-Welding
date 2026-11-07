"""G1.5 release gate (CP-58: kinematic-only programs are refused by every release gate; ADR-17 scopes)."""
import copy
import json

import pytest

from awm_core import jsonio
from awm_verify.release import ReleaseError, release, release_gate

SCOPES = ('simulation', 'trial', 'production')


def _example():
    return json.loads((jsonio.SCHEMA_DIR / 'v2' / 'examples' / 'program.example.json').read_text())


def _checked(doc):
    doc = copy.deepcopy(doc)
    doc['collision_checked'] = True
    for seg in doc['segments']:
        seg['collision_checked'] = True
    for ev in doc['events']:
        ev['wps_id'], ev['preset'] = 'WPS-T', 1
    return doc


@pytest.mark.parametrize('scope', SCOPES)
def test_kinematic_only_program_refused_for_every_scope(scope):
    doc = _example()
    assert doc['collision_checked'] is False
    with pytest.raises(ReleaseError, match='collision_checked'):
        release_gate(doc, scope)


@pytest.mark.parametrize('scope', SCOPES)
def test_gate_refuses_unchecked_segment_missing_flag_and_missing_process(scope):
    ok = _checked(_example())
    release_gate(ok, scope)
    bad = copy.deepcopy(ok)
    bad['segments'][0]['collision_checked'] = False
    with pytest.raises(ReleaseError, match='G001'):
        release_gate(bad, scope)
    bad = copy.deepcopy(ok)
    del bad['collision_checked']
    with pytest.raises(ReleaseError, match='collision_checked'):
        release_gate(bad, scope)
    bad = copy.deepcopy(ok)
    bad['events'][0]['preset'] = None
    with pytest.raises(ReleaseError, match='process'):
        release_gate(bad, scope)


def test_unknown_scope_refused():
    with pytest.raises(ReleaseError, match='scope'):
        release_gate(_checked(_example()), 'demo')


def test_release_calls_the_gate_first(job):
    """A v2 kinematic-only program in the job: release() refuses it before any other check, nothing written."""
    from awm_verify.release import is_locked
    job.program_json('P001').write_text(json.dumps(_example()))
    for scope in ('simulation', 'production'):
        with pytest.raises(ReleaseError, match='collision_checked'):
            release(job, 'P001', 'A. Approver', scope)
    assert not job.release_json.exists() and not is_locked(job, 'P001')


# ---------------------------------------------------------------------------------------- CP-62 (review g1-18 I2)

def _override_ref():
    return {'file': 'cad_check_override.json', 'sha256': '1' * 64, 'cad_check_sha256': '2' * 64, 'check_id': 4,
            'by': 'adm', 'created_at': '2026-11-07T18:00:00+07:00', 'reason': 'OI-23: kinematic planning only'}


def _write_cad_check(job_dir, check4='green', status=None):
    """A schema-valid cad_check.json in ``job_dir`` whose inputs (one stub STEP) are current."""
    step = job_dir / 'model.step'
    step.write_text('stub step')
    doc = json.loads((jsonio.SCHEMA_DIR / 'v2' / 'examples' / 'cad_check.example.json').read_text())
    doc.update({k: v for k, v in jsonio.make_header('test', [step], job_dir).items()})
    doc['schema_version'] = '2.0'
    for c in doc['checks']:
        if c['id'] == 4:
            c['status'] = check4
    doc['status'] = status or ('red' if check4 == 'red' else 'yellow')
    jsonio.write_json(job_dir / 'cad_check.json', doc, 'v2/cad_check')
    return step


@pytest.mark.parametrize('scope', SCOPES)
def test_program_with_cp62_override_never_releasable(scope):
    doc = _example()
    doc['cad_check_override'] = _override_ref()
    with pytest.raises(ReleaseError, match='CP-62.*never releasable'):
        release_gate(doc, scope)
    forged = _checked(doc)                                 # schema-invalid forgery: override + collision_checked true
    with pytest.raises(ReleaseError, match='CP-62.*never releasable'):
        release_gate(forged, scope)


@pytest.mark.parametrize('scope', SCOPES)
def test_job_with_red_check_4_refused_with_or_without_override(tmp_path, scope):
    _write_cad_check(tmp_path, check4='red')
    ok = _checked(_example())
    with pytest.raises(ReleaseError, match='check 4'):
        release_gate(ok, scope, job_dir=tmp_path)
    (tmp_path / 'cad_check_override.json').write_text('{}')  # an override in the job changes nothing
    with pytest.raises(ReleaseError, match='check 4'):
        release_gate(ok, scope, job_dir=tmp_path)


@pytest.mark.parametrize('scope', SCOPES)
def test_job_with_stale_cad_check_refused(tmp_path, scope):
    step = _write_cad_check(tmp_path, check4='green')
    ok = _checked(_example())
    release_gate(ok, scope, job_dir=tmp_path)              # current and not red: passes
    step.write_text('changed step')
    with pytest.raises(ReleaseError, match='cad_check.json is stale'):
        release_gate(ok, scope, job_dir=tmp_path)
    step.unlink()
    with pytest.raises(ReleaseError, match='cad_check.json is stale'):
        release_gate(ok, scope, job_dir=tmp_path)


def test_job_with_unreadable_cad_check_refused(tmp_path):
    (tmp_path / 'cad_check.json').write_text('{not json')
    with pytest.raises(ReleaseError, match='cad_check.json'):
        release_gate(_checked(_example()), 'simulation', job_dir=tmp_path)
    (tmp_path / 'cad_check.json').write_text('{"status": "green"}')
    with pytest.raises(ReleaseError, match='cad_check.json'):
        release_gate(_checked(_example()), 'simulation', job_dir=tmp_path)


def test_job_without_cad_check_keeps_old_behaviour(tmp_path):
    release_gate(_checked(_example()), 'simulation', job_dir=tmp_path)
    with pytest.raises(ReleaseError, match='collision_checked'):
        release_gate(_example(), 'simulation', job_dir=tmp_path)


def test_release_reads_cad_check_of_the_program_job(job):
    """release(): the v1 program of the conftest job is releasable today; a red check 4 in its job folder stops it
    before anything is written, for both CLI scopes."""
    from awm_verify.release import is_locked
    _write_cad_check(job.dir, check4='red')
    for scope in ('simulation', 'production'):
        with pytest.raises(ReleaseError, match='check 4'):
            release(job, 'P001', 'A. Approver', scope)
    assert not job.release_json.exists() and not is_locked(job, 'P001')


# ---------------------------------------------------------------------------------- review g1-19 I-1

@pytest.mark.parametrize('scope', SCOPES)
def test_cp60_job_without_cad_check_refused(tmp_path, scope):
    """A CP-60 job (placement source job_cad, or an override record) must have its cross-check (rule 4)."""
    ok = _checked(_example())
    (tmp_path / 'placement.json').write_text(json.dumps({'source': 'job_cad'}))
    with pytest.raises(ReleaseError, match='CP-60 job.*no cad_check.json'):
        release_gate(ok, scope, job_dir=tmp_path)
    (tmp_path / 'placement.json').unlink()
    (tmp_path / 'cad_check_override.json').write_text('{}')
    with pytest.raises(ReleaseError, match='CP-60 job.*no cad_check.json'):
        release_gate(ok, scope, job_dir=tmp_path)


@pytest.mark.parametrize('source', ['study', 'fixture_cad'])
def test_manual_or_copied_placement_without_cad_check_keeps_old_behaviour(tmp_path, source):
    (tmp_path / 'placement.json').write_text(json.dumps({'source': source}))
    release_gate(_checked(_example()), 'simulation', job_dir=tmp_path)


def test_unreadable_placement_without_cad_check_refused(tmp_path):
    (tmp_path / 'placement.json').write_text('{not json')
    with pytest.raises(ReleaseError, match='placement.json'):
        release_gate(_checked(_example()), 'simulation', job_dir=tmp_path)
