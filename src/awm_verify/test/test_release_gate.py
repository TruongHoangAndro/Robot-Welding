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
