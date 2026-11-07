"""CP-62: recorded Admin override of cross-check 4 for kinematic-only planning (schema + D07 / D09 reference)."""
import copy
import json

import pytest

from awm_core import jsonio
from awm_core.jsonio import SchemaValidationError

V2 = jsonio.SCHEMA_DIR / 'v2' / 'examples'


def example(name):
    return json.loads((V2 / f'{name}.example.json').read_text())


def test_override_example_validates():
    jsonio.validate(example('cad_check_override'), 'v2/cad_check_override')


@pytest.mark.parametrize('field', ['by', 'role', 'reason', 'check_id', 'cad_check_sha256', 'scope', 'findings'])
def test_override_required_fields(field):
    doc = example('cad_check_override')
    del doc[field]
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'v2/cad_check_override')


@pytest.mark.parametrize('field,value', [
    ('check_id', 1),            # only check 4 may be overridden
    ('role', 'engineer'),       # only Admin
    ('scope', 'release'),       # kinematic planning only
    ('reason', ''),             # a reason is required
    ('cad_check_sha256', 'abc'),
])
def test_override_refuses_out_of_scope(field, value):
    doc = example('cad_check_override')
    doc[field] = value
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'v2/cad_check_override')


def test_override_findings_not_empty():
    doc = example('cad_check_override')
    doc['findings'] = []
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'v2/cad_check_override')


REF = {'file': 'cad_check_override.json',
       'sha256': '1111111111111111111111111111111111111111111111111111111111111111',
       'cad_check_sha256': '2222222222222222222222222222222222222222222222222222222222222222',
       'check_id': 4,
       'by': 'adm', 'created_at': '2026-11-07T18:00:00+07:00', 'reason': 'OI-23 kinematic planning only'}


def test_path_records_override_reference():
    doc = example('path')
    doc['collision_checked'] = False
    doc['cad_check_override'] = copy.deepcopy(REF)
    jsonio.validate(doc, 'v2/path')
    doc['cad_check_override']['check_id'] = 2
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'v2/path')


def test_program_records_override_reference():
    doc = example('program')
    doc['collision_checked'] = False
    doc['cad_check_override'] = copy.deepcopy(REF)
    jsonio.validate(doc, 'v2/program')
    del doc['cad_check_override']['sha256']
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'v2/program')


@pytest.mark.parametrize('name', ['path', 'program'])
@pytest.mark.parametrize('field', ['by', 'created_at', 'reason'])
def test_reference_keeps_who_when_why(name, field):
    """Review I1: the audit trail survives a later DELETE / new override."""
    doc = example(name)
    doc['collision_checked'] = False
    doc['cad_check_override'] = copy.deepcopy(REF)
    del doc['cad_check_override'][field]
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, f'v2/{name}')


@pytest.mark.parametrize('name', ['path', 'program'])
def test_override_only_with_collision_unchecked(name):
    """Review I2: an override reference is only valid on a kinematic-only document."""
    doc = example(name)
    doc['cad_check_override'] = copy.deepcopy(REF)
    doc['collision_checked'] = False
    jsonio.validate(doc, f'v2/{name}')
    doc['collision_checked'] = True
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, f'v2/{name}')
    if name == 'path':
        del doc['collision_checked']
        with pytest.raises(SchemaValidationError):
            jsonio.validate(doc, f'v2/{name}')
