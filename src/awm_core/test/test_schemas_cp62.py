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
       'check_id': 4}


def test_path_records_override_reference():
    doc = example('path')
    doc['cad_check_override'] = copy.deepcopy(REF)
    jsonio.validate(doc, 'v2/path')
    doc['cad_check_override']['check_id'] = 2
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'v2/path')


def test_program_records_override_reference():
    doc = example('program')
    doc['cad_check_override'] = copy.deepcopy(REF)
    jsonio.validate(doc, 'v2/program')
    del doc['cad_check_override']['sha256']
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'v2/program')
