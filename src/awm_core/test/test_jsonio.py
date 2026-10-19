"""Schema loading, header, validated write/read and stale-input detection."""

import json

import pytest

from awm_core import jsonio
from awm_core.jsonio import SchemaValidationError, StaleInputError

POSE = {'position_m': [0.0, 0.0, 0.0], 'quat_xyzw': [0.0, 0.0, 0.0, 1.0]}
SHA = '0' * 64


def _calibration(header):
    doc = dict(header)
    doc.update({'calibration_id': 'CAL-NOMINAL', 'status': 'NOMINAL',
                'base_T_fixture': POSE, 'fixture_T_part': POSE, 'tcp': {'tcp_mig': POSE}})
    return doc


def test_all_schemas_load_and_are_valid():
    names = jsonio.schema_names()
    for expected in ['part', 'edges', 'seams', 'wps', 'process', 'seam_geometry', 'path',
                     'calibration', 'program', 'verify_report', 'record', 'release']:
        assert expected in names
    for name in names:
        schema = jsonio.load_schema(name)
        assert 'common.schema.json' not in json.dumps(schema), name
        for key in ('schema_version', 'created_by', 'created_at', 'inputs'):
            assert key in schema['required'], (name, key)


def test_header_and_roundtrip(tmp_path):
    src = tmp_path / 'in.txt'
    src.write_text('hello')
    header = jsonio.make_header('test 0.1', [src], tmp_path)
    assert header['inputs'][0]['name'] == 'in.txt'
    assert len(header['inputs'][0]['sha256']) == 64
    assert header['created_at'].endswith('Z')
    out = jsonio.write_json(tmp_path / 'cal.json', _calibration(header), 'calibration')
    doc = jsonio.read_json(out, 'calibration', base_dir=tmp_path)
    assert doc['calibration_id'] == 'CAL-NOMINAL'


def test_invalid_document_is_not_written(tmp_path):
    header = jsonio.make_header('test', [], tmp_path)
    doc = _calibration(header)
    doc['status'] = 'GUESSED'
    with pytest.raises(SchemaValidationError, match='status'):
        jsonio.write_json(tmp_path / 'cal.json', doc, 'calibration')
    assert not (tmp_path / 'cal.json').exists()


def test_unknown_field_rejected(tmp_path):
    doc = _calibration(jsonio.make_header('test', [], tmp_path))
    doc['speed_mm_s'] = 3
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'calibration')


def test_stale_input_detected(tmp_path):
    src = tmp_path / 'in.txt'
    src.write_text('v1')
    out = jsonio.write_json(tmp_path / 'cal.json',
                            _calibration(jsonio.make_header('t', [src], tmp_path)), 'calibration')
    src.write_text('v2')
    with pytest.raises(StaleInputError, match='in.txt'):
        jsonio.read_json(out, 'calibration', base_dir=tmp_path)
    jsonio.read_json(out, 'calibration', base_dir=tmp_path, check=False)
    src.unlink()
    with pytest.raises(StaleInputError, match='missing'):
        jsonio.read_json(out, 'calibration', base_dir=tmp_path)


def test_input_outside_base_dir_uses_relative_name(tmp_path):
    (tmp_path / 'cell').mkdir()
    (tmp_path / 'jobs' / 'A_1').mkdir(parents=True)
    wps = tmp_path / 'cell' / 'w.json'
    wps.write_text('{}')
    entry = jsonio.input_entry(wps, tmp_path / 'jobs' / 'A_1')
    assert entry['name'] == '../../cell/w.json'


def test_invalid_json_reported(tmp_path):
    bad = tmp_path / 'bad.json'
    bad.write_text('{not json')
    with pytest.raises(SchemaValidationError, match='invalid JSON'):
        jsonio.read_json(bad, 'calibration')


def test_nan_refused(tmp_path):
    doc = _calibration(jsonio.make_header('t', [], tmp_path))
    doc['base_T_fixture'] = {'position_m': [float('nan'), 0, 0], 'quat_xyzw': [0, 0, 0, 1]}
    with pytest.raises(ValueError):
        jsonio.write_json(tmp_path / 'c.json', doc, 'calibration')


def test_release_requires_lock(tmp_path):
    doc = jsonio.make_header('t', [], tmp_path)
    doc.update({'program_rev': 'P001', 'program_sha256': SHA, 'report_sha256': SHA,
                'scope': 'simulation', 'records': [], 'approved_by': 'tester',
                'approved_at': '2026-10-19T00:00:00Z', 'locked': True})
    jsonio.validate(doc, 'release')
    doc['locked'] = False
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'release')
