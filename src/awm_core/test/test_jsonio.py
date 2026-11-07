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


def test_contract_examples_validate():
    """Every hand-made contract example (T0.5) matches its schema: examples/<schema>.example.json."""
    examples = sorted((jsonio.SCHEMA_DIR / 'examples').glob('*.example.json'))
    assert examples
    for path in examples:
        doc = json.loads(path.read_text())
        jsonio.validate(doc, path.name[:-len('.example.json')])


def test_v2_contract_schemas_and_examples():
    """Contract freeze T0.5: every v2 schema loads, carries the header and has a valid example."""
    v2 = jsonio.SCHEMA_DIR / 'v2'
    names = sorted(p.name[:-len('.schema.json')] for p in v2.glob('*.schema.json')
                   if p.name != 'common.schema.json')
    assert names == ['active', 'axis_pick', 'cad_check', 'cad_check_override', 'cad_map', 'cad_roles', 'cell_scene', 'part', 'path',
                     'placement', 'program', 'release', 'seam_axis', 'seams']
    cell_data = {'cad_map'}  # hand / UI edited cell data with its own `schema` tag, not a job document
    for name in names:
        schema = jsonio.load_schema(f'v2/{name}')
        assert 'common.schema.json' not in json.dumps(schema), name
        example = json.loads((v2 / 'examples' / f'{name}.example.json').read_text())
        jsonio.validate(example, f'v2/{name}')
        if name in cell_data:
            continue
        for key in ('schema_version', 'created_by', 'created_at', 'inputs'):
            assert key in schema['required'], (name, key)
        assert example['schema_version'].startswith('2.')


def test_v2_refuses_v1_and_partial_records():
    import pytest
    v2 = jsonio.SCHEMA_DIR / 'v2' / 'examples'
    part = json.loads((v2 / 'part.example.json').read_text())
    part['schema_version'] = '1.0'
    with pytest.raises(jsonio.SchemaValidationError):
        jsonio.validate(part, 'v2/part')
    part = json.loads((v2 / 'part.example.json').read_text())
    del part['source']['component']
    with pytest.raises(jsonio.SchemaValidationError):
        jsonio.validate(part, 'v2/part')
    path = json.loads((v2 / 'path.example.json').read_text())
    del path['context']['git']
    with pytest.raises(jsonio.SchemaValidationError):
        jsonio.validate(path, 'v2/path')


def test_v2_release_scopes_trial_and_promotion():
    """E-02 = A (Truong 2026-11-03): scopes simulation / trial / production; production promotes a trial bundle."""
    v2 = jsonio.SCHEMA_DIR / 'v2'
    base = json.loads((v2 / 'examples' / 'release.example.json').read_text())
    trial = dict(base, scope='trial')
    jsonio.validate(trial, 'v2/release')
    production = dict(base, scope='production')
    with pytest.raises(SchemaValidationError):
        jsonio.validate(production, 'v2/release')  # no promoted_from
    production['promoted_from'] = {'program_rev': base['program_rev'], 'release_sha256': SHA}
    jsonio.validate(production, 'v2/release')
    with pytest.raises(SchemaValidationError):
        jsonio.validate(dict(trial, promoted_from=production['promoted_from']), 'v2/release')
    active = json.loads((v2 / 'examples' / 'active.example.json').read_text())
    jsonio.validate(dict(active, scope='trial'), 'v2/active')


def test_v2_path_minor_2_1_sync_and_kinematic_only():
    """G1.3 (CP-58 / CP-59): mode sync, per-point status, collision_checked false, null clearance, failing ranges."""
    import copy
    v2 = jsonio.SCHEMA_DIR / 'v2' / 'examples'
    base = json.loads((v2 / 'path.example.json').read_text())
    doc = copy.deepcopy(base)
    doc['collision_checked'] = False
    seam = doc['seams'][0]
    seam['mode'] = 'sync'
    for pt in seam['points']:
        pt['min_clearance_mm'] = None
    jsonio.validate(doc, 'v2/path')
    bad = copy.deepcopy(doc)
    bad['seams'][0]['points'][0]['status'] = 'maybe'
    with pytest.raises(SchemaValidationError):
        jsonio.validate(bad, 'v2/path')
    bad = copy.deepcopy(doc)
    bad['rejected'][0]['failing_ranges'][0]['status'] = 'collision?'
    with pytest.raises(SchemaValidationError):
        jsonio.validate(bad, 'v2/path')
    bad = copy.deepcopy(doc)
    bad['seams'][0]['mode'] = 'free'
    with pytest.raises(SchemaValidationError):
        jsonio.validate(bad, 'v2/path')
    old = copy.deepcopy(base)  # a 2.0 document without the new fields stays valid (MINOR)
    del old['collision_checked']
    del old['seams'][0]['planning']
    for pt in old['seams'][0]['points']:
        del pt['status']
    jsonio.validate(old, 'v2/path')


def _program_2_1():
    """2.1 program built from the 2.0 example: one 7-joint robot_positioner trajectory per segment (G1.5)."""
    import copy
    v2 = jsonio.SCHEMA_DIR / 'v2' / 'examples'
    doc = copy.deepcopy(json.loads((v2 / 'program_v2_0.example.json').read_text()))
    doc['schema_version'] = '2.1'
    doc['collision_checked'] = False
    for seg in doc['segments']:
        hold = seg['hold_deg']
        seg['controller'] = 'robot_positioner'
        seg['collision_checked'] = False
        for pt in seg['points']:
            if len(pt['positions_deg']) == 1:
                pt['positions_deg'] = hold + pt['positions_deg']
            else:
                pt['positions_deg'] = pt['positions_deg'] + hold
            pt['velocities_deg_s'] = [0.0] * 7
            pt['accelerations_deg_s2'] = [0.0] * 7
        seg['hold_deg'] = []
        if seg['kind'] in ('weld', 'positioner_weld'):
            seg['positioner_mode'] = 'C'
    for ev in doc['events']:
        ev['wps_id'] = None
        ev['preset'] = None
    return doc


def test_v2_program_minor_2_1_robot_positioner():
    """G1.5 (CP-59): controller robot_positioner = 7 values, hold_deg empty, positioner_mode on weld segments,
    collision_checked required from 2.1, process fields nullable; 2.0 documents stay valid (MINOR)."""
    import copy
    v2 = jsonio.SCHEMA_DIR / 'v2' / 'examples'
    jsonio.validate(json.loads((v2 / 'program_v2_0.example.json').read_text()), 'v2/program')
    example = json.loads((v2 / 'program.example.json').read_text())
    assert example['schema_version'] == '2.1' and example['collision_checked'] is False
    assert {s['controller'] for s in example['segments']} == {'robot_positioner'}
    jsonio.validate(example, 'v2/program')
    doc = _program_2_1()
    jsonio.validate(doc, 'v2/program')
    bad = copy.deepcopy(doc)
    del bad['collision_checked']
    with pytest.raises(SchemaValidationError):
        jsonio.validate(bad, 'v2/program')
    bad = copy.deepcopy(doc)
    bad['segments'][1]['points'][0]['positions_deg'] = bad['segments'][1]['points'][0]['positions_deg'][:6]
    with pytest.raises(SchemaValidationError):
        jsonio.validate(bad, 'v2/program')
    bad = copy.deepcopy(doc)
    bad['segments'][1]['points'][0]['velocities_deg_s'].append(0.0)
    with pytest.raises(SchemaValidationError):
        jsonio.validate(bad, 'v2/program')
    bad = copy.deepcopy(doc)
    bad['segments'][1]['hold_deg'] = [0.0]
    with pytest.raises(SchemaValidationError):
        jsonio.validate(bad, 'v2/program')
    bad = copy.deepcopy(doc)
    weld = next(s for s in bad['segments'] if s['kind'] == 'positioner_weld')
    del weld['positioner_mode']
    with pytest.raises(SchemaValidationError):
        jsonio.validate(bad, 'v2/program')
    weld['positioner_mode'] = 'robot_only'
    with pytest.raises(SchemaValidationError):
        jsonio.validate(bad, 'v2/program')
    old = json.loads((v2 / 'program_v2_0.example.json').read_text())
    old['events'][0]['preset'] = None   # null process data only from 2.1
    with pytest.raises(SchemaValidationError):
        jsonio.validate(old, 'v2/program')
