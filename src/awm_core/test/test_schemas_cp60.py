"""CP-60 / CP-61 schema additions: cad_roles, axis_pick, cad_check (new) and the minor changes."""
import copy
import json

import pytest

from awm_core import jsonio
from awm_core.jsonio import SchemaValidationError

V2 = jsonio.SCHEMA_DIR / 'v2' / 'examples'


def example(name):
    return json.loads((V2 / f'{name}.example.json').read_text())


@pytest.mark.parametrize('name', ['cad_roles', 'axis_pick', 'cad_check'])
def test_new_examples_validate(name):
    jsonio.validate(example(name), f'v2/{name}')


def test_cad_roles_refuses_unknown_role():
    doc = example('cad_roles')
    doc['assignments'][0]['role'] = 'fixture'
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'v2/cad_roles')


def test_axis_pick_needs_edge_id():
    doc = example('axis_pick')
    doc['edge_id'] = 'X1'
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'v2/axis_pick')


def test_cad_check_status_enum():
    doc = example('cad_check')
    doc['status'] = 'orange'
    with pytest.raises(SchemaValidationError):
        jsonio.validate(doc, 'v2/cad_check')


def test_placement_job_cad_with_rotary_fixture_object():
    doc = example('placement')
    doc['source'] = 'job_cad'
    doc['axis_pick'] = 'axis_pick.json'
    doc['phi_deg'] = 30.0
    obj = copy.deepcopy(doc['objects'][0])
    obj.update(object_id='holder_x', role='rotary_fixture', padding_mm=3.0)
    obj['mesh'] = dict(obj['mesh'], face_ids=False)
    doc['objects'].append(obj)
    jsonio.validate(doc, 'v2/placement')


def test_topology_with_components_and_shell_faces():
    topo = {
        'schema_version': '2.0', 'created_by': 't', 'created_at': '2026-11-06T00:00:00Z', 'inputs': [],
        'unit': 'mm', 'frame': 'part',
        'source': {'file': 'model.step', 'component': None, 'up_axis': '+Y', 'components': True},
        'tessellation': {'face_linear_deflection_mm': 0.05, 'face_angular_deflection_deg': 5.0,
                         'edge_chord_mm': 0.05},
        'solids': [{'solid_id': 'B1', 'name': 'a', 'volume_mm3': 1.0, 'bbox_min_mm': [0, 0, 0],
                    'bbox_max_mm': [1, 1, 1], 'component_index': 3, 'component_path': 'Fixture / Holder:1'}],
        'faces': [{'face_id': 'F1', 'solid_id': None, 'surface_type': 'plane', 'area_mm2': 1.0}],
        'edges': [{'edge_id': 'E1', 'solid_id': None, 'faces': ['F1'], 'curve_type': 'circle', 'length_mm': 1.0,
                   'closed': True, 'tangent_neighbours': [], 'fingerprint': 'x', 'joint_edge_id': None}],
        'joints': [],
    }
    jsonio.validate(topo, 'view_topology')


def test_cad_map_positioner_axis_with_pick():
    doc = example('cad_map')
    pick = example('axis_pick')
    doc['frames']['positioner_axis']['pick'] = {k: pick[k] for k in
                                                ('component_path', 'edge_id', 'radius_mm', 'reversed', 'method')}
    jsonio.validate(doc, 'v2/cad_map')
