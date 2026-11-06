"""CP-60 apply-roles: roles.json + axis pick compiled into a cad_map/0.1 dict (no OCC in roles_map itself)."""
import pytest

from awm_core import jsonio

BOX = {'min': [0, 0, 0], 'max': [1, 1, 1]}
EYE = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def node(index, parent, name, assembly=False, depth=1):
    return {'index': index, 'parent': parent, 'instance_name': name, 'product_name': name.split(':')[0],
            'is_assembly': assembly, 'bbox_mm': BOX, 'depth': depth, 'placement_mm': EYE}


@pytest.fixture
def tree():
    # root / UR:1 (assembly: Link:1, Tool:1) ; Pos:1 (assembly: Disc:1, Holder:1) ; Frame:1 ; Part:1 ; Bolt:1
    return [node(0, None, 'cell', True, 0), node(1, 0, 'UR:1', True), node(2, 1, 'Link:1', depth=2),
            node(3, 1, 'Tool:1', depth=2), node(4, 0, 'Pos:1', True), node(5, 4, 'Disc:1', depth=2),
            node(6, 4, 'Holder:1', depth=2), node(7, 0, 'Frame:1'), node(8, 0, 'Part:1'), node(9, 0, 'Bolt:1')]


@pytest.fixture
def roles():
    return {'schema_version': '2.0', 'created_by': 't', 'created_at': '2026-11-06T00:00:00Z', 'inputs': [],
            'source': {'file': 'cell.step', 'sha256': 'a' * 64, 'up_axis': '+Y'}, 'scope': 'cell',
            'assignments': [{'path': 'UR:1', 'role': 'robot'}, {'path': 'UR:1 / Tool:1', 'role': 'tool'},
                            {'path': 'Pos:1', 'role': 'rotary_fixture'}, {'path': 'Part:1', 'role': 'exclude'},
                            {'path': 'Bolt:1', 'role': 'exclude'}],
            'collision': {'Holder:1': 'mesh'}, 'contact_tolerance_mm': 1.0}


@pytest.fixture
def pick():
    return {'component_path': 'Pos:1 / Disc:1', 'edge_id': 'E7', 'center_file_mm': [1.0, 2.0, 3.0],
            'axis_file': [1.0, 0.0, 0.0], 'radius_mm': 60.0, 'reversed': False, 'method': 'pick'}


def test_compiled_map_has_no_nested_entries(tree, roles, pick):
    from awm_geometry import scene
    from awm_geometry.roles_map import compile_cad_map

    cmap, _report = compile_cad_map(tree, roles, pick)
    jsonio.validate(cmap, 'v2/cad_map')
    direct = scene.resolve_roles(tree, cmap)          # does not raise: covered, nothing nested
    by_path = {label: role for role, label in direct.values()}
    assert by_path == {'UR:1 / Link:1': 'ignore', 'UR:1 / Tool:1': 'head_cad', 'Pos:1': 'positioner_rotating',
                       'Frame:1': 'cell_static', 'Part:1': 'ignore', 'Bolt:1': 'ignore'}
    assert cmap['frames']['ur_base']['component'] == 'UR:1'
    assert cmap['frames']['positioner_axis']['point_file_mm'] == [1.0, 2.0, 3.0]
    assert cmap['frames']['positioner_axis']['pick']['edge_id'] == 'E7'
    assert cmap['collision'] == {'Holder:1': 'mesh'} and cmap['contact_tolerance_mm'] == 1.0
    assert cmap['source'] == {'file': 'cell.step', 'sha256': 'a' * 64, 'unit': 'mm', 'up_axis': '+Y'}


def test_unassigned_component_becomes_static_and_excluded_listed(tree, roles, pick):
    from awm_geometry.roles_map import compile_cad_map

    _cmap, report = compile_cad_map(tree, roles, pick)
    assert report['unassigned_static'] == ['Frame:1']
    assert report['excluded'] == ['Part:1', 'Bolt:1']


def test_invalid_roles_refused(tree, roles, pick):
    from awm_geometry.roles_map import compile_cad_map
    from awm_geometry.scene import MapError

    roles['assignments'] = [a for a in roles['assignments'] if a['role'] != 'robot']
    with pytest.raises(MapError, match='robot'):
        compile_cad_map(tree, roles, pick)


def test_ambiguous_robot_name_refused(tree, roles, pick):
    from awm_geometry.roles_map import compile_cad_map
    from awm_geometry.scene import MapError

    tree[5]['instance_name'] = 'UR:1'                 # nested component with the robot's instance name
    with pytest.raises(MapError, match='UR:1'):
        compile_cad_map(tree, roles, pick)
