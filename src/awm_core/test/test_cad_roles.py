"""CP-60 role logic on an inspect-assembly component tree (pure Python, no OCC)."""
import pytest

from awm_core.cad_roles import (apply_template, effective_roles, from_cad_map, nodes_with_role, rel_paths,
                                robot_node, update_template, validate_roles)

BOX = {'min': [0, 0, 0], 'max': [1, 1, 1]}


def node(index, parent, name, assembly=False, depth=1):
    return {'index': index, 'parent': parent, 'instance_name': name, 'product_name': name.split(':')[0],
            'is_assembly': assembly, 'bbox_mm': BOX, 'depth': depth,
            'placement_mm': [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]}


@pytest.fixture
def tree_small():
    # root / R (robot) ; A (assembly) / B, C ; D ; E
    return [node(0, None, 'root', True, 0), node(1, 0, 'R'), node(2, 0, 'A', True), node(3, 2, 'B', depth=2),
            node(4, 2, 'C', depth=2), node(5, 0, 'D'), node(6, 0, 'E')]


@pytest.fixture
def tree_dup():
    return [node(0, None, 'root', True, 0), node(1, 0, 'X', True), node(2, 0, 'Y', True),
            node(3, 1, 'Holder:1', depth=2), node(4, 2, 'Holder:1', depth=2)]


def idx(tree, path):
    return rel_paths(tree).index(path)


def doc(pairs):
    return {'assignments': [{'path': p, 'role': r} for p, r in pairs]}


def test_child_role_overrides_parent(tree_small):
    eff = effective_roles(tree_small, [{'path': 'A', 'role': 'rotary_fixture'}, {'path': 'A / B', 'role': 'part'}])
    assert eff[idx(tree_small, 'A / B')] == 'part'
    assert eff[idx(tree_small, 'A / C')] == 'rotary_fixture'
    assert eff[idx(tree_small, 'D')] == 'static'


def test_exclude_inherits(tree_small):
    eff = effective_roles(tree_small, [{'path': 'A', 'role': 'exclude'}])
    assert eff[idx(tree_small, 'A / B')] == 'exclude'


def test_paths_distinguish_duplicate_names(tree_dup):
    assert {'X / Holder:1', 'Y / Holder:1'} <= set(rel_paths(tree_dup))


def test_cell_scope_needs_one_robot_and_no_part(tree_small):
    assert any('robot' in p for p in validate_roles(tree_small, doc([('A', 'rotary_fixture')]), 'cell'))
    problems = validate_roles(tree_small, doc([('R', 'robot'), ('A', 'rotary_fixture'), ('A / B', 'part')]), 'cell')
    assert any('part' in p for p in problems)
    assert validate_roles(tree_small, doc([('R', 'robot'), ('A', 'rotary_fixture')]), 'cell') == []


def test_job_scope_needs_part_and_rotary_fixture(tree_small):
    assert any('part' in p for p in validate_roles(tree_small, doc([('A', 'rotary_fixture')]), 'job'))
    assert any('rotary_fixture' in p for p in validate_roles(tree_small, doc([('A / B', 'part')]), 'job'))
    assert validate_roles(tree_small, doc([('A', 'rotary_fixture'), ('A / B', 'part')]), 'job') == []
    assert any('robot' in p for p in
               validate_roles(tree_small, doc([('A', 'rotary_fixture'), ('A / B', 'part'), ('R', 'robot'),
                                               ('D', 'robot')]), 'job'))


def test_unknown_path_role_and_duplicates_reported(tree_small):
    problems = validate_roles(tree_small, doc([('A', 'rotary_fixture'), ('A / B', 'part'), ('Q', 'part'),
                                               ('D', 'fixture'), ('D', 'static')]), 'job')
    text = '\n'.join(problems)
    assert "'Q'" in text and "'fixture'" in text and 'twice' in text


def test_robot_node_and_nodes_with_role(tree_small):
    eff = effective_roles(tree_small, [{'path': 'R', 'role': 'robot'}, {'path': 'A', 'role': 'rotary_fixture'}])
    assert robot_node(tree_small, eff) == idx(tree_small, 'R')
    assert nodes_with_role(tree_small, eff, 'rotary_fixture') == [idx(tree_small, 'A')]
    assert nodes_with_role(tree_small, eff, 'static') == [idx(tree_small, 'D'), idx(tree_small, 'E')]


def test_from_cad_map_matches_old_roles(tree_small):
    cmap = {'frames': {'ur_base': {'component': 'R', 'use': 'placement'}},
            'roles': {'positioner_rotating': ['C'], 'part': [{'path': 'A / B'}], 'cell_static': ['D'],
                      'ignore': ['R', 'E']}}
    eff = effective_roles(tree_small, from_cad_map(tree_small, cmap))
    got = {p: eff[i] for i, p in enumerate(rel_paths(tree_small)) if p}
    assert got == {'R': 'robot', 'A': 'static', 'A / B': 'part', 'A / C': 'rotary_fixture', 'D': 'static',
                   'E': 'exclude'}


def test_template_round_trip_skips_ambiguous_names(tree_dup):
    tpl = update_template({'version': 0, 'roles': {}}, tree_dup,
                          [{'path': 'X', 'role': 'rotary_fixture'}, {'path': 'X / Holder:1', 'role': 'part'}])
    assert tpl['version'] == 1 and tpl['roles'] == {'X': 'rotary_fixture'}   # Holder:1 occurs twice
    assert apply_template(tree_dup, {'version': 1, 'roles': {'X': 'rotary_fixture', 'Holder:1': 'part'}}) == [
        {'path': 'X', 'role': 'rotary_fixture'}]


def test_uniform_nodes_skip_mixed_assemblies(tree_small):
    from awm_core.cad_roles import uniform_nodes

    eff = effective_roles(tree_small, [{'path': 'A / B', 'role': 'part'}])   # A static (default), B part, C static
    assert [rel_paths(tree_small)[i] for i in uniform_nodes(tree_small, eff, 'static')] == ['R', 'A / C', 'D', 'E']


def test_role_of_path_longest_prefix():
    from awm_core.cad_roles import role_of_path

    a = [{'path': 'Fix:1', 'role': 'rotary_fixture'}, {'path': 'Fix:1 / Pipe:1', 'role': 'part'}]
    assert role_of_path('Fix:1 / Pipe:1 / body', a) == 'part'
    assert role_of_path('Fix:1 / Holder:1', a) == 'rotary_fixture'
    assert role_of_path('Fix:10 / X', a) == 'static'          # no prefix match on 'Fix:1'
    assert role_of_path('Other:1', a) == 'static'
