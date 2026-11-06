"""CP-60 cell import: compile ``roles.json`` (v2/cad_roles) and the positioner-axis pick (v2/axis_pick) into the
``cad_map/0.1`` role map that ``export-scene`` reads (one scene code path; the hand maps of the spike stay readable).

Role translation: ``robot`` -> ``ignore`` (+ ``frames.ur_base.component``: the robot comes from ``ur_description``,
its CAD placement gives the base, ADR-16), ``tool`` -> ``head_cad``, ``rotary_fixture`` -> ``positioner_rotating``,
``static`` (also unassigned) -> ``cell_static``, ``exclude`` -> ``ignore``. A subtree whose nodes all share one role
becomes one entry (by path); a node with mixed descendants is split into its children, so no entry is nested in
another (``scene.resolve_roles`` rule). No OCC here.
"""

from __future__ import annotations

from typing import Any, Dict, List, Tuple

from awm_core import cad_roles
from awm_geometry.occ_io import GeometryError

MAP_SCHEMA = 'cad_map/0.1'
TO_MAP_ROLE = {'robot': 'ignore', 'tool': 'head_cad', 'rotary_fixture': 'positioner_rotating',
               'static': 'cell_static', 'exclude': 'ignore', 'part': 'part'}
PICK_KEYS = ('component_path', 'edge_id', 'radius_mm', 'reversed', 'method')


class RolesError(GeometryError):
    """roles.json / axis pick do not fit the assembly (same meaning as scene.MapError)."""


def _map_error(text: str) -> Exception:
    from awm_geometry.scene import MapError  # scene imports OCC; only needed on the error path
    return MapError(text)


def compile_cad_map(tree: List[Dict[str, Any]], roles: Dict[str, Any], pick: Dict[str, Any]
                    ) -> Tuple[Dict[str, Any], Dict[str, List[str]]]:
    """``(cad_map dict, report)``; report = ``{'excluded': [...], 'unassigned_static': [...]}`` (paths)."""
    problems = cad_roles.validate_roles(tree, roles, 'cell')
    if problems:
        raise _map_error('roles do not fit the cell assembly:\n  ' + '\n  '.join(problems))
    assignments = roles.get('assignments') or []
    eff = cad_roles.effective_roles(tree, assignments)
    rel = cad_roles.rel_paths(tree)
    robot = cad_roles.robot_node(tree, eff)
    robot_name = tree[robot]['instance_name']
    same = [n['index'] for n in tree if n['parent'] is not None and n['instance_name'] == robot_name]
    if len(same) != 1:
        raise _map_error(f'robot component name {robot_name!r} occurs {len(same)} times; the base is found by '
                         'instance name (rename the duplicate in the CAD)')
    children: Dict[int, List[int]] = {}
    for n in tree:
        if n['parent'] is not None:
            children.setdefault(n['parent'], []).append(n['index'])

    def subtree_roles(i: int) -> set:
        out = {eff[i]}
        for c in children.get(i, []):
            out |= subtree_roles(c)
        return out

    entries: Dict[str, List[Any]] = {'part': [], 'positioner_rotating': [], 'cell_static': [], 'ignore': []}

    def emit(i: int) -> None:
        kinds = subtree_roles(i)
        if len(kinds) == 1:
            role = TO_MAP_ROLE[eff[i]]
            entry = {'path': rel[i], 'depth': tree[i]['depth']} if role == 'head_cad' else {'path': rel[i]}
            entries.setdefault(role, []).append(entry)
        else:
            for c in children.get(i, []):
                emit(c)

    for n in tree:
        if n['parent'] is not None and tree[n['parent']]['parent'] is None:
            emit(n['index'])
    own = {a['path'] for a in assignments}
    report = {'excluded': [rel[i] for i in cad_roles.uniform_nodes(tree, eff, cad_roles.EXCLUDE)],
              'unassigned_static': [rel[i] for i in cad_roles.uniform_nodes(tree, eff, 'static')
                                    if rel[i] not in own]}
    src = roles['source']
    cmap: Dict[str, Any] = {
        'schema': MAP_SCHEMA,
        'source': {'file': src['file'], 'sha256': src['sha256'], 'unit': 'mm', 'up_axis': src['up_axis']},
        'match': 'instance_name', 'default': 'deny',
        'frames': {'world': {'origin_from': 'ur_base'},
                   'ur_base': {'component': robot_name, 'use': 'placement'},
                   'positioner_axis': {'point_file_mm': [float(v) for v in pick['center_file_mm']],
                                       'direction_file': [float(v) for v in pick['axis_file']],
                                       'pick': {k: pick[k] for k in PICK_KEYS}}},
        'roles': entries,
    }
    if roles.get('collision'):
        cmap['collision'] = dict(roles['collision'])
    if roles.get('contact_tolerance_mm') is not None:
        cmap['contact_tolerance_mm'] = float(roles['contact_tolerance_mm'])
    return cmap, report
