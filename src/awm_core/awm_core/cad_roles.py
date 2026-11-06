"""CP-60 component roles on an ``inspect-assembly`` tree (pure Python, no OCC).

A tree is the ``components`` list of ``awm_geometry.cli inspect-assembly``: dicts with ``index``, ``parent``,
``instance_name``, ``product_name``, ``is_assembly``, ``bbox_mm``, ``depth``. Nodes are addressed by their path below
the root (instance names joined with ``' / '``; names are never split on ``/``).

Roles (decided 2026-11-06): ``robot``, ``tool``, ``rotary_fixture``, ``part``; unassigned = ``static`` (obstacle).
``exclude`` removes a component from the computation (an action, listed in reports). A role on a node applies to its
descendants unless a descendant has its own.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

ROLES = ('robot', 'tool', 'rotary_fixture', 'part', 'static')
EXCLUDE = 'exclude'
ASSIGNABLE = ROLES + (EXCLUDE,)
SCOPES = ('cell', 'job')
#: Old ``cad_map/0.1`` role -> CP-60 role (spike maps stay readable).
FROM_OLD_ROLE = {'part': 'part', 'positioner_rotating': 'rotary_fixture', 'cell_static': 'static',
                 'head_cad': 'tool', 'ignore': EXCLUDE}
_ENTRY_KEYS = {'name', 'path', 'depth'}

Tree = List[Dict[str, Any]]


def _name(node: Dict[str, Any]) -> str:
    return node.get('instance_name') or node.get('product_name') or ''


def rel_paths(tree: Tree) -> List[str]:
    """Path of every node below its root ('' for a root)."""
    rel: List[str] = []
    for node in tree:
        parent = node['parent']
        if parent is None:
            rel.append('')
        else:
            rel.append(_name(node) if rel[parent] == '' else f'{rel[parent]} / {_name(node)}')
    return rel


def has_geometry(node: Dict[str, Any]) -> bool:
    return not node.get('is_assembly') and node.get('bbox_mm') is not None


def effective_roles(tree: Tree, assignments: List[Dict[str, str]]) -> Dict[int, str]:
    """Every node index -> role or ``exclude``: its own assignment, else its parent's, else ``static``."""
    by_path = {a['path']: a['role'] for a in assignments}
    rel = rel_paths(tree)
    out: Dict[int, str] = {}
    for node in tree:
        i, parent = node['index'], node['parent']
        own = by_path.get(rel[i]) if parent is not None else None
        out[i] = own or (out[parent] if parent is not None else 'static')
    return out


def nodes_with_role(tree: Tree, eff: Dict[int, str], role: str) -> List[int]:
    """Top-most nodes with ``role`` (a node whose parent has the same role is left out)."""
    return [n['index'] for n in tree if n['parent'] is not None and eff[n['index']] == role
            and (tree[n['parent']]['parent'] is None or eff[n['parent']] != role)]


def role_of_path(path: str, assignments: List[Dict[str, str]]) -> str:
    """Effective role of a component path from the assignments alone (longest assigned ancestor path, else
    ``static``); used where only D20 component paths are at hand (awm_seam)."""
    best, role = -1, 'static'
    for a in assignments:
        p = a['path']
        if (path == p or path.startswith(p + ' / ')) and len(p) > best:
            best, role = len(p), a['role']
    return role


def uniform_nodes(tree: Tree, eff: Dict[int, str], role: str) -> List[int]:
    """Top-most nodes whose whole subtree has ``role`` (an assembly with mixed descendants is split)."""
    kids: Dict[int, List[int]] = {}
    for n in tree:
        if n['parent'] is not None:
            kids.setdefault(n['parent'], []).append(n['index'])

    def uniform(i: int) -> bool:
        return eff[i] == role and all(uniform(c) for c in kids.get(i, []))

    out: List[int] = []

    def walk(i: int) -> None:
        if uniform(i):
            out.append(i)
        else:
            for c in kids.get(i, []):
                walk(c)

    for n in tree:
        if n['parent'] is not None and tree[n['parent']]['parent'] is None:
            walk(n['index'])
    return out


def robot_node(tree: Tree, eff: Dict[int, str]) -> Optional[int]:
    """The single top-most ``robot`` node, None if there is none (validate_roles reports more than one)."""
    hits = nodes_with_role(tree, eff, 'robot')
    return hits[0] if len(hits) == 1 else None


def validate_roles(tree: Tree, doc: Dict[str, Any], scope: str) -> List[str]:
    """Problems of a roles document for ``scope`` ('cell' or 'job'); an empty list means valid."""
    if scope not in SCOPES:
        raise ValueError(f'scope {scope!r} not in {SCOPES}')
    rel = rel_paths(tree)
    known = {p for i, p in enumerate(rel) if tree[i]['parent'] is not None}
    problems: List[str] = []
    seen: Dict[str, int] = {}
    for a in doc.get('assignments') or []:
        path, role = a.get('path'), a.get('role')
        seen[path] = seen.get(path, 0) + 1
        if path not in known:
            problems.append(f'path {path!r} is not a component of the file')
        if role not in ASSIGNABLE:
            problems.append(f'{path!r}: role {role!r} not in {list(ASSIGNABLE)}')
    problems += [f'path {p!r} is assigned twice' for p, k in seen.items() if k > 1]
    if problems:
        return problems
    eff = effective_roles(tree, doc.get('assignments') or [])
    robots = nodes_with_role(tree, eff, 'robot')
    rotary = nodes_with_role(tree, eff, 'rotary_fixture')
    parts = [n['index'] for n in tree if has_geometry(n) and eff[n['index']] == 'part']
    if scope == 'cell':
        if len(robots) != 1:
            problems.append(f'the cell needs exactly one robot component (found {len(robots)})')
        if not rotary:
            problems.append('the cell needs at least one rotary_fixture component (the positioner)')
        if any(eff[n['index']] == 'part' for n in tree if n['parent'] is not None):
            problems.append('the cell has no part role: set the part to exclude (parts come with the job)')
    else:
        if not parts:
            problems.append('the job needs at least one part component with geometry')
        if not rotary:
            problems.append('the job needs at least one rotary_fixture component (positioner flange / fixture)')
        if len(robots) > 1:
            problems.append(f'at most one robot component per file (found {len(robots)})')
    return problems


def _name_counts(tree: Tree) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for n in tree:
        if n['parent'] is not None:
            counts[_name(n)] = counts.get(_name(n), 0) + 1
    return counts


def apply_template(tree: Tree, template: Dict[str, Any]) -> List[Dict[str, str]]:
    """Assignments from a name template (instance name -> role); names that occur more than once are skipped."""
    counts, rel = _name_counts(tree), rel_paths(tree)
    roles = template.get('roles') or {}
    return [{'path': rel[n['index']], 'role': roles[_name(n)]} for n in tree
            if n['parent'] is not None and _name(n) in roles and counts[_name(n)] == 1]


def update_template(template: Dict[str, Any], tree: Tree, assignments: List[Dict[str, str]]) -> Dict[str, Any]:
    """Template with the unique-name assignments added (later wins); version + 1."""
    counts, rel = _name_counts(tree), rel_paths(tree)
    by_path = {p: tree[i] for i, p in enumerate(rel)}
    roles = dict(template.get('roles') or {})
    for a in assignments:
        n = by_path.get(a['path'])
        if n is not None and counts.get(_name(n)) == 1:
            roles[_name(n)] = a['role']
    return {'version': int(template.get('version') or 0) + 1, 'roles': roles}


def _find(tree: Tree, rel: List[str], crit: Dict[str, Any]) -> List[int]:
    return [n['index'] for n in tree if n['parent'] is not None
            and ('name' not in crit or n['instance_name'] == crit['name'])
            and ('path' not in crit or rel[n['index']] == crit['path'])
            and ('depth' not in crit or n['depth'] == crit['depth'])]


def from_cad_map(tree: Tree, cmap: Dict[str, Any]) -> List[Dict[str, str]]:
    """Assignments equivalent to an old ``cad_map/0.1`` role map; ``frames.ur_base.component`` becomes ``robot``.

    Raises ValueError for an entry that matches no component or more than one.
    """
    rel = rel_paths(tree)
    out: Dict[str, str] = {}
    for old, entries in (cmap.get('roles') or {}).items():
        if old not in FROM_OLD_ROLE:
            raise ValueError(f'unknown old role {old!r}')
        for entry in entries or []:
            crit = {'name': entry} if isinstance(entry, str) else dict(entry)
            if not isinstance(crit, dict) or not set(crit) <= _ENTRY_KEYS:
                raise ValueError(f'{old}: bad entry {entry!r}')
            hits = _find(tree, rel, crit)
            if len(hits) != 1:
                raise ValueError(f'{old}: {entry!r} matches {len(hits)} components (expected 1)')
            out[rel[hits[0]]] = FROM_OLD_ROLE[old]
    robot = ((cmap.get('frames') or {}).get('ur_base') or {}).get('component')
    if robot:
        hits = _find(tree, rel, {'name': robot})
        if len(hits) != 1:
            raise ValueError(f'frames.ur_base.component {robot!r} matches {len(hits)} components (expected 1)')
        out[rel[hits[0]]] = 'robot'
    return [{'path': p, 'role': r} for p, r in out.items()]


# --- file layout of the CP-60 imports (relative to the repository root / the job folder) -------------------------
CELL_CAD_DIR = ('cell', 'cad')
CELL_STEP = 'cell.step'
CELL_TREE = 'cell_components.json'
ROLES_FILE = 'roles.json'
AXIS_FILE = 'axis_pick.json'
CELL_MAP = 'cad_map.yaml'
TEMPLATE_FILE = ('cell', 'roles_template.json')
JOB_TREE = ('cache', 'components.json')
CHECK_FILE = 'cad_check.json'
#: Cross-check tolerances of checks 1-3 (decided 2026-11-06, = F8: both sides are nominal CAD).
CHECK_TOL_MM = 0.05
CHECK_TOL_DEG = 0.1
