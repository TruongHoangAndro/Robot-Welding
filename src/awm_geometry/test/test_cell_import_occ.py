"""CP-60 cell import: inspect-assembly -> roles + axis pick -> apply-roles -> export-scene without a job.

Run with ``$AWM_OCC_PYTHON -m pytest`` (skipped without OCC; colcon runs it through test_occ_suite_in_conda.py).
The golden test needs ``AWM_CELL_STEP`` (the cell CAD) and compares with ``docs/generated/spike/cad_map_cell.yaml``.
"""

import json
import math
import os
from pathlib import Path

import numpy as np
import pytest

from awm_core import jsonio

REPO = Path(__file__).resolve().parents[3]


def _write_roles(path, step, up_axis, assignments, **extra):
    from awm_core.hashing import sha256_file

    doc = jsonio.make_header('test', [step], path.parent)
    doc.update({'schema_version': '2.0', 'source': {'file': step.name, 'sha256': sha256_file(step), 'up_axis': up_axis},
                'scope': 'cell', 'assignments': assignments, **extra})
    jsonio.write_json(path, doc, 'v2/cad_roles')


def test_cell_only_export_writes_frames_with_pick(job_assembly, tmp_path):
    from awm_geometry import cli, miniyaml, testparts

    _job, topo, _tree = job_assembly
    step = _job.model_step
    tree_json = tmp_path / 'cell_components.json'
    assert cli.main(['inspect-assembly', '--step', str(step), '--out', str(tree_json)]) == 0
    disc = {s['solid_id'] for s in topo['solids'] if s['component_path'] == 'Positioner:1 / Disc:1'}
    rim = next(e['edge_id'] for e in topo['edges'] if e['solid_id'] in disc and e['curve_type'] == 'circle'
               and abs(e['length_mm'] - 2 * math.pi * testparts.JOB_DISC_R_MM) < 1e-3)
    pick = tmp_path / 'axis_pick.json'
    assert cli.main(['pick-circle', '--step', str(step), '--components', '--edge', rim, '--up-axis', '+Z',
                     '--out', str(pick)]) == 0
    roles = tmp_path / 'roles.json'
    _write_roles(roles, step, '+Z', [{'path': 'Robot:1', 'role': 'robot'},
                                     {'path': 'Positioner:1 / Disc:1', 'role': 'rotary_fixture'},
                                     {'path': 'Fixture:1', 'role': 'rotary_fixture'},
                                     {'path': 'Part:1', 'role': 'exclude'}])
    cmap = tmp_path / 'cad_map.yaml'
    assert cli.main(['apply-roles', '--step', str(step), '--tree', str(tree_json), '--roles', str(roles),
                     '--axis', str(pick), '--out', str(cmap)]) == 0
    repo = tmp_path / 'repo'
    assert cli.main(['export-scene', '--step', str(step), '--map', str(cmap), '--repo', str(repo)]) == 0
    fr = miniyaml.load(repo / 'cell' / 'model' / 'frames.yaml')
    axis = fr['world_T_positioner_axis']
    assert axis['method'] == 'pick' and axis['edge_id'] == rim
    assert abs(axis['radius_mm'] - testparts.JOB_DISC_R_MM) < 1e-6
    assert abs(abs(axis['direction'][0]) - 1.0) < 1e-9
    assert not (repo / 'jobs').exists()
    manifest = json.loads((repo / 'cell' / 'model' / 'scene_manifest.json').read_text())
    roles_seen = {o['component']: o['role'] for o in manifest['objects']}
    assert roles_seen.get('Positioner:1 / Disc:1') == 'positioner_rotating'
    assert roles_seen.get('Positioner:1 / Base:1') == 'cell_static'
    assert all(o['part_max_depth_mm'] is None for o in manifest['objects'])


@pytest.mark.skipif(not os.environ.get('AWM_CELL_STEP'), reason='AWM_CELL_STEP not set')
def test_golden_cell_import_reproduces_axis_line(tmp_path, occ):
    """Roles from the spike map (part excluded) + a circle of holder Part1 on the axis -> same axis line and base."""
    from awm_core import cad_roles
    from awm_geometry import cli, miniyaml, scene

    step = Path(os.environ['AWM_CELL_STEP'])
    old_map = miniyaml.load(REPO / 'docs' / 'generated' / 'spike' / 'cad_map_cell.yaml')
    tree_json = tmp_path / 'cell_components.json'
    assert cli.main(['inspect-assembly', '--step', str(step), '--out', str(tree_json), '--no-cylinders']) == 0
    tree = json.loads(tree_json.read_text())['components']
    assignments = [dict(a, role='exclude') if a['role'] == 'part' else a
                   for a in cad_roles.from_cad_map(tree, old_map)]
    roles = tmp_path / 'roles.json'
    _write_roles(roles, step, '+Y', assignments, collision=old_map.get('collision', {}),
                 contact_tolerance_mm=old_map['contact_tolerance_mm'])
    # circle of Part1:1 on the old axis line (largest radius): the test finds it, the UI user clicks it
    from OCC.Core.TopAbs import TopAbs_EDGE

    from awm_geometry import circle_pick, part, topology
    from awm_geometry.occ_io import GeometryError

    loaded = part.load_part(step, 'Part1:1')
    edges = topology.index_map(loaded.shape, TopAbs_EDGE)
    p0 = np.array(old_map['frames']['positioner_axis']['point_file_mm'], dtype=float)
    d0 = np.array(old_map['frames']['positioner_axis']['direction_file'], dtype=float)
    best = None
    for k in range(1, edges.Size() + 1):
        try:
            fr = circle_pick.circle_frame(circle_pick.edge_by_id(loaded.shape, f'E{k}'))
        except GeometryError:
            continue
        c, a = np.array(fr['center_mm']), np.array(fr['axis'])
        if abs(abs(a @ d0) - 1.0) < 1e-6 and np.linalg.norm(np.cross(c - p0, d0)) < 0.05 \
                and (best is None or fr['radius_mm'] > best[1]):
            best = (f'E{k}', fr['radius_mm'])
    assert best is not None, 'no circle of Part1:1 on the spike axis line'
    pick = tmp_path / 'axis_pick.json'
    assert cli.main(['pick-circle', '--step', str(step), '--component', 'Part1:1', '--edge', best[0],
                     '--up-axis', '+Y', '--out', str(pick)]) == 0
    best = (pick, None)
    cmap = tmp_path / 'cad_map.yaml'
    assert cli.main(['apply-roles', '--step', str(step), '--tree', str(tree_json), '--roles', str(roles),
                     '--axis', str(best[0]), '--out', str(cmap)]) == 0
    repo = tmp_path / 'repo'
    assert cli.main(['export-scene', '--step', str(step), '--map', str(cmap), '--repo', str(repo)]) == 0
    new = miniyaml.load(repo / 'cell' / 'model' / 'frames.yaml')
    ref = scene.cell_frames(json.loads(tree_json.read_text())['components'], old_map)
    a_new = np.array(new['world_T_positioner_axis']['direction'])
    p_new = np.array(new['world_T_positioner_axis']['point_mm'])
    angle = math.degrees(math.acos(min(1.0, abs(float(a_new @ ref['axis_direction'])))))
    off = np.linalg.norm(np.cross(p_new - ref['axis_point_mm'], ref['axis_direction']))
    assert angle <= 0.1 and off <= 0.05
    np.testing.assert_allclose(new['world_T_base']['position_mm'],
                               scene.pose_mm(ref['world_T_base'])['position_mm'], atol=1e-6)
