"""CP-61 TCP teach: ``export-head --tcp-component --tcp-edge`` (nozzle-mouth circle -> tcp_laser).

Synthetic head of test_head_occ (flange, mount, body with a nozzle r 10 ending at NOZZLE_END in tool0, a long cable
behind the flange). Run with ``$AWM_OCC_PYTHON -m pytest`` (skipped without OCC).
"""

import json
import os
from pathlib import Path

import numpy as np
import pytest

from awm_core import frames
from awm_geometry import miniyaml

from test_head_occ import CABLE, NOZZLE_END, POSE_A, _head_step, _map

TOL = 1e-6
REPO = Path(__file__).resolve().parents[3]


@pytest.fixture(scope='module')
def head(tmp_path_factory, occ):
    root = tmp_path_factory.mktemp('teach')
    step = _head_step(root / 'head.step', POSE_A, cable=True)
    map_path = root / 'cad_map_head.yaml'
    map_path.write_text(miniyaml.dumps(_map(step, tcp={'nozzle_len_mm': 40.0})))
    return root, step, map_path


def _circle_at(step, component, point_file, tmp_path):
    """Edge id of the circle of ``component`` centred at ``point_file`` (file mm): what the Admin clicks."""
    from awm_geometry import cli

    view = tmp_path / f'view_{component}'
    assert cli.main(['export-view', '--step', str(step), '--component', component, '--up-axis', '+Z',
                     '--out', str(view)]) == 0
    topo = json.loads((view / 'topology.json').read_text())
    for e in (e for e in topo['edges'] if e['curve_type'] == 'circle'):
        out = view / f'{e["edge_id"]}.json'
        assert cli.main(['pick-circle', '--step', str(step), '--component', component, '--edge', e['edge_id'],
                         '--up-axis', '+Z', '--out', str(out)]) == 0
        if np.linalg.norm(np.array(json.loads(out.read_text())['center_file_mm']) - point_file) < 1e-6:
            return e['edge_id']
    raise AssertionError(f'no circle of {component} at {point_file}')


def _teach(root, step, map_path, edge, component='nozzle:1', *extra):
    from awm_geometry import cli

    repo = root / f'repo_{component}_{edge}_{len(extra)}'
    code = cli.main(['export-head', '--step', str(step), '--map', str(map_path), '--repo', str(repo),
                     '--tcp-component', component, '--tcp-edge', edge, '--taught-by', 'admin', *extra])
    head_yaml = repo / 'cell' / 'model' / 'head' / 'head.yaml'
    return code, (miniyaml.load(head_yaml) if head_yaml.is_file() else None)


def _file(p_tool0):
    return (POSE_A @ np.append(p_tool0, 1.0))[:3]


def test_teach_tip_equals_known_tip(head, tmp_path):
    root, step, map_path = head
    edge = _circle_at(step, 'nozzle:1', _file(NOZZLE_END), tmp_path)
    code, doc = _teach(root, step, map_path, edge)
    assert code == 0
    np.testing.assert_allclose(doc['tool0_T_tcp_mm']['xyz'], NOZZLE_END, atol=TOL)
    np.testing.assert_allclose(doc['beam_axis_tool0'], [0.0, 0.0, 1.0], atol=TOL)
    np.testing.assert_allclose(doc['beam_exit_tool0_mm'], NOZZLE_END - [0.0, 0.0, 40.0], atol=TOL)


def test_stickout_moves_tcp_along_beam(head, tmp_path):
    root, step, map_path = head
    edge = _circle_at(step, 'nozzle:1', _file(NOZZLE_END), tmp_path)
    code, doc = _teach(root, step, map_path, edge, 'nozzle:1', '--stickout-mm', '5')
    assert code == 0
    np.testing.assert_allclose(doc['tool0_T_tcp_mm']['xyz'], NOZZLE_END + [0.0, 0.0, 5.0], atol=TOL)


def test_reverse_flips_beam(head, tmp_path):
    """Reverse on the nozzle mouth: the beam points into the head; the TCP stays at the tip (z > 0), axis -Z."""
    root, step, map_path = head
    edge = _circle_at(step, 'nozzle:1', _file(NOZZLE_END), tmp_path)
    code, doc = _teach(root, step, map_path, edge, 'nozzle:1', '--tcp-reverse')
    assert code == 0
    np.testing.assert_allclose(doc['beam_axis_tool0'], [0.0, 0.0, -1.0], atol=TOL)


def test_cable_end_edge_refused(head, tmp_path):
    root, step, map_path = head
    r, base, _length = CABLE
    edge = _circle_at(step, 'cable:1', _file(np.array(base)), tmp_path)
    code, doc = _teach(root, step, map_path, edge, 'cable:1')
    assert code != 0 and doc is None


def test_head_yaml_records_teach_ui(head, tmp_path):
    root, step, map_path = head
    edge = _circle_at(step, 'nozzle:1', _file(NOZZLE_END), tmp_path)
    code, doc = _teach(root, step, map_path, edge, 'nozzle:1', '--stickout-mm', '2.5')
    assert code == 0
    est = doc['estimates']['tcp']
    assert est['method'] == 'teach_ui' and est['edge_id'] == edge and est['component'] == 'nozzle:1'
    assert est['stickout_mm'] == 2.5 and est['taught_by'] == 'admin' and est['taught_at']
    assert est['behind_flange'] is False


def test_non_circle_edge_refused(head, tmp_path):
    from awm_geometry import cli

    root, step, map_path = head
    view = tmp_path / 'v'
    assert cli.main(['export-view', '--step', str(step), '--component', 'mount:1', '--up-axis', '+Z',
                     '--out', str(view)]) == 0
    line = next(e['edge_id'] for e in json.loads((view / 'topology.json').read_text())['edges']
                if e['curve_type'] == 'line')
    code, doc = _teach(root, step, map_path, line, 'mount:1')
    assert code != 0 and doc is None


@pytest.mark.skipif(not os.environ.get('AWM_CELL_STEP'), reason='AWM_CELL_STEP not set')
def test_golden_teach_matches_marked_tip(tmp_path, occ):
    """Teaching the LightWELD nozzle-mouth circle gives the TCP set in cad_map_head.yaml (Truong's mark, 2026-11-06)
    within 0.05 mm / 0.1 deg (stick-out 0)."""
    import math

    from awm_geometry import circle_pick, part, topology
    from awm_geometry.occ_io import GeometryError
    from OCC.Core.TopAbs import TopAbs_EDGE

    step = Path(os.environ['AWM_CELL_STEP']).parent / 'Dau_han_robot.step'
    if not step.is_file():
        pytest.skip(f'{step} not present')
    old = miniyaml.load(REPO / 'docs' / 'generated' / 'spike' / 'cad_map_head.yaml')
    tip = np.array(old['frames']['tcp']['point_file_mm'])
    axis = np.array(old['frames']['tcp']['axis_file'])
    cmap = dict(old)
    cmap['frames'] = dict(old['frames'], tcp={k: v for k, v in old['frames']['tcp'].items()
                                             if k not in ('point_file_mm', 'axis_file')})
    cmap['source'] = dict(old['source'], file=step.name)
    map_path = tmp_path / 'cad_map_head.yaml'
    map_path.write_text(miniyaml.dumps(cmap))
    loaded = part.load_part(step, 'LightWELD:1')
    edges = topology.index_map(loaded.shape, TopAbs_EDGE)
    found = None
    for k in range(1, edges.Size() + 1):
        try:
            fr = circle_pick.circle_frame(circle_pick.edge_by_id(loaded.shape, f'E{k}'))
        except GeometryError:
            continue
        if np.linalg.norm(np.array(fr['center_mm']) - tip) < 0.05 and abs(abs(np.array(fr['axis']) @ axis) - 1) < 1e-4:
            found = f'E{k}'
            break
    assert found, 'no circle of LightWELD:1 at the marked tip'
    from awm_geometry import cli
    repo = tmp_path / 'repo'
    assert cli.main(['export-head', '--step', str(step), '--map', str(map_path), '--repo', str(repo),
                     '--tcp-component', 'LightWELD:1', '--tcp-edge', found, '--taught-by', 'test']) == 0
    doc = miniyaml.load(repo / 'cell' / 'model' / 'head' / 'head.yaml')
    np.testing.assert_allclose(doc['tool0_T_tcp_mm']['xyz'], [43.8285, 0.0, 279.6060], atol=0.05)
    beam = np.array(doc['beam_axis_tool0'])
    assert math.degrees(math.acos(min(1.0, float(beam @ [0.866019, 0.0, 0.500011])))) <= 0.1
