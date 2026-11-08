"""G1.22: pure checks of ``scripts/golden_rebuild.py`` (no OCC, no CAD, no ROS graph) and the golden data file.

The full rebuild needs the cell STEP and runs in the session fixture of the golden tests (``conftest.py``); these tests
pin the refusal rules: wrong edge / radius, TCP off, unexpected cross-check reds, partial or rejected seams, a foreign
output folder, a changed fingerprint.
"""

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

WS = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('golden_rebuild', WS / 'scripts' / 'golden_rebuild.py')
gr = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gr)
DATA = Path(__file__).resolve().parent / 'data' / 'golden_564082.yaml'


@pytest.fixture(scope='module')
def data():
    return gr.load_data(DATA)


def test_data_file_is_the_default_and_complete(data):
    assert gr.DEFAULT_DATA.resolve() == DATA
    assert data['job']['travel_speed_mm_s'] == pytest.approx(2.3)
    assert [s['label'] for s in data['job']['seams']] == ['J1', 'J2']
    # decision 2026-11-08: the duplicate welding tool at the top level is excluded (OI-23)
    assert {'path': 'ASSY, Welding tool:1', 'role': 'exclude'} in data['cell']['roles']


def test_data_file_refusals(tmp_path):
    bad = tmp_path / 'g.yaml'
    bad.write_text('schema: other/0.1\n')
    with pytest.raises(gr.GoldenError, match='schema'):
        gr.load_data(bad)
    bad.write_text(DATA.read_text().replace('travel_speed_mm_s: 2.3', 'travel_speed_mm_s: 0'))
    with pytest.raises(gr.GoldenError, match='travel_speed'):
        gr.load_data(bad)
    bad.write_text(DATA.read_text().replace('joint_edge_id: JE11', 'jointid: JE11'))
    with pytest.raises(gr.GoldenError, match='joint_edge_id'):
        gr.load_data(bad)
    with pytest.raises(gr.GoldenError, match='not found'):
        gr.load_data(tmp_path / 'missing.yaml')


def test_check_circle():
    gr.check_circle({'edge_id': 'E3', 'radius_mm': 15.0004}, 'E3', 15.0, 0.001, 'axis')
    with pytest.raises(gr.GoldenError, match='radius'):
        gr.check_circle({'edge_id': 'E3', 'radius_mm': 15.01}, 'E3', 15.0, 0.001, 'axis')
    with pytest.raises(gr.GoldenError, match='expected .E3.'):
        gr.check_circle({'edge_id': 'E4', 'radius_mm': 15.0}, 'E3', 15.0, 0.001, 'axis')
    with pytest.raises(gr.GoldenError, match='radius'):
        gr.check_circle({'edge_id': 'E3'}, 'E3', 15.0, 0.001, 'axis')


def test_check_tcp():
    teach = {'component': 'C:1', 'edge_id': 'E7', 'radius_mm': 2.5}
    rec = {'method': 'teach_ui', 'component': 'C:1', 'edge_id': 'E7', 'mouth_radius_mm': 2.5}
    head = {'tool0_T_tcp_mm': {'xyz': [43.8288, 0.0, 279.6056]}, 'estimates': {'tcp': dict(rec)}}
    assert gr.check_tcp(head, teach, [43.829, 0.0, 279.606], 0.01, 0.001) == [43.8288, 0.0, 279.6056]
    head['tool0_T_tcp_mm']['xyz'] = [43.829, 0.0, 279.62]
    with pytest.raises(gr.GoldenError, match='from the expected'):
        gr.check_tcp(head, teach, [43.829, 0.0, 279.606], 0.01, 0.001)
    with pytest.raises(gr.GoldenError, match='tool0_T_tcp_mm'):
        gr.check_tcp({}, teach, [0, 0, 0], 0.01, 0.001)
    ok = {'tool0_T_tcp_mm': {'xyz': [43.829, 0.0, 279.606]}}
    for change, match in (({'mouth_radius_mm': 3.0}, 'radius'), ({'edge_id': 'E8'}, 'not the teach'),
                          ({'method': 'map'}, 'not the teach'), ({'mouth_radius_mm': None}, 'radius')):
        with pytest.raises(gr.GoldenError, match=match):
            gr.check_tcp(dict(ok, estimates={'tcp': dict(rec, **change)}), teach, [43.829, 0.0, 279.606], 0.01,
                         0.001)


def _topo():
    return {'solids': [{'solid_id': 'S1', 'component_path': '564082-Assembly-3 parts:2 / Pipe:1'},
                       {'solid_id': 'S2', 'component_path': '564082-Assembly-3 parts:2 / Solid:1'},
                       {'solid_id': 'S3', 'component_path': 'Part1:1'}],
            'joints': [{'joint_edge_id': 'JE9', 'edge_a': 'E10', 'edge_b': 'E20', 'closed': True,
                        'length_mm': 83.7863, 'solid_a': 'S1', 'solid_b': 'S2'},
                       {'joint_edge_id': 'JE11', 'edge_a': 'E30', 'edge_b': 'E40', 'closed': True,
                        'length_mm': 83.786, 'solid_a': 'S2', 'solid_b': 'S1'},
                       {'joint_edge_id': 'JE1', 'edge_a': 'E1', 'edge_b': 'E2', 'closed': True,
                        'length_mm': 83.786, 'solid_a': 'S3', 'solid_b': 'S1'}]}


def test_resolve_seams(data):
    picks = gr.resolve_seams(_topo(), data['job'], 0.05)
    assert [(p['seam_id'], p['edges'], p['sequence'], p['positioner_mode']) for p in picks] == [
        ('S01', ['E10'], 1, 'sync'), ('S02', ['E30'], 2, 'sync')]
    assert all(p['joint_type'] == 'butt' and p['reversed'] is False and p['arcs'] == 1 for p in picks)


@pytest.mark.parametrize('change, match', [
    (lambda t: t['joints'].pop(0), 'not in the job topology'),
    (lambda t: t['joints'][0].update(closed=False), 'not closed'),
    (lambda t: t['joints'][1].update(length_mm=80.0), 'length'),
    (lambda t: t['joints'][0].update(solid_b='S3'), 'not to a part component'),
])
def test_resolve_seams_refusals(data, change, match):
    topo = _topo()
    change(topo)
    with pytest.raises(gr.GoldenError, match=match):
        gr.resolve_seams(topo, data['job'], 0.05)


def test_check_cross_check():
    check = {'status': 'red', 'checks': [{'id': 1, 'status': 'green'}, {'id': 4, 'status': 'red', 'name': 'swept'}]}
    assert [c['id'] for c in gr.check_cross_check(check, [4])] == [4]
    with pytest.raises(gr.GoldenError, match=r'reds \[4\], expected \[\]'):
        gr.check_cross_check(check, [])
    check['checks'][0]['status'] = 'red'
    with pytest.raises(gr.GoldenError, match=r'reds \[1, 4\]'):
        gr.check_cross_check(check, [4])


def _axis(*flags):
    return {'seams': [{'seam_id': f'S0{k + 1}', 'coaxial': {'is_coaxial': f, 'axis_angle_deg': 0.0}}
                      for k, f in enumerate(flags)]}


def test_check_coaxial(data):
    assert gr.check_coaxial(_axis(True, False), data['job']) == {'S01': True, 'S02': False}
    with pytest.raises(gr.GoldenError, match='coaxial = False, expected True'):
        gr.check_coaxial(_axis(False, False), data['job'])
    with pytest.raises(gr.GoldenError, match='coaxial = True, expected False'):
        gr.check_coaxial(_axis(True, True), data['job'])
    with pytest.raises(gr.GoldenError, match='missing'):
        gr.check_coaxial(_axis(True), data['job'])
    with pytest.raises(gr.GoldenError, match='is_coaxial'):
        gr.check_coaxial({'seams': [{'seam_id': 'S01', 'coaxial': True}, {'seam_id': 'S02', 'coaxial': {}}]},
                         data['job'])


def _path(status='ok'):
    pts = [{'status': 'ok'}] * 3 + [{'status': status}]
    return {'rejected': [], 'seams': [
        {'seam_id': 'S01', 'mode': 'sync', 'points': pts, 'planning': {'max_travel_speed_mm_s': 6.96}},
        {'seam_id': 'S02', 'mode': 'sync', 'points': pts, 'planning': {'max_travel_speed_mm_s': 2.39}}]}


def test_check_path(data):
    out = gr.check_path(_path(), data['job'])
    assert [(s['label'], s['coverage_pct'], s['max_travel_speed_mm_s']) for s in out] == [
        ('J1', 100.0, 6.96), ('J2', 100.0, 2.39)]
    with pytest.raises(gr.GoldenError, match='3 / 4 points ok'):
        gr.check_path(_path('reach'), data['job'])
    rej = _path()
    rej['rejected'] = [{'seam_id': 'S02', 'reason': 'unreachable'}]
    with pytest.raises(gr.GoldenError, match='rejected'):
        gr.check_path(rej, data['job'])


def test_head_map(data):
    m = gr.head_map(data['head'], 'ab' * 32)
    assert m['source'] == {'file': 'head.step', 'sha256': 'ab' * 32, 'unit': 'mm', 'up_axis': '+Y'}
    assert m['match'] == 'instance_name' and m['default'] == 'deny'
    assert m['roles']['head_cad'] == [{'path': 'UR5E/UR7E:1 / ASSY, Welding tool:1', 'depth': 2}]
    assert 'point_file_mm' not in m['frames']['tcp']  # the TCP comes from the teach, never typed


def test_robot_frame_transform():
    from awm_core import frames, units
    pose = frames.xyz_rpy_to_matrix([-100.0, 518.4, -153.4], [units.deg_to_rad(v) for v in (10.0, -20.0, 30.0)])
    tree = [{'index': 0, 'parent': None, 'instance_name': ''},
            {'index': 1, 'parent': 0, 'instance_name': 'UR5E/UR7E:1', 'placement_mm': pose.tolist()}]
    t = gr.robot_frame_transform(tree, 'UR5E/UR7E:1')
    back = frames.xyz_rpy_to_matrix(t[:3], [units.deg_to_rad(v) for v in t[3:]])
    assert np.allclose(back @ pose, np.eye(4), atol=1e-9)
    with pytest.raises(gr.GoldenError, match='matches 0'):
        gr.robot_frame_transform(tree, 'Robot:1')


def test_prepare_out_and_fingerprint(tmp_path):
    out = tmp_path / 'g'
    gr.prepare_out(out)
    assert (out / gr.MARKER).is_file()
    (out / 'x.txt').write_text('1')
    gr.prepare_out(out)  # a golden folder is emptied
    assert not (out / 'x.txt').exists()
    foreign = tmp_path / 'repo'
    foreign.mkdir()
    (foreign / 'cell').mkdir()
    with pytest.raises(gr.GoldenError, match='refused'):
        gr.prepare_out(foreign)
    assert (foreign / 'cell').is_dir()
    step = tmp_path / 'cell.step'
    step.write_text('ISO-10303-21;')
    fp = gr.fingerprint(step, DATA)
    assert set(fp) == {'cell_step_sha256', 'data_sha256', 'script_sha256', 'sources_sha256', 'environment_sha256'}
    assert not gr.up_to_date(out, fp)
    (out / gr.SUMMARY).write_text(json.dumps({'status': 'ok', 'fingerprint': fp}))
    assert gr.up_to_date(out, fp)
    step.write_text('ISO-10303-21; changed')
    assert not gr.up_to_date(out, gr.fingerprint(step, DATA))


def test_source_digest_ignores_tests(tmp_path):
    pkg = tmp_path / 'src' / 'awm_core'
    (pkg / 'awm_core').mkdir(parents=True)
    (pkg / 'test').mkdir()
    (pkg / 'awm_core' / 'a.py').write_text('x = 1\n')
    d0 = gr.source_digest(tmp_path, ('awm_core',))
    (pkg / 'test' / 'test_a.py').write_text('pass\n')
    assert gr.source_digest(tmp_path, ('awm_core',)) == d0
    (pkg / 'awm_core' / 'a.py').write_text('x = 2\n')
    assert gr.source_digest(tmp_path, ('awm_core',)) != d0


def test_no_part_data_in_the_script():
    """Design rule 2: component names, edge / joint ids and speeds of the golden part live in the data file only."""
    text = (WS / 'scripts' / 'golden_rebuild.py').read_text()
    for token in ('564082-Assembly', 'Part1:1', 'E3955', 'JE9', 'JE11', 'LightWELD:1', 'UR5E', '2.3'):
        assert token not in text, token
    assert not math.isnan(0.0)


def _pkg(root, name, deps):
    d = root / 'src' / name
    d.mkdir(parents=True)
    (d / 'package.xml').write_text('<package format="3"><name>%s</name>%s</package>' % (
        name, ''.join(f'<{kind}>{dep}</{kind}>' for kind, dep in deps)))


def test_fingerprint_packages_are_the_dependency_closure(tmp_path):
    _pkg(tmp_path, 'awm_api', [('depend', 'awm_core'), ('exec_depend', 'awm_verify'), ('depend', 'rclpy')])
    _pkg(tmp_path, 'awm_path', [('depend', 'awm_bringup'), ('test_depend', 'awm_testonly')])
    _pkg(tmp_path, 'awm_program', [('build_depend', 'awm_path')])
    _pkg(tmp_path, 'awm_verify', [('depend', 'awm_cad')])
    for name in ('awm_core', 'awm_bringup', 'awm_cad', 'awm_testonly', 'awm_web_unused'):
        _pkg(tmp_path, name, [])
    assert gr.fingerprint_packages(tmp_path) == ['awm_api', 'awm_bringup', 'awm_cad', 'awm_core', 'awm_path',
                                                 'awm_program', 'awm_verify']
    real = gr.fingerprint_packages(WS)
    assert {'awm_bringup', 'awm_geometry', 'awm_seam', 'awm_description', 'awm_core'} <= set(real)


def test_environment_digest(monkeypatch, tmp_path):
    env = gr.environment(WS)
    assert set(env) == {'dockerfile_sha256', 'occ_python', 'occ_version', 'ur_analytic_ik', 'ur_description'}
    assert env['dockerfile_sha256'] == gr.sha256_file(WS / 'docker' / 'Dockerfile')
    assert all(isinstance(v, str) and v for v in env.values()), env
    monkeypatch.setattr(gr, 'environment', lambda ws=WS: dict(env, occ_version='0.0'))
    step = tmp_path / 'cell.step'
    step.write_text('x')
    monkeypatch.setattr(gr, 'source_digest', lambda ws, packages=None: 'src')
    a = gr.fingerprint(step, DATA)
    monkeypatch.setattr(gr, 'environment', lambda ws=WS: env)
    assert gr.fingerprint(step, DATA)['environment_sha256'] != a['environment_sha256']
