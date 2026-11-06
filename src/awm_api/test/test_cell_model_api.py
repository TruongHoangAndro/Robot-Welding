"""CP-60 / CP-61 cell model API (S10): import, roles, positioner-axis pick, save, TCP teach; Admin writes, Engineer
reads, Operator neither writes nor uses S10 files. Fake CLIs (no OCC)."""

import json
from pathlib import Path

from conftest import auth, wait_run

CELL = '/api/v1/cell/model'


def tree(*paths):
    """Minimal inspect-assembly tree: root + top components named by ``paths`` (one level)."""
    comps = [{'index': 0, 'parent': None, 'depth': 0, 'instance_name': '', 'product_name': 'cell',
              'is_assembly': True, 'bbox_mm': None, 'placement_mm': None}]
    for i, p in enumerate(paths, 1):
        comps.append({'index': i, 'parent': 0, 'depth': 1, 'instance_name': p, 'product_name': p.split(':')[0],
                      'is_assembly': False, 'bbox_mm': {'min': [0, 0, 0], 'max': [1, 1, 1]},
                      'placement_mm': [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]})
    return {'schema_version': '2.0', 'source': {'file': 'cell.step', 'sha256': '0' * 64}, 'components': comps}


def imported_cell(client, h, argv_log=None, monkeypatch=None):
    if argv_log is not None:
        monkeypatch.setenv('FAKE_ARGV_LOG', str(argv_log))
    r = client.post(CELL, headers=h, files={'step': ('cell.step', b'ISO-10303-21; cell', 'application/step')},
                    data={'up_axis': '+Y'})
    assert r.status_code == 202, r.text
    run = wait_run(client, h, r.json()['run_id'])
    assert run['state'] == 'succeeded', run
    return run


def argv_lines(path: Path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_cell_import_admin_only(client):
    for user in ('op', 'eng'):
        r = client.post(CELL, headers=auth(client, user),
                        files={'step': ('cell.step', b'x', 'application/step')}, data={'up_axis': '+Y'})
        assert r.status_code == 403, (user, r.text)


def test_cell_import_runs_inspect_and_cell_view(client, repo, tmp_path, monkeypatch):
    h = auth(client, 'adm')
    log = tmp_path / 'argv.log'
    imported_cell(client, h, log, monkeypatch)
    cmds = [a[0] for a in argv_lines(log)]
    assert cmds == ['inspect-assembly', 'export-cell-view']
    cad = repo / 'cell' / 'cad'
    assert (cad / 'cell.step').is_file() and (cad / 'cell_components.json').is_file()
    status = client.get(CELL, headers=h).json()
    assert status['step']['file'] == 'cell.step' and status['up_axis'] == '+Y'
    assert status['roles']['state'] == 'missing' and status['axis'] is None


def test_engineer_can_read_cell_files_operator_cannot_edit(client, repo):
    h = auth(client, 'adm')
    imported_cell(client, h)
    e = auth(client, 'eng')
    assert client.get(f'{CELL}/files/cell_components.json', headers=e).status_code == 200
    assert client.get(f'{CELL}/files/../../secrets', headers=e).status_code in (404, 422)
    r = client.put(f'{CELL}/roles', headers=e, json={'assignments': []})
    assert r.status_code == 403


def test_roles_put_returns_problems_422(client, repo):
    h = auth(client, 'adm')
    imported_cell(client, h)
    (repo / 'cell' / 'cad' / 'cell_components.json').write_text(json.dumps(tree('UR:1', 'Pos:1', 'Frame:1')))
    r = client.put(f'{CELL}/roles', headers=h, json={'assignments': [{'path': 'Pos:1', 'role': 'rotary_fixture'}]})
    assert r.status_code == 422 and 'robot' in r.text
    ok = {'assignments': [{'path': 'UR:1', 'role': 'robot'}, {'path': 'Pos:1', 'role': 'rotary_fixture'}]}
    r = client.put(f'{CELL}/roles', headers=h, json=ok)
    assert r.status_code == 200, r.text
    roles = json.loads((repo / 'cell' / 'cad' / 'roles.json').read_text())
    assert roles['scope'] == 'cell' and roles['source']['up_axis'] == '+Y'
    template = json.loads((repo / 'cell' / 'roles_template.json').read_text())
    assert template['roles'] == {'UR:1': 'robot', 'Pos:1': 'rotary_fixture'}
    got = client.get(f'{CELL}/roles', headers=h).json()
    assert got['assignments'] == ok['assignments'] and got['prefilled'] is False


def test_roles_get_prefilled_from_template(client, repo):
    h = auth(client, 'adm')
    imported_cell(client, h)
    (repo / 'cell' / 'cad' / 'cell_components.json').write_text(json.dumps(tree('UR:1', 'Pos:1')))
    (repo / 'cell' / 'roles_template.json').write_text(json.dumps({'version': 2, 'roles': {'Pos:1': 'rotary_fixture'}}))
    got = client.get(f'{CELL}/roles', headers=h).json()
    assert got['prefilled'] is True and got['assignments'] == [{'path': 'Pos:1', 'role': 'rotary_fixture'}]


def test_component_view_and_axis_put(client, repo, tmp_path, monkeypatch):
    h = auth(client, 'adm')
    imported_cell(client, h)
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    r = client.post(f'{CELL}/component_view', headers=h, json={'component_path': 'Pos:1 / Disc:1'})
    assert r.status_code == 202, r.text
    key = r.json()['key']
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    assert client.get(f'{CELL}/views/{key}/topology.json', headers=h).status_code == 200
    r = client.put(f'{CELL}/axis', headers=h, json={'component_path': 'Pos:1 / Disc:1', 'edge_id': 'E7',
                                                    'reverse': True})
    assert r.status_code == 202, r.text
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    view_argv, pick_argv = argv_lines(log)
    assert view_argv[0] == 'export-view' and view_argv[view_argv.index('--component') + 1] == 'Disc:1'
    assert pick_argv[0] == 'pick-circle' and '--reverse' in pick_argv
    assert pick_argv[pick_argv.index('--edge') + 1] == 'E7' and pick_argv[pick_argv.index('--up-axis') + 1] == '+Y'
    assert client.get(CELL, headers=h).json()['axis']['edge_id'] == 'E7'


def test_save_runs_apply_roles_then_export_scene(client, repo, tmp_path, monkeypatch):
    h = auth(client, 'adm')
    imported_cell(client, h)
    r = client.post(f'{CELL}/save', headers=h, json={})
    assert r.status_code == 409 and 'roles' in r.text           # roles and axis needed first
    cad = repo / 'cell' / 'cad'
    (cad / 'roles.json').write_text('{}')
    (cad / 'axis_pick.json').write_text('{}')
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    r = client.post(f'{CELL}/save', headers=h, json={})
    assert r.status_code == 202, r.text
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    a, b = argv_lines(log)
    assert a[0] == 'apply-roles' and b[0] == 'export-scene' and '--job' not in b
    assert b[b.index('--map') + 1] == a[a.index('--out') + 1]


def test_tcp_put_passes_stickout_and_user(client, repo, tmp_path, monkeypatch):
    h = auth(client, 'adm')
    cad = repo / 'cell' / 'cad'
    r = client.put('/api/v1/cell/head/tcp', headers=h, json={'component': 'LightWELD:1', 'edge_id': 'E12',
                                                              'reverse': False, 'stickout_mm': 1.5})
    assert r.status_code == 409                                 # no head STEP / map in cell/cad yet
    cad.mkdir(parents=True, exist_ok=True)
    (cad / 'head.step').write_text('ISO-10303-21; head')
    (cad / 'cad_map_head.yaml').write_text('schema: cad_map/0.1\n')
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    r = client.put('/api/v1/cell/head/tcp', headers=h, json={'component': 'LightWELD:1', 'edge_id': 'E12',
                                                              'reverse': False, 'stickout_mm': 1.5})
    assert r.status_code == 202, r.text
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    (argv,) = argv_lines(log)
    assert argv[0] == 'export-head' and argv[argv.index('--tcp-edge') + 1] == 'E12'
    assert argv[argv.index('--stickout-mm') + 1] == '1.5' and argv[argv.index('--taught-by') + 1] == 'adm'
    assert '--tcp-reverse' not in argv
    e = auth(client, 'eng')
    assert client.put('/api/v1/cell/head/tcp', headers=e, json={'component': 'LightWELD:1', 'edge_id': 'E12',
                                                                'reverse': False, 'stickout_mm': 0}).status_code == 403
