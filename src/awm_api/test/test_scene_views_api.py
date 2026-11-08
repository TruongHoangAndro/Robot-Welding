"""G1.20 (close-out B2 M4, m5): the cell scene manifest for the S4 cell view (static objects in world, rotating
objects under positioner_flange) with read-only mesh serving, and cached component / head edge views."""

import json

from conftest import auth, wait_run
from test_cell_model_api import CELL, argv_lines, imported_cell


def scene(repo):
    model = repo / 'cell' / 'model'
    model.mkdir(parents=True, exist_ok=True)
    pose = {'position_mm': [100.0, -150.0, -500.0], 'quat_xyzw': [0.0, 0.0, 0.0, 1.0]}
    objs = [
        {'object_id': 'frame_1', 'role': 'cell_static', 'component': 'Frame:1', 'parent_frame': 'world',
         'pose_mm': pose, 'collision': 'hull', 'mesh': {'file': 'frame_1.stl', 'sha256': '0' * 64}},
        {'object_id': 'holder_1', 'role': 'positioner_rotating', 'component': 'Holder:1',
         'parent_frame': 'positioner_flange', 'pose_mm': pose, 'collision': 'hull',
         'mesh': {'file': 'holder_1.stl', 'sha256': '1' * 64}},
    ]
    (model / 'scene_manifest.json').write_text(json.dumps({'schema_version': '2.0', 'unit': 'mm', 'objects': objs}))
    for o in objs:
        (model / o['mesh']['file']).write_bytes(b'solid x\nendsolid x\n')
    (model / 'other.stl').write_bytes(b'solid y\nendsolid y\n')
    return model


def test_scene_lists_objects_with_frames_and_mesh_urls(client, repo):
    scene(repo)
    for user in ('op', 'eng', 'adm'):
        r = client.get('/api/v1/cell/scene', headers=auth(client, user))
        assert r.status_code == 200, r.text
    d = r.json()
    assert d['unit'] == 'mm'
    by = {o['object_id']: o for o in d['objects']}
    assert by['frame_1']['parent_frame'] == 'world' and by['holder_1']['parent_frame'] == 'positioner_flange'
    assert by['frame_1']['pose_mm']['position_mm'] == [100.0, -150.0, -500.0]
    assert by['holder_1']['mesh_url'] == '/api/v1/cell/meshes/scene/holder_1.stl'


def test_scene_missing_404(client):
    r = client.get('/api/v1/cell/scene', headers=auth(client, 'eng'))
    assert r.status_code == 404 and r.json()['type'] == '/problems/no-scene'


def test_scene_mesh_served_only_when_listed(client, repo):
    scene(repo)
    h = auth(client, 'op')
    assert client.get('/api/v1/cell/meshes/scene/frame_1.stl', headers=h).content.startswith(b'solid x')
    assert client.get('/api/v1/cell/meshes/scene/other.stl', headers=h).status_code == 403
    assert client.get('/api/v1/cell/meshes/scene/..%2Fscene_manifest.json', headers=h).status_code in (403, 404)
    assert client.get('/api/v1/cell/meshes/scene/frame_1.stl').status_code == 401


def test_component_view_reuses_cached_view(client, repo, tmp_path, monkeypatch):
    h = auth(client, 'adm')
    imported_cell(client, h)
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    body = {'component_path': 'Pos:1 / Disc:1'}
    r = client.post(f'{CELL}/component_view', headers=h, json=body)
    assert r.status_code == 202
    key = r.json()['key']
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    again = client.post(f'{CELL}/component_view', headers=h, json=body)
    assert again.status_code == 200 and again.json() == {'key': key, 'cached': True}
    exports = lambda: [a for a in argv_lines(log) if a[0] == 'export-view']  # noqa: E731
    assert len(exports()) == 1                             # no second export
    # a new cell STEP makes the cached view unusable -> exported again
    (repo / 'cell' / 'cad' / 'cell.step').write_text('ISO-10303-21; cell v2')
    third = client.post(f'{CELL}/component_view', headers=h, json=body)
    assert third.status_code == 202
    wait_run(client, h, third.json()['run_id'])
    assert len(exports()) == 2


# -- review I2 / m7: the cache hit needs the D20 schema, the current exporter version and the STEP input by name ------

def _view(client, h, body=None):
    return client.post(f'{CELL}/component_view', headers=h, json=body or {'component_path': 'Pos:1 / Disc:1'})


def _exported(client, h, repo):
    r = _view(client, h)
    assert r.status_code == 202, r.text
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    path = repo / 'cell' / 'cad' / 'views' / r.json()['key'] / 'topology.json'
    return path, json.loads(path.read_text())


def test_cache_miss_when_exporter_version_differs(client, repo):
    h = auth(client, 'adm')
    imported_cell(client, h)
    path, topo = _exported(client, h, repo)
    assert _view(client, h).status_code == 200
    topo['created_by'] = 'awm_geometry 0.0-old (pythonocc 0.0-fake)'
    path.write_text(json.dumps(topo))
    assert _view(client, h).status_code == 202


def test_cache_miss_when_schema_version_differs(client, repo):
    h = auth(client, 'adm')
    imported_cell(client, h)
    path, topo = _exported(client, h, repo)
    topo['schema_version'] = '1.0'
    path.write_text(json.dumps(topo))
    assert _view(client, h).status_code == 202


def test_cache_matches_the_step_input_by_name(client, repo):
    h = auth(client, 'adm')
    imported_cell(client, h)
    path, topo = _exported(client, h, repo)
    sha = topo['inputs'][0]['sha256']
    topo['inputs'] = [{'name': 'other.step', 'sha256': sha}]  # right hash under another name: no hit
    path.write_text(json.dumps(topo))
    assert _view(client, h).status_code == 202


def test_cache_non_dict_inputs_no_500(client, repo):
    h = auth(client, 'adm')
    imported_cell(client, h)
    path, topo = _exported(client, h, repo)
    topo['inputs'] = ['cell.step', 7] + topo['inputs']
    path.write_text(json.dumps(topo))
    r = _view(client, h)
    assert r.status_code == 202
    wait_run(client, h, r.json()['run_id'])
    topo['inputs'] = 'cell.step'
    path.write_text(json.dumps(topo))
    assert _view(client, h).status_code == 202


def test_cache_miss_when_up_axis_differs(client, repo):
    h = auth(client, 'adm')
    imported_cell(client, h)
    _exported(client, h, repo)
    meta = repo / 'cell' / 'cad' / 'cell_meta.json'
    doc = json.loads(meta.read_text())
    doc['up_axis'] = '+Z'
    meta.write_text(json.dumps(doc))
    assert _view(client, h).status_code == 202


def test_head_view_reuses_cached_view(client, repo, tmp_path, monkeypatch):
    h = auth(client, 'adm')
    cad = repo / 'cell' / 'cad'
    cad.mkdir(parents=True, exist_ok=True)
    (cad / 'head.step').write_text('ISO-10303-21; head')
    (cad / 'cad_map_head.yaml').write_text('roles: {}\n')
    log = tmp_path / 'argv.log'
    monkeypatch.setenv('FAKE_ARGV_LOG', str(log))
    r = client.post('/api/v1/cell/head/view', headers=h, json={'component': 'Nozzle:1'})
    assert r.status_code == 202, r.text
    assert wait_run(client, h, r.json()['run_id'])['state'] == 'succeeded'
    again = client.post('/api/v1/cell/head/view', headers=h, json={'component': 'Nozzle:1'})
    assert again.status_code == 200 and again.json() == {'key': r.json()['key'], 'cached': True}
    assert [a[0] for a in argv_lines(log)].count('export-view') == 1
