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
    assert len(argv_lines(log)) == 1                       # no second export
    # a new cell STEP makes the cached view unusable -> exported again
    (repo / 'cell' / 'cad' / 'cell.step').write_text('ISO-10303-21; cell v2')
    third = client.post(f'{CELL}/component_view', headers=h, json=body)
    assert third.status_code == 202
    wait_run(client, h, third.json()['run_id'])
    assert len(argv_lines(log)) == 2
