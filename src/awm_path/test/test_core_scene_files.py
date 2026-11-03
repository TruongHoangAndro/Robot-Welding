"""WP-S S.4 C1 / C2 (pure part): job scene (D19 v2) and cell manifest files, scene record and hash, clearance
padding (TST-002 Rev 0.3 §5.1 F4, F5, §9 records). Synthetic data only."""

import json
import struct

import numpy as np
import pytest

from awm_core.hashing import sha256_file
from awm_path.core.scene_files import (CLEARANCE_METHOD, DEFAULT_CLEARANCE_LINKS, SceneFileError,
                                       clearance_padding_mm, load_cell_manifest, load_job_scene, part_object_files,
                                       scene_hash, world_T_object)

HEX = 'ab' * 32


def _write_stl(path, tris):
    with open(path, 'wb') as fh:
        fh.write(b'\0' * 80 + struct.pack('<I', len(tris)))
        for t in tris:
            fh.write(struct.pack('<3f', 0, 0, 0) + struct.pack('<9f', *np.asarray(t, float).reshape(-1)) + b'\0\0')


def _job(tmp_path, *, pose=([10.0, 20.0, 30.0], [0.0, 0.0, 0.0, 1.0]), tamper=False):
    job = tmp_path / 'jobs' / 'J_A'
    (job / 'cache').mkdir(parents=True)
    stl = job / 'cache' / 'collision.stl'
    _write_stl(stl, [[[0, 0, 0], [1, 0, 0], [0, 1, 0]], [[0, 0, 0], [0, 1, 0], [0, 0, 1]]])
    (job / 'model.step').write_text('step')
    digest = sha256_file(stl)
    doc = {
        'schema_version': '2.0', 'created_by': 'test', 'created_at': '2026-11-03T00:00:00Z',
        'inputs': [{'name': 'model.step', 'sha256': sha256_file(job / 'model.step')},
                   {'name': 'cache/collision.stl', 'sha256': digest}],
        'source': 'fixture_cad', 'verified': False,
        'flange_T_part': {'position_mm': pose[0], 'quat_xyzw': pose[1]},
        'objects': [{'object_id': 'part', 'role': 'part', 'parent_frame': 'positioner_flange',
                     'pose_mm': {'position_mm': pose[0], 'quat_xyzw': pose[1]},
                     'mesh': {'file': 'cache/collision.stl', 'sha256': digest, 'face_ids': True},
                     'padding_mm': 0.0}],
        'scene_sha256': HEX,
    }
    (job / 'placement.json').write_text(json.dumps(doc))
    if tamper:
        _write_stl(stl, [[[0, 0, 0], [2, 0, 0], [0, 1, 0]]])
    return job


def test_load_job_scene_reads_objects_in_mm(tmp_path) -> None:
    job = _job(tmp_path)
    scene = load_job_scene(job / 'placement.json', job)
    assert scene.scene_sha256 == HEX
    assert scene.file_sha256 == sha256_file(job / 'placement.json')
    (obj,) = scene.objects
    assert obj.object_id == 'part' and obj.parent_frame == 'positioner_flange' and obj.face_ids
    assert np.allclose(obj.parent_T_obj_mm[:3, 3], [10.0, 20.0, 30.0])
    assert obj.triangles.shape == (2, 3) and obj.vertices_mm.shape == (4, 3)
    assert obj.mesh_sha256 == sha256_file(job / 'cache' / 'collision.stl')
    assert obj.mesh_file == job / 'cache' / 'collision.stl'


def test_part_object_files(tmp_path) -> None:
    job = _job(tmp_path)
    scene = load_job_scene(job / 'placement.json', job)
    obj, stl, faces = part_object_files(scene)
    assert obj.object_id == 'part' and stl == job / 'cache' / 'collision.stl'
    assert faces == job / 'cache' / 'collision_faces.json'
    doc = json.loads((job / 'placement.json').read_text())
    doc['objects'][0]['mesh']['face_ids'] = False
    (job / 'placement.json').write_text(json.dumps(doc))
    with pytest.raises(SceneFileError, match='one part object'):
        part_object_files(load_job_scene(job / 'placement.json', job))


def test_load_job_scene_refuses_stale_mesh(tmp_path) -> None:
    job = _job(tmp_path, tamper=True)
    with pytest.raises(SceneFileError, match='collision.stl'):
        load_job_scene(job / 'placement.json', job)


def test_load_job_scene_refuses_invalid_schema(tmp_path) -> None:
    job = _job(tmp_path)
    doc = json.loads((job / 'placement.json').read_text())
    doc['objects'][0]['parent_frame'] = 'tool0'
    (job / 'placement.json').write_text(json.dumps(doc))
    with pytest.raises(SceneFileError):
        load_job_scene(job / 'placement.json', job)


def test_world_T_object_composes_the_flange() -> None:
    flange = np.eye(4)
    flange[:3, 3] = [100.0, 0.0, 0.0]
    obj_t = np.eye(4)
    obj_t[:3, 3] = [0.0, 5.0, 0.0]
    from awm_path.core.scene_files import JobObject
    o = JobObject('part', 'part', 'positioner_flange', obj_t, np.zeros((3, 3)), np.zeros((1, 3), int), HEX, True)
    assert np.allclose(world_T_object(o, flange)[:3, 3], [100.0, 5.0, 0.0])
    w = JobObject('fix', 'fixture', 'world', obj_t, np.zeros((3, 3)), np.zeros((1, 3), int), HEX, False)
    assert np.allclose(world_T_object(w, flange)[:3, 3], [0.0, 5.0, 0.0])


def test_load_cell_manifest_ids(tmp_path) -> None:
    p = tmp_path / 'scene_manifest.json'
    p.write_text(json.dumps({'scene_sha256': HEX, 'created_at': 'x', 'objects': [{'object_id': 'a'}]}))
    cell = load_cell_manifest(p)
    assert cell.scene_sha256 == HEX and cell.file_sha256 == sha256_file(p) and cell.object_ids == ('a',)
    p.write_text(json.dumps({'objects': []}))
    with pytest.raises(SceneFileError, match='scene_sha256'):
        load_cell_manifest(p)


def test_scene_hash_is_canonical_and_content_based() -> None:
    a = {'cell': {'scene_sha256': HEX}, 'padding': {'b': 3.0, 'a': 0.0}}
    b = {'padding': {'a': 0.0, 'b': 3.0}, 'cell': {'scene_sha256': HEX}}
    assert scene_hash(a) == scene_hash(b)
    assert len(scene_hash(a)) == 64
    assert scene_hash(a) != scene_hash({**a, 'padding': {'a': 0.0, 'b': 5.0}})


def test_clearance_padding_inflates_only_the_clearance_links() -> None:
    f4 = {'head_body': 3.0, 'head_mount': 3.0, 'conduit': 3.0, 'nozzle_body': 3.0, 'nozzle_tip': 0.0,
          'forearm_link': 3.0}
    pad = clearance_padding_mm(f4, DEFAULT_CLEARANCE_LINKS, 5.0)
    assert pad == {'head_body': 5.0, 'head_mount': 5.0, 'conduit': 5.0, 'nozzle_body': 0.0, 'nozzle_tip': 0.0,
                   'forearm_link': 0.0}
    with pytest.raises(ValueError, match='not_a_link'):
        clearance_padding_mm(f4, ('head_body', 'not_a_link'), 5.0)
    with pytest.raises(ValueError):
        clearance_padding_mm(f4, DEFAULT_CLEARANCE_LINKS, -1.0)


def test_default_clearance_links_follow_tst002_f5() -> None:
    """F5 / DES-001 Rev 0.6 §5.4: head body, mount and conduit (not the nozzle, which is at the contact point)."""
    assert DEFAULT_CLEARANCE_LINKS == ('head_body', 'head_mount', 'conduit')
    assert 'unpadded' in CLEARANCE_METHOD and 'validity' in CLEARANCE_METHOD
