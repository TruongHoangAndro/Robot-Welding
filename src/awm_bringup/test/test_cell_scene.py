"""Pure logic of the cell scene loader (no ROS graph): manifest, padding, positioner pose, allowed pairs."""

import math
import struct
import subprocess
from pathlib import Path

import numpy as np
import pytest
import yaml

from awm_bringup import cell_scene as cs
from awm_core.hashing import sha256_file

SHARE = Path(__file__).resolve().parents[1]


def _write_stl(path: Path, tris) -> None:
    data = b'\0' * 80 + struct.pack('<I', len(tris))
    for t in tris:
        data += struct.pack('<3f', 0, 0, 0) + b''.join(struct.pack('<3f', *v) for v in t) + b'\0\0'
    path.write_bytes(data)


def _manifest(tmp_path, objects):
    p = tmp_path / 'scene.yaml'
    p.write_text(yaml.safe_dump({'objects': objects}))
    return p


POSE = {'position_mm': [10.0, 0.0, 0.0], 'quat_xyzw': [0.0, 0.0, 0.0, 1.0]}


def test_nominal_manifest_loads():
    objs = cs.load_manifest(SHARE / 'config' / 'cell_scene_nominal.yaml')
    assert [o.object_id for o in objs] == ['pismo_frame']
    assert objs[0].parent_frame == 'world' and objs[0].role == 'cell_static'
    top = objs[0].parent_T_obj_mm[2, 3] + objs[0].box_mm[2] / 2
    assert top == pytest.approx(0.0)


def test_mesh_object_and_hash(tmp_path):
    (tmp_path / 'm').mkdir()
    _write_stl(tmp_path / 'm' / 'h.stl', [[(0, 0, 0), (10, 0, 0), (0, 10, 0)]])
    digest = sha256_file(tmp_path / 'm' / 'h.stl')
    objs = cs.load_manifest(_manifest(tmp_path, [{
        'object_id': 'holder_a', 'parent_frame': 'positioner_flange', 'pose_mm': POSE,
        'mesh': {'file': 'm/h.stl', 'sha256': digest}, 'padding_mm': 3.0}]))
    assert objs[0].role == 'positioner_rotating' and objs[0].mesh_sha256 == digest
    assert objs[0].vertices_mm.max() == pytest.approx(10.0) and objs[0].triangles.shape == (1, 3)
    with pytest.raises(cs.ManifestError, match='sha256'):
        cs.load_manifest(_manifest(tmp_path, [{
            'object_id': 'holder_a', 'parent_frame': 'world', 'pose_mm': POSE,
            'mesh': {'file': 'm/h.stl', 'sha256': '0' * 64}}]))


@pytest.mark.parametrize('bad', [
    {'object_id': 'Bad-Id', 'parent_frame': 'world', 'pose_mm': POSE, 'box_mm': [1, 1, 1]},
    {'object_id': 'a', 'parent_frame': 'tool0', 'pose_mm': POSE, 'box_mm': [1, 1, 1]},
    {'object_id': 'a', 'parent_frame': 'world', 'role': 'positioner_rotating', 'pose_mm': POSE, 'box_mm': [1, 1, 1]},
    {'object_id': 'a', 'parent_frame': 'world', 'pose_mm': POSE},
    {'object_id': 'a', 'parent_frame': 'world', 'pose_mm': POSE, 'box_mm': [1, 0, 1]},
    {'object_id': 'a', 'parent_frame': 'world', 'box_mm': [1, 1, 1],
     'pose_mm': {'position_mm': [0, 0, 0], 'quat_xyzw': [0, 0, 0, 2]}},
])
def test_manifest_errors(tmp_path, bad):
    with pytest.raises(cs.ManifestError):
        cs.load_manifest(_manifest(tmp_path, [bad]))


def test_empty_manifest(tmp_path):
    assert cs.load_manifest(_manifest(tmp_path, [])) == []


def test_link_padding_except_nozzle_tip():
    pad = cs.link_padding_m(['head_body', 'nozzle_body', 'nozzle_tip', 'shoulder_link'], 3.0, ['nozzle_tip'])
    assert pad == {'head_body': 0.003, 'nozzle_body': 0.003, 'nozzle_tip': 0.0, 'shoulder_link': 0.003}
    with pytest.raises(ValueError):
        cs.link_padding_m(['a'], -1.0, [])


def test_rotating_object_follows_positioner():
    obj = cs.SceneObject('h', 'positioner_rotating', 'positioner_flange',
                         np.array([[1, 0, 0, 0], [0, 1, 0, 50.0], [0, 0, 1, 0], [0, 0, 0, 1.0]]))
    flange = cs.world_T_flange_mm(np.array([700.0, 0, 300]), np.array([0, 0, 0, 1.0]),
                                  np.array([1.0, 0, 0]), math.pi / 2)
    t = cs.world_pose_mm(obj, flange)
    assert np.allclose(t[:3, 3], [700, 0, 350])  # +Y 50 mm rotated 90 deg about X -> +Z
    static = cs.SceneObject('p', 'cell_static', 'world', np.eye(4))
    assert np.allclose(cs.world_pose_mm(static, flange), np.eye(4))


@pytest.fixture(scope='module')
def urdf_xml():
    share = subprocess.run(['ros2', 'pkg', 'prefix', 'awm_description'], capture_output=True, text=True)
    if share.returncode != 0:
        pytest.skip('awm_description not built')
    xacro = Path(share.stdout.strip()) / 'share' / 'awm_description' / 'urdf' / 'awm_cell_v2.urdf.xacro'
    return subprocess.run(['xacro', str(xacro)], capture_output=True, text=True, check=True).stdout


def test_world_fixed_links_and_allowed_pairs(urdf_xml):
    fixed = cs.world_fixed_links(urdf_xml)
    assert {'world', 'base_link', 'base_link_inertia', 'positioner_axis'} <= fixed
    assert not ({'shoulder_link', 'tool0', 'positioner_flange', 'nozzle_tip', 'head_body'} & fixed)
    links = cs.collision_links(urdf_xml)
    assert {'head_mount', 'head_body', 'nozzle_body', 'nozzle_tip', 'conduit', 'base_link_inertia'} <= set(links)
    objs = [cs.SceneObject('pismo_frame', 'cell_static', 'world', np.eye(4)),
            cs.SceneObject('holder', 'positioner_rotating', 'positioner_flange', np.eye(4))]
    pairs = cs.allowed_pairs(objs, fixed, links)
    assert pairs == [('base_link_inertia', 'pismo_frame')]  # never a moving link, never a rotating object


def test_merge_acm():
    names, m = cs.merge_acm(['a', 'b'], [[False, True], [True, False]], [('a', 'obj')])
    assert names == ['a', 'b', 'obj']
    assert m[0][1] and m[1][0] and m[0][2] and m[2][0] and not m[1][2] and not m[2][2]
