"""Integration (WP-S S.4 C1 / C2): MoveGroupCellScene against ``sim_mock.launch.py`` (cell model v2, mock hardware).

Synthetic test data only (no part data): a cell manifest with one far box, a job with a 40 mm "part" cube placed
4 mm beside the head body of a test pose, and a "clamp" cube on the positioner flange that turns into the head body
at 90 deg. Positioner at a test pose, axis +X.
"""

import json
import struct

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from awm_bringup.testing import SimLaunch  # noqa: E402
from awm_core.hashing import sha256_file  # noqa: E402
from awm_path.backends.move_group import MoveGroupCellScene  # noqa: E402
from awm_path.core.kinematics import UrAnalyticKinematics, ik_tcp  # noqa: E402
from awm_path.core.scene_files import SceneFileError  # noqa: E402
from awm_path.core.states import config_label  # noqa: E402

POSITIONER_MM = np.array([430.0, 400.0, 450.0])
TCP_MM = np.array([430.0, 133.0, 100.0])          # test pose: beam down (-Z world), tool0 x -> -Y world
HEAD_HALF_D_MM = 17.5                              # H-BOX head body half depth (tool0 y = world -X here)
GAP_MM = 4.0
CUBE_MM = 40.0
HOME_DEG = [0.0, -90.0, 57.29577951308232, -57.29577951308232, -90.0, 0.0]


def _cube(path, size):
    h = size / 2.0
    v = np.array([[x, y, z] for x in (-h, h) for y in (-h, h) for z in (-h, h)])
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = [v[[a, b, c]] for a, b, c, d in faces] + [v[[a, c, d]] for a, b, c, d in faces]
    with open(path, 'wb') as fh:
        fh.write(b'\0' * 80 + struct.pack('<I', len(tris)))
        for t in tris:
            fh.write(struct.pack('<3f', 0, 0, 0) + struct.pack('<9f', *t.reshape(-1)) + b'\0\0')


def _files(tmp_path):
    cell = tmp_path / 'cell_scene.json'
    cell.write_text(json.dumps({'scene_sha256': 'cd' * 32, 'objects': [
        {'object_id': 'far_box', 'parent_frame': 'world', 'box_mm': [50, 50, 50],
         'pose_mm': {'position_mm': [-900, 0, 0], 'quat_xyzw': [0, 0, 0, 1]}}]}))
    job = tmp_path / 'job'
    (job / 'cache').mkdir(parents=True)
    stl = job / 'cache' / 'cube.stl'
    _cube(stl, CUBE_MM)
    digest = sha256_file(stl)
    part_world = TCP_MM + [-(HEAD_HALF_D_MM + GAP_MM + CUBE_MM / 2.0), 0.0, 120.0]
    obj = lambda oid, role, pos: {  # noqa: E731
        'object_id': oid, 'role': role, 'parent_frame': 'positioner_flange',
        'pose_mm': {'position_mm': [float(x) for x in pos], 'quat_xyzw': [0.0, 0.0, 0.0, 1.0]},
        'mesh': {'file': 'cache/cube.stl', 'sha256': digest, 'face_ids': False}, 'padding_mm': 0.0}
    doc = {'schema_version': '2.0', 'created_by': 'test', 'created_at': '2026-11-03T00:00:00Z',
           'inputs': [{'name': 'cache/cube.stl', 'sha256': digest}], 'source': 'study', 'verified': False,
           'flange_T_part': {'position_mm': [float(x) for x in part_world - POSITIONER_MM],
                             'quat_xyzw': [0.0, 0.0, 0.0, 1.0]},
           'objects': [obj('part', 'part', part_world - POSITIONER_MM), obj('clamp_test', 'clamp', [0, 0, 267])],
           'scene_sha256': 'ef' * 32}
    (job / 'placement.json').write_text(json.dumps(doc))
    return cell, job


@pytest.fixture(scope='module')
def backend(tmp_path_factory):
    tmp = tmp_path_factory.mktemp('cell_backend')
    cell, job = _files(tmp)
    args = {'cell_manifest': str(cell), 'positioner_xyz_mm': '%g %g %g' % tuple(POSITIONER_MM)}
    with SimLaunch(log_dir=tmp, launch_file='sim_mock.launch.py', launch_args=args) as sim:
        rclpy.init()
        node = rclpy.create_node('awm_path_cell_backend_test')
        b = MoveGroupCellScene(node, cell, job / 'placement.json', job, head_model='H-BOX (test)')
        try:
            try:
                b.load(timeout_s=180.0)
            except Exception:
                print(sim.log_tail(80))
                raise
            yield b, tmp, cell, job
        finally:
            node.destroy_node()
            rclpy.shutdown()


def _q_beside_cube(b):
    """Joint state with tcp_laser at TCP_MM, beam down, tool0 x -> -Y world (analytic IK, home configuration)."""
    kin = UrAnalyticKinematics()
    world = b.world_T_links(HOME_DEG, 0.0, ['base_link', 'tool0', 'tcp_laser'])
    tool0_T_tcp = np.linalg.inv(world['tool0']) @ world['tcp_laser']
    target = np.eye(4)
    target[:3, :3] = [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
    target[:3, 3] = TCP_MM
    base_T_tcp = np.linalg.inv(world['base_link']) @ target
    sols = [q for q in ik_tcp(kin, base_T_tcp, tool0_T_tcp) if config_label(q) == config_label(HOME_DEG)]
    assert sols
    return sols[0]


def test_record_and_scene_hash(backend) -> None:
    b, _, cell, job = backend
    rec = b.record
    assert rec['content']['cell']['scene_sha256'] == 'cd' * 32
    assert rec['files']['cell']['file_sha256'] == sha256_file(cell)
    assert rec['content']['job']['scene_sha256'] == 'ef' * 32
    assert [o['object_id'] for o in rec['content']['job']['objects']] == ['part', 'clamp_test']
    assert rec['content']['f4_padding_mm']['nozzle_tip'] == 0.0
    assert rec['content']['f4_padding_mm']['head_body'] == pytest.approx(3.0)
    assert rec['content']['clearance_links'] == ['head_body', 'head_mount', 'conduit']
    assert rec['content']['head_model'] == 'H-BOX (test)'
    assert len(rec['content']['robot_description_sha256']) == 64
    assert len(b.scene_hash) == 64 and rec['scene_hash'] == b.scene_hash
    assert b.theta_deg == pytest.approx(0.0, abs=1e-9)


def test_validity_and_rotating_job_objects(backend) -> None:
    b, *_ = backend
    q = _q_beside_cube(b)
    (r,) = b.check([q], 0.0)
    assert r.valid, r.contacts
    (r,) = b.check([q], 90.0)            # moves the positioner first; the clamp cube is now inside the head body
    assert b.theta_deg == pytest.approx(90.0, abs=0.01)
    assert not r.valid and any('clamp_test' in c for c in r.contacts), r.contacts
    (r,) = b.check([q], 0.0)
    assert r.valid, r.contacts


def test_clearance_by_inflated_links_and_padding_restored(backend) -> None:
    b, *_ = backend
    q = _q_beside_cube(b)
    assert b.check([q], 0.0)[0].valid                                    # F4: 4 mm gap > 3 mm padding
    (c5,) = b.check_clearance([q], 0.0, 5.0)
    assert not c5.valid and any('head_body' in c and 'part' in c for c in c5.contacts), c5.contacts
    (c3,) = b.check_clearance([q], 0.0, 3.5)
    assert c3.valid, c3.contacts
    assert b.check([q], 0.0)[0].valid                                    # F4 padding restored after the check
    assert b.f4_padding_mm() == b.record['content']['f4_padding_mm']


def test_wrong_joint_count_is_refused(backend) -> None:
    b, *_ = backend
    with pytest.raises(ValueError):
        b.check([[0.0] * 7], 0.0)


def test_cell_manifest_must_match_the_loaded_scene(backend) -> None:
    b, tmp, cell, job = backend
    other = tmp / 'other_cell.json'
    other.write_text(cell.read_text().replace('-900', '-901'))
    wrong = MoveGroupCellScene(b.node, other, job / 'placement.json', job)
    with pytest.raises(SceneFileError, match='manifest'):
        wrong.load(timeout_s=30.0)
