"""Torch orientation and approach / retract (pure numpy, no ROS)."""

import numpy as np
import pytest

from awm_path.poses import (PathError, offset_along_tool_z, seam_poses, tool_pose, torch_axis)

T = np.array([1.0, 0.0, 0.0])        # travel along +x
N_FLOOR = np.array([0.0, 0.0, 1.0])  # plate top
N_WALL = np.array([0.0, 1.0, 0.0])   # web side facing +y (concave root along x)


def _motion(**kw):
    m = {'travel_speed_mm_s': 6.0, 'work_angle_deg': 45.0, 'travel_angle_deg': 0.0,
         'stickout_mm': 15.0, 'focus_offset_mm': 0.0, 'weave_width_mm': 0.0, 'weave_freq_hz': 0.0}
    m.update(kw)
    return m


def test_bisector_axis():
    z = torch_axis(T, N_FLOOR, N_WALL, 'bisector', 45.0, 0.0)
    assert np.allclose(z, -(N_FLOOR + N_WALL) / np.sqrt(2))


def test_reference_face_work_angle():
    z45 = torch_axis(T, N_FLOOR, N_WALL, 'F1', 45.0, 0.0)
    assert np.allclose(z45, -(N_FLOOR + N_WALL) / np.sqrt(2))
    z30 = torch_axis(T, N_FLOOR, N_WALL, 'F1', 30.0, 0.0)
    # 30 deg from the floor: the torch lies flatter, pointing more towards the wall.
    assert np.isclose(np.degrees(np.arcsin(-z30 @ N_FLOOR)), 30.0)
    assert -z30 @ N_WALL > -z45 @ N_WALL


def test_travel_angle_push():
    z = torch_axis(T, N_FLOOR, N_WALL, 'bisector', 45.0, 10.0)
    assert np.isclose(np.degrees(np.arcsin(z @ T)), 10.0)  # tip leans forward (push)
    zd = torch_axis(T, N_FLOOR, N_WALL, 'bisector', 45.0, -10.0)
    assert zd @ T < 0


def test_single_face_rejected():
    with pytest.raises(PathError, match='single adjacent face'):
        torch_axis(T, N_FLOOR, None, 'bisector', 45.0, 0.0)


def test_tool_pose_frames():
    z = torch_axis(T, N_FLOOR, N_WALL, 'bisector', 45.0, 10.0)
    pose = tool_pose([0.1, 0.0, 0.01], T, z)
    r = pose[:3, :3]
    assert np.allclose(r.T @ r, np.eye(3)) and np.isclose(np.linalg.det(r), 1.0)
    assert np.allclose(r[:, 2], z) and r[:, 0] @ T > 0.98
    up = tool_pose([0.1, 0.0, 0.01], T, z, tool_roll='up')
    assert np.allclose(up[:3, 2], z) and up[2, 0] > 0
    laser = tool_pose([0.1, 0.0, 0.01], T, z, focus_offset_m=0.001)
    assert np.allclose(laser[:3, 3] - pose[:3, 3], 0.001 * z)
    with pytest.raises(PathError):
        tool_pose([0, 0, 0], T, z, tool_roll='sideways')


def test_approach_offset():
    z = torch_axis(T, N_FLOOR, N_WALL, 'bisector', 45.0, 0.0)
    pose = tool_pose([0.0, 0.0, 0.0], T, z)
    app = offset_along_tool_z(pose, 0.05)
    assert np.allclose(app[:3, 3], -0.05 * z)
    assert np.allclose(app[:3, :3], pose[:3, :3])


def _geometry(n=11, length=0.05, corner=False):
    pts = []
    for k in range(n):
        s = length * k / (n - 1)
        t = T if not corner or k < n // 2 else np.array([0.0, -1.0, 0.0])
        n2 = N_WALL if not corner or k < n // 2 else np.array([1.0, 0.0, 0.0])
        pts.append({'s_m': s, 'p_m': [s, 0.0, 0.0], 't': t.tolist(), 'n1': N_FLOOR.tolist(),
                    'n2': n2.tolist()})
    return {'seam_id': 'S01', 'length_m': length, 'closed': False, 'faces': ['F1', 'F2'],
            'points': pts}


SEAM = {'seam_id': 'S01', 'joint_type': 'fillet', 'weld_side': 'bisector'}


def test_seam_poses():
    sp = seam_poses(_geometry(), SEAM, {'process': 'MIG', 'motion': _motion()}, 0.05)
    assert len(sp.weld) == 11
    assert np.allclose(sp.approach[:3, 3], sp.weld[0][:3, 3] - 0.05 * sp.weld[0][:3, 2])
    assert np.allclose(sp.retract[:3, 3], sp.weld[-1][:3, 3] - 0.05 * sp.weld[-1][:3, 2])


@pytest.mark.parametrize('geometry, seam, motion, msg', [
    (_geometry(), SEAM, _motion(weave_width_mm=2.0), 'weave'),
    (_geometry(), dict(SEAM, joint_type='butt'), _motion(), 'not supported'),
    (_geometry(corner=True), SEAM, _motion(), 'jumps'),
])
def test_seam_poses_rejections(geometry, seam, motion, msg):
    with pytest.raises(PathError, match=msg):
        seam_poses(geometry, seam, {'process': 'MIG', 'motion': motion}, 0.05)
