"""Unit tests of the pure D09 replay core (G1.7): no rclpy, synthetic programs (test data, not part data)."""

import copy
import math

import pytest

from awm_bringup import replay

ROBOT = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
POS = ['positioner_joint']
HOME = [0.0, -90.0, 57.0, -57.0, -90.0, 0.0, 0.0]


def _seg(sid, start, end, duration=2.0, kind='air', controller='robot_positioner'):
    mid = [(a + b) / 2.0 for a, b in zip(start, end)]
    return {'segment_id': sid, 'controller': controller, 'kind': kind, 'seam_id': None, 'hold_deg': [],
            'tcp_speed_mm_s': None, 'collision_checked': False,
            'points': [
                {'time_from_start_s': 0.0, 'positions_deg': list(start), 'velocities_deg_s': [0.0] * 7,
                 'accelerations_deg_s2': [0.0] * 7},
                {'time_from_start_s': duration / 2, 'positions_deg': mid, 'velocities_deg_s': [1.0] * 7,
                 'accelerations_deg_s2': [0.0] * 7},
                {'time_from_start_s': duration, 'positions_deg': list(end), 'velocities_deg_s': [0.0] * 7,
                 'accelerations_deg_s2': [0.0] * 7}]}


def _program():
    p1 = [5.0, -85.0, 60.0, -55.0, -88.0, 10.0, 30.0]
    return {'schema_version': '2.1', 'program_rev': 'P001',
            'joint_names': {'robot': list(ROBOT), 'positioner': list(POS)},
            'home': {'robot_deg': HOME[:6], 'positioner_deg': 0.0},
            'segments': [_seg('G001', HOME, p1), _seg('G002', p1, HOME, kind='index')],
            'events': []}


def test_joint_order_and_rad_conversion():
    prog = replay.ReplayProgram.from_doc(_program())
    assert prog.joint_names == ROBOT + POS
    seg = prog.segments[0]
    t, pos, vel, acc = seg.trajectory_rad()
    assert t == [0.0, 1.0, 2.0]
    assert pos[2][6] == pytest.approx(math.radians(30.0))
    assert vel[1][0] == pytest.approx(math.radians(1.0))
    assert len(acc[0]) == 7


def test_refuses_split_controllers():
    doc = _program()
    doc['segments'][0]['controller'] = 'robot'
    with pytest.raises(replay.ReplayRefused, match='robot_positioner'):
        replay.ReplayProgram.from_doc(doc)


def test_refuses_old_major_and_wrong_point_width():
    doc = _program()
    doc['schema_version'] = '1.0'
    with pytest.raises(replay.ReplayRefused, match='schema_version'):
        replay.ReplayProgram.from_doc(doc)
    doc = _program()
    doc['segments'][0]['points'][1]['positions_deg'] = [0.0] * 6
    with pytest.raises(replay.ReplayRefused, match='7 values'):
        replay.ReplayProgram.from_doc(doc)


def test_refuses_discontinuous_segments():
    doc = _program()
    doc['segments'][1]['points'][0]['positions_deg'][3] += 1.0
    with pytest.raises(replay.ReplayRefused, match='G002.*does not start where G001 ends'):
        replay.ReplayProgram.from_doc(doc)


def test_refuses_nonzero_end_velocity():
    doc = _program()
    doc['segments'][0]['points'][-1]['velocities_deg_s'][2] = 0.5
    with pytest.raises(replay.ReplayRefused, match='rest'):
        replay.ReplayProgram.from_doc(doc)


def test_start_state_check():
    prog = replay.ReplayProgram.from_doc(_program())
    seg = prog.segments[0]
    current = dict(zip(ROBOT + POS, HOME))
    assert replay.check_start_state(seg, prog.joint_names, current, tol_deg=0.05) == pytest.approx(0.0)
    near = dict(current, wrist_2_joint=-90.04)
    assert replay.check_start_state(seg, prog.joint_names, near, tol_deg=0.05) == pytest.approx(0.04)
    off = dict(current, positioner_joint=0.2)
    with pytest.raises(replay.StartStateMismatch, match='positioner_joint.*0.2'):
        replay.check_start_state(seg, prog.joint_names, off, tol_deg=0.05)
    missing = {k: v for k, v in current.items() if k != 'elbow_joint'}
    with pytest.raises(replay.StartStateMismatch, match='elbow_joint'):
        replay.check_start_state(seg, prog.joint_names, missing, tol_deg=0.05)


def test_segment_selection():
    prog = replay.ReplayProgram.from_doc(_program())
    assert [s.segment_id for s in prog.select(None)] == ['G001', 'G002']
    assert [s.segment_id for s in prog.select('G002')] == ['G002']
    assert [s.segment_id for s in prog.select('G001-G002')] == ['G001', 'G002']
    with pytest.raises(replay.ReplayRefused, match='G009'):
        prog.select('G009')


def test_end_deviation_and_tracking_summary():
    prog = replay.ReplayProgram.from_doc(_program())
    seg = prog.segments[0]
    end = dict(zip(prog.joint_names, seg.end_deg))
    end['wrist_3_joint'] += 0.01
    dev = replay.deviation_deg(seg.end_deg, prog.joint_names, end)
    assert dev == pytest.approx(0.01)
    tr = replay.TrackingStats()
    tr.add([0.0, 0.001, -0.0015, 0.0, 0.0, 0.0, 0.0])
    tr.add([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(0.1)])
    assert tr.max_deg == pytest.approx(0.1)
    assert tr.samples == 2


def test_config_defaults_and_validation(tmp_path):
    cfg = replay.load_config(None)
    assert cfg.start_state_tolerance_deg > 0 and cfg.final_tolerance_deg > 0
    assert cfg.controller_action.endswith('/follow_joint_trajectory')
    bad = tmp_path / 'bad.yaml'
    bad.write_text('program_replay:\n  start_state_tolerance_deg: -1\n')
    with pytest.raises(ValueError, match='start_state_tolerance_deg'):
        replay.load_config(str(bad))


def test_program_from_file_validates_schema(tmp_path):
    import json
    good = copy.deepcopy(_program())
    path = tmp_path / 'program.json'
    path.write_text(json.dumps(good))
    # The synthetic program has no header: schema validation is off for this call, on by default.
    prog = replay.load_program(path, validate_schema=False)
    assert prog.program_rev == 'P001'
    with pytest.raises(replay.ReplayRefused, match='schema'):
        replay.load_program(path)


# --- review I1: mock hardware only, only the 7-joint controller action ---------------------------------------------

def test_mock_hardware_check():
    ok = [('ur5e', 'mock_components/GenericSystem'), ('awm_positioner', 'mock_components/GenericSystem')]
    replay.check_mock_hardware(ok)
    with pytest.raises(replay.ReplayRefused, match='ur5e.*ur_robot_driver/URPositionHardwareInterface'):
        replay.check_mock_hardware([('ur5e', 'ur_robot_driver/URPositionHardwareInterface'), ok[1]])
    with pytest.raises(replay.ReplayRefused, match='no hardware component'):
        replay.check_mock_hardware([])
    with pytest.raises(replay.ReplayRefused, match='awm_positioner'):
        replay.check_mock_hardware([ok[0], ('awm_positioner', '')])


def test_config_accepts_only_the_7_joint_action(tmp_path):
    other = tmp_path / 'other.yaml'
    other.write_text('program_replay:\n  controller_action: /scaled_joint_trajectory_controller/follow_joint_trajectory\n')
    with pytest.raises(replay.ReplayRefused, match='robot_positioner_trajectory_controller'):
        replay.load_config(str(other))
