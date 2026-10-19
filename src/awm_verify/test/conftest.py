"""Synthetic program revisions for the record / release / check tests (no ROS, no OCC)."""

import numpy as np
import pytest

from awm_core import jsonio
from awm_core.hashing import sha256_file
from awm_core.repository import Repository

HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
          'wrist_2_joint', 'wrist_3_joint']
TIMING = {'pre_flow_s': 0.3, 'post_flow_s': 0.5, 'crater_fill_s': 0.3, 'arc_ok_timeout_s': 1.0}


def _segment(seg_id, kind, seam, q0, q1, duration, speed=None):
    pts = [{'t_s': duration * k / 4, 'q_rad': list(np.asarray(q0) + (np.asarray(q1) -
                                                                   np.asarray(q0)) * k / 4)}
           for k in range(5)]
    return {'segment_id': seg_id, 'kind': kind, 'seam_id': seam, 'duration_s': duration,
            'tcp_speed_m_s': speed, 'points': pts}


def program_doc(job_dir, inputs):
    a = list(np.asarray(HOME) + [0.2, 0.1, 0, 0, 0, 0])
    b = list(np.asarray(a) + [0, 0.02, 0, 0, 0, 0])
    c = list(np.asarray(b) + [0.05, 0, 0, 0, 0, 0])
    d = list(np.asarray(c) + [0, -0.02, 0, 0, 0, 0])
    segs = [_segment('G001', 'air', None, HOME, a, 2.0),
            _segment('G002', 'approach', 'S01', a, b, 1.0),
            _segment('G003', 'weld', 'S01', b, c, 10.0, 0.006),
            _segment('G004', 'retract', 'S01', c, d, 1.0),
            _segment('G005', 'air', None, d, HOME, 2.0)]
    ev = {'process': 'MIG', 'wps_id': 'WPS-X', 'parameters': {'current_A': 180}, 'timing': TIMING}
    doc = jsonio.make_header('test', inputs, job_dir)
    doc.update({'program_rev': 'P001', 'part_no': 'P', 'part_rev': 'A', 'frame': 'base_link',
                'tool': 'tcp_mig',
                'calibration': {'calibration_id': 'C', 'sha256': 'a' * 64, 'status': 'NOMINAL'},
                'joint_names': JOINTS, 'home_joints': HOME, 'segments': segs,
                'events': [dict(ev, event_id='V001', type='WeldStart', seam_id='S01',
                                after_segment='G002'),
                           dict(ev, event_id='V002', type='WeldEnd', seam_id='S01',
                                after_segment='G003')],
                'cycle_time_s': 17.1,
                'limits': {'velocity_scaling': 0.2, 'acceleration_scaling': 0.2}})
    return doc


def write_report(job, rev='P001', result='PASS'):
    checks = {k: {'ok': result == 'PASS', 'detail': 'test'}
              for k in ('inputs', 'continuity', 'limits', 'events_at_rest', 'collision',
                        'replay')}
    doc = jsonio.make_header('test', [job.program_json(rev)], job.dir)
    doc.update({'program_rev': rev, 'program_sha256': sha256_file(job.program_json(rev)),
                'checks': checks,
                'replay': {'executed': True, 'segments_ok': 5, 'segments_total': 5,
                           'max_tracking_error_rad': 0.0, 'measured_time_s': 16.0},
                'cycle_time_s': 17.1, 'seams': [{'seam_id': 'S01', 'weld_time_s': 10.0,
                                                 'length_m': 0.06}],
                'result': result})
    jsonio.write_json(job.report_json(rev), doc, 'verify_report')


@pytest.fixture
def job(tmp_path):
    job = Repository(tmp_path).job('P', 'A')
    job.dir.mkdir(parents=True)
    (job.dir / 'path.json').write_text('{"stub": true}')
    jsonio.write_json(job.program_json('P001'), program_doc(job.dir, [job.dir / 'path.json']),
                      'program')
    return job
