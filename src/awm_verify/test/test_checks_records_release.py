"""Static checks (A71), records (A72 / A73) and release (A74) without ROS."""

import json
import os

import numpy as np
import pytest

from awm_core import jsonio
from awm_path.params import ChainParams
from awm_verify.records import RecordError, latest_records, template, write_record
from awm_verify.records import main as record_main
from awm_verify.release import ReleaseError, is_locked, release
from awm_verify.verify import (check_continuity, check_events, check_limits,
                               joint_position_limits)

from conftest import program_doc, write_report


def test_static_checks_pass(job):
    doc = json.loads(job.program_json('P001').read_text())
    assert check_continuity(doc)[0]
    assert check_events(doc)[0]
    ok, detail = check_limits(doc, ChainParams())
    assert ok, detail


def test_static_checks_detect_problems(job):
    doc = json.loads(job.program_json('P001').read_text())
    bad = json.loads(json.dumps(doc))
    bad['segments'][2]['points'][0]['q_rad'][0] += 0.01
    assert 'gap' in check_continuity(bad)[1]
    bad = json.loads(json.dumps(doc))
    bad['events'][0]['after_segment'] = 'G003'
    assert not check_events(bad)[0]
    bad = json.loads(json.dumps(doc))
    for p in bad['segments'][0]['points']:
        p['t_s'] *= 0.05  # air move 20x faster than planned
    ok, detail = check_limits(bad, ChainParams())
    assert not ok and 'G001' in detail


def test_ur_position_limits():
    lim = joint_position_limits(ChainParams().joint_names)
    assert np.allclose(lim[0], [-2 * np.pi, 2 * np.pi])
    assert np.allclose(lim[2], [-np.pi, np.pi])  # UR5e elbow


def test_record_template_and_cli(job, capsys):
    assert template('dry_run')['speed_override_pct'] == 25
    with pytest.raises(RecordError):
        template('paint')
    assert record_main(['--template', 'trial_weld']) == 0
    assert json.loads(capsys.readouterr().out)['record_type'] == 'trial_weld'
    assert record_main(['--job', str(job.dir), '--rev', 'P001', '--type', 'dry_run',
                        '--operator', 'T. Tester', '--result', 'PASS', '--seam', 'S01=PASS',
                        '--speed-override-pct', '25']) == 0
    assert latest_records(job, 'P001')['dry_run']['doc']['operator'] == 'T. Tester'


def test_record_validation(job):
    with pytest.raises(RecordError, match='not in program'):
        write_record(job, 'P001', 'dry_run', 'X', 'PASS',
                     [{'seam_id': 'S07', 'result': 'PASS', 'note': ''}], 25, '')
    with pytest.raises(RecordError, match='failed seam'):
        write_record(job, 'P001', 'dry_run', 'X', 'PASS',
                     [{'seam_id': 'S01', 'result': 'FAIL', 'note': ''}], 25, '')
    with pytest.raises(RecordError, match='not found'):
        write_record(job, 'P009', 'dry_run', 'X', 'PASS', [], 25, '')


def test_release_simulation_and_lock(job):
    with pytest.raises(ReleaseError, match='no verification report'):
        release(job, 'P001', 'A. Approver', 'simulation')
    write_report(job, result='FAIL')
    with pytest.raises(ReleaseError, match='FAIL'):
        release(job, 'P001', 'A. Approver', 'simulation')
    write_report(job)
    doc = release(job, 'P001', 'A. Approver', 'simulation')
    assert doc['scope'] == 'simulation' and 'not valid for production' in doc['note']
    jsonio.read_json(job.release_json, 'release', base_dir=job.dir)
    assert is_locked(job, 'P001')
    assert not os.access(job.report_json('P001'), os.W_OK)
    with pytest.raises(ReleaseError, match='already released'):
        release(job, 'P001', 'A. Approver', 'production')


def test_production_needs_records(job):
    write_report(job)
    with pytest.raises(ReleaseError, match='dry_run record'):
        release(job, 'P001', 'A. Approver', 'production')
    seams = [{'seam_id': 'S01', 'result': 'PASS', 'note': ''}]
    write_record(job, 'P001', 'dry_run', 'T. Tester', 'PASS', seams, 25, '')
    with pytest.raises(ReleaseError, match='trial_weld record'):
        release(job, 'P001', 'A. Approver', 'production')
    write_record(job, 'P001', 'trial_weld', 'W. Welder', 'PASS', seams, 100, 'coupon OK')
    doc = release(job, 'P001', 'A. Approver', 'production')
    assert [r['record_type'] for r in doc['records']] == ['dry_run', 'trial_weld']


def test_report_of_other_program_version_refused(job):
    write_report(job)
    doc = program_doc(job.dir, [job.dir / 'path.json'])
    doc['cycle_time_s'] = 99.0
    jsonio.write_json(job.program_json('P001'), doc, 'program')
    # The report's input hash catches it first; the sha comparison is the second barrier.
    with pytest.raises((ReleaseError, jsonio.StaleInputError), match='program'):
        release(job, 'P001', 'A. Approver', 'simulation')


def test_changed_input_refused(job):
    write_report(job)
    (job.dir / 'path.json').write_text('{"stub": false}')
    with pytest.raises(jsonio.StaleInputError):
        release(job, 'P001', 'A. Approver', 'simulation')
