"""awm_process: WPS lookup and process.json generation."""

import json

import pytest

from awm_core import jsonio
from awm_core.repository import Repository
from awm_process import ProcessError, assign, find_wps, load_library, select_process
from awm_process.cli import main

MOTION = {'travel_speed_mm_s': 6.0, 'work_angle_deg': 45.0, 'travel_angle_deg': 10.0,
          'stickout_mm': 15.0, 'focus_offset_mm': 0.0, 'weave_width_mm': 0.0,
          'weave_freq_hz': 0.0}
TIMING = {'pre_flow_s': 0.3, 'post_flow_s': 0.5, 'crater_fill_s': 0.3, 'arc_ok_timeout_s': 1.0}


def _wps(repo, wps_id, process='MIG', status='SAMPLE', t=(3, 12), leg=(3, 6), material='S235JR'):
    doc = jsonio.make_header('test', [], repo.root)
    doc.update({'wps_id': wps_id, 'status': status, 'description': 'test', 'process': process,
                'material': material, 'joint_types': ['fillet'],
                'thickness_mm': {'min': t[0], 'max': t[1]}, 'leg_mm': {'min': leg[0], 'max': leg[1]},
                'parameters': {'current_A': 180, 'voltage_V': 22}, 'motion': MOTION,
                'timing': TIMING})
    jsonio.write_json(repo.wps_dir / f'{wps_id}.json', doc, 'wps')


def _seam(seam_id, seq, hint='MIG', t=10.0, leg=4.0, material='S235JR'):
    return {'seam_id': seam_id, 'edges': ['E1'], 'reversed': False, 'joint_type': 'fillet',
            'weld_side': 'bisector', 'leg_mm': leg, 'sequence': seq, 'process_hint': hint,
            'material': material, 'thickness_mm': t, 'start_m': [0, 0, 0], 'end_m': [0.1, 0, 0],
            'length_m': 0.1, 'closed': False, 'fingerprint': 'c' * 64}


@pytest.fixture
def repo(tmp_path):
    repo = Repository(tmp_path)
    repo.wps_dir.mkdir(parents=True)
    _wps(repo, 'WPS-MIG-A')
    _wps(repo, 'WPS-MIG-B', status='APPROVED')
    _wps(repo, 'WPS-LASER-A', process='LASER', t=(0.5, 4), leg=(0.5, 3))
    return repo


def _job(repo, seams):
    job = repo.job('P', 'A')
    job.dir.mkdir(parents=True)
    (job.dir / 'edges.stub').write_text('x')
    doc = jsonio.make_header('test', [job.dir / 'edges.stub'], job.dir)
    doc.update({'part_no': 'P', 'part_rev': 'A', 'seams': seams})
    jsonio.write_json(job.seams_json, doc, 'seams')
    return job


def test_select_process():
    assert select_process(_seam('S01', 1, hint='LASER'), 'MIG')[0] == 'LASER'
    proc, why = select_process(_seam('S01', 1, hint=None), 'MIG')
    assert proc == 'MIG' and 'default' in why


def test_find_wps_prefers_approved(repo):
    lib = load_library(repo)
    wps, note = find_wps(_seam('S01', 1), 'MIG', lib, allow_sample=True)
    assert wps.wps_id == 'WPS-MIG-B' and 'preferred over' in note
    wps, _ = find_wps(_seam('S01', 1), 'MIG', lib, allow_sample=False)
    assert wps.wps_id == 'WPS-MIG-B'


def test_find_wps_reasons(repo):
    lib = load_library(repo)
    wps, why = find_wps(_seam('S01', 1, t=30.0), 'MIG', lib, allow_sample=True)
    assert wps is None and 'thickness' in why
    wps, why = find_wps(_seam('S01', 1, t=2.0, leg=2.0), 'LASER', lib, allow_sample=False)
    assert wps is None and 'SAMPLE not allowed' in why


def test_assign_writes_process_json(repo):
    job = _job(repo, [_seam('S01', 1), _seam('S02', 2, hint='LASER', t=2.0, leg=2.0),
                      _seam('S03', 3, material='AlMg3')])
    doc = assign(job, allow_sample_wps=True)
    rows = {r['seam_id']: r for r in doc['seams']}
    assert rows['S01']['wps_id'] == 'WPS-MIG-B'
    assert rows['S02']['wps_id'] == 'WPS-LASER-A' and rows['S02']['process'] == 'LASER'
    assert doc['unassigned'][0]['seam_id'] == 'S03'
    names = [i['name'] for i in doc['inputs']]
    assert names[0] == 'seams.json'
    assert '../../cell/wps/WPS-LASER-A.json' in names
    jsonio.read_json(job.process_json, 'process', base_dir=job.dir)
    # Changing a used WPS makes process.json stale.
    wps = repo.wps_dir / 'WPS-LASER-A.json'
    data = json.loads(wps.read_text())
    data['parameters']['power_W'] = 1500
    jsonio.write_json(wps, data, 'wps')
    with pytest.raises(jsonio.StaleInputError):
        jsonio.read_json(job.process_json, 'process', base_dir=job.dir)


def test_library_errors(tmp_path):
    with pytest.raises(ProcessError, match='not found'):
        load_library(Repository(tmp_path))
    repo = Repository(tmp_path)
    repo.wps_dir.mkdir(parents=True)
    _wps(repo, 'WPS-A')
    (repo.wps_dir / 'WPS-A.json').rename(repo.wps_dir / 'other.json')
    with pytest.raises(ProcessError, match='file name'):
        load_library(repo)


def test_cli_exit_codes(repo):
    job = _job(repo, [_seam('S01', 1)])
    assert main(['--job', str(job.dir)]) == 0
    job2 = repo.job('Q', 'A')
    job2.dir.mkdir(parents=True)
    (job2.dir / 'edges.stub').write_text('x')
    doc = jsonio.make_header('test', [job2.dir / 'edges.stub'], job2.dir)
    doc.update({'part_no': 'Q', 'part_rev': 'A', 'seams': [_seam('S01', 1, t=40.0)]})
    jsonio.write_json(job2.seams_json, doc, 'seams')
    assert main(['--job', str(job2.dir)]) == 2
    assert main(['--job', str(repo.root / 'jobs' / 'missing_A')]) == 1
