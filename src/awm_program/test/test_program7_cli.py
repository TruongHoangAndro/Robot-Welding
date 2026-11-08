"""G1.5: ``python3 -m awm_program.cli generate`` on the synthetic ring job (``awm_path/test/g13_job.py``, H-BOX head).

Done when (master plan G1.5): TCP speed relative to the part within 5 % on the constant-speed part of every weld
segment with θ moving (FK of robot q + θ through the cell chain, part frame); WeldStart / WeldEnd with robot and
positioner at rest; FK of every weld point = D07 TCP pose (sync 0.01 mm / 0.01°, Mode C F8 0.05 mm / 0.1°);
refusals (stale D07, rejected arc, travel speed over the limit with the number, acceleration, active D08).
Needs the robot description (xacro, ament share) and ur-analytic-ik: runs inside awm_dev (``scripts/dx``).
"""
import contextlib
import copy
import io
import json
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('xacro', reason='xacro not available (run inside awm_dev)')

from awm_core import frames, jsonio, units  # noqa: E402
from awm_path.core.kinematics import pose_error  # noqa: E402
from awm_path.core.states import config_label  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'awm_path' / 'test'))
from g13_job import make_ring_job  # noqa: E402

SPEED = '3.0'


def _plan(job, out: Path, speed: str = SPEED) -> int:
    from awm_path import cli
    with contextlib.redirect_stdout(io.StringIO()):
        return cli.main(['plan', '--job', str(job.dir), '--collision', 'off', '--out', str(out), '--head-model', 'hbox',
                         '--travel-speed-mm-s', speed])


def _gen(job, *extra):
    from awm_program import cli
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        rc = cli.main(['generate', '--job', str(job.dir), *extra])
    return rc, out.getvalue(), err.getvalue()


def _job(root: Path, mode='sync', arcs=1, angles=None):
    job = make_ring_job(root, mode=mode, arcs=arcs, angles=angles)
    assert _plan(job, job.path_json) == 0
    return job


@pytest.fixture(scope='module')
def ring(tmp_path_factory):
    job = _job(tmp_path_factory.mktemp('sync'))
    rc, out, err = _gen(job)
    assert rc == 0, out + err
    return job, out


@pytest.fixture(scope='module')
def cell(ring):
    from awm_path.description import load_planning_description
    return load_planning_description(ring[0].repo.cell_dir / 'model' / 'frames.yaml', 'hbox').sync_cell


def _program(job, rev='P001'):
    return jsonio.read_json(job.program_json(rev), 'v2/program', base_dir=job.dir)


def _ftp(job):
    from awm_program.core.program7 import pose_matrix
    return pose_matrix(json.loads((job.dir / 'seam_axis.json').read_text())['flange_T_part'])


def _tcp_part(cell, ftp, q7):
    from awm_program.core.program7 import tcp_in_part
    return tcp_in_part(cell, ftp, np.asarray(q7))


# --------------------------------------------------------------------------------------------------------------

def test_cli_protocol_and_valid_2_1_program(ring):
    job, out = ring
    lines = out.strip().splitlines()
    prog = [float(ln.split()[1]) for ln in lines if ln.startswith('progress ')]
    assert prog and prog == sorted(prog) and prog[-1] == 1.0
    last = lines[-1].split()
    assert last[:2] == ['result', 'ok']
    kv = dict(x.split('=', 1) for x in last[2:])
    assert kv['program_rev'] == 'P001' and kv['collision_checked'] == 'false'
    assert Path(kv['out']) == job.program_json('P001')
    doc = _program(job)
    assert doc['schema_version'] == '2.1' and doc['collision_checked'] is False
    assert float(kv['cycle_time_s']) == pytest.approx(doc['cycle_time_s'], abs=0.01)
    assert doc['joint_names']['positioner'] == ['positioner_joint'] and len(doc['joint_names']['robot']) == 6
    assert [s['kind'] for s in doc['segments']] == ['air', 'approach', 'weld', 'retract', 'air']
    for seg in doc['segments']:
        assert seg['controller'] == 'robot_positioner' and seg['hold_deg'] == [] and seg['collision_checked'] is False
        for p in seg['points']:
            assert len(p['positions_deg']) == len(p['velocities_deg_s']) == len(p['accelerations_deg_s2']) == 7
    weld = doc['segments'][2]
    assert weld['positioner_mode'] == 'sync' and weld['tcp_speed_mm_s'] == pytest.approx(float(SPEED))
    gen = doc['generator']
    assert gen['travel_speeds'][0]['travel_speed_mm_s'] == pytest.approx(float(SPEED))
    assert gen['travel_speeds'][0]['source'] and gen['planner_params_sha256'] and gen['head_model'] == 'box'
    assert doc['context']['calibration']['status'] == 'NOMINAL'
    path = json.loads(job.path_json.read_text())
    assert doc['context']['calibration']['sha256'] == path['context']['calibration']['sha256']
    assert {i['name'] for i in doc['inputs']} >= {'path.json', 'seam_axis.json', 'seams.json'}
    assert doc['cycle_time_s'] == pytest.approx(sum(s['points'][-1]['time_from_start_s'] for s in doc['segments']))


def test_continuous_from_home_to_home(ring):
    doc = _program(ring[0])
    home = doc['home']['robot_deg'] + [doc['home']['positioner_deg']]
    segs = doc['segments']
    assert segs[0]['points'][0]['positions_deg'] == pytest.approx(home, abs=1e-9)
    assert segs[-1]['points'][-1]['positions_deg'] == pytest.approx(home, abs=1e-9)
    for a, b in zip(segs, segs[1:]):
        assert a['points'][-1]['positions_deg'] == b['points'][0]['positions_deg']
    for seg in segs:
        t = [p['time_from_start_s'] for p in seg['points']]
        assert t[0] == 0.0 and np.all(np.diff(t) > 0)


def test_events_only_with_robot_and_positioner_at_rest(ring):
    doc = _program(ring[0])
    segs = {s['segment_id']: i for i, s in enumerate(doc['segments'])}
    assert [e['type'] for e in doc['events']] == ['WeldStart', 'WeldEnd']
    for ev in doc['events']:
        i = segs[ev['after_segment']]
        before, after = doc['segments'][i], doc['segments'][i + 1]
        assert before['points'][-1]['velocities_deg_s'] == [0.0] * 7
        assert after['points'][0]['velocities_deg_s'] == [0.0] * 7
        assert ev['wps_id'] is None and ev['preset'] is None   # no D06 in Goal 1: never fabricated
    start, end = doc['events']
    assert doc['segments'][segs[start['after_segment']] + 1]['kind'] == 'weld'
    assert doc['segments'][segs[end['after_segment']]]['kind'] == 'weld'


def test_weld_fk_equals_d07_and_tcp_speed_relative_to_part(ring, cell):
    job = ring[0]
    doc = _program(job)
    path = json.loads(job.path_json.read_text())['seams'][0]
    weld = doc['segments'][2]
    q7 = np.array([p['positions_deg'] for p in weld['points']])
    t = np.array([p['time_from_start_s'] for p in weld['points']])
    assert np.ptp(q7[:, 6]) > 300.0                                    # θ moving during the weld
    tcp = _tcp_part(cell, _ftp(job), q7)
    from awm_program.core.program7 import pose_matrix
    for k, p in enumerate(path['points']):
        dp, dr = pose_error(tcp[k], pose_matrix(p['pose_mm']))
        assert dp <= 0.01 and units.rad_to_deg(dr) <= 0.01, (k, dp, dr)
    rec = doc['generator']['weld_segments'][0]
    s = np.array([p['s_mm'] for p in path['points']])
    lo, hi = rec['constant_speed_s_mm']
    inner = (s[:-1] >= lo - 1e-9) & (s[1:] <= hi + 1e-9)
    assert inner.sum() > 0.8 * len(inner)
    speed = np.linalg.norm(np.diff(tcp[:, :3, 3], axis=0), axis=1) / np.diff(t)
    assert np.abs(speed[inner] / float(SPEED) - 1.0).max() <= 0.05
    assert np.all(speed[~inner] <= float(SPEED) * 1.05)               # ramps never above the travel speed


def test_every_segment_inside_the_limits_and_spline_consistent(ring):
    from awm_program.core import spline
    job = ring[0]
    doc = _program(job)
    lim = doc['generator']['limits']
    vl, al = np.array(lim['velocity_deg_s']), np.array(lim['acceleration_deg_s2'])
    for seg in doc['segments']:
        t = np.array([p['time_from_start_s'] for p in seg['points']])
        q = np.array([p['positions_deg'] for p in seg['points']])
        v = np.array([p['velocities_deg_s'] for p in seg['points']])
        a = np.array([p['accelerations_deg_s2'] for p in seg['points']])
        v2, a2 = spline.clamped_cubic(t, q)
        # stored values are rounded to 1e-9; the shortest interval (closing overlap end) amplifies that in a
        assert np.abs(v - v2).max() <= 1e-5, (seg['segment_id'], np.abs(v - v2).max())
        assert np.abs(a - a2).max() <= 1e-2, (seg['segment_id'], np.abs(a - a2).max())
        vmax, amax = spline.max_abs(t, q, v, a)
        scale = 1.0 if seg['kind'] in ('weld', 'positioner_weld') else 0.25
        assert np.all(vmax <= scale * vl + 1e-6) and np.all(amax <= scale * al + 1e-6), seg['segment_id']


def test_approach_and_retract_40_mm_along_the_beam_on_the_weld_branch(ring, cell):
    job = ring[0]
    doc = _program(job)
    ftp = _ftp(job)
    path = json.loads(job.path_json.read_text())['seams'][0]
    from awm_program.core.program7 import pose_matrix
    for seg, end_pose, at in ((doc['segments'][1], path['points'][0], -1), (doc['segments'][3], path['points'][-1], 0)):
        q7 = np.array([p['positions_deg'] for p in seg['points']])
        assert np.ptp(q7[:, 6]) == 0.0                                  # θ held
        assert np.abs(np.diff(q7, axis=0)).max() <= 5.0 + 1e-9
        assert {config_label(q) for q in q7[:, :6]} == {path['config_label']}
        tcp = _tcp_part(cell, ftp, q7)
        seam_pt = pose_matrix(end_pose['pose_mm'])
        assert np.linalg.norm(tcp[at][:3, 3] - seam_pt[:3, 3]) <= 1e-6
        far = tcp[0] if at == -1 else tcp[-1]
        off = far[:3, 3] - seam_pt[:3, 3]
        assert np.linalg.norm(off) == pytest.approx(40.0, abs=1e-6)
        assert np.allclose(off / 40.0, -seam_pt[:3, 2], atol=1e-6)     # against the beam (tool +Z) = seam +Z
        for k in range(len(tcp)):                                        # straight line, constant orientation
            d = tcp[k][:3, 3] - seam_pt[:3, 3]
            assert np.linalg.norm(np.cross(d, seam_pt[:3, 2])) <= 1e-6
            assert frames.rotation_angle_between(tcp[k][:3, :3], seam_pt[:3, :3]) <= 1e-6   # arccos round-off


def test_new_revision_each_time_never_overwrites(ring):
    job = ring[0]
    before = job.program_json('P001').read_bytes()
    rc, out, err = _gen(job)
    assert rc == 0 and 'program_rev=P002' in out
    assert job.program_json('P001').read_bytes() == before


def test_mode_c_and_mode_b(tmp_path):
    from awm_path.description import load_planning_description
    job = _job(tmp_path / 'c', mode='C')
    rc, out, err = _gen(job)
    assert rc == 0, out + err
    doc = _program(job)
    weld = doc['segments'][2]
    assert weld['kind'] == 'positioner_weld' and weld['positioner_mode'] == 'C'
    q7 = np.array([p['positions_deg'] for p in weld['points']])
    assert np.ptp(q7[:, :6], axis=0).max() == 0.0 and np.ptp(q7[:, 6]) > 300.0
    cell = load_planning_description(job.repo.cell_dir / 'model' / 'frames.yaml', 'hbox').sync_cell
    path = json.loads(job.path_json.read_text())['seams'][0]
    tcp = _tcp_part(cell, _ftp(job), q7)
    from awm_program.core.program7 import pose_matrix
    for k, p in enumerate(path['points']):
        dp, dr = pose_error(tcp[k], pose_matrix(p['pose_mm']))
        assert dp <= 0.05 and units.rad_to_deg(dr) <= 0.1

    job = make_ring_job(tmp_path / 'b', mode='B', arcs=2, angles=[0.0, 180.0])
    from awm_path import cli as pcli
    with contextlib.redirect_stdout(io.StringIO()):
        prc = pcli.main(['plan', '--job', str(job.dir), '--collision', 'off', '--out', str(job.path_json),
                         '--head-model', 'hbox', '--travel-speed-mm-s', SPEED])
    rc, out, err = _gen(job)
    if prc == 2:                       # an arc rejected by the planner: refused, nothing written
        assert rc == 1 and 'rejected' in out and not job.programs_dir.exists()
        return
    assert rc == 0, out + err
    doc = _program(job)
    welds = [s for s in doc['segments'] if s['kind'] == 'weld']
    assert [w['arc_index'] for w in welds] == [1, 2] and {w['positioner_mode'] for w in welds} == {'B'}
    for w in welds:
        assert np.ptp([p['positions_deg'][6] for p in w['points']]) == 0.0
    assert 'index' in [s['kind'] for s in doc['segments']] or 'air' in [s['kind'] for s in doc['segments']]
    assert len(doc['events']) == 4


# ------------------------------------------------------------------------------------------------- refusals

def _copy_job(tmp_path):
    return _job(tmp_path / 'r')


def test_refuses_stale_d07(tmp_path):
    job = _copy_job(tmp_path)
    seams = json.loads(job.seams_json.read_text())
    seams['created_by'] = 'edited after planning'
    job.seams_json.write_text(json.dumps(seams))
    rc, out, err = _gen(job)
    assert rc == 1 and out.strip().splitlines()[-1].startswith('result refused') and 'stale' in out.lower()
    assert not job.programs_dir.exists()


def _tampered(job, tmp_path, fn):
    doc = json.loads(job.path_json.read_text())
    fn(doc)
    out = job.dir / 'path_tampered.json'
    out.write_text(json.dumps(doc))
    return out


def test_refuses_rejected_arc(tmp_path):
    job = _copy_job(tmp_path)

    def reject(doc):
        a = doc['seams'][0]
        doc['rejected'] = [{'seam_id': a['seam_id'], 'arc': a['arc'], 'mode': a['mode'], 'reason': 'ik',
                            'coverage_fraction': 0.5, 'failing_points': [{'index': 3, 's_mm': 3.0, 'reasons': {'ik': 1}}],
                            'pitch_mm': 1.0, 'failing_ranges': [{'s_from_mm': 3.0, 's_to_mm': 9.0, 'status': 'ik',
                                                                 'points': 7}], 'attempts': []}]
    p = _tampered(job, tmp_path, reject)
    rc, out, err = _gen(job, '--path', str(p))
    assert rc == 1 and 'rejected' in out and 'S01 arc 1/1' in out and 'no partial' in out
    assert not job.programs_dir.exists()


def test_refuses_travel_speed_over_the_limit_with_the_number(tmp_path):
    job = make_ring_job(tmp_path / 'fast')
    assert _plan(job, job.path_json, '10.0') == 0           # ω = 43 deg/s > 30 deg/s at 10 mm/s on r 13.3 mm
    path = json.loads(job.path_json.read_text())['seams'][0]
    assert path['planning']['speed_within_limits'] is False
    rc, out, err = _gen(job)
    vmax = path['planning']['max_travel_speed_mm_s']
    assert rc == 1 and 'S01 arc 1/1' in out and f'{vmax:.2f}' in out and 'never' in out
    assert not job.programs_dir.exists()


def test_refuses_acceleration_over_the_limit_with_the_number(ring):
    """Core: a too steep weld ramp breaks the positioner acceleration limit → refused with the feasible speed.
    G1.12: the program refuses a ramp other than the D07 one, so the D07 record is edited to the same ramp (its speed
    record still says "within limits" at ramp 5: the defensive check of the program must still catch it)."""
    from awm_path.core.kinematics import UrAnalyticKinematics
    from awm_path.description import load_planning_description
    from awm_program.core.program7 import ProgramParams, ProgramRefused, build_program
    from awm_program.description import load_program_cell
    job = ring[0]
    desc = load_planning_description(job.repo.cell_dir / 'model' / 'frames.yaml', 'hbox')
    pcell = load_program_cell(desc)
    path = json.loads(job.path_json.read_text())
    path['planner']['params']['sync']['weld_ramp_accel_mm_s2'] = 500.0
    with pytest.raises(ProgramRefused, match=r'positioner_joint acceleration .* max feasible travel speed \d+\.\d\d'):
        build_program(path, {'S01': 1}, _ftp(job), pcell, UrAnalyticKinematics(),
                      ProgramParams(weld_ramp_accel_mm_s2=500.0))
    with pytest.raises(ProgramRefused) as exc:                     # review 1.a: no "lower travel speed" advice
        build_program(path, {'S01': 1}, _ftp(job), pcell, UrAnalyticKinematics(),
                      ProgramParams(weld_ramp_accel_mm_s2=500.0))
    assert '≤ the maximum' in str(exc.value) and 'lower travel speed' not in str(exc.value)


def test_refuses_when_an_active_calibration_exists(tmp_path):
    job = _copy_job(tmp_path)
    cal = job.repo.active_calibration
    cal.parent.mkdir(parents=True, exist_ok=True)
    cal.write_text('{}')
    rc, out, err = _gen(job)
    assert rc == 1 and 'calibration' in out
    assert not job.programs_dir.exists()


def test_refuses_d07_with_another_description(tmp_path):
    job = _copy_job(tmp_path)

    def other(doc):
        doc['planner']['params']['description']['cell']['positioner_velocity_deg_s'] = 99.0
    p = _tampered(job, tmp_path, other)
    rc, out, err = _gen(job, '--path', str(p))
    assert rc == 1 and 'description' in out


# ----------------------------------------------------------------------------------- CP-62 override (g1-18 I2)

def _with_override(job):
    """Record a CP-62 override in the job (format of the v2 examples) and its reference in D07, as the plan run of
    ``awm_api`` does; return the reference."""
    from awm_core.hashing import sha256_file
    check = json.loads((jsonio.SCHEMA_DIR / 'v2' / 'examples' / 'cad_check.example.json').read_text())
    (job.dir / 'cad_check.json').write_text(json.dumps(check))
    rec = json.loads((jsonio.SCHEMA_DIR / 'v2' / 'examples' / 'cad_check_override.example.json').read_text())
    rec['cad_check_sha256'] = sha256_file(job.dir / 'cad_check.json')
    rec['inputs'] = [{'name': 'cad_check.json', 'sha256': rec['cad_check_sha256']}]
    jsonio.validate(rec, 'v2/cad_check_override')
    (job.dir / 'cad_check_override.json').write_text(json.dumps(rec))
    ref = {'file': 'cad_check_override.json', 'sha256': sha256_file(job.dir / 'cad_check_override.json'),
           'cad_check_sha256': rec['cad_check_sha256'], 'check_id': 4, 'by': rec['by'],
           'created_at': rec['created_at'], 'reason': rec['reason']}
    path = json.loads(job.path_json.read_text())
    assert path['collision_checked'] is False
    path['cad_check_override'] = ref
    jsonio.validate(path, 'v2/path')
    job.path_json.write_text(json.dumps(path))
    return ref


def test_no_override_in_d07_none_in_d09(ring):
    assert 'cad_check_override' not in json.loads(ring[0].path_json.read_text())
    assert 'cad_check_override' not in _program(ring[0])


def test_override_in_d07_is_carried_into_d09(tmp_path):
    from awm_core.hashing import sha256_file
    job = _copy_job(tmp_path)
    ref = _with_override(job)
    rc, out, err = _gen(job)
    assert rc == 0, out + err
    doc = _program(job)
    jsonio.validate(doc, 'v2/program')
    assert doc['cad_check_override'] == ref                    # incl. by / created_at / reason, exact copy
    assert {'by', 'created_at', 'reason'} <= set(doc['cad_check_override'])
    assert doc['collision_checked'] is False
    assert doc['generator']['path']['sha256'] == sha256_file(job.path_json)
    # the awm_api runner's carry step is a no-op on this program: same reference, same recorded path.json hash
    assert doc['cad_check_override'] == json.loads(job.path_json.read_text())['cad_check_override']


def test_refuses_d07_override_voided_by_a_changed_cross_check(tmp_path):
    job = _copy_job(tmp_path)
    _with_override(job)
    (job.dir / 'cad_check.json').write_text('{"changed": true}')
    rc, out, err = _gen(job)
    assert rc == 1 and 'CP-62' in out and 'void' in out
    assert not job.programs_dir.exists()
    _with_override(job)
    (job.dir / 'cad_check_override.json').unlink()
    rc, out, err = _gen(job)
    assert rc == 1 and 'CP-62' in out and 'void' in out
    assert not job.programs_dir.exists()


# ---------------------------------------------------------------------------------- review g1-19 M-1

def test_d07_deleted_mid_run_is_refused_not_a_traceback(tmp_path, monkeypatch):
    from awm_program.core import program7
    job = _copy_job(tmp_path)
    orig = program7.build_program

    def build_then_delete(*a, **k):
        out = orig(*a, **k)
        job.path_json.unlink()
        return out
    monkeypatch.setattr(program7, 'build_program', build_then_delete)
    rc, out, err = _gen(job)
    assert rc == 1 and out.strip().splitlines()[-1].startswith('result refused') and 'D07' in out
    assert 'Traceback' not in err
    assert not job.programs_dir.exists()


def test_d07_hash_in_inputs_and_generator_is_the_hash_of_the_bytes_used(ring):
    from awm_core.hashing import sha256_bytes
    job = ring[0]
    doc = _program(job)
    sha = sha256_bytes(job.path_json.read_bytes())
    assert doc['generator']['path']['sha256'] == sha
    assert {i['name']: i['sha256'] for i in doc['inputs']}['path.json'] == sha
