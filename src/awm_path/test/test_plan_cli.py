"""G1.3 CLI ``python3 -m awm_path.cli plan`` (agreed with the web agent): refusals, progress lines, D07 v2 2.1,
reproducible payload. Needs the robot description (xacro, ament share): runs inside awm_dev with the workspace
overlay (``scripts/dx``)."""
import copy
import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from awm_core import jsonio  # noqa: E402
from awm_path import cli  # noqa: E402
from g13_job import make_ring_job  # noqa: E402

pytest.importorskip('xacro', reason='xacro not available (run inside awm_dev)')


def _run(args, capsys):
    rc = cli.main(args)
    out = capsys.readouterr()
    return rc, out.out, out.err


def _payload(doc):
    d = copy.deepcopy(doc)
    for k in ('created_at', 'run_id'):
        d.pop(k)
    return d


@pytest.fixture(scope='module')
def ring(tmp_path_factory):
    root = tmp_path_factory.mktemp('repo')
    return make_ring_job(root)


@pytest.fixture(scope='module')
def planned(ring, tmp_path_factory):
    out = tmp_path_factory.mktemp('out') / 'path.json'
    import io
    import contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = cli.main(['plan', '--job', str(ring.dir), '--collision', 'off', '--out', str(out),
                       '--head-model', 'hbox'])
    return rc, buf.getvalue(), out


def test_collision_on_is_refused(ring, tmp_path, capsys):
    rc, out, err = _run(['plan', '--job', str(ring.dir), '--collision', 'on', '--out', str(tmp_path / 'p.json'),
                         '--head-model', 'hbox'], capsys)
    assert rc == 1 and 'Goal 2' in err
    assert not (tmp_path / 'p.json').exists()


def test_stale_seam_axis_is_refused(tmp_path, capsys):
    job = make_ring_job(tmp_path / 'repo')
    pl = json.loads((job.dir / 'placement.json').read_text())
    pl['flange_T_part']['position_mm'] = [1.0, 0.0, 0.0]
    (job.dir / 'placement.json').write_text(json.dumps(pl))
    rc, out, err = _run(['plan', '--job', str(job.dir), '--collision', 'off', '--out', str(tmp_path / 'p.json'),
                         '--head-model', 'hbox'], capsys)
    assert rc == 1 and 'stale' in err.lower() and 're-plan' in err
    assert not (tmp_path / 'p.json').exists()


def test_unknown_seam_is_refused(ring, tmp_path, capsys):
    rc, _, err = _run(['plan', '--job', str(ring.dir), '--collision', 'off', '--out', str(tmp_path / 'p.json'),
                       '--head-model', 'hbox', '--seams', 'S09'], capsys)
    assert rc == 1 and 'S09' in err


def test_plan_writes_valid_kinematic_only_d07(planned):
    rc, stdout, out = planned
    assert rc == 0, stdout
    lines = stdout.strip().splitlines()
    prog = [ln for ln in lines if ln.startswith('progress ')]
    assert prog and all(0.0 <= float(ln.split()[1]) <= 1.0 for ln in prog)
    assert float(prog[-1].split()[1]) == 1.0
    assert lines[-1].startswith('result ')
    doc = jsonio.read_json(out, 'v2/path')
    assert doc['schema_version'] == '2.1'
    assert doc['collision_checked'] is False
    assert doc['planner']['scene_backend'].startswith('none')
    assert doc['planner']['params']['collision'] == 'off'
    assert {i['name'] for i in doc['inputs']} >= {'seam_axis.json', 'seams.json', 'placement.json',
                                                  '../../cell/model/frames.yaml'}
    assert doc['rejected'] == []
    (seam,) = doc['seams']
    assert seam['mode'] == 'sync'
    pts = seam['points']
    assert all(p['min_clearance_mm'] is None and p['status'] == 'ok' for p in pts)
    q7 = np.array([p['q_deg'] + [p['theta_deg']] for p in pts])
    assert np.abs(np.diff(q7, axis=0)).max() <= 5.0 + 1e-9
    assert seam['planning']['fk_max_error_mm'] <= 0.01 and seam['planning']['fk_max_error_deg'] <= 0.01
    assert pts[-1]['s_mm'] == pytest.approx(seam['arc']['end_s_mm'])
    assert seam['timed'][0]['kind'] == 'weld'
    t = [p['t_s'] for p in seam['timed'][0]['points']]
    assert t[0] == 0.0 and np.all(np.diff(t) > 0)
    assert t[-1] == pytest.approx((pts[-1]['s_mm'] - pts[0]['s_mm']) / seam['travel_speed_mm_s'])


def test_same_inputs_give_identical_payload(ring, planned, tmp_path):
    out2 = tmp_path / 'path2.json'
    import io
    import contextlib
    with contextlib.redirect_stdout(io.StringIO()):
        assert cli.main(['plan', '--job', str(ring.dir), '--collision', 'off', '--out', str(out2),
                         '--head-model', 'hbox']) == 0
    a = json.loads(planned[2].read_text())
    b = json.loads(out2.read_text())
    assert _payload(a) == _payload(b)


def test_mode_c_and_mode_b_through_the_cli(tmp_path, capsys):
    job = make_ring_job(tmp_path / 'c', mode='C')
    rc, out, err = _run(['plan', '--job', str(job.dir), '--collision', 'off', '--out', str(tmp_path / 'c.json'),
                         '--head-model', 'hbox'], capsys)
    assert rc == 0, err
    seam = json.loads((tmp_path / 'c.json').read_text())['seams'][0]
    assert seam['mode'] == 'C' and seam['timed'][0]['kind'] == 'positioner_weld'
    assert np.ptp(np.array([p['q_deg'] for p in seam['points']]), axis=0).max() == 0.0
    job = make_ring_job(tmp_path / 'b', mode='B', arcs=2, angles=[0.0, 180.0])
    rc, out, err = _run(['plan', '--job', str(job.dir), '--collision', 'off', '--out', str(tmp_path / 'b.json'),
                         '--head-model', 'hbox'], capsys)
    doc = json.loads((tmp_path / 'b.json').read_text())
    assert rc in (0, 2)
    for s in doc['seams']:
        assert {p['theta_deg'] for p in s['points']} == {[0.0, 180.0][s['arc']['index'] - 1]}
    for r in doc['rejected']:
        assert r['failing_ranges'] and r['reason']
