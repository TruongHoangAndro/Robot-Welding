"""G1.5 golden run: plan J1 + J2 of the golden part (G1.3 CLI) and generate the synchronised 7-joint program.

Golden data from the session fixture ``golden`` (``conftest.py``, G1.22: repository rebuilt from the cell STEP by
``scripts/golden_rebuild.py``, skipped with ``AWM_CELL_STEP not set`` otherwise) and ``AWM_OCC_PYTHON`` (see
``awm_path/test/test_golden_g13.py``; the golden files are linked into a temporary repository, never written).
``AWM_G15_SPEED`` = requested travel speed (default 3.0 mm/s, lead brief).

Flow: plan at the requested speed → generate. If the generator refuses because a weld would break a joint /
positioner velocity or acceleration limit, the refusal must name the arc and the largest feasible travel speed; the
test then re-plans at that speed (an Engineer decision, recorded) and generates again. Checks on the program: FK of
every weld point = D07 pose (sync 0.01 mm / 0.01°, Mode C 0.05 mm / 0.1°), TCP speed relative to the part within 5 %
on the constant-speed part, events with robot and positioner at rest, every segment inside the limits.
Machine-readable summary printed as ``G15 {json}`` (``pytest -s``).
"""
import contextlib
import io
import json
import os
import re
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('xacro', reason='xacro not available (run inside awm_dev)')

from awm_core import jsonio, units  # noqa: E402
from awm_path.core.kinematics import pose_error  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'awm_path' / 'test'))
from test_golden_g13 import _repo  # noqa: E402

SPEED = os.environ.get('AWM_G15_SPEED', '3.0')


def _plan(job: Path, speed: str):
    from awm_path import cli
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = cli.main(['plan', '--job', str(job), '--collision', 'off', '--out', str(job / 'path.json'),
                       '--travel-speed-mm-s', speed])
    return rc, buf.getvalue().strip().splitlines()[-1]


def _gen(job: Path):
    from awm_program import cli
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        rc = cli.main(['generate', '--job', str(job)])
    return rc, out.getvalue().strip().splitlines()[-1]


def _check(job: Path, doc, path):
    from awm_path.description import load_planning_description
    from awm_program.core import spline
    from awm_program.core.program7 import pose_matrix, tcp_in_part
    model = job.parent.parent / 'cell' / 'model'
    cell = load_planning_description(model / 'frames.yaml', 'hcad', str((model / 'head').resolve())).sync_cell
    ftp = pose_matrix(json.loads((job / 'seam_axis.json').read_text())['flange_T_part'])
    arcs = {(a['seam_id'], a['arc']['index']): a for a in path['seams']}
    recs = {w['segment_id']: w for w in doc['generator']['weld_segments']}
    lim = doc['generator']['limits']
    vl, al = np.array(lim['velocity_deg_s']), np.array(lim['acceleration_deg_s2'])
    out = []
    for seg in doc['segments']:
        t = np.array([p['time_from_start_s'] for p in seg['points']])
        q7 = np.array([p['positions_deg'] for p in seg['points']])
        v = np.array([p['velocities_deg_s'] for p in seg['points']])
        a = np.array([p['accelerations_deg_s2'] for p in seg['points']])
        vmax, amax = spline.max_abs(t, q7, v, a)
        weld = seg['kind'] in ('weld', 'positioner_weld')
        scale = 1.0 if weld else 0.25
        assert np.all(vmax <= scale * vl + 1e-6) and np.all(amax <= scale * al + 1e-6), seg['segment_id']
        if not weld:
            continue
        arc = arcs[(seg['seam_id'], seg['arc_index'])]
        rec = recs[seg['segment_id']]
        tcp = tcp_in_part(cell, ftp, q7)
        errs = [pose_error(tcp[k], pose_matrix(p['pose_mm'])) for k, p in enumerate(arc['points'])]
        e_mm = max(e[0] for e in errs)
        e_deg = max(float(units.rad_to_deg(e[1])) for e in errs)
        tol = (0.05, 0.1) if seg['positioner_mode'] == 'C' else (0.01, 0.01)
        assert e_mm <= tol[0] and e_deg <= tol[1], (seg['seam_id'], e_mm, e_deg)
        s = np.array([p['s_mm'] for p in arc['points']])
        lo, hi = rec['constant_speed_s_mm']
        inner = (s[:-1] >= lo - 1e-9) & (s[1:] <= hi + 1e-9)
        sp = np.linalg.norm(np.diff(tcp[:, :3, 3], axis=0), axis=1) / np.diff(t)
        dev = float(np.abs(sp[inner] / seg['tcp_speed_mm_s'] - 1.0).max())
        assert dev <= 0.05, (seg['seam_id'], dev)
        out.append({'seam_id': seg['seam_id'], 'mode': seg['positioner_mode'], 'points': len(q7),
                    'duration_s': round(float(t[-1]), 2), 'tcp_speed_mm_s': seg['tcp_speed_mm_s'],
                    'speed_dev_max_pct': round(100 * dev, 3), 'fk_mm': e_mm, 'fk_deg': e_deg,
                    'theta_travel_deg': round(float(np.ptp(q7[:, 6])), 1),
                    'vmax_ratio': round(float((vmax / vl).max()), 3), 'amax_ratio': round(float((amax / al).max()), 3)})
    idx = {s['segment_id']: i for i, s in enumerate(doc['segments'])}
    for ev in doc['events']:
        i = idx[ev['after_segment']]
        assert doc['segments'][i]['points'][-1]['velocities_deg_s'] == [0.0] * 7
        assert doc['segments'][i + 1]['points'][0]['velocities_deg_s'] == [0.0] * 7
    return out


@pytest.mark.parametrize('modes', [('sync', 'sync'), ('C', 'sync')], ids=['sync', 'J1-C'])
def test_golden_program(tmp_path, golden, modes):
    job = _repo(tmp_path, golden, modes)
    record = {'modes': modes, 'requested_speed_mm_s': float(SPEED)}
    rc, line = _plan(job, SPEED)
    assert rc == 0, line
    rc, line = _gen(job)
    if rc == 1:
        assert line.startswith('result refused')
        m = re.search(r'max feasible travel speed (\d+\.\d\d) mm/s', line) or \
            re.search(r'max_travel_speed_mm_s = (\d+\.\d\d)', line)
        assert m, line
        record['refusal'] = line
        feasible = m.group(1)
        record['replanned_speed_mm_s'] = float(feasible)
        rc, pline = _plan(job, feasible)
        assert rc == 0, pline
        rc, line = _gen(job)
    assert rc == 0, line
    kv = dict(x.split('=', 1) for x in line.split()[2:])
    doc = jsonio.read_json(Path(kv['out']), 'v2/program', base_dir=job)
    path = json.loads((job / 'path.json').read_text())
    assert doc['collision_checked'] is False
    record['cycle_time_s'] = doc['cycle_time_s']
    record['segments'] = [(s['segment_id'], s['kind'], s['seam_id'], len(s['points']),
                           s['points'][-1]['time_from_start_s']) for s in doc['segments']]
    record['welds'] = _check(job, doc, path)
    record['generator_weld_segments'] = doc['generator']['weld_segments']
    print('G15 ' + json.dumps(record, default=float))
