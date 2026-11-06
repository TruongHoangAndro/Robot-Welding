"""G1.3 golden run: plan CLI on a copy of the spike job (golden part 564082), J1 + J2 in ``sync``, J1 in ``C``.

Golden data only through environment variables (skipped with the reason when unset): ``AWM_SPIKE_REPO`` (repository
root with cell/model/frames.yaml, cell/model/head/ and jobs/<job>/), ``AWM_SPIKE_JOB`` (default 564082_A),
``AWM_SPIKE_J1`` / ``AWM_SPIKE_J2`` (joint edge ids, default JE2 / JE4 = the spike record) and ``AWM_OCC_PYTHON``
(OCC sampler for the seams). The spike files are linked into a temporary repository, never written.

Acceptance (master plan G1.3): every arc 100 % in its mode or rejected with failing s ranges and reasons; |Δq| ≤ 5°
on all 7 joints; FK of q reproduces the planned TCP pose (sync ≤ 0.01 mm / 0.01°, Mode C within the F8 tolerance of
a robot at rest); reproducible payload.
"""
import contextlib
import copy
import io
import json
import os
import shutil
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from awm_core import frames, jsonio, occ_bridge, units  # noqa: E402

pytest.importorskip('xacro', reason='xacro not available (run inside awm_dev)')

_JOB_FILES = ('model.step', 'part.json', 'placement.json', 'cache/model.brep', 'cache/collision.stl',
              'cache/collision_faces.json', 'cache/view/topology.json')


def _repo(tmp_path: Path, modes):
    root = os.environ.get('AWM_SPIKE_REPO')
    if not root:
        pytest.skip('AWM_SPIKE_REPO not set (golden-part spike job)')
    if not os.environ.get(occ_bridge.ENV_VAR):
        pytest.skip(f'{occ_bridge.ENV_VAR} not set (OCC sampler)')
    from awm_seam import joint_cli
    src_repo = Path(root)
    name = os.environ.get('AWM_SPIKE_JOB', '564082_A')
    src = src_repo / 'jobs' / name
    dst = tmp_path / 'repo' / 'jobs' / name
    (dst / 'cache' / 'view').mkdir(parents=True)
    for rel in _JOB_FILES:
        os.symlink(src / rel, dst / rel)
    model = tmp_path / 'repo' / 'cell' / 'model'
    model.mkdir(parents=True)
    shutil.copyfile(src_repo / 'cell' / 'model' / 'frames.yaml', model / 'frames.yaml')
    os.symlink(src_repo / 'cell' / 'model' / 'head', model / 'head')
    topo = json.loads((dst / 'cache' / 'view' / 'topology.json').read_text())
    joints = {j['joint_edge_id']: j for j in topo['joints']}
    ids = (os.environ.get('AWM_SPIKE_J1', 'JE2'), os.environ.get('AWM_SPIKE_J2', 'JE4'))
    seams = [{'seam_id': f'S0{k + 1}', 'edges': [joints[ids[k]]['edge_a']], 'joint_type': 'butt', 'sequence': k + 1,
              'arcs': 1, 'arc_overlap_mm': 2.0, 'positioner_mode': m} for k, m in enumerate(modes)]
    picks = tmp_path / 'picks.json'
    picks.write_text(json.dumps({'seams': seams}))
    with contextlib.redirect_stdout(io.StringIO()):
        assert joint_cli.main(['build', '--job', str(dst), '--picks', str(picks), '--pitch-mm', '1.0']) == 0
    return dst


def _plan(job_dir: Path, out: Path):
    from awm_path import cli
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = cli.main(['plan', '--job', str(job_dir), '--collision', 'off', '--out', str(out)])
    return rc, buf.getvalue()


def _check(job_dir: Path, doc, tol_mm: float, tol_deg: float):
    from awm_path.core.batch_kin import batch_fk
    from awm_path.core.kinematics import pose_error
    from awm_path.core.sync import rot_x
    from awm_path.description import load_planning_description
    model = job_dir.parent.parent / 'cell' / 'model'
    cell = load_planning_description(model / 'frames.yaml', 'hcad', str(model / 'head')).sync_cell
    ax = json.loads((job_dir / 'seam_axis.json').read_text())
    ftp = frames.make_transform(ax['flange_T_part']['position_mm'],
                                frames.quat_to_matrix(ax['flange_T_part']['quat_xyzw']))
    for s in doc['seams']:
        pts = s['points']
        q = np.array([p['q_deg'] for p in pts])
        q7 = np.column_stack([q, [p['theta_deg'] for p in pts]])
        assert np.abs(np.diff(q7, axis=0)).max() <= 5.0 + 1e-9
        assert pts[0]['s_mm'] == pytest.approx(s['arc']['start_s_mm'])
        assert pts[-1]['s_mm'] == pytest.approx(s['arc']['end_s_mm'])
        assert all(p['status'] == 'ok' and p['min_clearance_mm'] is None for p in pts)
        fk = batch_fk(q) @ cell.tool0_T_tcp
        for k, p in enumerate(pts):
            pose = frames.make_transform(p['pose_mm']['position_mm'], frames.quat_to_matrix(p['pose_mm']['quat_xyzw']))
            dp, dr = pose_error(fk[k], cell.base_link_T_axis @ rot_x(p['theta_deg']) @ ftp @ pose)
            assert dp <= tol_mm and units.rad_to_deg(dr) <= tol_deg, (s['seam_id'], k, dp, dr)
    for r in doc['rejected']:
        assert r['reason'] and r['failing_ranges'] and r['failing_points']


def _payload(doc):
    d = copy.deepcopy(doc)
    d.pop('created_at')
    d.pop('run_id')
    return d


def test_golden_sync_j1_j2(tmp_path):
    job = _repo(tmp_path, ('sync', 'sync'))
    rc, out = _plan(job, tmp_path / 'a.json')
    assert rc in (0, 2), out
    doc = jsonio.read_json(tmp_path / 'a.json', 'v2/path')
    assert doc['collision_checked'] is False
    assert {s['seam_id'] for s in doc['seams']} | {r['seam_id'] for r in doc['rejected']} == {'S01', 'S02'}
    _check(job, doc, 0.01, 0.01)
    rc2, _ = _plan(job, tmp_path / 'b.json')
    assert rc2 == rc
    assert _payload(doc) == _payload(json.loads((tmp_path / 'b.json').read_text()))
    print('GOLDEN', out.strip().splitlines()[-1])


def test_golden_mode_c_j1(tmp_path):
    from awm_path.core.sync import SyncParams
    job = _repo(tmp_path, ('C',))
    rc, out = _plan(job, tmp_path / 'c.json')
    assert rc in (0, 2), out
    doc = jsonio.read_json(tmp_path / 'c.json', 'v2/path')
    for s in doc['seams']:
        assert s['mode'] == 'C' and s['timed'][0]['kind'] == 'positioner_weld'
        assert np.ptp(np.array([p['q_deg'] for p in s['points']]), axis=0).max() == 0.0
    p = SyncParams()
    _check(job, doc, p.mode_c_tol_mm, p.mode_c_tol_deg)
    print('GOLDEN', out.strip().splitlines()[-1])
