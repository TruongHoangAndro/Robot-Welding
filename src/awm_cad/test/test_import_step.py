"""awm_cad: API validation with a mocked geometry CLI, and the real CLI when OCC is configured."""

import json
import os
import subprocess
from pathlib import Path

import pytest

from awm_cad import export_view, import_step
from awm_cad.cli import main
from awm_core import jsonio, occ_bridge

POSE_SHA = 'a' * 64


def _fake_cli(job_dir: Path, part_no='P1', part_rev='A'):
    """Write what the geometry CLI would write, return a CompletedProcess."""
    job_dir.mkdir(parents=True, exist_ok=True)
    (job_dir / 'cache').mkdir(exist_ok=True)
    step = job_dir / 'model.step'
    step.write_text('ISO-10303-21; fake')
    edges = jsonio.make_header('fake', [step], job_dir)
    edges.update({'edges': [], 'faces': []})
    jsonio.write_json(job_dir / 'cache' / 'edges.json', edges, 'edges')
    part = jsonio.make_header('fake', [step], job_dir)
    part.update({'part_no': part_no, 'part_rev': part_rev, 'step_file': 'x.step',
                 'step_sha256': POSE_SHA, 'unit_source': 'mm',
                 'bbox_m': {'min': [0, 0, 0], 'max': [1, 1, 1]},
                 'counts': {'solids': 1, 'shells': 1, 'faces': 6, 'edges': 12},
                 'checks': [], 'healed': False,
                 'mesh': {'file': 'cache/mesh.stl', 'sha256': POSE_SHA, 'linear_deflection_m': 1e-4,
                          'angular_deflection_rad': 0.17, 'triangles': 12},
                 'brep_cache': {'file': 'cache/model.brep', 'sha256': POSE_SHA},
                 'edges_file': {'file': 'cache/edges.json', 'sha256': POSE_SHA}})
    jsonio.write_json(job_dir / 'part.json', part, 'part')
    return subprocess.CompletedProcess([], 0, stdout='banner\n{"edges": 0}\n', stderr='')


def test_import_validates_outputs(tmp_path, monkeypatch):
    calls = []

    def fake(args, occ_python_cmd=None):
        calls.append(args)
        return _fake_cli(Path(args[args.index('--job') + 1]))

    monkeypatch.setattr(occ_bridge, 'run_geometry_cli', fake)
    result = import_step(tmp_path, tmp_path / 'in.step', 'P1', 'A')
    assert result.job.dir == tmp_path / 'jobs' / 'P1_A'
    assert calls[0][0] == 'import'
    assert result.summary == {'edges': 0}


def test_import_detects_mismatch(tmp_path, monkeypatch):
    monkeypatch.setattr(occ_bridge, 'run_geometry_cli',
                        lambda args, occ_python_cmd=None: _fake_cli(
                            Path(args[args.index('--job') + 1]), part_no='OTHER'))
    with pytest.raises(RuntimeError, match='does not match'):
        import_step(tmp_path, tmp_path / 'in.step', 'P1', 'A')


def test_cli_reports_geometry_error(tmp_path, monkeypatch):
    def fail(args, occ_python_cmd=None):
        raise occ_bridge.GeometryCliError('boom')
    monkeypatch.setattr(occ_bridge, 'run_geometry_cli', fail)
    assert main(['--repo', str(tmp_path), '--step', 'x.step', '--part-no', 'P',
                 '--part-rev', 'A']) == 1


@pytest.mark.skipif(not os.environ.get(occ_bridge.ENV_VAR), reason='AWM_OCC_PYTHON not set')
def test_real_import(tmp_path):
    step = tmp_path / 'plate.step'
    occ_bridge.run_geometry_cli(['make-test-part', '--kind', 'plate_tjoint', '--out', str(step)])
    assert main(['--repo', str(tmp_path / 'repo'), '--step', str(step), '--part-no', 'PLATE',
                 '--part-rev', 'A']) == 0
    part = json.loads((tmp_path / 'repo' / 'jobs' / 'PLATE_A' / 'part.json').read_text())
    assert part['counts']['solids'] == 1


def _example_topology():
    return json.loads((Path(jsonio.SCHEMA_DIR) / 'examples' / 'view_topology.example.json').read_text())


def _job_with_step(tmp_path):
    job = tmp_path / 'jobs' / 'P_A'
    job.mkdir(parents=True)
    (job / 'model.step').write_text('ISO-10303-21; fake')
    return job


def test_export_view_validates_topology(tmp_path, monkeypatch):
    job = _job_with_step(tmp_path)
    calls = []

    def fake(args, occ_python_cmd=None):
        calls.append(args)
        out = Path(args[args.index('--out') + 1])
        doc = _example_topology()
        doc['inputs'] = [jsonio.input_entry(job / 'model.step', job)]
        jsonio.write_json(out / 'topology.json', doc, 'view_topology')
        (out / 'view.glb').write_bytes(b'glTF')
        return subprocess.CompletedProcess([], 0, stdout='banner\n{"solids": 2}\n', stderr='')

    monkeypatch.setattr(occ_bridge, 'run_geometry_cli', fake)
    summary = export_view(job, component='C', up_axis='+Y')
    assert summary == {'solids': 2}
    args = calls[0]
    assert args[0] == 'export-view'
    assert args[args.index('--step') + 1] == str(job / 'model.step')
    assert args[args.index('--job') + 1] == str(job)
    assert args[args.index('--out') + 1] == str(job / 'cache' / 'view')
    assert args[args.index('--component') + 1] == 'C'
    assert '--up-axis=+Y' in args


def test_export_view_rejects_stale_input(tmp_path, monkeypatch):
    job = _job_with_step(tmp_path)

    def fake(args, occ_python_cmd=None):
        out = Path(args[args.index('--out') + 1])
        jsonio.write_json(out / 'topology.json', _example_topology(), 'view_topology')
        return subprocess.CompletedProcess([], 0, stdout='{}\n', stderr='')

    monkeypatch.setattr(occ_bridge, 'run_geometry_cli', fake)
    with pytest.raises(jsonio.StaleInputError):
        export_view(job)


@pytest.mark.skipif(not os.environ.get(occ_bridge.ENV_VAR), reason='AWM_OCC_PYTHON not set')
def test_real_import_and_export_view(tmp_path):
    step = tmp_path / 'tube_rod.step'
    occ_bridge.run_geometry_cli(['make-test-part', '--kind', 'tube_rod', '--out', str(step)])
    result = import_step(tmp_path / 'repo', step, 'TR', 'A')
    summary = export_view(result.job.dir)
    assert summary['solids'] == 2 and len(summary['joints']) == 1
    topo = json.loads((result.job.cache_dir / 'view' / 'topology.json').read_text())
    assert topo['source']['file'] == 'model.step'
    catalogue = json.loads(result.job.edges_json.read_text())
    assert [e['id'] for e in catalogue['edges']] == [e['edge_id'] for e in topo['edges']]
    assert [e['fingerprint'] for e in catalogue['edges']] == [e['fingerprint'] for e in topo['edges']]
