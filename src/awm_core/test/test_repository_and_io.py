"""Repository layout, STL reader, OCC bridge error handling."""

import struct
import sys

import numpy as np
import pytest

from awm_core import occ_bridge
from awm_core.repository import JobPaths, Repository
from awm_core.stl import read_stl


def test_job_paths(tmp_path):
    repo = Repository(tmp_path)
    job = repo.job('PLATE-01', 'A')
    assert job.dir == tmp_path / 'jobs' / 'PLATE-01_A'
    assert job.repo.root == tmp_path
    assert job.next_program_rev() == 'P001'
    job.program_dir('P001').mkdir(parents=True)
    job.program_dir('P002').mkdir(parents=True)
    assert job.program_revs() == ['P001', 'P002']
    assert job.next_program_rev() == 'P003'
    assert job.report_json('P002') == job.dir / 'programs' / 'P002' / 'verify' / 'report.json'
    with pytest.raises(ValueError):
        job.program_dir('rev1')
    with pytest.raises(ValueError):
        repo.job('../evil', 'A')


def test_init_from_template(tmp_path):
    template = tmp_path / 'tpl'
    (template / 'cell' / 'wps').mkdir(parents=True)
    (template / 'cell' / 'wps' / 'WPS-X.json').write_text('{}')
    repo = Repository(tmp_path / 'repo')
    repo.init_from_template(template)
    assert repo.exists()
    assert (repo.wps_dir / 'WPS-X.json').is_file()
    with pytest.raises(FileExistsError):
        repo.init_from_template(template)
    assert isinstance(repo.jobs(), list)


def _write_binary_stl(path, tris):
    with open(path, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', len(tris)))
        for tri in tris:
            f.write(struct.pack('<3f', 0, 0, 0))
            for v in tri:
                f.write(struct.pack('<3f', *v))
            f.write(struct.pack('<H', 0))


def test_read_stl_binary_and_ascii(tmp_path):
    tris = [[(0, 0, 0), (1, 0, 0), (0, 1, 0)], [(1, 0, 0), (1, 1, 0), (0, 1, 0)]]
    _write_binary_stl(tmp_path / 'b.stl', tris)
    verts, faces = read_stl(tmp_path / 'b.stl')
    assert verts.shape == (4, 3) and faces.shape == (2, 3)
    ascii_lines = ['solid t']
    for tri in tris:
        ascii_lines += ['facet normal 0 0 1', 'outer loop']
        ascii_lines += [f'vertex {v[0]} {v[1]} {v[2]}' for v in tri]
        ascii_lines += ['endloop', 'endfacet']
    ascii_lines.append('endsolid t')
    (tmp_path / 'a.stl').write_text('\n'.join(ascii_lines))
    verts2, faces2 = read_stl(tmp_path / 'a.stl')
    assert np.allclose(np.sort(verts2, axis=0), np.sort(verts, axis=0))
    assert faces2.shape == (2, 3)


def test_occ_bridge_requires_interpreter(monkeypatch):
    monkeypatch.delenv(occ_bridge.ENV_VAR, raising=False)
    with pytest.raises(occ_bridge.GeometryCliError, match='no OCC interpreter'):
        occ_bridge.interpreter_command('')
    monkeypatch.setenv(occ_bridge.ENV_VAR, 'conda run -n awm_occ python')
    assert occ_bridge.interpreter_command(None) == ['conda', 'run', '-n', 'awm_occ', 'python']
    assert occ_bridge.interpreter_command('/x/python') == ['/x/python']


def test_occ_bridge_missing_interpreter(monkeypatch):
    monkeypatch.setattr(occ_bridge, 'geometry_env', lambda: {})
    with pytest.raises(occ_bridge.GeometryCliError, match='cannot start'):
        occ_bridge.run_geometry_cli(['info'], occ_python_cmd='/nonexistent/python3')


def test_occ_bridge_nonzero_exit_reports_stderr(monkeypatch):
    # The real interpreter without awm_geometry on the path: "No module named awm_geometry".
    monkeypatch.setattr(occ_bridge, 'geometry_env', lambda: {'PATH': '/usr/bin:/bin'})
    with pytest.raises(occ_bridge.GeometryCliError, match='awm_geometry'):
        occ_bridge.run_geometry_cli(['info'], occ_python_cmd=sys.executable)


def test_jobpaths_from_dir(tmp_path):
    job = JobPaths(tmp_path / 'jobs' / 'X_A')
    assert job.repo.root == tmp_path
