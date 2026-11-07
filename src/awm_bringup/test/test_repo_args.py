"""G1.16 (close-out A2): ``sim_mock repo:=<root>`` resolves frames, cell manifest and head from the repository.

Always on: a tiny repository is generated in ``tmp_path`` (test data only: a rotated positioner frame, one box
object, the synthetic head of awm_description). Checks the values taken from ``cell/model/frames.yaml`` (positioner
axis expressed in the positioner_axis frame), every refusal (explicit pose / manifest / head_dir args, missing or
malformed files, a manifest from another export, head rules) and one ``ros2 launch`` refusal before any node starts.
"""

import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

from awm_bringup.cell_config import REPO_CONFLICT_ARGS, RepoArgError, repo_launch_args
from awm_core.frames import quat_to_matrix

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'awm_description' / 'test'))
from head_fixture import write_synthetic_head_dir  # noqa: E402

INPUTS = [{'name': '../cad/cell.step', 'sha256': 'a' * 64}, {'name': '../cad/cad_map.yaml', 'sha256': 'b' * 64}]
S = math.sqrt(0.5)
# Test data: base shifted and turned 30 deg about Z; positioner frame turned 90 deg about Z, axis along world +Y.
BASE = {'position_mm': [12.5, -40.0, 3.0], 'quat_xyzw': [0.0, 0.0, math.sin(math.radians(15)),
                                                         math.cos(math.radians(15))]}
AXIS = {'position_mm': [-142.0, -475.0, 75.0], 'quat_xyzw': [0.0, 0.0, S, S], 'direction': [0.0, 1.0, 0.0]}
EMPTY = {name: '' for name in REPO_CONFLICT_ARGS}


def make_repo(root: Path, head: bool = True) -> Path:
    model = root / 'cell' / 'model'
    model.mkdir(parents=True)
    frames = {'schema_version': '2.0', 'created_by': 'test', 'created_at': '2026-11-07T00:00:00Z', 'inputs': INPUTS,
              'schema': 'awm_cell_frames/0.1', 'unit': 'mm', 'world_T_base': dict(BASE),
              'world_T_positioner_axis': dict(AXIS)}
    (model / 'frames.yaml').write_text(yaml.safe_dump(frames))
    manifest = {'schema_version': '2.0', 'created_by': 'test', 'created_at': '2026-11-07T00:00:00Z',
                'inputs': INPUTS, 'unit': 'mm', 'frames_file': 'frames.yaml',
                'objects': [{'object_id': 'box_test', 'role': 'cell_static', 'parent_frame': 'world',
                             'pose_mm': {'position_mm': [0, 0, -100], 'quat_xyzw': [0, 0, 0, 1]},
                             'box_mm': [100, 100, 10]}]}
    (model / 'scene_manifest.json').write_text(json.dumps(manifest))
    if head:
        write_synthetic_head_dir(model / 'head')
    return root


def _edit(path: Path, fn) -> None:
    if path.suffix == '.json':
        doc = json.loads(path.read_text())
        fn(doc)
        path.write_text(json.dumps(doc))
    else:
        doc = yaml.safe_load(path.read_text())
        fn(doc)
        path.write_text(yaml.safe_dump(doc))


def _vec(text):
    return np.asarray([float(v) for v in text.split()])


def test_values_from_repo(tmp_path):
    repo = make_repo(tmp_path / 'repo')
    out = repo_launch_args(str(repo), dict(EMPTY, head_model=''))
    model = repo.resolve() / 'cell' / 'model'
    assert np.allclose(_vec(out['base_xyz_mm']), BASE['position_mm'], atol=1e-12)
    assert np.allclose(_vec(out['base_quat_xyzw']), BASE['quat_xyzw'], atol=1e-15)
    assert np.allclose(_vec(out['positioner_xyz_mm']), AXIS['position_mm'], atol=1e-12)
    assert np.allclose(_vec(out['positioner_quat_xyzw']), AXIS['quat_xyzw'], atol=1e-15)
    # world +Y expressed in a frame turned +90 deg about Z is the local +X
    local = _vec(out['positioner_axis_xyz'])
    assert np.allclose(local, [1.0, 0.0, 0.0], atol=1e-12)
    assert np.allclose(quat_to_matrix(AXIS['quat_xyzw']) @ local, AXIS['direction'], atol=1e-12)
    assert out['cell_manifest'] == str(model / 'scene_manifest.json')
    assert out['head_model'] == 'hcad' and out['head_dir'] == str(model / 'head')


def test_relative_repo_is_made_absolute(tmp_path, monkeypatch):
    make_repo(tmp_path / 'repo')
    monkeypatch.chdir(tmp_path)
    out = repo_launch_args('repo', dict(EMPTY, head_model=''))
    assert Path(out['head_dir']).is_absolute() and Path(out['cell_manifest']).is_absolute()


@pytest.mark.parametrize('name', sorted(REPO_CONFLICT_ARGS))
def test_explicit_args_refused(tmp_path, name):
    repo = make_repo(tmp_path / 'repo')
    with pytest.raises(RepoArgError, match=name):
        repo_launch_args(str(repo), dict(EMPTY, head_model='', **{name: '1 2 3'}))


def test_missing_repo(tmp_path):
    with pytest.raises(RepoArgError, match='not a folder'):
        repo_launch_args(str(tmp_path / 'nope'), dict(EMPTY, head_model=''))


@pytest.mark.parametrize('name', ['frames.yaml', 'scene_manifest.json'])
def test_missing_file(tmp_path, name):
    repo = make_repo(tmp_path / 'repo')
    (repo / 'cell' / 'model' / name).unlink()
    with pytest.raises(RepoArgError, match=name):
        repo_launch_args(str(repo), dict(EMPTY, head_model=''))


@pytest.mark.parametrize('edit, match', [
    (lambda d: d.pop('world_T_base'), 'world_T_base'),
    (lambda d: d.pop('world_T_positioner_axis'), 'world_T_positioner_axis'),
    (lambda d: d['world_T_positioner_axis'].pop('direction'), 'direction'),
    (lambda d: d['world_T_positioner_axis'].update(direction=[0, 0, 0]), 'direction'),
    (lambda d: d['world_T_base'].update(quat_xyzw=[0, 0, 0, 2]), 'unit quaternion'),
    (lambda d: d['world_T_base'].update(position_mm=[1, 2]), 'position_mm'),
    (lambda d: d.update(unit='m'), 'unit'),
])
def test_bad_frames(tmp_path, edit, match):
    repo = make_repo(tmp_path / 'repo')
    _edit(repo / 'cell' / 'model' / 'frames.yaml', edit)
    with pytest.raises(RepoArgError, match=match):
        repo_launch_args(str(repo), dict(EMPTY, head_model=''))


@pytest.mark.parametrize('edit, match', [
    (lambda d: d.update(inputs=[{'name': '../cad/cell.step', 'sha256': 'c' * 64}]), 'same export'),
    (lambda d: d.update(frames_file='other.yaml'), 'frames_file'),
    (lambda d: d.update(objects=[]), 'no objects'),
])
def test_manifest_from_another_export(tmp_path, edit, match):
    repo = make_repo(tmp_path / 'repo')
    _edit(repo / 'cell' / 'model' / 'scene_manifest.json', edit)
    with pytest.raises(RepoArgError, match=match):
        repo_launch_args(str(repo), dict(EMPTY, head_model=''))


def test_no_head_refused_unless_hbox(tmp_path):
    repo = make_repo(tmp_path / 'repo', head=False)
    for value in ('', 'hcad'):
        with pytest.raises(RepoArgError, match='head.yaml.*head_model:=hbox'):
            repo_launch_args(str(repo), dict(EMPTY, head_model=value))
    out = repo_launch_args(str(repo), dict(EMPTY, head_model='hbox'))
    assert out['head_model'] == 'hbox' and out['head_dir'] == ''


def test_explicit_hbox_with_head_present(tmp_path):
    repo = make_repo(tmp_path / 'repo')
    out = repo_launch_args(str(repo), dict(EMPTY, head_model='hbox'))
    assert out['head_model'] == 'hbox' and out['head_dir'] == ''


def test_changed_head_mesh_refused(tmp_path):
    repo = make_repo(tmp_path / 'repo')
    with open(repo / 'cell' / 'model' / 'head' / 'head_body.stl', 'ab') as f:
        f.write(b'x')
    with pytest.raises(RepoArgError, match='SHA-256'):
        repo_launch_args(str(repo), dict(EMPTY, head_model=''))


def test_bad_head_model_value(tmp_path):
    repo = make_repo(tmp_path / 'repo')
    with pytest.raises(RepoArgError, match='head_model'):
        repo_launch_args(str(repo), dict(EMPTY, head_model='hfoo'))


def test_launch_refuses_pose_args_with_repo(tmp_path):
    """The launch stops before any node starts and names the conflicting argument."""
    repo = make_repo(tmp_path / 'repo')
    proc = subprocess.run(['ros2', 'launch', 'awm_bringup', 'sim_mock.launch.py', 'headless:=true',
                           f'repo:={repo}', 'base_xyz_mm:=0 0 0'],
                          capture_output=True, text=True, timeout=120, check=False)
    out = proc.stdout + proc.stderr
    assert proc.returncode != 0, out
    assert 'base_xyz_mm' in out and 'repo' in out, out
    assert 'process started' not in out, out


def test_launch_refuses_missing_frames(tmp_path):
    repo = make_repo(tmp_path / 'repo')
    (repo / 'cell' / 'model' / 'frames.yaml').unlink()
    proc = subprocess.run(['ros2', 'launch', 'awm_bringup', 'sim_mock.launch.py', 'headless:=true',
                           f'repo:={repo}'], capture_output=True, text=True, timeout=120, check=False)
    out = proc.stdout + proc.stderr
    assert proc.returncode != 0 and 'frames.yaml' in out and 'process started' not in out, out
