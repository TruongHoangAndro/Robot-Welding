"""Review 2.b: the simulation runners refuse a head without a parametric nozzle (F9 impossible) before they start
sim_mock: the description is loaded offline and checked with contact_params_for; exit 1 with one ``error:`` line.
Needs xacro and the ament share (runs inside awm_dev). Synthetic job and head only."""
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / 'awm_description' / 'test'))

from g13_job import make_ring_job  # noqa: E402
from head_fixture import write_synthetic_head_dir  # noqa: E402


@pytest.fixture
def no_launch(monkeypatch):
    import awm_bringup.testing

    def _boom(*_a, **_k):
        raise AssertionError('sim_mock must not be launched for a refused head')
    monkeypatch.setattr(awm_bringup.testing, 'SimLaunch', _boom)


def _setup(tmp_path, nozzle_len_mm):
    make_ring_job(tmp_path / 'repo')
    head = write_synthetic_head_dir(tmp_path / 'head', nozzle_len_mm=nozzle_len_mm)
    return tmp_path / 'repo', head


def test_preflight_passes_a_head_with_a_nozzle_and_refuses_one_without(tmp_path):
    from awm_path.gate_g2 import preflight_head
    repo, head = _setup(tmp_path, 40.0)
    assert preflight_head(repo, 'H-CAD', str(head), [], 'rising') is None
    assert preflight_head(repo, 'H-BOX', None, [], 'rising') is None
    bare = write_synthetic_head_dir(tmp_path / 'bare', nozzle_len_mm=0.0)
    msg = preflight_head(repo, 'H-CAD', str(bare), [], 'rising')
    assert msg is not None and 'nozzle_len_mm 0' in msg and 'before launching sim_mock' in msg


def _argv(runner, repo, head, out):
    common = ['--repo', str(repo), '--job', 'TEST-RING_A', '--head-model', 'H-CAD', '--head-dir', str(head)]
    return {
        'gate_g2': common + ['--poses', str(out / 'poses.yaml'), '--out-dir', str(out)],
        'spike_s5': ['run'] + common + ['--seam-id', 'S01', '--out-dir', str(out)],
        'spike_s5b': common + ['--input', str(out / 'in.yaml'), '--out-dir', str(out)],
        'spike_s6': common + ['--input', str(out / 'in.yaml'), '--out-dir', str(out)],
        'scene_timing': common + ['--joint', 'JE1', '--n1', 'F1', '--n2', 'F2', '--arc-start-deg', '0',
                                  '--arc-end-deg', '90', '--out-json', str(out / 'o.json'),
                                  '--out-csv', str(out / 'o.csv')],
    }[runner]


@pytest.mark.parametrize('runner', ['gate_g2', 'spike_s5', 'spike_s5b', 'spike_s6', 'scene_timing'])
def test_runner_refuses_a_no_nozzle_head_before_launching(runner, tmp_path, no_launch, capsys):
    import importlib
    repo, head = _setup(tmp_path, 0.0)
    out = tmp_path / 'out'
    rc = importlib.import_module(f'awm_path.{runner}').main(_argv(runner, repo, head, out))
    assert rc == 1
    err = capsys.readouterr().err
    assert err.startswith('error: ') and 'nozzle_len_mm 0' in err
    assert not out.exists()          # nothing written
