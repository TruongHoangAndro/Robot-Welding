"""When run from an interpreter without OCC (colcon test), run the OCC test module in $AWM_OCC_PYTHON."""

import importlib.util
import os
import subprocess
from pathlib import Path

import pytest

from awm_core import occ_bridge

HERE = Path(__file__).resolve().parent

# The only allowed skip: tests that need the large cell CAD file, which is not in git (run them with AWM_CELL_STEP set).
ALLOWED_SKIP_REASON = 'AWM_CELL_STEP not set'


def unexpected_skips(stdout: str) -> list:
    """Return the `pytest -rs` skip lines whose reason is not ALLOWED_SKIP_REASON."""
    return [line for line in stdout.splitlines()
            if line.startswith('SKIPPED [') and not line.endswith(ALLOWED_SKIP_REASON)]


def test_skip_without_cell_step_is_allowed():
    out = 'SKIPPED [1] test_joints_occ.py:117: AWM_CELL_STEP not set\n13 passed, 1 skipped in 1.10s'
    assert unexpected_skips(out) == []


def test_skip_for_any_other_reason_is_reported():
    out = ('SKIPPED [12] test_geometry_occ.py:20: could not import \'OCC\'\n'
           'SKIPPED [1] test_joints_occ.py:117: AWM_CELL_STEP not set\n1 passed, 13 skipped in 0.20s')
    assert unexpected_skips(out) == ["SKIPPED [12] test_geometry_occ.py:20: could not import 'OCC'"]


@pytest.mark.skipif(importlib.util.find_spec('OCC') is not None,
                    reason='OCC importable here: test_geometry_occ.py runs directly')
@pytest.mark.skipif(not os.environ.get(occ_bridge.ENV_VAR), reason=f'{occ_bridge.ENV_VAR} not set')
def test_occ_suite_passes_in_conda_env(tmp_path):
    argv = occ_bridge.interpreter_command(None) + [
        '-m', 'pytest', '-q', '-rs', '-p', 'no:cacheprovider', '--basetemp', str(tmp_path / 'occ'),
        str(HERE / 'test_geometry_occ.py'), str(HERE / 'test_joints_occ.py'),
        str(HERE / 'test_view_occ.py'), str(HERE / 'test_parity_occ.py'),
        str(HERE / 'test_assembly_tree_occ.py'), str(HERE / 'test_scene_occ.py'),
        str(HERE / 'test_cell_view_occ.py'), str(HERE / 'test_joint_seam_occ.py')]
    proc = subprocess.run(argv, cwd=str(HERE), env=occ_bridge.geometry_env(),
                          capture_output=True, text=True, timeout=900, check=False)
    # OCCT writes its own (late-flushed) banners to stdout: look for pytest's summary line.
    summary = [line for line in proc.stdout.splitlines() if ' passed' in line or ' failed' in line]
    assert proc.returncode == 0, (f'OCC test suite failed in conda env:\n{summary}\n'
                                  f'{proc.stdout[-3000:]}\n{proc.stderr[-2000:]}')
    assert summary and ' passed' in summary[-1] and 'failed' not in summary[-1], summary
    assert unexpected_skips(proc.stdout) == [], summary
