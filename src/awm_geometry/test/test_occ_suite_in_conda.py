"""When run from an interpreter without OCC (colcon test), run the OCC test module in $AWM_OCC_PYTHON."""

import importlib.util
import os
import subprocess
from pathlib import Path

import pytest

from awm_core import occ_bridge

HERE = Path(__file__).resolve().parent


@pytest.mark.skipif(importlib.util.find_spec('OCC') is not None,
                    reason='OCC importable here: test_geometry_occ.py runs directly')
@pytest.mark.skipif(not os.environ.get(occ_bridge.ENV_VAR), reason=f'{occ_bridge.ENV_VAR} not set')
def test_occ_suite_passes_in_conda_env(tmp_path):
    argv = occ_bridge.interpreter_command(None) + [
        '-m', 'pytest', '-q', '-p', 'no:cacheprovider', '--basetemp', str(tmp_path / 'occ'),
        str(HERE / 'test_geometry_occ.py')]
    proc = subprocess.run(argv, cwd=str(HERE), env=occ_bridge.geometry_env(),
                          capture_output=True, text=True, timeout=900, check=False)
    # OCCT writes its own (late-flushed) banners to stdout: look for pytest's summary line.
    summary = [line for line in proc.stdout.splitlines() if ' passed' in line or ' failed' in line]
    assert proc.returncode == 0, (f'OCC test suite failed in conda env:\n{summary}\n'
                                  f'{proc.stdout[-3000:]}\n{proc.stderr[-2000:]}')
    assert summary and 'skipped' not in summary[-1] and 'failed' not in summary[-1], summary
