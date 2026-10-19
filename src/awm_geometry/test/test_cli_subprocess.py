"""Runs the geometry CLI the way ROS packages do: via awm_core.occ_bridge in a subprocess.

Works from the ROS interpreter (no OCC) when AWM_OCC_PYTHON points to the conda interpreter,
so ``colcon test`` also exercises the real OCC code path.
"""

import json
import os
from pathlib import Path

import pytest

from awm_core import jsonio, occ_bridge

pytestmark = pytest.mark.skipif(
    not os.environ.get(occ_bridge.ENV_VAR) or not Path(
        os.environ.get(occ_bridge.ENV_VAR, '').split()[0] or '/nonexistent').exists(),
    reason=f'{occ_bridge.ENV_VAR} not set to an existing interpreter')


def test_info():
    out = occ_bridge.run_geometry_cli(['info'])
    info = json.loads(out.stdout)
    assert info['occ_version'].startswith('8.')


def test_import_via_bridge(tmp_path):
    step = tmp_path / 'p.step'
    occ_bridge.run_geometry_cli(['make-test-part', '--kind', 'pipe_flange', '--out', str(step)])
    job = tmp_path / 'jobs' / 'PIPE_A'
    out = occ_bridge.run_geometry_cli(['import', '--job', str(job), '--step', str(step),
                                       '--part-no', 'PIPE', '--part-rev', 'A'])
    summary = json.loads(out.stdout.strip().splitlines()[-1])
    assert summary['checks_failed'] == []
    part = jsonio.read_json(job / 'part.json', 'part', base_dir=job)
    assert part['part_no'] == 'PIPE'


def test_bridge_error_has_stderr(tmp_path):
    with pytest.raises(occ_bridge.GeometryCliError, match='STEP file not found'):
        occ_bridge.run_geometry_cli(['import', '--job', str(tmp_path / 'j'), '--step',
                                     str(tmp_path / 'missing.step'), '--part-no', 'X',
                                     '--part-rev', 'A'])
