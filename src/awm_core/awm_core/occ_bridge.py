"""Call the OCCT geometry CLI (`awm_geometry`) in its own interpreter (ADR-02, CP-2).

ROS-side code never imports OCC. The interpreter command comes from the ``occ_python_cmd``
parameter, or the ``AWM_OCC_PYTHON`` environment variable when the parameter is empty.
"""

from __future__ import annotations

import importlib.util
import os
import shlex
import subprocess
from pathlib import Path
from typing import List, Optional, Sequence

ENV_VAR = 'AWM_OCC_PYTHON'


class GeometryCliError(RuntimeError):
    """The geometry CLI could not be started or returned a non-zero exit code."""


def _package_root(module: str) -> Path:
    spec = importlib.util.find_spec(module)
    if spec is None or not spec.submodule_search_locations:
        raise GeometryCliError(f'Python package {module!r} is not importable; build and source '
                               'the workspace (colcon build, source install/setup.bash)')
    return Path(list(spec.submodule_search_locations)[0]).resolve().parent


def interpreter_command(occ_python_cmd: Optional[str] = None) -> List[str]:
    """Return the interpreter command as an argv list."""
    cmd = (occ_python_cmd or '').strip() or os.environ.get(ENV_VAR, '').strip()
    if not cmd:
        raise GeometryCliError(
            f'no OCC interpreter configured: set parameter occ_python_cmd or env {ENV_VAR}')
    return shlex.split(cmd)


def geometry_env() -> dict:
    """Environment for the geometry interpreter: only our two pure packages on PYTHONPATH."""
    roots = []
    for module in ('awm_geometry', 'awm_core'):
        root = str(_package_root(module))
        if root not in roots:
            roots.append(root)
    env = {k: v for k, v in os.environ.items()
           if k not in ('PYTHONPATH', 'PYTHONHOME', 'AMENT_PREFIX_PATH')}
    env['PYTHONPATH'] = os.pathsep.join(roots)
    env['PYTHONNOUSERSITE'] = '1'
    return env


def run_geometry_cli(args: Sequence[str], occ_python_cmd: Optional[str] = None,
                     timeout_s: float = 900.0) -> subprocess.CompletedProcess:
    """Run ``<interpreter> -m awm_geometry.cli <args>``; raise GeometryCliError on failure."""
    argv = interpreter_command(occ_python_cmd) + ['-m', 'awm_geometry.cli'] + [str(a) for a in args]
    try:
        proc = subprocess.run(argv, env=geometry_env(), capture_output=True, text=True,
                              timeout=timeout_s, check=False)
    except FileNotFoundError as exc:
        raise GeometryCliError(f'cannot start OCC interpreter {argv[0]!r}: {exc}') from exc
    except subprocess.TimeoutExpired as exc:
        raise GeometryCliError(f'geometry CLI timed out after {timeout_s:.0f} s: '
                               f'{" ".join(argv[3:])}') from exc
    if proc.returncode != 0:
        raise GeometryCliError(
            f'geometry CLI failed (exit {proc.returncode}): {" ".join(argv[3:])}\n'
            f'{proc.stderr.strip()}')
    return proc
