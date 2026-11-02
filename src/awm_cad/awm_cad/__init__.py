"""APLSMN601 M1 CAD import (A11-A14) and D20 view model export (A13).

The B-Rep work runs in the OCC interpreter (awm_geometry); this package validates the results and
offers the Python API / CLI used by the Workbench and the chain runner.
"""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Union

from awm_core import jsonio, occ_bridge
from awm_core.repository import JobPaths, Repository

__version__ = '0.1.0'

PathLike = Union[str, Path]


@dataclass
class ImportResult:
    job: JobPaths
    summary: Dict[str, Any]


def import_step(repo_root: PathLike, step_file: PathLike, part_no: str, part_rev: str,
                occ_python_cmd: Optional[str] = None,
                linear_deflection_m: float = 1.0e-4) -> ImportResult:
    """Create / refresh the job package of a part from a STEP file (A11-A14).

    Returns the job paths and the CLI summary. Raises ``GeometryCliError`` if OCC fails or a
    mandatory model check (valid B-Rep, at least one solid) fails; the part record is written in
    both cases.
    """
    job = Repository(repo_root).job(part_no, part_rev)
    out = occ_bridge.run_geometry_cli(
        ['import', '--job', str(job.dir), '--step', str(step_file), '--part-no', part_no,
         '--part-rev', part_rev, '--linear-deflection-m', repr(float(linear_deflection_m))],
        occ_python_cmd=occ_python_cmd)
    summary = json.loads(out.stdout.strip().splitlines()[-1])
    part = jsonio.read_json(job.part_json, 'part', base_dir=job.dir)
    jsonio.read_json(job.edges_json, 'edges', base_dir=job.dir)
    if part['part_no'] != part_no or part['part_rev'] != part_rev:
        raise RuntimeError(f'part.json does not match the request: {job.part_json}')
    return ImportResult(job, summary)


def export_view(step_file: PathLike, out_dir: PathLike, component: Optional[str] = None,
                up_axis: str = '+Z', occ_python_cmd: Optional[str] = None) -> Dict[str, Any]:
    """Export the D20 view model (``view.glb`` + ``topology.json``, mm) of a STEP file or component.

    Runs ``awm_geometry.cli export-view`` in the OCC interpreter, then validates ``topology.json``
    against ``view_topology`` (inputs relative to the STEP file's folder, normally the job folder).
    Returns the CLI summary. Raises ``GeometryCliError`` if the export fails.
    """
    step = Path(step_file)
    out = Path(out_dir)
    args = ['export-view', '--step', str(step), '--out', str(out), '--up-axis', up_axis]
    if component is not None:
        args += ['--component', component]
    proc = occ_bridge.run_geometry_cli(args, occ_python_cmd=occ_python_cmd)
    summary = json.loads(proc.stdout.strip().splitlines()[-1])
    jsonio.read_json(out / 'topology.json', 'view_topology', base_dir=step.resolve().parent)
    if not (out / 'view.glb').is_file():
        raise RuntimeError(f'view.glb missing in {out}')
    return summary
