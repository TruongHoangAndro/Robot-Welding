"""Joint sampler of :mod:`awm_seam.joint_seams` backed by the conda CAD process (ADR-15).

Runs ``<AWM_OCC_PYTHON | occ_python_cmd> -m awm_geometry.cli sample-joints`` through ``awm_core.occ_bridge``
and exchanges JSON files; OCC is never imported here.
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path
from typing import Dict, Optional

from awm_core import occ_bridge
from awm_core.repository import JobPaths
from awm_seam.joint_seams import Sampler


def occ_sampler(occ_python_cmd: Optional[str] = None, timeout_s: float = 600.0) -> Sampler:
    """Return a sampler callable ``(job, request) -> output document``."""

    def run(job: JobPaths, request: Dict) -> Dict:
        with tempfile.TemporaryDirectory(prefix='awm_seam_') as tmp:
            req, out = Path(tmp) / 'request.json', Path(tmp) / 'samples.json'
            req.write_text(json.dumps(request), encoding='utf-8')
            occ_bridge.run_geometry_cli(['sample-joints', '--job', job.dir, '--request', req, '--out', out],
                                        occ_python_cmd, timeout_s)
            return json.loads(out.read_text(encoding='utf-8'))

    return run
