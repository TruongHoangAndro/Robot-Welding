"""Job repository layout (DES-001 §5.2).

The repository root is always passed in (ROS parameter ``repo_root`` / CLI ``--repo``); nothing here
knows an absolute path.
"""

from __future__ import annotations

import re
import shutil
from pathlib import Path
from typing import List, Optional, Union

PathLike = Union[str, Path]

_SAFE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')
_PROGRAM_REV = re.compile(r'^P(\d{3})$')


def _check_token(value: str, what: str) -> str:
    if not _SAFE.match(value):
        raise ValueError(f'{what} {value!r} must match {_SAFE.pattern}')
    return value


class Repository:
    """Repository root with ``cell/``, ``jobs/`` and ``logs/``."""

    def __init__(self, root: PathLike) -> None:
        self.root = Path(root)

    @property
    def cell_dir(self) -> Path:
        return self.root / 'cell'

    @property
    def wps_dir(self) -> Path:
        return self.cell_dir / 'wps'

    @property
    def calibration_dir(self) -> Path:
        return self.cell_dir / 'calibration'

    @property
    def active_calibration(self) -> Path:
        return self.calibration_dir / 'active.json'

    @property
    def io_map(self) -> Path:
        return self.cell_dir / 'io_map.yaml'

    @property
    def jobs_dir(self) -> Path:
        return self.root / 'jobs'

    def job_id(self, part_no: str, part_rev: str) -> str:
        return f'{_check_token(part_no, "part number")}_{_check_token(part_rev, "part revision")}'

    def job(self, part_no: str, part_rev: str) -> 'JobPaths':
        return JobPaths(self.jobs_dir / self.job_id(part_no, part_rev))

    def jobs(self) -> List['JobPaths']:
        if not self.jobs_dir.is_dir():
            return []
        return [JobPaths(p) for p in sorted(self.jobs_dir.iterdir()) if p.is_dir()]

    def exists(self) -> bool:
        return self.cell_dir.is_dir() and self.jobs_dir.is_dir()

    def init_from_template(self, template_dir: PathLike, overwrite: bool = False) -> None:
        """Create the repository layout and copy ``cell/`` from a template directory."""
        template = Path(template_dir)
        if not (template / 'cell').is_dir():
            raise FileNotFoundError(f'template has no cell/ directory: {template}')
        if self.cell_dir.exists() and not overwrite:
            raise FileExistsError(f'repository already initialised: {self.root}')
        self.root.mkdir(parents=True, exist_ok=True)
        shutil.copytree(template / 'cell', self.cell_dir, dirs_exist_ok=True)
        self.jobs_dir.mkdir(exist_ok=True)
        (self.root / 'logs').mkdir(exist_ok=True)


class JobPaths:
    """All file locations of one job package ``jobs/<PartNo>_<PartRev>/``."""

    def __init__(self, job_dir: PathLike) -> None:
        self.dir = Path(job_dir)

    @property
    def repo(self) -> Repository:
        return Repository(self.dir.parent.parent)

    @property
    def model_step(self) -> Path:
        return self.dir / 'model.step'

    @property
    def part_json(self) -> Path:
        return self.dir / 'part.json'

    @property
    def cache_dir(self) -> Path:
        return self.dir / 'cache'

    @property
    def brep_cache(self) -> Path:
        return self.cache_dir / 'model.brep'

    @property
    def mesh_stl(self) -> Path:
        return self.cache_dir / 'mesh.stl'

    @property
    def edges_json(self) -> Path:
        return self.cache_dir / 'edges.json'

    @property
    def seam_geometry_json(self) -> Path:
        return self.cache_dir / 'seam_geometry.json'

    @property
    def seams_json(self) -> Path:
        return self.dir / 'seams.json'

    @property
    def process_json(self) -> Path:
        return self.dir / 'process.json'

    @property
    def path_json(self) -> Path:
        return self.dir / 'path.json'

    @property
    def programs_dir(self) -> Path:
        return self.dir / 'programs'

    @property
    def release_json(self) -> Path:
        return self.dir / 'release.json'

    @property
    def inspections_dir(self) -> Path:
        return self.dir / 'inspections'

    def program_dir(self, rev: str) -> Path:
        if not _PROGRAM_REV.match(rev):
            raise ValueError(f'program revision {rev!r} must look like P001')
        return self.programs_dir / rev

    def program_json(self, rev: str) -> Path:
        return self.program_dir(rev) / 'program.json'

    def verify_dir(self, rev: str) -> Path:
        return self.program_dir(rev) / 'verify'

    def report_json(self, rev: str) -> Path:
        return self.verify_dir(rev) / 'report.json'

    def records_dir(self, rev: str) -> Path:
        return self.verify_dir(rev) / 'records'

    def program_revs(self) -> List[str]:
        if not self.programs_dir.is_dir():
            return []
        return sorted(p.name for p in self.programs_dir.iterdir()
                      if p.is_dir() and _PROGRAM_REV.match(p.name))

    def latest_program_rev(self) -> Optional[str]:
        revs = self.program_revs()
        return revs[-1] if revs else None

    def next_program_rev(self) -> str:
        latest = self.latest_program_rev()
        number = int(_PROGRAM_REV.match(latest).group(1)) + 1 if latest else 1
        return f'P{number:03d}'
