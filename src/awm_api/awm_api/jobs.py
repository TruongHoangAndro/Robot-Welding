"""Read-side helpers for the job repository (DES-001 §5.2): ids, status, documents, safe file access."""

from __future__ import annotations

import datetime as _dt
import json
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from awm_core.hashing import sha256_file

from awm_api.problems import ProblemError

JOB_ID_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*_[A-Za-z0-9][A-Za-z0-9._-]*$')
TOKEN_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')
SCOPES = ('simulation', 'trial', 'production')

#: Documents reported in the job detail (name → SHA-256), job-relative.
DOCUMENTS = ('part.json', 'seams.json', 'seam_axis.json', 'process.json', 'placement.json', 'path.json',
             'cache/view/view.glb', 'cache/view/topology.json')

NO_PLACEMENT_NOTE = (
    'No placement: this job has no placement.json. Goal 1 uses the golden-part placement exported from the cell '
    'CAD (awm_geometry export-scene, source fixture_cad); a newly uploaded STEP has none, so seam_axis.json and '
    'planning are not available until a placement exists. The API never invents one.')


def job_dir(jobs_dir: Path, job_id: str) -> Path:
    """Existing job folder for ``job_id`` or a 404 / 400 problem."""
    if not JOB_ID_RE.match(job_id):
        raise ProblemError(400, 'bad-job-id', detail=f'job id {job_id!r} must look like <PartNo>_<PartRev>')
    path = jobs_dir / job_id
    if not (path / 'part.json').is_file():
        raise ProblemError(404, 'job-not-found', detail=f'job {job_id} not found')
    return path


def iso_mtime(ts: float) -> str:
    return _dt.datetime.fromtimestamp(ts, _dt.timezone.utc).isoformat(timespec='seconds').replace('+00:00', 'Z')


def read_json(path: Path) -> Optional[Dict[str, Any]]:
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def documents(job: Path) -> Dict[str, str]:
    return {name: sha256_file(job / name) for name in DOCUMENTS if (job / name).is_file()}


def program_revs(job: Path) -> List[str]:
    d = job / 'programs'
    if not d.is_dir():
        return []
    return sorted(p.name for p in d.iterdir() if p.is_dir() and re.match(r'^P[0-9]{3}$', p.name))


def active_pointers(job: Path) -> Dict[str, Optional[str]]:
    out: Dict[str, Optional[str]] = {}
    for scope in SCOPES:
        doc = read_json(job / f'active_{scope}.json')
        out[scope] = (doc.get('program_rev') or doc.get('release') or f'active_{scope}.json') if doc else None
    return out


def placement_info(job: Path) -> Optional[Dict[str, Any]]:
    doc = read_json(job / 'placement.json')
    if doc is None:
        return None
    return {'source': doc.get('source'), 'verified': doc.get('verified')}


def seam_axis_state(job: Path, repo_root: Path) -> str:
    """``missing`` | ``ok`` | ``stale`` (an input hash of seam_axis.json differs or its input is gone, CP-58 d2)."""
    return document_state(job, 'seam_axis.json', repo_root)


def document_state(job: Path, name: str, repo_root: Path, doc: Optional[Dict[str, Any]] = None) -> str:
    """``missing`` | ``ok`` | ``stale`` for a derived job document by its recorded input hashes. Inputs that resolve
    outside the repository (e.g. a linked head folder) are not ours to check and are skipped."""
    if doc is None:
        doc = read_json(job / name)
    if doc is None:
        return 'missing'
    bases = [job, (job / name).parent]  # names are job-relative (jsonio.make_header(..., job)); doc folder fallback
    root = repo_root.resolve()
    for item in doc.get('inputs', []):
        cands = [(b / item.get('name', '')).resolve() for b in bases]
        path = next((c for c in cands if c.is_file()), cands[0])
        if root not in path.parents:
            continue  # outside the repository: not ours to check
        if not path.is_file() or sha256_file(path) != item.get('sha256'):
            return 'stale'
    return 'ok'


def latest_program(job: Path, repo_root: Path) -> Tuple[Optional[str], Optional[Dict[str, Any]], Optional[str]]:
    """(rev, D09, state) of the newest program revision with a readable ``program.json``; state ok | stale."""
    for rev in reversed(program_revs(job)):
        doc = read_json(job / 'programs' / rev / 'program.json')
        if doc is not None:
            return rev, doc, document_state(job, f'programs/{rev}/program.json', repo_root, doc)
    return None, None, None


def status(job: Path, importing: bool = False) -> str:
    if importing:
        return 'importing'
    if not (job / 'part.json').is_file():
        return 'error'
    active = active_pointers(job)
    for scope in ('production', 'trial', 'simulation'):
        if active[scope]:
            return f'released_{scope}'
    if any((job / 'programs' / rev / 'verify' / 'report.json').is_file() for rev in program_revs(job)):
        return 'simulated'
    if (job / 'path.json').is_file():
        return 'planned'
    return 'draft'


def summary(job: Path) -> Dict[str, Any]:
    part = read_json(job / 'part.json') or {}
    part_no, _, part_rev = job.name.rpartition('_')
    stamps = [(job / n).stat().st_mtime for n in DOCUMENTS if (job / n).is_file()] or [job.stat().st_mtime]
    return {'job_id': job.name, 'part_no': part.get('part_no', part_no), 'part_rev': part.get('part_rev', part_rev),
            'status': status(job), 'updated_at': iso_mtime(max(stamps))}


def detail(job: Path, repo_root: Path, active_run: Optional[str]) -> Dict[str, Any]:
    out = summary(job)
    placement = placement_info(job)
    out.update({
        'documents': documents(job), 'programs': program_revs(job), 'active': active_pointers(job),
        'active_run': active_run, 'placement': placement,
        'placement_note': None if placement else NO_PLACEMENT_NOTE,
        'seam_axis': seam_axis_state(job, repo_root),
        'path': document_state(job, 'path.json', repo_root),
        'collision_checked': False,  # Goal 1 (CP-58): nothing in this backend checks collision
    })
    return out


def safe_file(job: Path, rel: str) -> Path:
    """Job-relative path of a referenced file; no traversal and no symlink leaving the job (403)."""
    parts = Path(rel).parts
    if not rel or Path(rel).is_absolute() or '..' in parts or '\\' in rel:
        raise ProblemError(403, 'forbidden-path', detail=f'path {rel!r} is not a job-relative file path')
    path = (job / rel)
    resolved = path.resolve()
    if job.resolve() not in resolved.parents:
        raise ProblemError(403, 'forbidden-path', detail=f'path {rel!r} leaves the job folder')
    if not resolved.is_file():
        raise ProblemError(404, 'file-not-found', detail=f'{rel} not found in job {job.name}')
    return resolved
