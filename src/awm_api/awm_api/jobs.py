"""Read-side helpers for the job repository (DES-001 §5.2): ids, status, documents, safe file access."""

from __future__ import annotations

import datetime as _dt
import json
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from awm_core import cad_roles
from awm_core.hashing import sha256_file
from awm_core.staleness import document_state, document_verdict  # noqa: F401 - one staleness rule (G1.17)

from awm_api.problems import ProblemError

JOB_ID_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*_[A-Za-z0-9][A-Za-z0-9._-]*$')
TOKEN_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')
SCOPES = ('simulation', 'trial', 'production')

#: Documents reported in the job detail (name → SHA-256), job-relative.
DOCUMENTS = ('part.json', 'seams.json', 'seam_axis.json', 'process.json', 'placement.json', 'path.json',
             'cache/view/view.glb', 'cache/view/topology.json')

NO_PLACEMENT_ASSEMBLY = (
    'No placement yet: this job was imported as an assembly (positioner + fixture + part, CP-60). Save its roles, '
    'pick the positioner axis and run the CAD cross-check in S2: the cross-check computes placement.json '
    '(source job_cad). The API never invents a placement.')
NO_PLACEMENT_SINGLE = (
    'No placement: this job was imported as a single part, without the positioner and fixture, so the CAD cannot '
    'place it (CP-60). Re-import the job STEP as an assembly (positioner + fixture + part) to get a placement from '
    'the CAD, or enter a manual / copied placement in S2 (source study, test helper only, Assumption). The API '
    'never invents a placement.')
#: Kept for callers that have no job folder.
NO_PLACEMENT_NOTE = NO_PLACEMENT_SINGLE


def no_placement_note(job: Path) -> str:
    """Why a job has no placement and how to get one (CP-60)."""
    return NO_PLACEMENT_ASSEMBLY if job.joinpath(*cad_roles.JOB_TREE).is_file() else NO_PLACEMENT_SINGLE


def placement_hint(job: Path) -> str:
    return ('run the CAD cross-check (CP-60) to compute it' if job.joinpath(*cad_roles.JOB_TREE).is_file()
            else 'single-part job: enter a manual placement or re-import as an assembly (CP-60)')


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


def problems(job: Path, name: str, repo_root: Path, doc: Optional[Dict[str, Any]] = None) -> List[str]:
    """Reasons why a derived job document is stale (``document_verdict(...).problems``); empty when ok / missing."""
    return list(document_verdict(job, name, repo_root, doc).problems)


def program_info(job: Path, repo_root: Path) -> Optional[Dict[str, Any]]:
    """Newest program revision with its staleness state and reasons, or None (S2 / S4, close-out M3)."""
    rev, doc, _state = latest_program(job, repo_root)
    if rev is None:
        return None
    v = document_verdict(job, f'programs/{rev}/program.json', repo_root, doc)
    return {'rev': rev, 'state': v.state, 'problems': list(v.problems)}


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
        'placement_note': None if placement else no_placement_note(job),
        'seam_axis': seam_axis_state(job, repo_root),
        'path': document_state(job, 'path.json', repo_root),
        'path_problems': problems(job, 'path.json', repo_root),
        'program': program_info(job, repo_root),
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
