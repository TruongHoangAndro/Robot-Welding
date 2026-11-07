"""One staleness rule for derived job documents (G1.17, close-out A3; DES-001 §4.3, §5.2).

Every derived job document records ``inputs`` (``{name, sha256}``) with names relative to the job folder
(``jsonio.make_header(..., job.dir)``). This module is the single rule that decides whether such a document is still
valid; the planner CLI (``awm_path.cli``), the program generator (``awm_program.cli``) and the UI backend
(``awm_api.jobs.document_state``) use it so they give the same answer.

Rules, in this order, for each recorded input:

1. the input resolves (symlinks followed) outside the resolved repository root → stale (``outside the repository``).
   The repository is the unit that is backed up, released and hashed (DES-001 §5.2); an input outside it can change
   or vanish without the repository knowing, and skipping it would be a silent fallback (design rule 4). Planners
   refuse such inputs at plan time (:func:`outside_repo`);
2. not a file → stale (``missing``);
3. SHA-256 differs → stale (``changed since this file was created``).

Names resolve against the job folder only. In addition:

* a D07 (a document with a ``planner`` block) must carry ``planner.params.record_version`` equal to
  :data:`PLANNER_RECORD_VERSION`; missing / lower → "planned with an older planner; re-plan", higher → "planned with
  a newer planner";
* a D09 (a document with ``program_rev``) inherits the verdict of every JSON input that is a D07.

An unreadable or absent document is ``missing``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple, Union

from awm_core.hashing import sha256_file

PathLike = Union[str, Path]

#: Version of what a D07 means (planner output + head record). Bump it when the planner output or the head record
#: (``awm_path.core.head.HeadGeometry.as_record``, the head.yaml format read by the description) changes in a way
#: that makes an older D07 wrong even though its input hashes still match (e.g. c01ba5b changed the head record
#: derived from an unchanged head.yaml). 1 = first recorded version (G1.17); every D07 without it is older.
PLANNER_RECORD_VERSION = 1
#: Key inside ``planner.params`` (free-form and hashed into ``params_sha256``; no schema change).
RECORD_VERSION_KEY = 'record_version'
OLDER_PLANNER = 'planned with an older planner; re-plan'

MISSING, OK, STALE = 'missing', 'ok', 'stale'


@dataclass(frozen=True)
class Verdict:
    """Result of the staleness rule: ``state`` in {missing, ok, stale} and the ordered problems."""

    state: str
    problems: Tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        return self.state == OK

    def message(self) -> str:
        """One line for a refusal or a UI banner."""
        if self.state == OK:
            return 'ok'
        if self.state == MISSING:
            return 'missing' + (f' ({"; ".join(self.problems)})' if self.problems else '')
        return 'stale inputs: ' + '; '.join(self.problems) + '; re-plan'


def _inside(path: Path, root: Path) -> bool:
    return path == root or root in path.parents


def outside_repo(paths: Iterable[PathLike], repo_root: PathLike) -> List[Path]:
    """The given input paths (as given) whose resolved location is outside the resolved repository root."""
    root = Path(repo_root).resolve()
    return [Path(p) for p in paths if not _inside(Path(p).resolve(), root)]


def _read(path: Path) -> Optional[Dict[str, Any]]:
    try:
        doc = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None
    return doc if isinstance(doc, dict) else None


def is_d07(doc: Dict[str, Any]) -> bool:
    """A weld-path document (D07): has a ``planner`` block."""
    return isinstance(doc.get('planner'), dict)


def record_version_problem(doc: Dict[str, Any]) -> Optional[str]:
    """None if the D07 record version is current, else the problem text."""
    params = doc['planner'].get('params')
    got = params.get(RECORD_VERSION_KEY) if isinstance(params, dict) else None
    if not isinstance(got, int) or isinstance(got, bool) or got < PLANNER_RECORD_VERSION:
        return (f'{OLDER_PLANNER} (planner.params.{RECORD_VERSION_KEY} {got!r}, current '
                f'{PLANNER_RECORD_VERSION})')
    if got > PLANNER_RECORD_VERSION:
        return (f'planned with a newer planner (planner.params.{RECORD_VERSION_KEY} {got} > '
                f'{PLANNER_RECORD_VERSION}); update the software or re-plan')
    return None


def _input_problems(doc: Dict[str, Any], job: Path, root: Path, follow_d07: bool) -> List[str]:
    problems: List[str] = []
    for item in doc.get('inputs', []) or []:
        name = str(item.get('name', ''))
        path = (job / name).resolve()
        if not name or not _inside(path, root):
            problems.append(f'{name}: outside the repository ({path}); inputs must live in the repository '
                            '(replace the link by a copy)')
            continue
        if not path.is_file():
            problems.append(f'{name}: missing')
            continue
        if sha256_file(path) != item.get('sha256'):
            problems.append(f'{name}: changed since this file was created')
            continue
        if follow_d07 and path.suffix == '.json':
            sub = _read(path)
            if sub is not None and is_d07(sub):
                problems.extend(f'{name}: {p}' for p in _problems(sub, job, root, follow_d07=False))
    return problems


def _problems(doc: Dict[str, Any], job: Path, root: Path, follow_d07: bool) -> List[str]:
    problems = _input_problems(doc, job, root, follow_d07)
    if is_d07(doc):
        p = record_version_problem(doc)
        if p:
            problems.append(p)
    return problems


def document_verdict(job: PathLike, name: str, repo_root: PathLike,
                     doc: Optional[Dict[str, Any]] = None) -> Verdict:
    """Verdict for the job document ``name`` (relative to ``job`` or absolute); ``doc`` avoids a second read."""
    job = Path(job).resolve()
    if doc is None:
        doc = _read(job / name)
    if doc is None:
        return Verdict(MISSING)
    problems = _problems(doc, job, Path(repo_root).resolve(), follow_d07='program_rev' in doc)
    return Verdict(STALE, tuple(problems)) if problems else Verdict(OK)


def document_state(job: Path, name: str, repo_root: Path, doc: Optional[Dict[str, Any]] = None) -> str:
    """``missing`` | ``ok`` | ``stale`` — same signature as ``awm_api.jobs.document_state`` (drop-in)."""
    return document_verdict(job, name, repo_root, doc).state
