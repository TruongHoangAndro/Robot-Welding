"""One staleness rule for derived job documents (G1.17, close-out A3; DES-001 §4.3, §5.2).

Every derived job document records ``inputs`` (``{name, sha256}``) with names relative to the job folder
(``jsonio.make_header(..., job.dir)``). This module is the single rule that decides whether such a document is still
valid; the planner CLI (``awm_path.cli``), the program generator (``awm_program.cli``) and the UI backend
(``awm_api.jobs.document_state``) use it so they give the same answer.

Rules, in this order, for each recorded input:

1. the input resolves (symlinks followed) outside the resolved repository root → stale (``outside the repository``).
   The repository is the unit that is backed up, released and hashed (DES-001 §5.2); an input outside it can change
   or vanish without the repository knowing, and skipping it would be a silent fallback (design rule 4);
2. the input is reached through a symbolic link (its job-relative path, made absolute, differs from the resolved
   path) → stale (``symbolic link``). A link inside the repository can be retargeted later while the recorded
   target and its hash stay unchanged (review I1), so no link is allowed on an input path. Planners refuse linked
   or outside inputs at plan time (:func:`linked_or_outside`);
3. not a file → stale (``missing``);
4. SHA-256 differs → stale (``changed since this file was created``).

Names resolve against the job folder only. In addition:

* a D07 (a document with a ``planner`` block, or a schema 2.x path document without one) must carry ``planner.params.record_version`` equal to
  :data:`PLANNER_RECORD_VERSION`; missing / lower → "planned with an older planner; re-plan", higher → "planned with
  a newer planner";
* a D09 (a document with ``program_rev``) inherits the verdict of every JSON input that is a D07.

An unreadable or absent document is ``missing``.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple, Union

from awm_core.hashing import sha256_file

PathLike = Union[str, Path]

#: Version of what a D07 means (planner output + head record). Bump it when the planner output or the head record
#: (``awm_path.core.head.HeadGeometry.as_record``, the head.yaml format read by the description) changes in a way
#: that makes an older D07 wrong even though its input hashes still match (e.g. c01ba5b changed the head record
#: derived from an unchanged head.yaml). 1 = first recorded version (G1.17); every D07 without it is older.
#: The bump is a developer decision. A guard test (``awm_path/test/test_staleness_cli.py``, ``RECORD_FORMATS``) pins
#: the description record of two synthetic heads (H-BOX and a synthetic H-CAD head) by key sets and by value, so a
#: change of the rendered record fails until a new version is added; it cannot detect changes that do not show in
#: those two records, nor decide whether a change matters for existing D07 files.
PLANNER_RECORD_VERSION = 1
#: Key inside ``planner.params`` (free-form and hashed into ``params_sha256``; no schema change).
RECORD_VERSION_KEY = 'record_version'
OLDER_PLANNER = 'planned with an older planner; re-plan'
#: Fix named in every refusal of a linked or outside input.
LINK_FIX = 'inputs must be plain files inside the repository; replace the link by a copy (e.g. cp -rL)'

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


def _logical_rel(path: Path, roots: Tuple[Path, ...]) -> Optional[Path]:
    logical = Path(os.path.abspath(path))
    for root in roots:
        try:
            return logical.relative_to(root)
        except ValueError:
            continue
    return None


def linked_or_outside(paths: Iterable[PathLike], repo_root: PathLike) -> List[Path]:
    """The given input paths (as given) that resolve outside the repository root or contain a symbolic link below it.

    The root itself may be reached through a link (only the part below the root is compared).
    """
    real_root = Path(repo_root).resolve()
    roots = (Path(os.path.abspath(repo_root)), real_root)
    out: List[Path] = []
    for p in paths:
        p = Path(p)
        real = p.resolve()
        rel = _logical_rel(p, roots)
        if not _inside(real, real_root) or rel is None or rel != real.relative_to(real_root):
            out.append(p)
    return out


def _read(path: Path) -> Optional[Dict[str, Any]]:
    try:
        doc = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None
    return doc if isinstance(doc, dict) else None


def is_d07(doc: Dict[str, Any]) -> bool:
    """A weld-path document (D07): has a ``planner`` block, or is a schema 2.x path document (``seams`` and
    ``rejected``) whose planner block is missing — it then fails the record-version check."""
    if isinstance(doc.get('planner'), dict):
        return True
    return str(doc.get('schema_version', '')).startswith('2.') and 'seams' in doc and 'rejected' in doc


def record_version_problem(doc: Dict[str, Any]) -> Optional[str]:
    """None if the D07 record version is current, else the problem text."""
    planner = doc.get('planner')
    params = planner.get('params') if isinstance(planner, dict) else None
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
        logical = Path(os.path.abspath(job / name))
        path = logical.resolve()
        if not name or not _inside(path, root):
            problems.append(f'{name}: outside the repository ({path}); {LINK_FIX}')
            continue
        if logical != path:
            problems.append(f'{name}: reached through a symbolic link ({logical} -> {path}); {LINK_FIX}')
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
