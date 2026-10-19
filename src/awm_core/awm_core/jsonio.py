"""Schema-validated JSON read / write with the common header (DES-001 §5.2).

Every job-package file carries ``schema_version``, ``created_by``, ``created_at`` and ``inputs``
(list of ``{name, sha256}``). ``name`` is a POSIX path relative to a base directory chosen by the
caller (the job directory for job files, the repository root for cell files). Readers recompute
the input hashes and refuse stale inputs.
"""

from __future__ import annotations

import copy
import datetime as _dt
import json
import os
import tempfile
from functools import lru_cache
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple, Union

import jsonschema

from awm_core.hashing import sha256_file

PathLike = Union[str, Path]

SCHEMA_VERSION = '1.0'
SCHEMA_DIR = Path(__file__).resolve().parent / 'schemas'
_COMMON = 'common.schema.json'
_REF_PREFIX = _COMMON + '#/$defs/'


class SchemaValidationError(ValueError):
    """A document does not match its JSON schema."""


class StaleInputError(RuntimeError):
    """An input recorded in a document's header changed or is missing."""


def schema_names() -> List[str]:
    """Return the names of all document schemas (without ``.schema.json``)."""
    return sorted(p.name[:-len('.schema.json')] for p in SCHEMA_DIR.glob('*.schema.json')
                  if p.name != _COMMON)


def _inline_common(node: Any, defs: Dict[str, Any]) -> Any:
    """Replace ``common.schema.json#/$defs/X`` references by the definitions themselves.

    Done here instead of through a jsonschema registry so the same code works with the
    jsonschema versions of Ubuntu 24.04 (4.10) and conda-forge (4.2x).
    """
    if isinstance(node, dict):
        ref = node.get('$ref')
        if isinstance(ref, str) and ref.startswith(_REF_PREFIX):
            name = ref[len(_REF_PREFIX):]
            if name not in defs:
                raise KeyError(f'unknown common definition {name!r}')
            merged = _inline_common(copy.deepcopy(defs[name]), defs)
            extra = {k: _inline_common(v, defs) for k, v in node.items() if k != '$ref'}
            merged.update(extra)
            return merged
        return {k: _inline_common(v, defs) for k, v in node.items()}
    if isinstance(node, list):
        return [_inline_common(v, defs) for v in node]
    return node


@lru_cache(maxsize=None)
def load_schema(name: str) -> Dict[str, Any]:
    """Load a schema by name (e.g. ``'part'``) with the common definitions inlined."""
    path = SCHEMA_DIR / f'{name}.schema.json'
    if not path.is_file():
        raise FileNotFoundError(f'no JSON schema named {name!r} in {SCHEMA_DIR}')
    common = json.loads((SCHEMA_DIR / _COMMON).read_text())
    schema = json.loads(path.read_text())
    resolved = _inline_common(schema, common['$defs'])
    jsonschema.Draft202012Validator.check_schema(resolved)
    return resolved


def validate(doc: Dict[str, Any], schema: str) -> None:
    """Validate ``doc`` against schema ``schema``; raise SchemaValidationError with all errors."""
    validator = jsonschema.Draft202012Validator(load_schema(schema))
    errors = sorted(validator.iter_errors(doc), key=lambda e: list(e.absolute_path))
    if errors:
        lines = []
        for err in errors[:20]:
            where = '/'.join(str(p) for p in err.absolute_path) or '<root>'
            lines.append(f'  {where}: {err.message}')
        more = '' if len(errors) <= 20 else f'\n  ... {len(errors) - 20} more'
        raise SchemaValidationError(f'document does not match schema {schema!r}:\n'
                                    + '\n'.join(lines) + more)


def utc_now_iso() -> str:
    """Current UTC time, ISO 8601 with seconds and a ``Z`` suffix."""
    return _dt.datetime.now(_dt.timezone.utc).replace(microsecond=0).isoformat().replace(
        '+00:00', 'Z')


def input_entry(path: PathLike, base_dir: PathLike) -> Dict[str, str]:
    """Build one ``inputs`` entry for ``path`` relative to ``base_dir``."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f'input file not found: {path}')
    name = Path(os.path.relpath(path.resolve(), Path(base_dir).resolve())).as_posix()
    return {'name': name, 'sha256': sha256_file(path)}


def make_header(created_by: str, inputs: Iterable[PathLike], base_dir: PathLike) -> Dict[str, Any]:
    """Return the common header for a new document."""
    return {
        'schema_version': SCHEMA_VERSION,
        'created_by': created_by,
        'created_at': utc_now_iso(),
        'inputs': [input_entry(p, base_dir) for p in inputs],
    }


def check_inputs(doc: Dict[str, Any], base_dir: PathLike) -> None:
    """Raise StaleInputError if any recorded input is missing or has a different hash."""
    problems = []
    for entry in doc.get('inputs', []):
        path = Path(base_dir) / entry['name']
        if not path.is_file():
            problems.append(f'{entry["name"]}: missing')
        elif sha256_file(path) != entry['sha256']:
            problems.append(f'{entry["name"]}: changed since this file was created')
    if problems:
        raise StaleInputError('stale inputs:\n  ' + '\n  '.join(problems))


def write_json(path: PathLike, doc: Dict[str, Any], schema: str) -> Path:
    """Validate ``doc`` and write it atomically (temp file + rename). Return the path."""
    validate(doc, schema)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(doc, indent=2, sort_keys=False, allow_nan=False) + '\n'
    fd, tmp = tempfile.mkstemp(prefix='.' + path.name + '.', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as handle:
            handle.write(text)
        os.chmod(tmp, 0o644)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return path


def read_json(path: PathLike, schema: str, base_dir: Optional[PathLike] = None,
              check: bool = True) -> Dict[str, Any]:
    """Read and validate a document. If ``check`` and ``base_dir`` are given, verify its inputs."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f'{schema} file not found: {path}')
    try:
        doc = json.loads(path.read_text(encoding='utf-8'))
    except json.JSONDecodeError as exc:
        raise SchemaValidationError(f'{path}: invalid JSON: {exc}') from exc
    try:
        validate(doc, schema)
    except SchemaValidationError as exc:
        raise SchemaValidationError(f'{path}: {exc}') from exc
    if check and base_dir is not None:
        try:
            check_inputs(doc, base_dir)
        except StaleInputError as exc:
            raise StaleInputError(f'{path}: {exc}') from exc
    return doc


def pose_dict(position: Sequence[float], quat_xyzw: Sequence[float]) -> Dict[str, List[float]]:
    """Return a pose in the JSON layout used by all schemas."""
    return {'position_m': [float(v) for v in position], 'quat_xyzw': [float(v) for v in quat_xyzw]}


def split_pose(pose: Dict[str, Any]) -> Tuple[List[float], List[float]]:
    """Inverse of :func:`pose_dict`."""
    return list(pose['position_m']), list(pose['quat_xyzw'])
