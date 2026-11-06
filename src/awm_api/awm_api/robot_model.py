"""Robot description for the browser (G1.6, S4): the cell URDF the planner uses, with mesh URLs the browser can load.

One source: the URDF is rendered by ``awm_path.description.load_planning_description`` (head model ``hcad``, the
pose arguments from ``cell/model/frames.yaml``) in a subprocess (``python3 -m awm_api.describe``), so this API
process imports neither xacro nor the planner (B-11). The rendered URDF stays in metres / radians (REP 103, third
party); the API converts nothing.

Mesh URIs are rewritten:

* ``file://<head_dir>/<rel>``          → ``/api/v1/cell/meshes/head/<rel>``
* ``package://<pkg>/<rel>``            → ``/api/v1/cell/meshes/package/<pkg>/<rel>``

Any other URI refuses the description (500 ``description-failed``). The mesh endpoints serve read-only files below
the head folder or the share folder of a package **named by the rendered URDF**, with a mesh file extension; no
``..``, no absolute path, no symlink leaving the folder (403).
"""

from __future__ import annotations

import re
import subprocess
import threading
from pathlib import Path
from typing import Callable, Dict, Optional, Set, Tuple

from awm_core.hashing import sha256_file

from awm_api.problems import ProblemError
from awm_api.settings import Settings

API_MESHES = '/api/v1/cell/meshes'
MESH_SUFFIXES = ('.stl', '.dae', '.obj', '.glb', '.gltf', '.png', '.jpg', '.jpeg')
HEAD_MODEL = 'hcad'  # Goal 1 global constraint (master plan): the planner default head model
_FILENAME_RE = re.compile(r'filename="([^"]*)"')
_COMMENT_RE = re.compile(r'<!--.*?-->', re.S)


def strip_comments(urdf: str) -> str:
    """XML comments removed (the xacro banner names the machine's install path; the browser needs no comments)."""
    return _COMMENT_RE.sub('', urdf)
_PKG_RE = re.compile(r'^package://([A-Za-z0-9_]+)/(.+)$')


def _bad_rel(rel: str) -> bool:
    parts = Path(rel).parts
    return (not rel or rel.startswith('/') or '\\' in rel or '..' in parts or '\x00' in rel
            or any(p.startswith('.') for p in parts))


def safe_child(base: Path, rel: str) -> Path:
    """Existing mesh file ``rel`` below ``base`` (403 for traversal / type, 404 when missing)."""
    if _bad_rel(rel):
        raise ProblemError(403, 'forbidden-path', detail=f'{rel!r} is not a relative mesh path')
    if not rel.lower().endswith(MESH_SUFFIXES):
        raise ProblemError(403, 'forbidden-path', detail=f'{rel!r} is not a mesh file ({", ".join(MESH_SUFFIXES)})')
    root = base.resolve()
    path = (root / rel).resolve()
    if root not in path.parents:
        raise ProblemError(403, 'forbidden-path', detail=f'{rel!r} leaves its folder')
    if not path.is_file():
        raise ProblemError(404, 'mesh-not-found', detail=f'{rel} not found')
    return path


def rewrite_mesh_urls(urdf: str, head_dir: Path) -> Tuple[str, Set[str]]:
    """Rewrite the mesh URIs of ``urdf``; returns the new text and the packages it references."""
    heads = {f'file://{head_dir}/', f'file://{head_dir.resolve()}/'}
    packages: Set[str] = set()

    def sub(m: 're.Match[str]') -> str:
        uri = m.group(1)
        for prefix in heads:
            if uri.startswith(prefix):
                rel = uri[len(prefix):]
                if _bad_rel(rel):
                    break
                return f'filename="{API_MESHES}/head/{rel}"'
        pm = _PKG_RE.match(uri)
        if pm and not _bad_rel(pm.group(2)):
            packages.add(pm.group(1))
            return f'filename="{API_MESHES}/package/{pm.group(1)}/{pm.group(2)}"'
        raise ProblemError(500, 'description-failed', 'Robot description refused',
                           detail=f'mesh URI {uri!r} is neither in the head folder nor a package:// URI')

    return _FILENAME_RE.sub(sub, urdf), packages


def _ament_share(pkg: str) -> Path:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        return Path(get_package_share_directory(pkg))
    except PackageNotFoundError as exc:
        raise ProblemError(404, 'mesh-not-found', detail=f'package {pkg} not installed') from exc


class RobotModel:
    """Rendered + rewritten URDF, cached by the SHA-256 of ``frames.yaml`` and ``head.yaml``."""

    def __init__(self, settings: Settings, share: Optional[Callable[[str], Path]] = None) -> None:
        self.settings = settings
        shares = settings.package_shares
        if share is not None:
            self._share = share
        elif shares is not None:
            self._share = lambda pkg: Path(shares[pkg]) if pkg in shares else _missing(pkg)
        else:
            self._share = _ament_share
        self._lock = threading.Lock()
        self._key: Optional[Tuple[str, str, str]] = None
        self._doc: Optional[Dict] = None
        self._packages: Set[str] = set()

    def _inputs(self) -> Tuple[Path, Path]:
        frames, head = self.settings.frames_yaml, self.settings.head_dir
        missing = [str(p) for p in (frames, head / 'head.yaml') if not p.is_file()]
        if missing:
            raise ProblemError(409, 'no-cell-model', 'No cell model',
                               detail='the cell frames (export-scene) or the H-CAD head folder (export-head) are '
                                      f'missing: {", ".join(missing)}')
        return frames, head.resolve()

    def description(self) -> Dict:
        frames, head = self._inputs()
        key = (sha256_file(frames), sha256_file(head / 'head.yaml'), str(head))
        with self._lock:
            if self._key == key and self._doc is not None:
                return self._doc
            argv = self.settings.describe_argv() + ['describe', '--frames', str(frames), '--head-dir', str(head)]
            try:
                proc = subprocess.run(argv, capture_output=True, text=True, timeout=self.settings.describe_timeout_s,
                                      check=False)
            except (OSError, subprocess.TimeoutExpired) as exc:
                raise ProblemError(500, 'description-failed', 'Robot description failed', detail=str(exc)) from exc
            if proc.returncode != 0 or '<robot' not in proc.stdout:
                tail = ' | '.join([ln for ln in proc.stderr.splitlines() if ln.strip()][-4:])
                raise ProblemError(500, 'description-failed', 'Robot description failed',
                                   detail=tail or f'exit {proc.returncode}')
            urdf, packages = rewrite_mesh_urls(strip_comments(proc.stdout.strip()), head)
            self._doc = {'urdf': urdf, 'head_model': HEAD_MODEL, 'unit': 'm, rad (URDF, REP 103)',
                         'frames_sha256': key[0], 'head_yaml_sha256': key[1],
                         'note': 'Rendered like the planner (awm_path.description, head_model hcad). Display only.'}
            self._packages = packages
            self._key = key
            return self._doc

    def head_mesh(self, rel: str) -> Path:
        return safe_child(self.settings.head_dir, rel)

    def package_mesh(self, pkg: str, rel: str) -> Path:
        if pkg not in self._packages:
            self.description()  # render once if needed; refuses packages the URDF does not name
        if pkg not in self._packages:
            raise ProblemError(403, 'forbidden-path', detail=f'package {pkg!r} is not referenced by the description')
        return safe_child(self._share(pkg), rel)


def _missing(pkg: str) -> Path:
    raise ProblemError(404, 'mesh-not-found', detail=f'package {pkg} not installed')
