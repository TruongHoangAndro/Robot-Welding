"""Cell v2 configuration helpers shared by the launch, the scene tools and tests (no ROS graph needed)."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Dict, List, Optional, Union

import numpy as np
import yaml

from awm_core import units

PathLike = Union[str, Path]


def positioner_limits_path() -> Path:
    """Installed ``awm_description/config/positioner_limits.yaml`` (the only source of the positioner limits)."""
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('awm_description')) / 'config' / 'positioner_limits.yaml'


def load_positioner_limits(path: Optional[PathLike] = None) -> Dict[str, float]:
    """positioner_joint limits in deg, deg/s, deg/s^2."""
    data = yaml.safe_load(Path(path or positioner_limits_path()).read_text())
    lim = data['positioner_joint']
    for key in ('lower_deg', 'upper_deg', 'max_velocity_deg_s', 'max_acceleration_deg_s2'):
        lim[key] = float(lim[key])
    if not (lim['lower_deg'] < lim['upper_deg'] and lim['max_velocity_deg_s'] > 0
            and lim['max_acceleration_deg_s2'] > 0):
        raise ValueError(f'invalid positioner limits: {lim}')
    return lim


def positioner_moveit_limits(lim: Dict[str, float]) -> Dict[str, object]:
    """MoveIt ``robot_description_planning.joint_limits.positioner_joint`` entry (rad, rad/s, rad/s^2)."""
    return {'has_velocity_limits': True, 'max_velocity': units.deg_to_rad(lim['max_velocity_deg_s']),
            'has_acceleration_limits': True, 'max_acceleration': units.deg_to_rad(lim['max_acceleration_deg_s2'])}


HEAD_MESH_LINKS = ('head_body', 'head_mount')


class HeadDirError(ValueError):
    """The H-CAD head folder (head_model:=hcad) is unusable: path, head.yaml or a mesh file is missing or changed."""


def check_head_dir(head_dir: str) -> List[Path]:
    """Check an H-CAD head folder before xacro runs; return head.yaml and the mesh files it names.

    head_dir must be absolute (it becomes ``file://`` mesh URIs). head.yaml (awm_geometry export-head, A18) must name a
    mesh and a hull file for head_body and head_mount; each file must exist and match its SHA-256 in head.yaml.
    """
    value = (head_dir or '').strip()
    if not value or not Path(value).is_absolute():
        raise HeadDirError(f'head_model:=hcad needs head_dir:=<absolute folder with head.yaml>, got {head_dir!r}')
    folder = Path(value)
    head_yaml = folder / 'head.yaml'
    if not head_yaml.is_file():
        raise HeadDirError(f'{head_yaml} not found')
    doc = yaml.safe_load(head_yaml.read_text()) or {}
    files = [head_yaml]
    for link in HEAD_MESH_LINKS:
        entry = (doc.get('links') or {}).get(link)
        if not entry:
            raise HeadDirError(f'{head_yaml}: links.{link} missing')
        for kind in ('mesh', 'hull'):
            spec = entry.get(kind) or {}
            path = folder / str(spec.get('file', ''))
            if not spec.get('file') or not path.is_file():
                raise HeadDirError(f'{head_yaml}: links.{link}.{kind} file {path} not found')
            sha = hashlib.sha256(path.read_bytes()).hexdigest()
            if spec.get('sha256') and sha != spec['sha256']:
                raise HeadDirError(f'SHA-256 of {path} does not match {head_yaml} (file changed after export)')
            files.append(path)
    return files


# Conduit position args of awm_cell_v2.urdf.xacro (H-BOX, and H-CAD only when head.yaml has no cable exit).
CONDUIT_POSITION_ARGS = ('conduit_x_mm', 'conduit_y_mm', 'conduit_z_mm', 'conduit_r_deg', 'conduit_p_deg',
                         'conduit_y_deg')


def conduit_source(head_dir: str) -> str:
    """Conduit source of an H-CAD head folder, same rule as lightweld_hcad.xacro (G1.8 / review I4):
    ``explicit`` (cable_exit_tool0_mm + cable_axis_tool0 with cable_exit_source map or null; ``rear_estimate`` when
    cable_exit_source is rear_estimate), ``rear_estimate`` (estimates.tcp.geometry_estimate behind
    the flange) or ``parametric`` (neither: the conduit_* args)."""
    doc = yaml.safe_load((Path(head_dir.strip()) / 'head.yaml').read_text()) or {}
    if doc.get('cable_exit_tool0_mm') is not None and doc.get('cable_axis_tool0') is not None:
        label = doc.get('cable_exit_source')
        if label not in (None, 'map', 'rear_estimate'):
            raise ValueError(f'{head_dir}/head.yaml: cable_exit_source {label!r} unknown (expected map, rear_estimate '
                             'or null)')
        return 'rear_estimate' if label == 'rear_estimate' else 'explicit'
    est = ((doc.get('estimates') or {}).get('tcp') or {}).get('geometry_estimate') or {}
    if est.get('behind_flange') is True and est.get('tcp_tool0_mm') is not None \
            and est.get('beam_axis_tool0') is not None:
        return 'rear_estimate'
    return 'parametric'


def check_conduit_args(head_model: str, head_dir: str, set_args: List[str]) -> None:
    """Review M5: refuse conduit position args that the hcad xacro would ignore (head.yaml gives the cable exit)."""
    given = [a for a in set_args if a in CONDUIT_POSITION_ARGS]
    if head_model != 'hcad' or not given:
        return
    src = conduit_source(head_dir)
    if src != 'parametric':
        raise ValueError(f'{", ".join(given)} not used with head_model:=hcad: the conduit starts at the head.yaml cable '
                         f'exit (source {src}); only conduit_dia_mm / conduit_len_mm apply')


# --- G1.16 (close-out A2): sim_mock repo:=<root> -----------------------------------------------------------------

#: sim_mock pose arguments taken from ``cell/model/frames.yaml`` when ``repo:=`` is given.
REPO_POSE_ARGS = ('base_xyz_mm', 'base_quat_xyzw', 'positioner_xyz_mm', 'positioner_quat_xyzw', 'positioner_axis_xyz')
#: Arguments that ``repo:=`` sets itself; giving one of them together with ``repo:=`` is refused (no two sources).
REPO_CONFLICT_ARGS = REPO_POSE_ARGS + ('cell_manifest', 'head_dir')
HEAD_MODELS = ('hbox', 'hcad')
#: frames.yaml direction / point vs the pose of world_T_positioner_axis (= awm_seam.seam_axis.AXIS_CONVENTION_TOL).
AXIS_CONVENTION_TOL = 1e-6


class RepoArgError(ValueError):
    """``repo:=`` cannot be used: conflicting arguments, or a cell model file is missing, malformed or stale."""


def _fmt(values) -> str:
    """Full float precision (repr) so the URDF pose equals frames.yaml to the last digit."""
    return ' '.join(repr(float(v)) for v in values)


def _vector(entry: dict, key: str, n: int, where: str) -> np.ndarray:
    value = entry.get(key) if isinstance(entry, dict) else None
    try:
        vec = np.asarray([float(v) for v in value], dtype=float)
    except (TypeError, ValueError):
        vec = None
    if vec is None or vec.shape != (n,) or not np.all(np.isfinite(vec)):
        raise RepoArgError(f'{where}.{key}: expected {n} numbers, got {value!r}')
    return vec


def _unit_quat(entry: dict, where: str) -> np.ndarray:
    quat = _vector(entry, 'quat_xyzw', 4, where)
    if abs(np.linalg.norm(quat) - 1.0) > 1e-6:
        raise RepoArgError(f'{where}.quat_xyzw is not a unit quaternion: {quat.tolist()}')
    return quat


def frames_launch_args(frames_yaml: PathLike) -> Dict[str, str]:
    """sim_mock pose arguments from the export-scene ``frames.yaml`` (mm, quaternion xyzw).

    ``world_T_base`` -> base_link pose; ``world_T_positioner_axis`` -> positioner_axis pose (= positioner_flange at
    θ = 0); its world ``direction`` is expressed in the positioner_axis frame for ``positioner_axis_xyz``. Same rule
    as ``awm_path.gate_g2.launch_args_from_frames`` (planner side); here with checks and a clear refusal.
    """
    from awm_core.frames import quat_to_matrix

    path = Path(frames_yaml)
    try:
        doc = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise RepoArgError(f'{path}: cannot read ({exc})') from exc
    if not isinstance(doc, dict):
        raise RepoArgError(f'{path}: not a mapping')
    if doc.get('unit') != 'mm':
        raise RepoArgError(f'{path}: unit must be "mm", got {doc.get("unit")!r}')
    if not str(doc.get('schema_version', '')).startswith('2.'):
        raise RepoArgError(f'{path}: schema_version {doc.get("schema_version")!r} is not 2.x')
    for key in ('world_T_base', 'world_T_positioner_axis'):
        if not isinstance(doc.get(key), dict):
            raise RepoArgError(f'{path}: {key} missing')
    base, axis = doc['world_T_base'], doc['world_T_positioner_axis']
    where_b, where_a = f'{path.name} world_T_base', f'{path.name} world_T_positioner_axis'
    base_pos, base_q = _vector(base, 'position_mm', 3, where_b), _unit_quat(base, where_b)
    axis_pos, axis_q = _vector(axis, 'position_mm', 3, where_a), _unit_quat(axis, where_a)
    direction = _vector(axis, 'direction', 3, where_a)
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        raise RepoArgError(f'{where_a}.direction is the zero vector')
    rot = quat_to_matrix(axis_q)
    # Same convention as awm_seam.seam_axis.check_frames (copied, awm_bringup does not depend on awm_seam): the
    # positioner axis is +X of world_T_positioner_axis and point_mm its origin, within AXIS_CONVENTION_TOL.
    if np.linalg.norm(rot[:, 0] - direction / norm) > AXIS_CONVENTION_TOL:
        raise RepoArgError(f'{where_a}: direction {direction.tolist()} is not +X of world_T_positioner_axis '
                           '(the planner and seam_axis.json assume axis = +X)')
    if 'point_mm' in axis and np.linalg.norm(_vector(axis, 'point_mm', 3, where_a) - axis_pos) > AXIS_CONVENTION_TOL:
        raise RepoArgError(f'{where_a}: point_mm is not the origin of world_T_positioner_axis')
    local = rot.T @ (direction / norm)
    return {'base_xyz_mm': _fmt(base_pos), 'base_quat_xyzw': _fmt(base_q),
            'positioner_xyz_mm': _fmt(axis_pos), 'positioner_quat_xyzw': _fmt(axis_q),
            'positioner_axis_xyz': _fmt(local)}


def _check_manifest(manifest: Path, frames_yaml: Path) -> None:
    """The scene manifest must be readable, list objects and come from the same export as frames.yaml."""
    try:
        doc = yaml.safe_load(manifest.read_text(encoding='utf-8'))  # JSON is YAML
        frames = yaml.safe_load(frames_yaml.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise RepoArgError(f'{manifest}: cannot read ({exc})') from exc
    if not isinstance(doc, dict) or not doc.get('objects'):
        raise RepoArgError(f'{manifest}: no objects (re-run the cell import, S10)')
    named = doc.get('frames_file')
    if named != frames_yaml.name:
        raise RepoArgError(f'{manifest}: frames_file {named!r} is not {frames_yaml.name!r}')
    for path, d in ((manifest, doc), (frames_yaml, frames)):
        inputs = d.get('inputs') if isinstance(d, dict) else None
        if not isinstance(inputs, list) or not inputs:
            raise RepoArgError(f'{path}: inputs missing or empty; cannot check that the manifest and frames.yaml '
                               'are from the same export (re-run the cell import, S10)')
    if doc['inputs'] != frames['inputs']:
        raise RepoArgError(f'{manifest} and {frames_yaml} are not from the same export (inputs differ); re-run the '
                           'cell import (S10)')


def repo_launch_args(repo: str, given: Dict[str, str]) -> Dict[str, str]:
    """Resolve the sim_mock arguments from the job repository root (DES-001 §5.2, cell model written by A18 / CP-60).

    ``given``: the launch values of :data:`REPO_CONFLICT_ARGS` and ``head_model`` ('' = not set). Returns the pose
    arguments from ``<repo>/cell/model/frames.yaml``, ``cell_manifest`` = ``<repo>/cell/model/scene_manifest.json``,
    ``head_model`` / ``head_dir``: ``hcad`` with ``<repo>/cell/model/head`` when ``head/head.yaml`` exists; without a
    head only an explicit ``head_model:=hbox`` is accepted. Raises :class:`RepoArgError` (never a silent default).
    """
    conflicts = [n for n in REPO_CONFLICT_ARGS if str(given.get(n, '') or '').strip()]
    if conflicts:
        raise RepoArgError(f'repo:= sets {", ".join(conflicts)} from the repository cell model; do not pass '
                           f'{", ".join(c + ":=" for c in conflicts)} together with repo:=')
    head_model = str(given.get('head_model', '') or '').strip()
    if head_model not in ('',) + HEAD_MODELS:
        raise RepoArgError(f'head_model must be hbox or hcad, got {head_model!r}')
    root = Path(repo.strip()).expanduser()
    if not root.is_dir():
        raise RepoArgError(f'repo:={repo!r} is not a folder')
    root = root.resolve()
    model = root / 'cell' / 'model'
    frames_yaml, manifest = model / 'frames.yaml', model / 'scene_manifest.json'
    for path in (frames_yaml, manifest):
        if not path.is_file():
            raise RepoArgError(f'{path} not found: import the cell CAD first (S10, CP-60)')
    out = frames_launch_args(frames_yaml)
    _check_manifest(manifest, frames_yaml)
    out['cell_manifest'] = str(manifest)
    head_dir = model / 'head'
    if head_model == 'hbox':
        out.update(head_model='hbox', head_dir='')
    elif (head_dir / 'head.yaml').is_file():
        try:
            check_head_dir(str(head_dir))
        except HeadDirError as exc:
            raise RepoArgError(str(exc)) from exc
        out.update(head_model='hcad', head_dir=str(head_dir))
    else:
        raise RepoArgError(f'{head_dir / "head.yaml"} not found: import the head CAD (S10) or pass head_model:=hbox '
                           'explicitly for the parametric head')
    return out
