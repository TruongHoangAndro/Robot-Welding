"""Entry gate G2 = C1 scene sanity (WP-S S.4 C4; TST-002 Rev 0.3 draft §5.2 G2, §5.3 C1).

G2 passes when 10 / 10 deliberately colliding poses are invalid, 10 / 10 known free poses are valid, and the three
contact-zone poses are invalid (tip 0.5 mm into a non-seam face; rim into a corner wall; flat tip tilted 8° on a flat
face). The poses are **data** (``docs/generated/spike/g2_poses.yaml``), never source. This module parses the pose
file, turns a pose spec into a joint state, and summarises the results; the ROS-bound runner is
``awm_path.gate_g2``.

Pose file (YAML / JSON, mm, degrees)::

    schema: awm_g2_poses/0.1
    seam_faces: {<seam>: {n1: [<face_id>], n2: [<face_id>]}}   # D04 faces of the seams used by contact checks
    poses:
      - id: C01                                       # unique
        kind: colliding | free | contact_zone
        description: text (how the pose was built, what it should touch)
        theta_deg: 0.0                                # positioner angle
        tcp: {frame: part | world, position_mm: [x, y, z], beam_dir: [..], x_dir: [..]}   # or
        joints_deg: [q1, .., q6]
        label: <S±E±W±>                               # optional configuration label of the IK solution
        checks: [validity] | [validity, contact]      # contact needs `seam`
        seam: <seam>
        expect: {valid: false, objects: [<object_id>]}  # objects expected in a contact pair (reported)

Verdict of one pose = F4 validity, and the F9 contact check when listed (a node is valid only if both pass).
Pure Python, mm / deg (B-06).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from awm_path.core.kinematics import KinematicsBackend, ik_tcp
from awm_path.core.states import LABEL_ORDER, config_label

SCHEMA = 'awm_g2_poses/0.1'
KINDS = ('colliding', 'free', 'contact_zone')
#: Minimum number of poses per kind (TST-002 §5.2 G2) and the verdict each kind must get.
REQUIRED = {'colliding': 10, 'free': 10, 'contact_zone': 3}
EXPECTED_VALID = {'colliding': False, 'free': True, 'contact_zone': False}


class PoseSpecError(ValueError):
    """The pose file is malformed or a pose cannot be resolved."""


@dataclass(frozen=True)
class GatePose:
    """One G2 pose spec (see the module docstring)."""

    pose_id: str
    kind: str
    description: str
    theta_deg: float
    tcp: Optional[Mapping[str, Any]]
    joints_deg: Optional[Tuple[float, ...]]
    label: Optional[str]
    checks: Tuple[str, ...]
    seam: Optional[str]
    expect_valid: bool
    expect_objects: Tuple[str, ...]


@dataclass(frozen=True)
class GateResult:
    """Result of one pose: overall verdict, F4 contact pairs, contact result (or None), details for the log."""

    pose: GatePose
    valid: bool
    contacts: Tuple[str, ...]
    contact: Any
    details: Optional[Dict[str, Any]]

    @property
    def passed(self) -> bool:
        return self.valid == self.pose.expect_valid

    @property
    def expected_object_seen(self) -> Optional[bool]:
        if not self.pose.expect_objects:
            return None
        return any(o in pair.split('<->') for pair in self.contacts for o in self.pose.expect_objects)


def _vec(v: Any, n: int, where: str) -> Tuple[float, ...]:
    try:
        out = tuple(float(x) for x in v)
    except TypeError as exc:
        raise PoseSpecError(f'{where}: expected {n} numbers') from exc
    if len(out) != n or not all(np.isfinite(out)):
        raise PoseSpecError(f'{where}: expected {n} finite numbers, got {v!r}')
    return out


def load_pose_specs(doc: Mapping[str, Any]) -> Tuple[List[GatePose], Dict[str, Tuple[Tuple[str, ...],
                                                                                       Tuple[str, ...]]]]:
    """Parse a pose file (already loaded from YAML / JSON). Returns (poses, seam faces by seam name)."""
    if doc.get('schema') != SCHEMA:
        raise PoseSpecError(f'pose file schema {doc.get("schema")!r}, expected {SCHEMA!r}')
    seams = {}
    for name, f in (doc.get('seam_faces') or {}).items():
        n1, n2 = tuple(str(x) for x in f.get('n1') or ()), tuple(str(x) for x in f.get('n2') or ())
        if not n1 or not n2:
            raise PoseSpecError(f'seam_faces.{name}: n1 and n2 are required')
        seams[str(name)] = (n1, n2)
    poses: List[GatePose] = []
    seen = set()
    for i, p in enumerate(doc.get('poses') or []):
        where = f'poses[{i}]'
        pid = str(p.get('id', ''))
        if not pid or pid in seen:
            raise PoseSpecError(f'{where}: missing or duplicate id {pid!r}')
        seen.add(pid)
        kind = p.get('kind')
        if kind not in KINDS:
            raise PoseSpecError(f'{pid}: kind must be one of {KINDS}')
        checks = tuple(p.get('checks') or ())
        if not checks or checks[0] != 'validity' or not set(checks) <= {'validity', 'contact'}:
            raise PoseSpecError(f'{pid}: checks must be [validity] or [validity, contact]')
        seam = p.get('seam')
        if 'contact' in checks and seam not in seams:
            raise PoseSpecError(f'{pid}: contact check needs a seam listed in seam_faces')
        tcp, joints = p.get('tcp'), p.get('joints_deg')
        if (tcp is None) == (joints is None):
            raise PoseSpecError(f'{pid}: give exactly one of tcp / joints_deg')
        if tcp is not None:
            if tcp.get('frame') not in ('part', 'world'):
                raise PoseSpecError(f'{pid}: tcp.frame must be part or world')
            for key in ('position_mm', 'beam_dir', 'x_dir'):
                _vec(tcp.get(key), 3, f'{pid}.tcp.{key}')
        if 'contact' in checks and tcp is None:
            raise PoseSpecError(f'{pid}: a contact check needs a tcp pose')
        expect = p.get('expect') or {}
        if 'valid' not in expect:
            raise PoseSpecError(f'{pid}: expect.valid is required')
        if bool(expect['valid']) != EXPECTED_VALID[kind]:
            raise PoseSpecError(f'{pid}: a {kind} pose must expect valid = {EXPECTED_VALID[kind]}')
        label = p.get('label')
        if label is not None and label not in LABEL_ORDER:
            raise PoseSpecError(f'{pid}: unknown configuration label {label!r}')
        poses.append(GatePose(pid, kind, str(p.get('description', '')), float(p.get('theta_deg', 0.0)),
                              dict(tcp) if tcp is not None else None,
                              _vec(joints, 6, f'{pid}.joints_deg') if joints is not None else None,
                              label, checks, str(seam) if seam is not None else None, bool(expect['valid']),
                              tuple(str(o) for o in expect.get('objects') or ())))
    return poses, seams


def tool_pose(position_mm: Sequence[float], beam_dir: Sequence[float], x_dir: Sequence[float]) -> np.ndarray:
    """4x4 pose with Z = beam direction (tool Z, DES-001 §5.3) and X = ``x_dir`` made orthogonal to it."""
    z = np.asarray(beam_dir, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.asarray(x_dir, dtype=float)
    x = x - z * float(x @ z)
    if np.linalg.norm(x) < 1e-9:
        raise ValueError('x_dir is parallel to the beam direction')
    x = x / np.linalg.norm(x)
    t = np.eye(4)
    t[:3, :3] = np.stack([x, np.cross(z, x), z], axis=1)
    t[:3, 3] = np.asarray(position_mm, dtype=float)
    return t


def target_world_T_tcp(pose: GatePose, world_T_part: np.ndarray) -> np.ndarray:
    """``world_T_tcp`` (mm) of a tcp spec (``world_T_part`` at the pose's positioner angle)."""
    t = tool_pose(pose.tcp['position_mm'], pose.tcp['beam_dir'], pose.tcp['x_dir'])
    return world_T_part @ t if pose.tcp['frame'] == 'part' else t


def resolve_pose(pose: GatePose, world_T_base: np.ndarray, world_T_part: np.ndarray, tool0_T_tcp: np.ndarray,
                 kin: Optional[KinematicsBackend]) -> Tuple[np.ndarray, Dict[str, Any]]:
    """Joint state (deg) of a pose spec, and how it was obtained.

    tcp specs: all IK solutions sorted by configuration-label order then joint values; the one with ``label`` (or
    the first) is taken. Raises ``PoseSpecError`` when there is no IK solution (with that label).
    """
    if pose.joints_deg is not None:
        q = np.asarray(pose.joints_deg, dtype=float)
        return q, {'source': 'joints_deg', 'label': config_label(q)}
    target = np.linalg.inv(world_T_base) @ target_world_T_tcp(pose, world_T_part)
    sols = sorted(((LABEL_ORDER[config_label(q)], tuple(float(x) for x in q)) for q in ik_tcp(kin, target,
                                                                                                tool0_T_tcp)))
    if pose.label is not None:
        sols_l = [s for s in sols if s[0] == LABEL_ORDER[pose.label]]
    else:
        sols_l = sols
    if not sols_l:
        raise PoseSpecError(f'{pose.pose_id}: no IK solution' + (f' with label {pose.label}' if pose.label else ''))
    q = np.asarray(sols_l[0][1])
    return q, {'source': 'tcp', 'label': config_label(q), 'solutions': len(sols)}


def gate_summary(results: Sequence[GateResult]) -> Dict[str, Any]:
    """Counts per kind ([passed, total]), failing pose ids, kinds with too few poses, overall pass (G2)."""
    counts = {k: [0, 0] for k in KINDS}
    failing = []
    seen, with_objects = 0, 0
    for r in results:
        counts[r.pose.kind][1] += 1
        if r.passed:
            counts[r.pose.kind][0] += 1
        else:
            failing.append(r.pose.pose_id)
        if r.expected_object_seen is not None:
            with_objects += 1
            seen += int(r.expected_object_seen)
    missing = [k for k in KINDS if counts[k][1] < REQUIRED[k]]
    return {'pass': not failing and not missing, 'counts': counts, 'failing': failing, 'missing': missing,
            'required': dict(REQUIRED), 'expected_objects_seen': [seen, with_objects]}


__all__ = ['SCHEMA', 'KINDS', 'REQUIRED', 'PoseSpecError', 'GatePose', 'GateResult', 'load_pose_specs', 'tool_pose',
           'target_world_T_tcp', 'resolve_pose', 'gate_summary']
