"""D04 v2 seams from picked joint edges (PLAN T2.2, A21 / A22 / A24; CP-52, CP-53, CP-59).

Pure Python (no OCC). A picked edge id (``E<n>`` of either solid, from the D20 ``topology.json``) names the joint
between two solids through its ``joint_edge_id``; the seam is that joint. The exact seam samples (points, tangents
and the joint-aware exterior-face normals n1 / n2) come from the B-Rep in the conda process
(``awm_geometry.cli sample-joints``, called through :mod:`awm_seam.geometry_client`); this module validates the
Engineer's attributes and writes ``seams.json`` (schema ``v2/seams``, mm, part frame).

Conventions (stated in the schema): the forward weld direction is the curve direction of the joint edge of
``solid_a``; ``reversed`` flips it. ``start_offset_mm`` is the arc length from the first vertex of that edge to the
weld start, measured in the curve direction, so a start point stays where it is when the direction is reversed.
The software never splits a seam by itself: ``arcs`` and ``arc_overlap_mm`` are entered by the Engineer (CP-53).
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, fields
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from awm_core import cad_roles, jsonio
from awm_core.hashing import sha256_text
from awm_core.repository import JobPaths

CREATED_BY = 'awm_seam 0.2.0'
SEAMS_SCHEMA = 'v2/seams'
SCHEMA_VERSION = '2.1'
TOPOLOGY_SCHEMA = 'view_topology'
#: Sampling pitch ceiling along the seam (G1.2 task brief: "<= 1 mm arc length").
MAX_PITCH_MM = 1.0
POSITIONER_MODES = ('sync', 'C', 'B')
JOINT_TYPES = ('butt', 'fillet', 'lap', 'groove')
MAX_ARCS = 12
SEAM_ID = re.compile(r'^S[0-9]{2,3}$')
#: Agreement of the sampled length with the D20 joint length (mm); larger = sampler and D20 disagree.
LENGTH_TOL_MM = 1e-3

Sampler = Callable[[JobPaths, Dict], Dict]


class SeamDefinitionError(ValueError):
    """A seam request is invalid (unknown edge, not a joint, bad attribute, ...)."""


def topology_path(job: JobPaths):
    return job.dir / 'cache' / 'view' / 'topology.json'


@dataclass
class JointSeamRequest:
    """What the Engineer enters in S2 for one seam (UI / CLI / pick file)."""

    seam_id: str
    edges: List[str]
    joint_type: str
    sequence: int
    reversed: bool = False
    start_offset_mm: float = 0.0
    start_point_mm: Optional[List[float]] = None
    arcs: int = 1
    arc_overlap_mm: float = 0.0
    positioner_mode: str = 'sync'
    positioner_angles_deg: Optional[List[float]] = None
    leg_mm: Optional[float] = None
    note: str = ''

    @classmethod
    def from_dict(cls, data: Dict) -> 'JointSeamRequest':
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(data) - known)
        if unknown:
            raise SeamDefinitionError(f'unknown seam request field(s) {unknown}; allowed {sorted(known)}')
        missing = [k for k in ('seam_id', 'edges', 'joint_type', 'sequence') if k not in data]
        if missing:
            raise SeamDefinitionError(f'seam request without {missing}')
        req = cls(**data)
        req.edges = list(req.edges)
        return req


class Topology:
    """Read-only view of the D20 ``topology.json`` (edge -> joint lookup)."""

    def __init__(self, doc: Dict) -> None:
        self.doc = doc
        self.edges = {e['edge_id']: e for e in doc['edges']}
        self.joints = {j['joint_edge_id']: j for j in doc['joints']}

    @classmethod
    def load(cls, job: JobPaths) -> 'Topology':
        """Read and validate ``cache/view/topology.json``; refuse it if ``model.step`` changed."""
        return cls(jsonio.read_json(topology_path(job), TOPOLOGY_SCHEMA, base_dir=job.dir))

    def resolve_joint(self, edge_ids: Sequence[str]) -> Dict:
        """A21: the joint named by the picked edges (an edge of either solid of the joint)."""
        if not edge_ids:
            raise SeamDefinitionError('no edge picked')
        names = set()
        for eid in edge_ids:
            edge = self.edges.get(eid)
            if edge is None:
                raise SeamDefinitionError(f'unknown edge {eid}')
            jid = edge.get('joint_edge_id')
            if not jid:
                raise SeamDefinitionError(
                    f'edge {eid} is not a joint edge between two solids (Goal 1 welds joints only)')
            names.add(jid)
        if len(names) > 1:
            raise SeamDefinitionError(f'picked edges {list(edge_ids)} belong to two joints {sorted(names)}; '
                                      'pick one joint per seam')
        jid = names.pop()
        if jid not in self.joints:
            raise SeamDefinitionError(f'joint {jid} of the picked edges is not listed in topology.json')
        return self.joints[jid]


def check_request(req: JointSeamRequest, joint: Dict) -> None:
    """A22 attribute checks; raises SeamDefinitionError with the seam id."""
    sid = req.seam_id
    if not isinstance(sid, str) or not SEAM_ID.match(sid):
        raise SeamDefinitionError(f'seam id {sid!r} must look like S01 (user-assigned)')
    if req.joint_type not in JOINT_TYPES:
        raise SeamDefinitionError(f'{sid}: joint type {req.joint_type!r} not in {JOINT_TYPES}')
    if not isinstance(req.sequence, int) or req.sequence < 1:
        raise SeamDefinitionError(f'{sid}: sequence must be an integer >= 1')
    if req.leg_mm is not None and req.leg_mm < 0:
        raise SeamDefinitionError(f'{sid}: leg_mm must be >= 0')
    if req.positioner_mode not in POSITIONER_MODES:
        raise SeamDefinitionError(f'{sid}: positioner_mode {req.positioner_mode!r} not in {POSITIONER_MODES}')
    if not isinstance(req.arcs, int) or not 1 <= req.arcs <= MAX_ARCS:
        raise SeamDefinitionError(f'{sid}: arcs must be an integer 1..{MAX_ARCS} (entered by the Engineer)')
    if req.arc_overlap_mm < 0:
        raise SeamDefinitionError(f'{sid}: arc_overlap_mm must be >= 0')
    arc_len = float(joint['length_mm']) / req.arcs
    if req.arc_overlap_mm >= arc_len:
        raise SeamDefinitionError(f'{sid}: arc overlap {req.arc_overlap_mm:.2f} mm is not shorter than one arc '
                                  f'({arc_len:.2f} mm)')
    if req.positioner_mode == 'B':
        angles = req.positioner_angles_deg
        if angles is None or len(angles) != req.arcs:
            raise SeamDefinitionError(f'{sid}: mode B needs one angle per arc ({req.arcs}) in '
                                      'positioner_angles_deg')
    elif req.positioner_angles_deg is not None:
        raise SeamDefinitionError(f'{sid}: positioner_angles_deg is only for mode B')
    if req.start_offset_mm < 0 or req.start_offset_mm >= float(joint['length_mm']):
        raise SeamDefinitionError(f'{sid}: start_offset_mm must be in [0, joint length '
                                  f'{float(joint["length_mm"]):.2f} mm)')
    if req.start_point_mm is not None and req.start_offset_mm:
        raise SeamDefinitionError(f'{sid}: give the start as start_offset_mm or start_point_mm, not both')
    if not joint['closed'] and (req.start_offset_mm or req.start_point_mm is not None):
        raise SeamDefinitionError(f'{sid}: the start of an open joint is its end vertex (use reversed); '
                                  'partial seams are not supported')


def check_seam_list(requests: Sequence[JointSeamRequest], joints: Sequence[Dict]) -> None:
    """Unique seam ids and sequence numbers; one joint is welded by one seam."""
    for attr in ('seam_id', 'sequence'):
        values = [getattr(r, attr) for r in requests]
        dup = sorted({v for v in values if values.count(v) > 1})
        if dup:
            raise SeamDefinitionError(f'duplicate {attr}(s): {dup}')
    used: Dict[str, str] = {}
    for req, joint in zip(requests, joints):
        jid = joint['joint_edge_id']
        if jid in used:
            raise SeamDefinitionError(f'joint {jid} is used by {used[jid]} and {req.seam_id}')
        used[jid] = req.seam_id


def sampler_request(requests: Sequence[JointSeamRequest], joints: Sequence[Dict], pitch_mm: float) -> Dict:
    """Request file of ``awm_geometry.cli sample-joints``."""
    return {'pitch_mm': float(pitch_mm), 'seams': [
        {'seam_id': r.seam_id, 'joint_edge_id': j['joint_edge_id'], 'reversed': bool(r.reversed),
         'start_offset_mm': float(r.start_offset_mm),
         'start_point_mm': None if r.start_point_mm is None else [float(v) for v in r.start_point_mm]}
        for r, j in zip(requests, joints)]}


def seam_entry(req: JointSeamRequest, joint: Dict, topo: Topology, sampled: Dict) -> Dict:
    """One D04 v2 seam from the request, its joint and the sampler output."""
    sid = req.seam_id
    if sampled.get('seam_id') != sid or sampled.get('joint_edge_id') != joint['joint_edge_id']:
        raise SeamDefinitionError(f'{sid}: sampler returned {sampled.get("seam_id")} / '
                                  f'{sampled.get("joint_edge_id")}, expected {joint["joint_edge_id"]}')
    for key in ('solid_a', 'solid_b', 'edge_a', 'edge_b'):
        if sampled[key] != joint[key]:
            raise SeamDefinitionError(f'{sid}: sampler {key} {sampled[key]} differs from topology.json '
                                      f'{joint[key]}: re-export D20 from the same model')
    if abs(float(sampled['length_mm']) - float(joint['length_mm'])) > LENGTH_TOL_MM:
        raise SeamDefinitionError(f'{sid}: sampler length {sampled["length_mm"]:.4f} mm differs from '
                                  f'topology.json {joint["length_mm"]:.4f} mm')
    prints = sorted(topo.edges[joint[k]]['fingerprint'] for k in ('edge_a', 'edge_b'))
    points = [{'s_mm': float(p['s_mm']), 'p_mm': [float(v) for v in p['p_mm']],
               't': [float(v) for v in p['t']], 'n1': [float(v) for v in p['n1']],
               'n2': [float(v) for v in p['n2']]} for p in sampled['points']]
    entry = {
        'seam_id': sid,
        'edges': [joint['edge_a'], joint['edge_b']],
        'joint_edge_id': joint['joint_edge_id'],
        'solids': [joint['solid_a'], joint['solid_b']],
        'faces': {'n1': list(sampled['faces_n1']), 'n2': list(sampled['faces_n2'])},
        'reversed': bool(req.reversed),
        'range_mm': None,
        'closed': bool(sampled['closed']),
        'length_mm': float(sampled['length_mm']),
        'joint_type': req.joint_type,
        'sequence': req.sequence,
        'leg_mm': req.leg_mm,
        'arcs': req.arcs,
        'arc_overlap_mm': float(req.arc_overlap_mm),
        'positioner_mode': req.positioner_mode,
        'positioner_angles_deg': (None if req.positioner_angles_deg is None
                                  else [float(a) for a in req.positioner_angles_deg]),
        'start_offset_mm': float(sampled['start_offset_mm']),
        'fingerprint': sha256_text('|'.join(prints)),
        'geometry': {'spacing_mm': float(sampled['spacing_mm']), 'points': points},
    }
    if req.note:
        entry['note'] = req.note
    return entry


def _part_ids(job: JobPaths) -> Dict[str, str]:
    """part_no / part_rev of ``part.json`` (v1 or v2 record; only these two keys are read)."""
    doc = json.loads(job.part_json.read_text(encoding='utf-8'))
    try:
        return {'part_no': str(doc['part_no']), 'part_rev': str(doc['part_rev'])}
    except KeyError as exc:
        raise SeamDefinitionError(f'{job.part_json}: no {exc}') from exc


ROLES_FILE = cad_roles.ROLES_FILE


def load_roles(job: JobPaths) -> Optional[List[Dict]]:
    """Assignments of ``roles.json`` (CP-60 job import), None for a job without roles (G1 flow unchanged)."""
    path = job.dir / ROLES_FILE
    if not path.is_file():
        return None
    return jsonio.read_json(path, 'v2/cad_roles', base_dir=job.dir)['assignments']


def check_part_only(topo: Topology, req: JointSeamRequest, joint: Dict, assignments: List[Dict]) -> None:
    """CP-60 check 5: picked edges and both joint solids must belong to ``part`` components."""
    paths = {s['solid_id']: s.get('component_path') for s in topo.doc['solids']}

    def role(solid_id: str) -> Tuple[str, str]:
        path = paths.get(solid_id)
        if path is None:
            raise SeamDefinitionError(f'{req.seam_id}: solid {solid_id} has no component path in topology.json '
                                      '(re-import the job with components)')
        return cad_roles.role_of_path(path, assignments), path

    for eid in req.edges:
        r, path = role(topo.edges[eid]['solid_id'])
        if r != 'part':
            raise SeamDefinitionError(f'{req.seam_id}: edge {eid} is on {r} component {path}; only part edges can '
                                      'be welded')
    ra, pa = role(joint['solid_a'])
    rb, pb = role(joint['solid_b'])
    if ra != 'part' or rb != 'part':
        raise SeamDefinitionError(f'{req.seam_id}: joint {joint["joint_edge_id"]} joins {pa} ({ra}) and {pb} ({rb}); '
                                  'only joints between two part solids can be welded')


def build_seams(job: JobPaths, requests: Sequence[JointSeamRequest], pitch_mm: float,
                sampler: Sampler, created_by: str = CREATED_BY) -> Dict:
    """A21 + A22 + A24: resolve, validate, sample and write ``seams.json`` (full list, replaces the file)."""
    if not 0.0 < pitch_mm <= MAX_PITCH_MM:
        raise SeamDefinitionError(f'pitch {pitch_mm} mm outside (0, {MAX_PITCH_MM}] mm')
    topo = Topology.load(job)
    joints = [topo.resolve_joint(r.edges) for r in requests]
    roles = load_roles(job)
    if roles is not None:
        for req, joint in zip(requests, joints):
            check_part_only(topo, req, joint, roles)
    for req, joint in zip(requests, joints):
        check_request(req, joint)
    check_seam_list(requests, joints)
    seams: List[Dict] = []
    if requests:
        out = sampler(job, sampler_request(requests, joints, pitch_mm))
        by_id = {s.get('seam_id'): s for s in out.get('seams', [])}
        for req, joint in zip(requests, joints):
            if req.seam_id not in by_id:
                raise SeamDefinitionError(f'{req.seam_id}: no samples from the sampler')
            seams.append(seam_entry(req, joint, topo, by_id[req.seam_id]))
    doc = jsonio.make_header(created_by, [job.part_json, topology_path(job)], job.dir)
    doc['schema_version'] = SCHEMA_VERSION
    doc.update(_part_ids(job))
    doc['seams'] = sorted(seams, key=lambda s: s['sequence'])
    jsonio.write_json(job.seams_json, doc, SEAMS_SCHEMA)
    return doc


def load_seams(job: JobPaths, check: bool = True) -> Dict:
    """Read ``seams.json`` (v2); refuse it when ``part.json`` or ``topology.json`` changed since."""
    doc = jsonio.read_json(job.seams_json, SEAMS_SCHEMA, base_dir=job.dir, check=check)
    if not str(doc['schema_version']).startswith('2.'):
        raise SeamDefinitionError(f'{job.seams_json}: schema_version {doc["schema_version"]} is not 2.x')
    return doc


def requests_from_seams(doc: Dict) -> List[JointSeamRequest]:
    """The requests that rebuild an existing seam list (e.g. after a re-export of the part)."""
    out = []
    for s in doc['seams']:
        out.append(JointSeamRequest(
            seam_id=s['seam_id'], edges=list(s['edges']), joint_type=s['joint_type'], sequence=s['sequence'],
            reversed=s['reversed'], start_offset_mm=s.get('start_offset_mm', 0.0), arcs=s['arcs'],
            arc_overlap_mm=s['arc_overlap_mm'], positioner_mode=s.get('positioner_mode', 'sync'),
            positioner_angles_deg=s.get('positioner_angles_deg'), leg_mm=s.get('leg_mm'),
            note=s.get('note', '')))
    return out


