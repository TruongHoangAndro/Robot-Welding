"""Head geometry from the robot description in use (S.5c Task 0; CP-56 decision 2 and its "one source" gap).

The IK / kinematics TCP offset ``tool0_T_tcp`` and the F9 nozzle (radius, contact standoff s_c) are read from the
URDF that the planning graph uses (``/robot_description`` of ``move_group``), never from constants:

* ``tool0_T_tcp_mm`` = composition of the **fixed** joints on the chain ``tool0`` → ``tcp_laser``;
* ``nozzle_radius_mm`` / ``contact_standoff_mm`` = the collision cylinder of the ``nozzle_tip`` link (H-BOX:
  ``nozzle_dia_mm`` / ``contact_standoff_mm`` args of ``lightweld_hbox.xacro``): its axis must be the beam axis
  (tool Z of ``tcp_laser``) and its end nearest to ``tcp_laser`` lies s_c before it (on the nozzle side, tool −Z).

The same link names are used by every head model (H-BOX now, H-CAD in S.5c Task 2), so this reader serves both.

G1.13 (cell-sim G1.8): the conduit source ``<awm_conduit source="explicit|rear_estimate|parametric"/>`` written by
``head_conduit.xacro`` is read (exactly one, known value; no default) and recorded; ``parametric`` and
``rear_estimate`` are Assumptions (``CONDUIT_SOURCES``). H-CAD with head.yaml ``nozzle_len_mm`` 0 renders
``nozzle_body`` / ``nozzle_tip`` as empty links: the head is read with ``nozzle_geometry = 'none'`` (tool0_T_tcp
still valid for kinematics) and :func:`contact_params_for` **refuses** F9 (:class:`NoNozzleError`). Decision: no
fallback to the head mesh, because a disc taken from the mesh is a geometry the description does not state
(no silent fallback, CP-56 one source).
Pure Python (xml.etree + numpy), mm; URDF metres converted only through ``awm_core.units`` (ADR-14). No rclpy (B-06).
"""

from __future__ import annotations

import dataclasses
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from awm_core import units
from awm_core.frames import matrix_to_quat, rotation_angle_between, rpy_to_matrix
from awm_core.hashing import sha256_text
from awm_path.core.contact import ContactParams, min_rim_samples

#: Conduit sources of ``<awm_conduit source>`` (head_conduit.xacro) -> True when the position is an Assumption.
CONDUIT_SOURCES = {'explicit': False, 'rear_estimate': True, 'parametric': True}
#: Text per source for run records.
CONDUIT_SOURCE_TEXT = {
    'explicit': 'head.yaml cable exit (explicit keys)',
    'rear_estimate': 'refused automatic beam estimate at the cable connector (head.yaml geometry_estimate)',
    'parametric': 'xacro conduit_x/y/z, conduit_r/p/y args (H-BOX defaults)',
}


class NoNozzleError(ValueError):
    """F9 needs the parametric nozzle_tip cylinder, and the description has none."""


#: Tolerances of the nozzle-on-beam-axis check and of the FK cross-check (numerical only).
AXIS_POS_TOL_MM = 1e-6
AXIS_ANGLE_TOL_RAD = 1e-9
FK_POS_TOL_MM = 1e-6
FK_ROT_TOL_RAD = 1e-9


@dataclass(frozen=True)
class HeadGeometry:
    """Head values read from one robot description (mm).

    ``nozzle_geometry`` is ``'cylinder'`` (nozzle_tip cylinder: radius, s_c, tip length set) or ``'none'`` (H-CAD
    with head.yaml ``nozzle_len_mm`` 0: empty nozzle links, the three nozzle values are ``None``).
    ``conduit_source`` is the ``<awm_conduit source>`` of the description.
    """

    tool0_T_tcp_mm: np.ndarray
    nozzle_radius_mm: Optional[float]
    contact_standoff_mm: Optional[float]
    nozzle_tip_length_mm: Optional[float]
    robot_description_sha256: str
    tool_link: str
    tcp_link: str
    nozzle_link: str
    nozzle_geometry: str
    conduit_source: str

    def as_record(self) -> dict:
        """Plain dict for run records."""
        return {'source': 'robot_description', 'robot_description_sha256': self.robot_description_sha256,
                'chain': [self.tool_link, self.tcp_link], 'nozzle_link': self.nozzle_link,
                'tool0_T_tcp_mm': {'position_mm': [float(v) for v in self.tool0_T_tcp_mm[:3, 3]],
                                   'quat_xyzw': [float(v) for v in matrix_to_quat(self.tool0_T_tcp_mm[:3, :3])]},
                'nozzle_geometry': self.nozzle_geometry,
                'nozzle_radius_mm': self.nozzle_radius_mm, 'contact_standoff_mm': self.contact_standoff_mm,
                'nozzle_tip_length_mm': self.nozzle_tip_length_mm,
                'conduit_source': self.conduit_source,
                'conduit_source_assumption': CONDUIT_SOURCES[self.conduit_source]}


def _floats(text: Optional[str], n: int = 3) -> np.ndarray:
    v = np.zeros(n) if text is None else np.array([float(x) for x in text.split()], dtype=float)
    if v.shape != (n,):
        raise ValueError(f'expected {n} numbers, got {text!r}')
    return v


def _origin_mm(elem: Optional[ET.Element]) -> np.ndarray:
    """URDF ``<origin xyz rpy>`` (m, rad) -> 4x4 transform in mm."""
    t = np.eye(4)
    if elem is None:
        return t
    t[:3, :3] = rpy_to_matrix(*_floats(elem.get('rpy')))
    t[:3, 3] = units.m_to_mm(_floats(elem.get('xyz')))
    return t


def _chain_mm(joints: Dict[str, ET.Element], root: str, tip: str) -> np.ndarray:
    """root_T_tip (mm) through fixed joints only; ``joints`` = child link -> joint element."""
    chain = []
    link = tip
    while link != root:
        j = joints.get(link)
        if j is None:
            raise ValueError(f'no joint chain from {root!r} to {tip!r} (stops at {link!r})')
        if j.get('type') != 'fixed':
            raise ValueError(f'joint {j.get("name")!r} on the chain {root} -> {tip} is {j.get("type")!r}, '
                             'the head chain must be fixed')
        chain.append(j)
        link = j.find('parent').get('link')
    t = np.eye(4)
    for j in reversed(chain):
        t = t @ _origin_mm(j.find('origin'))
    return t


def conduit_source_of_urdf(root: ET.Element) -> str:
    """The single ``<awm_conduit source>`` of the description (G1.8 review M6). Missing, duplicated or unknown ->
    ``ValueError`` (no default source: every record must state where the conduit came from)."""
    tags = root.findall('awm_conduit')
    if len(tags) != 1:
        raise ValueError(f'the robot description has {len(tags)} <awm_conduit source=...> records, expected 1 '
                         '(awm_description head_conduit.xacro, G1.8): cannot state the conduit source')
    src = tags[0].get('source')
    if src not in CONDUIT_SOURCES:
        raise ValueError(f'<awm_conduit source={src!r}> is not one of {sorted(CONDUIT_SOURCES)}')
    return src


def _is_empty(link: ET.Element) -> bool:
    return not link.findall('collision') and not link.findall('visual')


def head_from_urdf(urdf_xml: str, *, tool_link: str = 'tool0', tcp_link: str = 'tcp_laser',
                   nozzle_link: str = 'nozzle_tip', nozzle_body_link: str = 'nozzle_body') -> HeadGeometry:
    """Read :class:`HeadGeometry` from a URDF string. Raises ``ValueError`` if a link is missing, the chain is not
    fixed, the conduit source is missing / unknown, ``nozzle_link`` has neither exactly one cylinder collision nor
    is empty, that cylinder is not on the beam axis / not on the nozzle side of ``tcp_link``, or the nozzle is
    half-empty: one of ``nozzle_link`` / ``nozzle_body_link`` empty and the other with geometry (review 2.c). A
    description without a ``nozzle_body_link`` link is accepted (synthetic test URDFs; H-BOX and H-CAD always
    render both links).

    An empty ``nozzle_link`` (and empty ``nozzle_body_link`` if present) = no parametric nozzle (H-CAD with
    head.yaml ``nozzle_len_mm`` 0): ``nozzle_geometry = 'none'``, nozzle values ``None``."""
    root = ET.fromstring(urdf_xml)
    links = {l.get('name'): l for l in root.findall('link')}
    for name in (tool_link, tcp_link, nozzle_link):
        if name not in links:
            raise ValueError(f'link {name!r} not in the robot description')
    conduit = conduit_source_of_urdf(root)
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    tool0_T_tcp = _chain_mm(joints, tool_link, tcp_link)
    tool0_T_nozzle = _chain_mm(joints, tool_link, nozzle_link)
    sha = sha256_text(urdf_xml)
    body = links.get(nozzle_body_link)
    if _is_empty(links[nozzle_link]):
        if body is not None and not _is_empty(body):
            raise ValueError(f'{nozzle_link!r} is empty but {nozzle_body_link!r} has geometry: half-empty nozzle '
                             '(expected both empty for head.yaml nozzle_len_mm 0, or a nozzle_tip cylinder)')
        return HeadGeometry(tool0_T_tcp, None, None, None, sha, tool_link, tcp_link, nozzle_link, 'none', conduit)
    if body is not None and _is_empty(body):
        raise ValueError(f'{nozzle_link!r} has geometry but {nozzle_body_link!r} is empty: half-empty nozzle '
                         '(expected both empty for head.yaml nozzle_len_mm 0, or both with geometry)')
    cyl = [c for c in links[nozzle_link].findall('collision') if c.find('geometry/cylinder') is not None]
    if len(cyl) != 1 or len(links[nozzle_link].findall('collision')) != 1:
        raise ValueError(f'{nozzle_link!r} must have exactly one collision geometry, a cylinder (or none at all '
                         'for head.yaml nozzle_len_mm 0)')
    c = cyl[0].find('geometry/cylinder')
    radius = float(units.m_to_mm(float(c.get('radius'))))
    length = float(units.m_to_mm(float(c.get('length'))))
    tcp_T_cyl = np.linalg.inv(tool0_T_tcp) @ tool0_T_nozzle @ _origin_mm(cyl[0].find('origin'))
    axis, centre = tcp_T_cyl[:3, 2], tcp_T_cyl[:3, 3]
    if (np.linalg.norm(np.cross(axis, [0.0, 0.0, 1.0])) > AXIS_ANGLE_TOL_RAD
            or np.linalg.norm(centre[:2]) > AXIS_POS_TOL_MM):
        raise ValueError(f'{nozzle_link!r} cylinder is not on the beam axis (tool Z of {tcp_link!r}): axis {axis}, '
                         f'centre {centre} mm')
    near_end = centre[2] + length / 2.0             # end nearest to tcp_laser (nozzle side = −Z)
    if near_end > AXIS_POS_TOL_MM:
        raise ValueError(f'{nozzle_link!r} reaches beyond {tcp_link!r} ({near_end} mm along the beam)')
    return HeadGeometry(tool0_T_tcp, radius, float(-near_end), length, sha, tool_link, tcp_link, nozzle_link,
                        'cylinder', conduit)


def contact_params_for(head: HeadGeometry, base: ContactParams = ContactParams()) -> ContactParams:
    """F9 parameters with the nozzle (radius, s_c) of the description (CP-56 decision 2); other values from ``base``.
    ``rim_samples`` is raised (never lowered) to :func:`awm_path.core.contact.min_rim_samples` so that the rim spacing
    stays ≤ 2·s_c for the configured nozzle (review M1).

    Raises :class:`NoNozzleError` for a head without a parametric nozzle (``nozzle_geometry 'none'``): F9 has no
    disc, and none is taken from the head mesh (G1.13 decision, no silent fallback).
    """
    if head.nozzle_geometry != 'cylinder':
        raise NoNozzleError(
            f'F9 contact check refused: the robot description has no parametric nozzle ({head.nozzle_link!r} without '
            'geometry, H-CAD head.yaml nozzle_len_mm 0), so there is no nozzle disc (radius, s_c); F9 does not fall '
            'back to the head mesh. Use a head.yaml with nozzle_len_mm > 0 (parametric nozzle, CP-56) or H-BOX')
    rim = max(int(base.rim_samples),
              min_rim_samples(head.nozzle_radius_mm, head.contact_standoff_mm, int(base.ring_samples)))
    return dataclasses.replace(base, nozzle_radius_mm=head.nozzle_radius_mm, standoff_mm=head.contact_standoff_mm,
                               rim_samples=rim)


#: Head model labels of the runners / records → description head_model (sim_mock ``head_model`` arg).
HEAD_MODEL_LABELS = {'hbox': 'hbox', 'hcad': 'hcad'}


def normalise_head_model(label: str) -> str:
    """``'H-BOX'`` / ``'hbox'`` → ``'hbox'``, ``'H-CAD'`` / ``'hcad'`` → ``'hcad'``; anything else is refused."""
    key = str(label).strip().lower().replace('-', '').replace('_', '')
    if key not in HEAD_MODEL_LABELS:
        raise ValueError(f'unknown head model {label!r}: expected H-BOX (hbox) or H-CAD (hcad)')
    return HEAD_MODEL_LABELS[key]


def head_model_of_urdf(urdf_xml: str, body_link: str = 'head_body') -> str:
    """Head model a description was rendered with: ``'hcad'`` if the ``head_body`` collision is a mesh (CAD hull),
    ``'hbox'`` if it is a box (TST-002 §3.1 envelope). Raises ``ValueError`` otherwise."""
    root = ET.fromstring(urdf_xml)
    links = [l for l in root.findall('link') if l.get('name') == body_link]
    if len(links) != 1:
        raise ValueError(f'link {body_link!r} not (uniquely) in the robot description')
    kinds = {g.tag for c in links[0].findall('collision') for geo in c.findall('geometry') for g in geo}
    if kinds == {'mesh'}:
        return 'hcad'
    if kinds == {'box'}:
        return 'hbox'
    raise ValueError(f'{body_link!r} collision geometry {sorted(kinds)} is neither a box (H-BOX) nor a mesh (H-CAD)')


def check_head_model(urdf_xml: str, label: str) -> str:
    """Refuse a description whose head is not the one named by the run label (review I1: an ``H-CAD`` record must
    not be computed on H-BOX geometry). Returns the normalised model."""
    want = normalise_head_model(label)
    have = head_model_of_urdf(urdf_xml)
    if have != want:
        raise ValueError(f'run labelled {label!r} ({want}) but the robot description in use has the {have} head: pass '
                         f'head_model:={want} (and head_dir for hcad) to the launch')
    return want


def check_against_fk(head: HeadGeometry, tool0_T_tcp_fk_mm: np.ndarray) -> Tuple[float, float]:
    """Cross-check with ``tool0_T_tcp`` from the running graph (e.g. ``/compute_fk``). Returns (mm, rad) errors;
    raises ``ValueError`` beyond the numerical tolerances."""
    fk = np.asarray(tool0_T_tcp_fk_mm, dtype=float)
    dp = float(np.linalg.norm(fk[:3, 3] - head.tool0_T_tcp_mm[:3, 3]))
    dr = float(rotation_angle_between(fk[:3, :3], head.tool0_T_tcp_mm[:3, :3]))
    if dp > FK_POS_TOL_MM or dr > FK_ROT_TOL_RAD:
        raise ValueError(f'tool0_T_tcp of the description and of FK differ by {dp} mm / {dr} rad')
    return dp, dr


__all__ = ['HeadGeometry', 'NoNozzleError', 'CONDUIT_SOURCES', 'CONDUIT_SOURCE_TEXT', 'conduit_source_of_urdf',
           'head_from_urdf', 'contact_params_for', 'check_against_fk', 'HEAD_MODEL_LABELS',
           'normalise_head_model', 'head_model_of_urdf', 'check_head_model']
