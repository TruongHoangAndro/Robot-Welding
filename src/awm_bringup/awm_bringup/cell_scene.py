"""Cell scene of the cell model v2: static cell + positioner rotating set as world objects, link padding.

Why world objects and not URDF links: MoveIt 2.12 (Jazzy) pads robot links only against world objects
(``CollisionRequest.pad_environment_collisions``); self-collision checks run on the unpadded robot
(``pad_self_collisions`` = false, not settable through ``check_state_validity``). TST-002 F4 needs 3 mm padding
between the head and the Pismo frame, jig and holders, so these must be world objects.

Cell scene manifest (YAML or JSON, mm / unit quaternion, ADR-14). Draft format of this package, to be aligned with
the A3 ``export-scene`` manifest (Part A); extra keys are ignored::

    objects:
      - object_id: pismo_frame            # ^[a-z][a-z0-9_]*$
        role: cell_static                 # cell_static | positioner_rotating (optional, from parent_frame)
        parent_frame: world               # world | positioner_flange
        pose_mm: {position_mm: [x, y, z], quat_xyzw: [x, y, z, w]}
        box_mm: [sx, sy, sz]              # box centred at the pose, or
        mesh: {file: meshes/a.stl, sha256: <hex>}   # STL in mm, path relative to the manifest

Rules applied by :func:`build_scene`:

- ``world`` objects are fixed; ``positioner_flange`` objects are placed at the *current* positioner angle and moved
  when ``/joint_states`` reports a new ``positioner_joint`` value.
- Padding (F4): every robot link with collision geometry gets ``padding_mm`` except the ``unpadded_links``
  (default ``nozzle_tip``).
- Allowed collisions: only between ``world`` objects and robot links that are rigidly fixed to ``world`` (UR
  ``base_link_inertia``): their relative pose never changes (same rule as SRDF "Adjacent / Never"). No entry is
  ever made for a moving link, so none between the head and the part / holders / jig (B-04).

Pure functions are testable without ROS; :func:`main` is the ROS node ``cell_scene_loader``.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np
import yaml

from awm_core import units
from awm_core.frames import axis_angle_to_matrix, compose, make_transform, matrix_to_quat, quat_to_matrix
from awm_core.hashing import sha256_file
from awm_core.stl import read_stl

PARENT_FRAMES = ('world', 'positioner_flange')
ROLES = {'world': 'cell_static', 'positioner_flange': 'positioner_rotating'}
_ID = re.compile(r'^[a-z][a-z0-9_]*$')


class ManifestError(ValueError):
    """The cell scene manifest is malformed or a mesh does not match its hash."""


@dataclass
class SceneObject:
    """One cell object in mm. Exactly one of ``box_mm`` / (``vertices_mm``, ``triangles``) is set."""

    object_id: str
    role: str
    parent_frame: str
    parent_T_obj_mm: np.ndarray
    box_mm: Optional[np.ndarray] = None
    vertices_mm: Optional[np.ndarray] = None
    triangles: Optional[np.ndarray] = None
    mesh_sha256: Optional[str] = None
    extra: Dict = field(default_factory=dict)


def parse_vector(text: str, n: int, name: str) -> np.ndarray:
    """``"1 2 3"`` (launch / xacro argument format) -> float array of length n."""
    values = [float(v) for v in str(text).replace(',', ' ').split()]
    if len(values) != n:
        raise ValueError(f'{name}: expected {n} numbers, got {text!r}')
    return np.asarray(values, dtype=float)


def _pose_mm(entry: dict, where: str) -> np.ndarray:
    try:
        pos = parse_vector(' '.join(map(str, entry['position_mm'])), 3, where)
        quat = parse_vector(' '.join(map(str, entry['quat_xyzw'])), 4, where)
    except (KeyError, TypeError) as exc:
        raise ManifestError(f'{where}: pose_mm needs position_mm and quat_xyzw') from exc
    if abs(np.linalg.norm(quat) - 1.0) > 1e-6:
        raise ManifestError(f'{where}: quat_xyzw is not a unit quaternion')
    return make_transform(pos, quat_to_matrix(quat))


def load_manifest(path: Path) -> List[SceneObject]:
    """Read and check a cell scene manifest. Mesh hashes are verified when given."""
    path = Path(path)
    data = yaml.safe_load(path.read_text()) or {}
    entries = data.get('objects') or []
    if not isinstance(entries, list):
        raise ManifestError(f'{path}: objects must be a list')
    objects: List[SceneObject] = []
    seen: Set[str] = set()
    for i, e in enumerate(entries):
        where = f'{path.name} objects[{i}]'
        oid = e.get('object_id', '')
        if not _ID.match(str(oid)) or oid in seen:
            raise ManifestError(f'{where}: bad or duplicate object_id {oid!r}')
        seen.add(oid)
        parent = e.get('parent_frame')
        if parent not in PARENT_FRAMES:
            raise ManifestError(f'{where}: parent_frame must be one of {PARENT_FRAMES}')
        role = e.get('role', ROLES[parent])
        if role != ROLES[parent]:
            raise ManifestError(f'{where}: role {role!r} does not match parent_frame {parent!r}')
        obj = SceneObject(oid, role, parent, _pose_mm(e.get('pose_mm') or {}, where),
                          extra={k: v for k, v in e.items()
                                 if k not in ('object_id', 'role', 'parent_frame', 'pose_mm', 'box_mm', 'mesh')})
        if ('box_mm' in e) == ('mesh' in e):
            raise ManifestError(f'{where}: give exactly one of box_mm / mesh')
        if 'box_mm' in e:
            obj.box_mm = parse_vector(' '.join(map(str, e['box_mm'])), 3, where)
            if np.any(obj.box_mm <= 0):
                raise ManifestError(f'{where}: box_mm must be positive')
        else:
            mesh_file = path.parent / e['mesh']['file']
            digest = sha256_file(mesh_file)
            expected = e['mesh'].get('sha256')
            if expected and expected != digest:
                raise ManifestError(f'{where}: {mesh_file.name} sha256 {digest} != manifest {expected}')
            obj.vertices_mm, obj.triangles = read_stl(mesh_file)
            obj.mesh_sha256 = digest
        objects.append(obj)
    return objects


def world_fixed_links(urdf_xml: str, root: str = 'world') -> Set[str]:
    """Links joined to ``root`` by fixed joints only (their pose never changes)."""
    from urdf_parser_py.urdf import URDF
    robot = URDF.from_xml_string(urdf_xml)
    fixed = {root}
    changed = True
    while changed:
        changed = False
        for j in robot.joints:
            if j.type == 'fixed' and j.parent in fixed and j.child not in fixed:
                fixed.add(j.child)
                changed = True
    return fixed


def collision_links(urdf_xml: str) -> List[str]:
    """Robot links that have collision geometry (the links padding applies to)."""
    from urdf_parser_py.urdf import URDF
    robot = URDF.from_xml_string(urdf_xml)
    return sorted(link.name for link in robot.links if link.collisions)


def link_padding_m(links: Iterable[str], padding_mm: float, unpadded: Sequence[str]) -> Dict[str, float]:
    """Padding per link in m (MoveIt): ``padding_mm`` everywhere except the ``unpadded`` links (F4)."""
    if padding_mm < 0:
        raise ValueError('padding_mm must be >= 0')
    return {link: (0.0 if link in unpadded else units.mm_to_m(padding_mm)) for link in links}


def world_T_flange_mm(axis_xyz_mm: np.ndarray, axis_quat_xyzw: np.ndarray, axis_dir: np.ndarray,
                      theta_rad: float) -> np.ndarray:
    """world_T_positioner_flange (mm) at the positioner angle, same chain as positioner.xacro."""
    world_T_axis = make_transform(axis_xyz_mm, quat_to_matrix(axis_quat_xyzw))
    return compose(world_T_axis, make_transform(np.zeros(3), axis_angle_to_matrix(axis_dir, theta_rad)))


def world_pose_mm(obj: SceneObject, flange_mm: np.ndarray) -> np.ndarray:
    """world_T_object (mm) for the given world_T_flange."""
    return obj.parent_T_obj_mm if obj.parent_frame == 'world' else compose(flange_mm, obj.parent_T_obj_mm)


def allowed_pairs(objects: Iterable[SceneObject], fixed_links: Set[str], links_with_geometry: Iterable[str]
                  ) -> List[Tuple[str, str]]:
    """ACM entries: world objects x robot links fixed to world (never moving relative to each other)."""
    rigid = sorted(set(links_with_geometry) & fixed_links)
    return [(link, o.object_id) for o in objects if o.parent_frame == 'world' for link in rigid]


def merge_acm(names: List[str], values: List[List[bool]], pairs: Iterable[Tuple[str, str]]
              ) -> Tuple[List[str], List[List[bool]]]:
    """Return a new full ACM (names, matrix) with the pairs set to allowed; other entries unchanged."""
    names = list(names)
    for a, b in pairs:
        for n in (a, b):
            if n not in names:
                names.append(n)
    size = len(names)
    m = [[False] * size for _ in range(size)]
    for i, row in enumerate(values):
        for j, v in enumerate(row):
            m[i][j] = bool(v)
    for a, b in pairs:
        i, j = names.index(a), names.index(b)
        m[i][j] = m[j][i] = True
    return names, m


def scene_summary(manifest: Path, objects: List[SceneObject], padding: Dict[str, float]) -> str:
    """JSON status published by the loader (record for the spike: manifest hash, objects, padding in mm)."""
    return json.dumps({
        'manifest': str(manifest),
        'manifest_sha256': sha256_file(manifest),
        'objects': [{'object_id': o.object_id, 'role': o.role, 'parent_frame': o.parent_frame,
                     'mesh_sha256': o.mesh_sha256} for o in objects],
        'link_padding_mm': {k: units.m_to_mm(v) for k, v in sorted(padding.items())},
    }, sort_keys=True)


# --------------------------------------------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------------------------------------------

def _ros_pose(t_mm: np.ndarray):
    from geometry_msgs.msg import Pose
    p = Pose()
    x, y, z = (float(v) for v in units.mm_to_m(t_mm[:3, 3]))
    p.position.x, p.position.y, p.position.z = x, y, z
    q = matrix_to_quat(t_mm[:3, :3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    return p


def _collision_object(obj: SceneObject, t_mm: np.ndarray, operation: int):
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
    from geometry_msgs.msg import Point, Pose
    co = CollisionObject()
    co.header.frame_id = 'world'
    co.id = obj.object_id
    co.pose = _ros_pose(t_mm)
    co.operation = operation
    if operation == CollisionObject.MOVE:
        return co
    identity = Pose()
    identity.orientation.w = 1.0
    if obj.box_mm is not None:
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in units.mm_to_m(obj.box_mm)])
        co.primitives.append(prim)
        co.primitive_poses.append(identity)
    else:
        mesh = Mesh()
        mesh.vertices = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))
                         for v in units.mm_to_m(obj.vertices_mm)]
        mesh.triangles = [MeshTriangle(vertex_indices=[int(i) for i in tri]) for tri in obj.triangles]
        co.meshes.append(mesh)
        co.mesh_poses.append(identity)
    return co


def main(args=None) -> None:
    """ROS node: load the manifest into move_group's planning scene, set padding / ACM, follow the positioner."""
    import rclpy
    from moveit_msgs.msg import (AllowedCollisionEntry, CollisionObject, LinkPadding, PlanningScene,
                                 PlanningSceneComponents)
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    class CellSceneLoader(Node):
        def __init__(self) -> None:
            super().__init__('cell_scene_loader')
            self.declare_parameter('manifest', '')
            self.declare_parameter('robot_description', '')
            self.declare_parameter('padding_mm', 3.0)
            self.declare_parameter('unpadded_links', ['nozzle_tip'])
            self.declare_parameter('positioner_xyz_mm', '0 0 0')
            self.declare_parameter('positioner_quat_xyzw', '0 0 0 1')
            self.declare_parameter('positioner_axis_xyz', '1 0 0')
            self.declare_parameter('positioner_joint', 'positioner_joint')
            self.declare_parameter('theta_update_deg', 0.01)
            g = lambda n: self.get_parameter(n).value  # noqa: E731
            self.manifest = Path(g('manifest'))
            self.objects = load_manifest(self.manifest) if g('manifest') else []
            urdf = g('robot_description')
            if not urdf:
                raise RuntimeError('cell_scene_loader: robot_description parameter is empty')
            self.padding = link_padding_m(collision_links(urdf), float(g('padding_mm')), list(g('unpadded_links')))
            self.pairs = allowed_pairs(self.objects, world_fixed_links(urdf), self.padding.keys())
            self.axis_xyz = parse_vector(g('positioner_xyz_mm'), 3, 'positioner_xyz_mm')
            self.axis_q = parse_vector(g('positioner_quat_xyzw'), 4, 'positioner_quat_xyzw')
            self.axis_dir = parse_vector(g('positioner_axis_xyz'), 3, 'positioner_axis_xyz')
            self.joint = g('positioner_joint')
            self.update_rad = units.deg_to_rad(float(g('theta_update_deg')))
            self.theta: Optional[float] = None
            self.get_client = self.create_client(GetPlanningScene, '/get_planning_scene')
            self.apply_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.status_pub = self.create_publisher(String, '/awm/cell_scene/status', latched)
            self.loaded = False
            self.pending = None
            self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
            self.create_timer(0.5, self._try_load)

        def _flange(self, theta: float) -> np.ndarray:
            return world_T_flange_mm(self.axis_xyz, self.axis_q, self.axis_dir, theta)

        def _on_joint_states(self, msg: JointState) -> None:
            if self.joint in msg.name:
                theta = msg.position[msg.name.index(self.joint)]
                if not self.loaded:
                    self.theta = theta
                elif abs(theta - self.theta) >= self.update_rad and self.pending is None:
                    self._move_rotating(theta)

        def _try_load(self) -> None:
            if self.loaded or self.pending is not None or self.theta is None:
                return
            if not (self.get_client.service_is_ready() and self.apply_client.service_is_ready()):
                return
            req = GetPlanningScene.Request()
            req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            self.pending = self.get_client.call_async(req)
            self.pending.add_done_callback(self._apply_initial)

        def _apply_initial(self, future) -> None:
            acm = future.result().scene.allowed_collision_matrix
            names, matrix = merge_acm(list(acm.entry_names), [list(e.enabled) for e in acm.entry_values],
                                      self.pairs)
            scene = PlanningScene(is_diff=True)
            scene.allowed_collision_matrix.entry_names = names
            scene.allowed_collision_matrix.entry_values = [AllowedCollisionEntry(enabled=row) for row in matrix]
            scene.allowed_collision_matrix.default_entry_names = list(acm.default_entry_names)
            scene.allowed_collision_matrix.default_entry_values = list(acm.default_entry_values)
            scene.link_padding = [LinkPadding(link_name=k, padding=v) for k, v in sorted(self.padding.items())]
            flange = self._flange(self.theta)
            scene.world.collision_objects = [_collision_object(o, world_pose_mm(o, flange), CollisionObject.ADD)
                                             for o in self.objects]
            done = self.apply_client.call_async(ApplyPlanningScene.Request(scene=scene))
            done.add_done_callback(self._initial_applied)

        def _initial_applied(self, future) -> None:
            self.pending = None
            if not future.result().success:
                self.get_logger().error('apply_planning_scene refused the cell scene; retrying')
                return
            self.loaded = True
            summary = (scene_summary(self.manifest, self.objects, self.padding) if self.manifest.is_file()
                       else json.dumps({'objects': [], 'link_padding_mm': {
                           k: units.m_to_mm(v) for k, v in sorted(self.padding.items())}}))
            self.status_pub.publish(String(data=summary))
            self.get_logger().info(
                f'cell scene loaded: {len(self.objects)} objects from {self.manifest}, '
                f'{len(self.pairs)} allowed world-fixed pairs, padding {self.get_parameter("padding_mm").value} mm '
                f'except {list(self.get_parameter("unpadded_links").value)}')

        def _move_rotating(self, theta: float) -> None:
            rotating = [o for o in self.objects if o.parent_frame == 'positioner_flange']
            self.theta = theta
            if not rotating:
                return
            flange = self._flange(theta)
            scene = PlanningScene(is_diff=True)
            scene.world.collision_objects = [_collision_object(o, world_pose_mm(o, flange), CollisionObject.MOVE)
                                             for o in rotating]
            self.pending = self.apply_client.call_async(ApplyPlanningScene.Request(scene=scene))
            self.pending.add_done_callback(lambda _f: setattr(self, 'pending', None))

    rclpy.init(args=args)
    node = CellSceneLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
