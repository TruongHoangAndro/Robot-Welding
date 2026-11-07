"""Scene backends on MoveIt2 ``move_group`` ``/check_state_validity`` (WP-S S.2; cell scene v2: S.4 C1 / C2).

States are converted from degrees to radians only through ``awm_core.units`` (ADR-14). One service call per state,
sequentially (batching is PLAN T1.8, only if TST-002 C5 > 30 s).
"""

from __future__ import annotations

from typing import List, Sequence

import numpy as np
from geometry_msgs.msg import Point

from awm_core import units
from awm_path.core.scene import StateCheck, validate_states
from awm_path.moveit_client import MoveItClient, MoveItServiceError, pose_msg


class MoveGroupSceneBackend:
    """``SceneBackend`` backed by a running ``move_group`` (mock hardware / planning-only graph)."""

    name = 'move_group/check_state_validity'

    def __init__(self, client: MoveItClient) -> None:
        self._client = client

    def check(self, states_deg: Sequence[Sequence[float]]) -> List[StateCheck]:
        states = validate_states(states_deg, n_joints=len(self._client.joint_names))
        if states.shape[1] != 6:
            raise ValueError(f'MoveGroupSceneBackend expects 6 robot joints, client has {states.shape[1]}')
        out: List[StateCheck] = []
        for q_deg in states:
            valid, contacts = self._client.state_valid(units.deg_to_rad(q_deg))
            out.append(StateCheck(valid=bool(valid), contacts=tuple(sorted(contacts))))
        return out


# ----------------------------------------------------------------------------------------------------------------
# Cell scene v2 (WP-S S.4 C1 / C2): cell manifest (cell_scene_loader) + job objects of D19, positioner angle,
# validity with F4 padding, clearance F5 by inflated links.
# ----------------------------------------------------------------------------------------------------------------

ARM_JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
              'wrist_3_joint')


class MoveGroupCellScene:
    """Cell scene backend on ``move_group`` of ``sim_mock.launch.py`` (cell model v2; mock hardware only).

    Scene ownership:

    * cell objects (static + positioner rotating set) — ``cell_scene_loader`` of ``awm_bringup`` from the cell
      manifest; this class only checks that the loaded manifest is the one it was given (SHA-256 of the file);
    * job objects (D19 ``placement.json`` v2: part with face ids, fixtures, clamps) — added here as world objects at
      ``world_T_positioner_flange(θ) · parent_T_object`` and moved with the positioner; ``world_T_positioner_flange``
      comes from ``/compute_fk`` (the URDF is the one source of the positioner pose).

    Positioner angle: :meth:`set_theta` moves the mock positioner through ``awm_bringup.scene_sync`` and waits until
    the cell scene applied it; every request is guarded (``check_positioner``), so a state is never reported valid
    against the rotating set at another angle.

    Validity (F4): ``check_state_validity`` with the padding the loader set (3 mm on every link except ``nozzle_tip``).
    Clearance (F5): :meth:`check_clearance` — a planning-scene padding diff sets the clearance links to exactly the
    clearance and every other link to 0, checks, and restores the F4 padding (``CLEARANCE_METHOD``).

    All public values mm / degrees; m / rad only through ``awm_core.units``. One request in flight at a time; the
    caller's node is spun only inside the calls (no background executor).
    """

    name = 'move_group/check_state_validity (cell scene v2)'

    def __init__(self, node, cell_manifest, placement, job_dir, *, group: str = 'laser_arm',
                 positioner_joint: str = 'positioner_joint',
                 clearance_links: Sequence[str] = None, head_model: str = 'H-BOX',
                 timeout_s: float = 30.0) -> None:
        from awm_bringup.scene_sync import SceneSync
        from awm_path.core.scene_files import DEFAULT_CLEARANCE_LINKS
        from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK, GetStateValidity
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String

        self.node = node
        self.cell_manifest = cell_manifest
        self.placement = placement
        self.job_dir = job_dir
        self.group = group
        self.positioner_joint = positioner_joint
        self.clearance_links = tuple(clearance_links or DEFAULT_CLEARANCE_LINKS)
        self.head_model = head_model
        self.timeout_s = float(timeout_s)
        self.sync = SceneSync(node)
        self._valid = node.create_client(GetStateValidity, '/check_state_validity')
        self._apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._fk = node.create_client(GetPositionFK, '/compute_fk')
        self._robot_description = None
        self._robot_description_semantic = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/robot_description', self._on_description, latched)
        node.create_subscription(String, '/robot_description_semantic', self._on_semantic, latched)
        self.padding_dirty = False
        self.job = None
        self.head = None
        self.record = None
        self.scene_hash = None
        self._f4 = None
        self._theta = None

    # -- plumbing ------------------------------------------------------------------------------------------------
    def _on_description(self, msg) -> None:
        self._robot_description = msg.data

    def _on_semantic(self, msg) -> None:
        self._robot_description_semantic = msg.data

    def _call(self, client, request):
        import rclpy
        if not client.wait_for_service(timeout_sec=self.timeout_s):
            raise MoveItServiceError(f'service {client.srv_name} not available')
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=self.timeout_s)
        if not fut.done():
            client.remove_pending_request(fut)
            raise MoveItServiceError(f'service {client.srv_name} timed out')
        if fut.exception() is not None:
            raise MoveItServiceError(f'{client.srv_name}: {fut.exception()}')
        return fut.result()

    def _robot_state(self, q_deg, theta_deg: float):
        from moveit_msgs.msg import RobotState
        rs = RobotState()
        rs.joint_state.name = list(ARM_JOINTS) + [self.positioner_joint]
        rs.joint_state.position = [float(v) for v in units.deg_to_rad(np.asarray(q_deg, dtype=float))] + [
            float(units.deg_to_rad(float(theta_deg)))]
        return rs

    def _apply_scene(self, scene) -> None:
        from moveit_msgs.srv import ApplyPlanningScene
        if not self._call(self._apply, ApplyPlanningScene.Request(scene=scene)).success:
            raise MoveItServiceError('apply_planning_scene refused the diff')

    # -- frames --------------------------------------------------------------------------------------------------
    def world_T_links(self, q_deg: Sequence[float], theta_deg: float, links: Sequence[str]):
        """``world_T_link`` (4x4, mm) of the given links for a robot state and positioner angle (``/compute_fk``)."""
        from moveit_msgs.msg import MoveItErrorCodes
        from moveit_msgs.srv import GetPositionFK
        from awm_core.frames import make_transform, quat_to_matrix
        req = GetPositionFK.Request()
        req.header.frame_id = 'world'
        req.fk_link_names = list(links)
        req.robot_state = self._robot_state(q_deg, theta_deg)
        res = self._call(self._fk, req)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MoveItServiceError(f'compute_fk failed (error {res.error_code.val})')
        out = {}
        for name, ps in zip(res.fk_link_names, res.pose_stamped):
            p, o = ps.pose.position, ps.pose.orientation
            out[name] = make_transform(units.m_to_mm(np.array([p.x, p.y, p.z])), quat_to_matrix([o.x, o.y, o.z, o.w]))
        return out

    @property
    def robot_description(self) -> str:
        """URDF of the running graph (``/robot_description``, latched); available after :meth:`load`."""
        if self._robot_description is None:
            raise MoveItServiceError('/robot_description not received yet (call load() first)')
        return self._robot_description

    def head_geometry(self):
        """Head values of the description in use (S.5c Task 0, CP-56 one source): ``tool0_T_tcp`` from the URDF
        chain tool0 -> tcp_laser, nozzle radius and s_c from the ``nozzle_tip`` cylinder; cross-checked with
        ``/compute_fk`` (raises ``ValueError`` on a mismatch)."""
        from awm_path.core.head import check_against_fk, head_from_urdf
        head = head_from_urdf(self.robot_description)
        w = self.world_T_links([0.0] * 6, 0.0, [head.tool_link, head.tcp_link])
        check_against_fk(head, np.linalg.inv(w[head.tool_link]) @ w[head.tcp_link])
        return head

    def _world_T_flange(self, theta_deg: float) -> np.ndarray:
        return self.world_T_links([0.0] * 6, theta_deg, ['positioner_flange'])['positioner_flange']

    # -- scene ---------------------------------------------------------------------------------------------------
    def load(self, timeout_s: float = 120.0) -> dict:
        """Wait for the cell scene, check its manifest, add the job objects at the current angle, build the record.

        Raises ``SceneFileError`` if the loaded cell manifest is not ``cell_manifest`` or a job file is stale.
        """
        import rclpy
        import time
        from awm_path.core.scene_files import (SceneFileError, load_cell_manifest, load_job_scene, scene_content,
                                               scene_hash)
        cell = load_cell_manifest(self.cell_manifest)
        job = load_job_scene(self.placement, self.job_dir)
        status = self.sync.wait_loaded(timeout_s)
        if status.get('manifest_sha256') != cell.file_sha256:
            raise SceneFileError(f'cell scene loaded from manifest {status.get("manifest")} '
                                 f'(sha256 {status.get("manifest_sha256")}), expected {cell.path} ({cell.file_sha256})')
        deadline = time.monotonic() + timeout_s
        while self._robot_description is None or self._robot_description_semantic is None:
            if time.monotonic() > deadline:
                raise MoveItServiceError('/robot_description or /robot_description_semantic not received')
            rclpy.spin_once(self.node, timeout_sec=0.05)
        from awm_path.core.head import check_head_model
        check_head_model(self._robot_description, self.head_model)   # review I1: label = description in use
        for c in (self._valid, self._apply, self._fk):
            if not c.wait_for_service(timeout_sec=timeout_s):
                raise MoveItServiceError(f'service {c.srv_name} not available')
        self.job = job
        self._f4 = {k: float(v) for k, v in sorted(status['link_padding_mm'].items())}
        self._theta = float(status['theta_deg'])
        self._place_job_objects(self._theta, add=True)
        content = scene_content(backend=self.name, group=self.group, head_model=self.head_model,
                                robot_description=self._robot_description,
                                robot_description_semantic=self._robot_description_semantic, cell=cell, job=job,
                                f4_padding_mm=self._f4, clearance_links=self.clearance_links)
        self.scene_hash = scene_hash(content)
        self.head = self.head_geometry()
        self.record = {
            'scene_hash': self.scene_hash,
            'content': content,
            'head': self.head.as_record(),
            'files': {'cell': {'file': str(cell.path), 'file_sha256': cell.file_sha256},
                      'placement': {'file': str(job.path), 'file_sha256': job.file_sha256}},
        }
        return self.record

    def _place_job_objects(self, theta_deg: float, add: bool) -> None:
        from moveit_msgs.msg import CollisionObject, PlanningScene
        from shape_msgs.msg import Mesh, MeshTriangle
        from awm_path.core.scene_files import world_T_object
        flange = self._world_T_flange(theta_deg)
        objs = []
        for o in self.job.objects:
            co = CollisionObject()
            co.id = o.object_id
            co.header.frame_id = 'world'
            co.pose = pose_msg(units.transform_mm_to_m(world_T_object(o, flange)))
            if add:
                mesh = Mesh()
                mesh.vertices = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in units.mm_to_m(o.vertices_mm)]
                mesh.triangles = [MeshTriangle(vertex_indices=[int(i) for i in t]) for t in o.triangles]
                co.meshes = [mesh]
                co.mesh_poses = [pose_msg(np.eye(4))]
                co.operation = CollisionObject.ADD
            else:
                co.operation = CollisionObject.MOVE
            objs.append(co)
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = objs
        self._apply_scene(scene)

    @property
    def theta_deg(self) -> float:
        """Positioner angle applied to the cell scene and the job objects (deg)."""
        return self._theta

    def set_theta(self, theta_deg: float) -> None:
        """Move the (mock) positioner and the job objects to ``theta_deg`` and wait until the scene applied it."""
        self._require_loaded()
        self.sync.set_positioner_and_wait(float(theta_deg), timeout_s=max(60.0, self.timeout_s))
        self._place_job_objects(float(theta_deg), add=False)
        self._theta = float(theta_deg)

    def _require_loaded(self) -> None:
        if self.record is None:
            raise RuntimeError('MoveGroupCellScene.load() has not been called')

    def _ensure_theta(self, theta_deg: float) -> None:
        from awm_bringup.scene_sync import TOL_DEG
        if abs(float(theta_deg) - self._theta) > TOL_DEG / 10:
            self.set_theta(theta_deg)

    def _check(self, states: np.ndarray, theta_deg: float) -> List[StateCheck]:
        from moveit_msgs.srv import GetStateValidity
        out: List[StateCheck] = []
        for q in states:
            req = GetStateValidity.Request(robot_state=self._robot_state(q, theta_deg), group_name=self.group)
            self.sync.check(req.robot_state)      # positioner angle must equal the applied one
            res = self._call(self._valid, req)
            contacts = sorted({f'{c.contact_body_1}<->{c.contact_body_2}' for c in res.contacts})
            out.append(StateCheck(valid=bool(res.valid), contacts=tuple(contacts)))
        return out

    def _ensure_f4_padding(self) -> None:
        """Re-apply the F4 padding if an earlier restore failed; raise (refuse the check) if that fails too."""
        if self.padding_dirty:
            self._set_padding(self._f4)
            self.padding_dirty = False

    def check(self, states_deg: Sequence[Sequence[float]], theta_deg: float) -> List[StateCheck]:
        """F4 validity of robot states (6 joints, deg) at the positioner angle ``theta_deg``.

        Refused (raises) while the F4 padding is not known to be in place (failed restore, see check_clearance)."""
        self._require_loaded()
        self._ensure_f4_padding()
        states = validate_states(states_deg, n_joints=6)
        self._ensure_theta(theta_deg)
        return self._check(states, theta_deg)

    def f4_padding_mm(self) -> dict:
        """Link padding of the loaded planning scene (mm), read back from move_group."""
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene
        client = self.node.create_client(GetPlanningScene, '/get_planning_scene')
        try:
            comp = PlanningSceneComponents(components=PlanningSceneComponents.LINK_PADDING_AND_SCALING)
            ps = self._call(client, GetPlanningScene.Request(components=comp)).scene
        finally:
            self.node.destroy_client(client)
        pad = {p.link_name: float(units.m_to_mm(p.padding)) for p in ps.link_padding}
        return {k: round(pad[k], 9) for k in sorted(self._f4)}

    def _set_padding(self, padding_mm: dict) -> None:
        from moveit_msgs.msg import LinkPadding, PlanningScene
        scene = PlanningScene(is_diff=True)
        scene.link_padding = [LinkPadding(link_name=k, padding=float(units.mm_to_m(v)))
                              for k, v in sorted(padding_mm.items())]
        self._apply_scene(scene)

    def check_clearance(self, states_deg: Sequence[Sequence[float]], theta_deg: float,
                        clearance_mm: float) -> List[StateCheck]:
        """F5: valid = no clearance link within ``clearance_mm`` of any world object (unpadded geometry otherwise).

        The F4 padding is restored afterwards, also on error. If the restore fails, ``padding_dirty`` stays set and
        every later check first re-applies the F4 padding (or raises).
        """
        from awm_path.core.scene_files import clearance_padding_mm
        self._require_loaded()
        self._ensure_f4_padding()
        states = validate_states(states_deg, n_joints=6)
        self._ensure_theta(theta_deg)
        pad = clearance_padding_mm(self._f4, self.clearance_links, clearance_mm)
        self.padding_dirty = True
        try:
            self._set_padding(pad)
            return self._check(states, theta_deg)
        finally:
            self._set_padding(self._f4)
            self.padding_dirty = False
