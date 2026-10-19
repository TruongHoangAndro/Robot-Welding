"""Synchronous client for the move_group services used offline (DES-001 §7.1).

The node is spun by a background executor so service calls can be awaited from plain code.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, MoveItErrorCodes,
                             PlanningScene, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPositionFK,
                             GetPositionIK, GetStateValidity)
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle

from awm_core import frames


class MoveItServiceError(RuntimeError):
    """A move_group service is unavailable or timed out."""


@dataclass
class TrajectoryPoints:
    """Joint trajectory as arrays: positions [N,6], time_from_start [N] (s)."""

    positions: np.ndarray
    times: np.ndarray

    @classmethod
    def from_msg(cls, traj, joint_names: Sequence[str]) -> 'TrajectoryPoints':
        jt = traj.joint_trajectory
        order = [list(jt.joint_names).index(j) for j in joint_names]
        pos = np.array([[p.positions[i] for i in order] for p in jt.points])
        t = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in jt.points])
        return cls(pos, t)


def pose_msg(t: np.ndarray) -> Pose:
    p = frames.matrix_to_pose(t)
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = p['position_m']
    (msg.orientation.x, msg.orientation.y, msg.orientation.z,
     msg.orientation.w) = p['quat_xyzw']
    return msg


def pose_from_msg(msg: Pose) -> np.ndarray:
    return frames.pose_to_matrix({
        'position_m': [msg.position.x, msg.position.y, msg.position.z],
        'quat_xyzw': [msg.orientation.x, msg.orientation.y, msg.orientation.z,
                      msg.orientation.w]})


class MoveItClient:
    """IK / FK / Cartesian / joint planning / validity / scene against a running move_group."""

    def __init__(self, node, group: str, tcp_link: str, base_frame: str,
                 joint_names: Sequence[str], timeout_s: float = 30.0) -> None:
        self.node = node
        self.group = group
        self.tcp_link = tcp_link
        self.base_frame = base_frame
        self.joint_names = list(joint_names)
        self.timeout_s = timeout_s
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self._clients = {
            'ik': node.create_client(GetPositionIK, '/compute_ik'),
            'fk': node.create_client(GetPositionFK, '/compute_fk'),
            'cartesian': node.create_client(GetCartesianPath, '/compute_cartesian_path'),
            'plan': node.create_client(GetMotionPlan, '/plan_kinematic_path'),
            'valid': node.create_client(GetStateValidity, '/check_state_validity'),
            'scene': node.create_client(ApplyPlanningScene, '/apply_planning_scene'),
        }

    def close(self) -> None:
        self._executor.shutdown()
        self._thread.join(timeout=2.0)

    def wait_ready(self, timeout_s: float = 90.0) -> None:
        for name, client in self._clients.items():
            if not client.wait_for_service(timeout_sec=timeout_s):
                raise MoveItServiceError(f'move_group service {client.srv_name} not available '
                                         f'after {timeout_s:.0f} s (is sim.launch.py running?)')

    def _call(self, key: str, request):
        client = self._clients[key]
        if not client.wait_for_service(timeout_sec=self.timeout_s):
            raise MoveItServiceError(f'service {client.srv_name} not available')
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(self.timeout_s):
            client.remove_pending_request(future)
            raise MoveItServiceError(f'service {client.srv_name} timed out')
        if future.exception() is not None:
            raise MoveItServiceError(f'{client.srv_name}: {future.exception()}')
        return future.result()

    def robot_state(self, joints: Sequence[float]) -> RobotState:
        state = RobotState()
        state.joint_state = JointState(name=list(self.joint_names),
                                       position=[float(q) for q in joints])
        state.is_diff = False
        return state

    def compute_ik(self, pose: np.ndarray, seed: Sequence[float], timeout_s: float = 0.5,
                   avoid_collisions: bool = True,
                   group: Optional[str] = None) -> Optional[np.ndarray]:
        """IK of the TCP for a base-frame pose; None if no (collision-free) solution."""
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = group or self.group
        r.robot_state = self.robot_state(seed)
        r.avoid_collisions = avoid_collisions
        r.ik_link_name = self.tcp_link
        r.pose_stamped = PoseStamped(pose=pose_msg(pose))
        r.pose_stamped.header.frame_id = self.base_frame
        r.timeout.sec = int(timeout_s)
        r.timeout.nanosec = int((timeout_s - int(timeout_s)) * 1e9)
        res = self._call('ik', req)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        names = list(res.solution.joint_state.name)
        return np.array([res.solution.joint_state.position[names.index(j)]
                         for j in self.joint_names])

    def compute_fk(self, joints: Sequence[float], links: Sequence[str]) -> Dict[str, np.ndarray]:
        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.fk_link_names = list(links)
        req.robot_state = self.robot_state(joints)
        res = self._call('fk', req)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MoveItServiceError(f'FK failed (error {res.error_code.val})')
        return {name: pose_from_msg(ps.pose) for name, ps in zip(res.fk_link_names,
                                                                  res.pose_stamped)}

    def cartesian_path(self, start: Sequence[float], poses: Sequence[np.ndarray], max_step_m: float,
                       revolute_jump_threshold: float, velocity_scaling: float,
                       acceleration_scaling: float) -> Tuple[float, Optional[TrajectoryPoints]]:
        """Collision-checked Cartesian path of the TCP through base-frame poses.

        Returns (fraction, trajectory). The caller decides; nothing below 100 % is accepted
        anywhere in the chain (fixes demo finding F01).
        """
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.start_state = self.robot_state(start)
        req.group_name = self.group
        req.link_name = self.tcp_link
        req.waypoints = [pose_msg(p) for p in poses]
        req.max_step = float(max_step_m)
        req.jump_threshold = 0.0
        req.revolute_jump_threshold = float(revolute_jump_threshold)
        req.prismatic_jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = float(velocity_scaling)
        req.max_acceleration_scaling_factor = float(acceleration_scaling)
        res = self._call('cartesian', req)
        if res.error_code.val != MoveItErrorCodes.SUCCESS or not res.solution.joint_trajectory.points:
            return float(res.fraction), None
        return float(res.fraction), TrajectoryPoints.from_msg(res.solution, self.joint_names)

    def plan_joint(self, start: Sequence[float], goal: Sequence[float], planning_time_s: float,
                   attempts: int, velocity_scaling: float,
                   acceleration_scaling: float) -> Optional[TrajectoryPoints]:
        """Collision-free joint-space air move (OMPL), time-parameterised by move_group."""
        req = GetMotionPlan.Request()
        m = req.motion_plan_request
        m.group_name = self.group
        m.start_state = self.robot_state(start)
        m.num_planning_attempts = int(attempts)
        m.allowed_planning_time = float(planning_time_s)
        m.max_velocity_scaling_factor = float(velocity_scaling)
        m.max_acceleration_scaling_factor = float(acceleration_scaling)
        m.pipeline_id = 'ompl'
        c = Constraints()
        for name, q in zip(self.joint_names, goal):
            c.joint_constraints.append(JointConstraint(joint_name=name, position=float(q),
                                                       tolerance_above=1e-4,
                                                       tolerance_below=1e-4, weight=1.0))
        m.goal_constraints = [c]
        res = self._call('plan', req)
        mr = res.motion_plan_response
        if mr.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        return TrajectoryPoints.from_msg(mr.trajectory, self.joint_names)

    def state_valid(self, joints: Sequence[float]) -> Tuple[bool, List[str]]:
        req = GetStateValidity.Request()
        req.robot_state = self.robot_state(joints)
        req.group_name = self.group
        res = self._call('valid', req)
        contacts = sorted({f'{c.contact_body_1}<->{c.contact_body_2}' for c in res.contacts})
        return bool(res.valid), contacts

    def set_part(self, vertices: np.ndarray, triangles: np.ndarray, base_T_part: np.ndarray,
                 object_id: str = 'part') -> None:
        """Add / replace the part mesh as a collision object (planning-scene diff)."""
        mesh = Mesh()
        mesh.vertices = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in vertices]
        mesh.triangles = [MeshTriangle(vertex_indices=[int(i) for i in tri]) for tri in triangles]
        obj = CollisionObject()
        obj.id = object_id
        obj.header.frame_id = self.base_frame
        obj.pose = pose_msg(base_T_part)
        obj.meshes = [mesh]
        obj.mesh_poses = [pose_msg(np.eye(4))]
        obj.operation = CollisionObject.ADD
        self._apply_world([obj])

    def remove_object(self, object_id: str = 'part') -> None:
        obj = CollisionObject()
        obj.id = object_id
        obj.header.frame_id = self.base_frame
        obj.operation = CollisionObject.REMOVE
        self._apply_world([obj])

    def _apply_world(self, objects: List[CollisionObject]) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        res = self._call('scene', ApplyPlanningScene.Request(scene=scene))
        if not res.success:
            raise MoveItServiceError('apply_planning_scene failed')


def init_node(name: str, argv: Optional[List[str]] = None):
    """rclpy.init + node with automatically declared overrides disabled (we declare explicitly)."""
    if not rclpy.ok():
        rclpy.init(args=argv)
    return rclpy.create_node(name)
