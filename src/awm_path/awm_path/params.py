"""Chain parameters (D16). Defaults here == awm_bringup/config/awm_params.yaml (checked by a test).

Nodes declare every field as a ROS parameter with this default; the YAML file overrides them.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import List

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
          'wrist_2_joint', 'wrist_3_joint']


@dataclass
class ChainParams:
    repo_root: str = ''
    occ_python_cmd: str = ''
    planning_group: str = 'weld_arm'
    ik_search_group: str = 'weld_arm_search'
    tcp_link: str = 'tcp_mig'
    base_frame: str = 'base_link'
    joint_names: List[str] = field(default_factory=lambda: list(JOINTS))
    home_joints_rad: List[float] = field(
        default_factory=lambda: [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
    default_process: str = 'MIG'
    allow_sample_wps: bool = True
    sample_spacing_m: float = 0.005
    approach_distance_m: float = 0.05
    tool_roll: str = 'travel'
    cartesian_max_step_m: float = 0.002
    revolute_jump_threshold_rad: float = 0.35
    cartesian_retries: int = 1
    ik_timeout_s: float = 0.5
    ik_seed_count: int = 24
    min_wrist_singularity_margin_rad: float = 0.15
    service_timeout_s: float = 30.0
    velocity_scaling: float = 0.2
    acceleration_scaling: float = 0.2
    weld_accel_m_s2: float = 0.5
    max_joint_velocity_rad_s: List[float] = field(default_factory=lambda: [3.14159] * 6)
    max_joint_acceleration_rad_s2: List[float] = field(default_factory=lambda: [5.0] * 6)
    trajectory_dt_s: float = 0.008
    air_planning_time_s: float = 5.0
    air_planning_attempts: int = 5
    air_planning_retries: int = 3
    replay_on_mock_hardware: bool = True
    replay_speed_scale: float = 1.0
    max_tracking_error_rad: float = 0.02
    start_state_tolerance_rad: float = 0.001
    controller_action: str = '/joint_trajectory_controller/follow_joint_trajectory'


def declare(node) -> ChainParams:
    """Declare all chain parameters on an rclpy node and return their values."""
    defaults = ChainParams()
    values = {}
    for f in fields(ChainParams):
        default = getattr(defaults, f.name)
        values[f.name] = node.declare_parameter(f.name, default).value
    params = ChainParams(**values)
    params.home_joints_rad = [float(v) for v in params.home_joints_rad]
    params.max_joint_velocity_rad_s = [float(v) for v in params.max_joint_velocity_rad_s]
    params.max_joint_acceleration_rad_s2 = [float(v) for v in params.max_joint_acceleration_rad_s2]
    return params


def from_yaml(path, **overrides) -> ChainParams:
    """Load the D16 YAML without ROS (tests, offline tools)."""
    import yaml
    data = yaml.safe_load(open(path, encoding='utf-8'))['/**']['ros__parameters']
    data.update(overrides)
    known = {f.name for f in fields(ChainParams)}
    unknown = set(data) - known
    if unknown:
        raise KeyError(f'unknown parameters in {path}: {sorted(unknown)}')
    return ChainParams(**data)
