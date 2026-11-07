"""Sim-1 bring-up of the cell model v2 (DES-001 §4.2 launch name ``sim_mock``; PLAN T0.4 subset for WP-S S.4).

    ros2 launch awm_bringup sim_mock.launch.py repo:=<repo root> [headless:=true]
        [trajectory_controllers:=split|robot_positioner] [head_model:=hbox]

    ros2 launch awm_bringup sim_mock.launch.py [headless:=true] [cell_manifest:=<file>]
        [base_xyz_mm:="x y z"] [base_quat_xyzw:="x y z w"]
        [positioner_xyz_mm:="x y z"] [positioner_quat_xyzw:="x y z w"] [positioner_axis_xyz:="x y z"]
        [padding_mm:=3.0] [<any H-BOX xacro arg>:=<mm or deg>]
        [head_model:=hbox|hcad] [head_dir:=<absolute folder with head.yaml, e.g. <repo_root>/cell/model/head>]
        [trajectory_controllers:=split|robot_positioner]

Repository cell model (G1.16, close-out A2): ``repo:=<root>`` loads, with nothing typed by hand, the poses of
``<root>/cell/model/frames.yaml`` (world_T_base, world_T_positioner_axis + direction), the scene
``<root>/cell/model/scene_manifest.json`` (same export as frames.yaml) and the head ``<root>/cell/model/head``
(head_model hcad when head.yaml is present; without it only an explicit ``head_model:=hbox`` is accepted). With
``repo:=`` the arguments base_*/positioner_*/cell_manifest/head_dir are refused; a missing or malformed file stops
the launch before any node starts (awm_bringup.cell_config.repo_launch_args). Without ``repo:=`` nothing changes.

Positioner limits: only awm_description/config/positioner_limits.yaml (URDF and MoveIt). The rotating set follows
positioner_joint; checkers use awm_bringup.scene_sync (set_positioner_and_wait, check_positioner).

UR5e + LightWELD head + positioner_joint on ros2_control mock hardware (one controller manager, two components:
joint_trajectory_controller for the arm, positioner_trajectory_controller for positioner_joint), MoveIt
move_group, and cell_scene_loader (cell manifest -> world objects, 3 mm padding on every link except nozzle_tip,
TST-002 F4). Mock hardware only: this launch never connects to a robot or a drive.

All pose / size arguments are mm, degrees and unit quaternions (ADR-14). Empty H-BOX args keep the xacro defaults
(TST-002 §3.1). Defaults of the poses are identity (Assumption until A3 frames.yaml, OI-23). The v1 MIG launch
``sim.launch.py`` is unchanged.

Trajectory controllers (G1.7): ``split`` (default) = joint_trajectory_controller (6 arm joints) +
positioner_trajectory_controller; ``robot_positioner`` = DES-001 §6.3 Option A1: one 7-joint
robot_positioner_trajectory_controller (arm joints then positioner_joint, one controller manager, mock hardware) used
by MoveIt and by the D09 replay CLI ``ros2 run awm_bringup replay_program``. Both claim the same command interfaces;
only one set is spawned.
In ``robot_positioner`` mode the split controllers are not running: the actions named by
``awm_bringup.scene_sync.CONTROLLER_ACTION`` (positioner_trajectory_controller) and by ``controller_action`` in
``config/awm_params.yaml`` (joint_trajectory_controller) do not exist. SceneSync fails at once with a message; use
``split`` for checkers / planners that move the positioner or the arm through those actions.

Head model (Goal 1 G1.1): ``head_model:=hbox`` (default, parametric envelope) or ``hcad`` (Fusion head CAD meshes and
tcp_laser from ``<head_dir>/head.yaml``, A18 export). For hcad the folder is checked first (head.yaml, mesh files and
their SHA-256: awm_bringup.cell_config.check_head_dir); nozzle_*, conduit_dia_mm and conduit_len_mm apply to both
models; for hcad the conduit capsule sits at the head.yaml cable exit and conduit_x/y/z, conduit_r/p/y are refused
(G1.8, review M5), unless head.yaml has no cable exit (parametric conduit from these args, Assumption, review I4).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from awm_bringup.cell_config import (REPO_CONFLICT_ARGS, check_conduit_args, check_head_dir, load_positioner_limits,
                                     positioner_moveit_limits, repo_launch_args)
from awm_core import units

POSE_ARGS = {
    'base_xyz_mm': ('0 0 0', 'world -> base_link position, mm (Assumption: identity)'),
    'base_quat_xyzw': ('0 0 0 1', 'world -> base_link rotation, unit quaternion x y z w'),
    'positioner_xyz_mm': ('0 0 0', 'world -> positioner_axis position, mm (Assumption: identity)'),
    'positioner_quat_xyzw': ('0 0 0 1', 'world -> positioner_axis rotation, unit quaternion x y z w'),
    'positioner_axis_xyz': ('1 0 0', 'positioner_joint axis in the positioner_axis frame'),
}
# xacro args of awm_cell_v2.urdf.xacro forwarded only when set (empty = xacro default).
XACRO_ARGS = (
    'mount_w_mm', 'mount_d_mm', 'mount_h_mm', 'head_w_mm', 'head_d_mm', 'head_h_mm',
    'head_axis_x_mm', 'head_axis_y_mm', 'nozzle_dia_mm', 'nozzle_len_mm', 'nozzle_tip_len_mm',
    'contact_standoff_mm', 'conduit_dia_mm', 'conduit_len_mm', 'conduit_x_mm', 'conduit_y_mm', 'conduit_z_mm',
    'conduit_r_deg', 'conduit_p_deg', 'conduit_y_deg',
)
# trajectory_controllers value -> (MoveIt controllers file, spawned trajectory controllers)
TRAJECTORY_CONTROLLERS = {
    'split': ('config/moveit_controllers_v2.yaml',
              ('joint_trajectory_controller', 'positioner_trajectory_controller')),
    'robot_positioner': ('config/moveit_controllers_v2_7axis.yaml', ('robot_positioner_trajectory_controller',)),
}


def _launch_setup(context, *args, **kwargs):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    headless = arg('headless').lower() in ('true', '1')
    repo = arg('repo').strip()
    values = {name: arg(name) for name in REPO_CONFLICT_ARGS}
    values['head_model'] = arg('head_model').strip()
    log = []
    if repo:
        # G1.16: everything from the repository cell model; conflicting args / missing files -> RepoArgError
        values = repo_launch_args(repo, values)
        log.append(LogInfo(msg=f'sim_mock repo:={repo}: frames, cell_manifest {values["cell_manifest"]}, '
                               f'head_model {values["head_model"]} {values["head_dir"]}'.rstrip()))
    else:
        values = {k: (v if str(v).strip() else POSE_ARGS[k][0]) if k in POSE_ARGS else v for k, v in values.items()}
        values['head_model'] = values['head_model'] or 'hbox'
    mappings = {name: values[name] for name in POSE_ARGS}
    mappings.update({name: arg(name) for name in XACRO_ARGS if arg(name).strip()})
    head_model = values['head_model']
    if head_model not in ('hbox', 'hcad'):
        raise ValueError(f'head_model must be hbox or hcad, got {head_model!r}')
    mappings['head_model'] = head_model
    if head_model == 'hcad':
        check_head_dir(values['head_dir'])  # clear error before xacro / MoveIt (missing or changed files)
        # conduit position args are ignored when head.yaml gives the cable exit (review M5): refuse them.
        check_conduit_args('hcad', values['head_dir'], [n for n in XACRO_ARGS if arg(n).strip()])
        mappings['head_dir'] = values['head_dir'].strip()
    padding_mm = float(arg('padding_mm'))
    traj = arg('trajectory_controllers').strip()
    if traj not in TRAJECTORY_CONTROLLERS:
        raise ValueError(f'trajectory_controllers must be one of {sorted(TRAJECTORY_CONTROLLERS)}, got {traj!r}')
    moveit_controllers_file, spawned = TRAJECTORY_CONTROLLERS[traj]
    manifest = values['cell_manifest'] or (FindPackageShare('awm_bringup').perform(context)
                                           + '/config/cell_scene_nominal.yaml')
    moveit_config = (
        MoveItConfigsBuilder('awm_cell_v2', package_name='awm_description')
        .robot_description(file_path='urdf/awm_cell_v2.urdf.xacro', mappings=mappings)
        .robot_description_semantic(file_path='srdf/awm_cell_v2.srdf.xacro')
        .robot_description_kinematics(file_path='config/kinematics_v2.yaml')
        .joint_limits(file_path='config/joint_limits_v2.yaml')
        .trajectory_execution(file_path=moveit_controllers_file)
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True,
                                publish_planning_scene=True,
                                publish_geometry_updates=True,
                                publish_state_updates=True,
                                publish_transforms_updates=True)
        .to_moveit_configs()
    )
    # positioner_joint limits: one source (awm_description/config/positioner_limits.yaml, also read by the xacro).
    moveit_config.joint_limits['robot_description_planning']['joint_limits']['positioner_joint'] = \
        positioner_moveit_limits(load_positioner_limits())
    controllers = FindPackageShare('awm_description').perform(context) + '/config/ros2_controllers_v2.yaml'
    # Padding on every robot link from the start; cell_scene_loader then sets nozzle_tip to 0 (F4).
    padding = {'robot_description_planning.default_robot_padding': units.mm_to_m(padding_mm)}

    nodes = log + [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='log',
             parameters=[moveit_config.robot_description]),
        Node(package='controller_manager', executable='ros2_control_node', output='log',
             parameters=[controllers],
             remappings=[('/controller_manager/robot_description', '/robot_description')]),
        Node(package='controller_manager', executable='spawner', output='log',
             arguments=['joint_state_broadcaster', *spawned, '--controller-manager', '/controller_manager']),
        Node(package='moveit_ros_move_group', executable='move_group', output='log',
             parameters=[moveit_config.to_dict(), padding, {'publish_robot_description_semantic': True}]),
        Node(package='awm_bringup', executable='cell_scene_loader', output='log',
             parameters=[{'manifest': manifest, 'padding_mm': padding_mm, 'unpadded_links': ['nozzle_tip']},
                         moveit_config.robot_description]),
    ]
    if not headless:
        rviz_config = FindPackageShare('ur_moveit_config').perform(context) + '/config/moveit.rviz'
        nodes.append(Node(package='rviz2', executable='rviz2', output='log', arguments=['-d', rviz_config],
                          parameters=[moveit_config.robot_description,
                                      moveit_config.robot_description_semantic,
                                      moveit_config.robot_description_kinematics,
                                      moveit_config.planning_pipelines,
                                      moveit_config.joint_limits]))
    return nodes


def generate_launch_description():
    decls = [
        DeclareLaunchArgument('repo', default_value='',
                              description='job repository root: poses from cell/model/frames.yaml, scene from '
                                          'cell/model/scene_manifest.json, head from cell/model/head (G1.16); '
                                          'refused together with base_*/positioner_*/cell_manifest/head_dir'),
        DeclareLaunchArgument('headless', default_value='false', description='true: no RViz (tests, CI)'),
        DeclareLaunchArgument('cell_manifest', default_value='',
                              description='cell scene manifest (mm); empty = config/cell_scene_nominal.yaml '
                                          '(Pismo box only, Assumption)'),
        DeclareLaunchArgument('head_model', default_value='',
                              description='hbox (parametric H-BOX, default) or hcad (Fusion head CAD, needs head_dir); '
                                          'with repo:= empty = hcad from cell/model/head'),
        DeclareLaunchArgument('head_dir', default_value='',
                              description='hcad only: absolute folder with head.yaml + STL meshes '
                                          '(A18 export, e.g. <repo_root>/cell/model/head)'),
        DeclareLaunchArgument('trajectory_controllers', default_value='split',
                              description='split (6-joint arm + positioner controllers, default) or robot_positioner '
                                          '(one 7-joint controller, DES-001 §6.3 Option A1, G1.7)'),
        DeclareLaunchArgument('padding_mm', default_value='3.0',
                              description='padding on every robot link except nozzle_tip (TST-002 F4, D16)'),
    ]
    # Empty default = not given: identity values of POSE_ARGS without repo:=, refused with repo:= (G1.16).
    decls += [DeclareLaunchArgument(n, default_value='', description=f'{t}; empty = {d!r} (refused with repo:=)')
              for n, (d, t) in POSE_ARGS.items()]
    decls += [DeclareLaunchArgument(n, default_value='', description='head xacro arg (H-BOX; nozzle_*, conduit_dia / len also H-CAD); empty = default')
              for n in XACRO_ARGS]
    return LaunchDescription(decls + [OpaqueFunction(function=_launch_setup)])
