"""Sim-1 bring-up of the cell model v2 (DES-001 §4.2 launch name ``sim_mock``; PLAN T0.4 subset for WP-S S.4).

    ros2 launch awm_bringup sim_mock.launch.py [headless:=true] [cell_manifest:=<file>]
        [base_xyz_mm:="x y z"] [base_quat_xyzw:="x y z w"]
        [positioner_xyz_mm:="x y z"] [positioner_quat_xyzw:="x y z w"] [positioner_axis_xyz:="x y z"]
        [padding_mm:=3.0] [<any H-BOX xacro arg>:=<mm or deg>]

UR5e + H-BOX head + positioner_joint on ros2_control mock hardware (one controller manager, two components:
joint_trajectory_controller for the arm, positioner_trajectory_controller for positioner_joint), MoveIt
move_group, and cell_scene_loader (cell manifest -> world objects, 3 mm padding on every link except nozzle_tip,
TST-002 F4). Mock hardware only: this launch never connects to a robot or a drive.

All pose / size arguments are mm, degrees and unit quaternions (ADR-14). Empty H-BOX args keep the xacro defaults
(TST-002 §3.1). Defaults of the poses are identity (Assumption until A3 frames.yaml, OI-23). The v1 MIG launch
``sim.launch.py`` is unchanged.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

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
    'positioner_lower_deg', 'positioner_upper_deg', 'positioner_velocity_deg_s',
    'mount_w_mm', 'mount_d_mm', 'mount_h_mm', 'head_w_mm', 'head_d_mm', 'head_h_mm',
    'head_axis_x_mm', 'head_axis_y_mm', 'nozzle_dia_mm', 'nozzle_len_mm', 'nozzle_tip_len_mm',
    'contact_standoff_mm', 'conduit_dia_mm', 'conduit_len_mm', 'conduit_x_mm', 'conduit_y_mm', 'conduit_z_mm',
    'conduit_r_deg', 'conduit_p_deg', 'conduit_y_deg',
)


def _launch_setup(context, *args, **kwargs):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    headless = arg('headless').lower() in ('true', '1')
    mappings = {name: arg(name) for name in POSE_ARGS}
    mappings.update({name: arg(name) for name in XACRO_ARGS if arg(name).strip()})
    padding_mm = float(arg('padding_mm'))
    manifest = arg('cell_manifest') or (FindPackageShare('awm_bringup').perform(context)
                                        + '/config/cell_scene_nominal.yaml')
    moveit_config = (
        MoveItConfigsBuilder('awm_cell_v2', package_name='awm_description')
        .robot_description(file_path='urdf/awm_cell_v2.urdf.xacro', mappings=mappings)
        .robot_description_semantic(file_path='srdf/awm_cell_v2.srdf.xacro')
        .robot_description_kinematics(file_path='config/kinematics_v2.yaml')
        .joint_limits(file_path='config/joint_limits_v2.yaml')
        .trajectory_execution(file_path='config/moveit_controllers_v2.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True,
                                publish_planning_scene=True,
                                publish_geometry_updates=True,
                                publish_state_updates=True,
                                publish_transforms_updates=True)
        .to_moveit_configs()
    )
    controllers = FindPackageShare('awm_description').perform(context) + '/config/ros2_controllers_v2.yaml'
    # Padding on every robot link from the start; cell_scene_loader then sets nozzle_tip to 0 (F4).
    padding = {'robot_description_planning.default_robot_padding': units.mm_to_m(padding_mm)}

    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='log',
             parameters=[moveit_config.robot_description]),
        Node(package='controller_manager', executable='ros2_control_node', output='log',
             parameters=[controllers],
             remappings=[('/controller_manager/robot_description', '/robot_description')]),
        Node(package='controller_manager', executable='spawner', output='log',
             arguments=['joint_state_broadcaster', 'joint_trajectory_controller',
                        'positioner_trajectory_controller', '--controller-manager', '/controller_manager']),
        Node(package='moveit_ros_move_group', executable='move_group', output='log',
             parameters=[moveit_config.to_dict(), padding, {'publish_robot_description_semantic': True}]),
        Node(package='awm_bringup', executable='cell_scene_loader', output='log',
             parameters=[{'manifest': manifest, 'padding_mm': padding_mm, 'unpadded_links': ['nozzle_tip'],
                          'positioner_xyz_mm': mappings['positioner_xyz_mm'],
                          'positioner_quat_xyzw': mappings['positioner_quat_xyzw'],
                          'positioner_axis_xyz': mappings['positioner_axis_xyz']},
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
        DeclareLaunchArgument('headless', default_value='false', description='true: no RViz (tests, CI)'),
        DeclareLaunchArgument('cell_manifest', default_value='',
                              description='cell scene manifest (mm); empty = config/cell_scene_nominal.yaml '
                                          '(Pismo box only, Assumption)'),
        DeclareLaunchArgument('padding_mm', default_value='3.0',
                              description='padding on every robot link except nozzle_tip (TST-002 F4, D16)'),
    ]
    decls += [DeclareLaunchArgument(n, default_value=d, description=t) for n, (d, t) in POSE_ARGS.items()]
    decls += [DeclareLaunchArgument(n, default_value='', description='H-BOX / positioner xacro arg; empty = default')
              for n in XACRO_ARGS]
    return LaunchDescription(decls + [OpaqueFunction(function=_launch_setup)])
