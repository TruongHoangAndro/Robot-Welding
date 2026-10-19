"""Simulation bring-up: cell model on ros2_control mock hardware + MoveIt move_group (+ RViz).

    ros2 launch awm_bringup sim.launch.py [headless:=true]

Mock hardware only (mock_components/GenericSystem): this launch never connects to a robot.
The ROS graph is isolated by the container environment (ROS_DOMAIN_ID=42, localhost discovery).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _moveit_config():
    return (
        MoveItConfigsBuilder('awm_cell', package_name='awm_description')
        .robot_description(file_path='urdf/awm_cell.urdf.xacro')
        .robot_description_semantic(file_path='srdf/awm_cell.srdf.xacro')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True,
                                publish_planning_scene=True,
                                publish_geometry_updates=True,
                                publish_state_updates=True,
                                publish_transforms_updates=True)
        .to_moveit_configs()
    )


def _launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1')
    moveit_config = _moveit_config()
    controllers = FindPackageShare('awm_description').perform(context) + \
        '/config/ros2_controllers.yaml'

    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='log', parameters=[moveit_config.robot_description]),
        Node(package='controller_manager', executable='ros2_control_node', output='log',
             parameters=[controllers],
             remappings=[('/controller_manager/robot_description', '/robot_description')]),
        Node(package='controller_manager', executable='spawner', output='log',
             arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']),
        Node(package='controller_manager', executable='spawner', output='log',
             arguments=['joint_trajectory_controller', '--controller-manager',
                        '/controller_manager']),
        Node(package='moveit_ros_move_group', executable='move_group', output='log',
             parameters=[moveit_config.to_dict(), {'publish_robot_description_semantic': True}]),
    ]
    if not headless:
        rviz_config = FindPackageShare('ur_moveit_config').perform(context) + '/config/moveit.rviz'
        nodes.append(Node(package='rviz2', executable='rviz2', output='log',
                          arguments=['-d', rviz_config],
                          parameters=[moveit_config.robot_description,
                                      moveit_config.robot_description_semantic,
                                      moveit_config.robot_description_kinematics,
                                      moveit_config.planning_pipelines,
                                      moveit_config.joint_limits]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='true: no RViz (tests, CI)'),
        OpaqueFunction(function=_launch_setup),
    ])
