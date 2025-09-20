#!/usr/bin/env python3
"""ROS 2 launch file (Python) equivalent to the prior XML version.

Starts:
 - robot_state_publisher (with xacro processed robot_description)
 - ros2_control_node (controller_manager) with controllers config
 - spawners for joint_state_broadcaster and diff_drive_controller
 - RViz2 with a predefined config
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Paths
    description_share = FindPackageShare('my_robot_description')
    bringup_share = FindPackageShare('my_robot_bringup')

    urdf_path = PathJoinSubstitution([description_share, 'urdf', 'my_robot.urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([description_share, 'rviz', 'urdf_config.rviz'])
    controllers_yaml = PathJoinSubstitution([bringup_share, 'config', 'my_robot_controller.yaml'])

    # Allow overriding rviz config via CLI
    rviz_arg = DeclareLaunchArgument(
        'rviz_config', default_value=rviz_config_path,
        description='Path to RViz configuration file'
    )

    rviz_cfg = LaunchConfiguration('rviz_config')

    # robot_description via xacro
    robot_description = Command(['xacro ', urdf_path])

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    ros2_control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_yaml, {'robot_description': robot_description}],
        output='screen',
        name='controller_manager'
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_pub,
        ros2_control,
        joint_state_spawner,
        diff_drive_spawner,
        rviz2,
    ])

