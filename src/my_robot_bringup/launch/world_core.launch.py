"""
Pi-side launch file.
Starts: robot_state_publisher and ros2_control_node.
Run this on the Raspberry Pi.

When Gazebo is not running on the Pi, we use mock_components/GenericSystem so
controller_manager is still available and controllers can be spawned locally.
"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from utils.EnvParams import EnvParams

def generate_launch_description():
    ## Environment parameters
    use_sim_time = EnvParams().USE_SIM_TIME == 'true'
    use_mock_hardware = EnvParams().USE_MOCK_HARDWARE == 'true'

    ## Simulation arguments
    use_lidar_sim = LaunchConfiguration('use_lidar_sim')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    lidar_sim_arg = DeclareLaunchArgument('use_lidar_sim', default_value='true')
    mock_hw_arg = DeclareLaunchArgument('use_mock_hardware', default_value=use_mock_hardware)


    ## Paths
    robot_description_pkg = get_package_share_directory('my_robot_description')
    bringup_pkg = get_package_share_directory('my_robot_bringup')
    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'my_wheelchair.urdf.xacro')
    controller_yaml = os.path.join(bringup_pkg, 'config', 'my_wheelchair_controller.yaml')

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' use_lidar_sim:=', use_lidar_sim,
            ' use_mock_hardware:=', use_mock_hardware,
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': use_sim_time},
            controller_yaml,
        ],
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    simple_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    delayed_joint_state_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[TimerAction(period=2.0, actions=[joint_state_spawner])],
        )
    )

    delayed_simple_velocity_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_spawner,
            on_start=[TimerAction(period=2.0, actions=[simple_velocity_controller_spawner])],
        )
    )

    return LaunchDescription([
        lidar_sim_arg,
        mock_hw_arg,
        robot_state_publisher_node,
        ros2_control_node,
        delayed_joint_state_spawner,
        delayed_simple_velocity_spawner,
    ])
