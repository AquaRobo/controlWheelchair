"""
Laptop-side launch file.
Starts: Gazebo simulator, robot spawner, RViz2, and controller spawners.
Run this on your laptop while world_core.launch.py runs on the Pi.
Both machines must be on the same ROS_DOMAIN_ID / network.

Note: controller_manager is created by the ros2_control Gazebo plugin when
Gazebo loads the robot, so the spawners must run here on the laptop.
"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
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
    pkg_worlds = get_package_share_directory('gazebo_worlds')

    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'my_wheelchair.urdf.xacro')
    rviz_config_path = os.path.join(robot_description_pkg, 'rviz', 'wheelchair_config.rviz')
    world_path = os.path.join(pkg_worlds, 'worlds', 'house_turtlebot.world')

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' use_lidar_sim:=', use_lidar_sim,
            ' use_mock_hardware:=', use_mock_hardware,
        ]),
        value_type=str
    )

    # robot_state_publisher is needed here too so Gazebo can read /robot_description
    # and RViz can render the model.  It mirrors the one on the Pi; both publish
    # the same static transforms so they are idempotent across the network.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{pkg_worlds}/models:{pkg_worlds}/custom_map'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments=[
            ('gz_args', [' -v 4', ' -r', f' {world_path}'])
        ],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'my_wheelchair'],
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Bridge must run co-located with Gazebo — gz-transport is local IPC only.
    # Once converted to ROS 2 topics, DDS propagates them across the network to the Pi.
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ]
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    simple_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_velocity_controller'],
        output='screen',
    )

    # Mirror the timing from the original world.launch.py:
    # wait 8 s after spawn for controller_manager to be ready, then spawn
    # joint_state_broadcaster; once that starts, wait 3 s for the velocity controller.
    delayed_joint_state_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gz_spawn_entity,
            on_start=[TimerAction(period=8.0, actions=[joint_state_spawner])],
        )
    )

    delayed_simple_velocity_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_spawner,
            on_start=[TimerAction(period=3.0, actions=[simple_velocity_controller_spawner])],
        )
    )

    return LaunchDescription([
        # lidar_sim_arg,
        mock_hw_arg,
        # set_gz_resource_path,
        # robot_state_publisher_node,
        # gazebo,
        # gz_spawn_entity,
        gz_ros2_bridge,
        delayed_joint_state_spawner,
        delayed_simple_velocity_spawner,
        rviz2_node,
    ])
