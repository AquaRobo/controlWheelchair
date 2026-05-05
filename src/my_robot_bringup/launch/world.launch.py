"""
Single-machine launch file (development / testing on one PC).
For split deployment:
  - Pi    → ros2 launch my_robot_bringup world_core.launch.py
  - Laptop → ros2 launch my_robot_bringup world_viz.launch.py
"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command , LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler, TimerAction, DeclareLaunchArgument
from utils.EnvParams import EnvParams

def generate_launch_description():
    ## Environment parameters
    use_sim_time = EnvParams().USE_SIM_TIME == 'true'
    share_root = os.path.dirname(get_package_share_directory('my_robot_description'))
    ## Simulation arguments
    use_lidar_sim = LaunchConfiguration('use_lidar_sim')
    use_imu_sim = LaunchConfiguration('use_imu_sim')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    lidar_sim_arg = DeclareLaunchArgument('use_lidar_sim', default_value='true')
    imu_sim_arg = DeclareLaunchArgument('use_imu_sim', default_value='true')
    mock_hw_arg = DeclareLaunchArgument('use_mock_hardware', default_value='true')
    # Package paths
    robot_description_pkg = get_package_share_directory('my_robot_description')
    pkg_worlds = get_package_share_directory('gazebo_worlds')

    # Paths
    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'my_wheelchair.urdf.xacro')
    rviz_config_path = os.path.join(robot_description_pkg, 'rviz', 'wheelchair_config.rviz')
    world_path = os.path.join(pkg_worlds, 'worlds', 'house_turtlebot.world')
    
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' use_lidar_sim:=', use_lidar_sim,
            ' use_imu_sim:=', use_imu_sim,
            ' use_mock_hardware:=', use_mock_hardware,
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}], 
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    simple_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_velocity_controller'],
        output='screen'
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', rviz_config_path], 
        parameters=[{'use_sim_time': use_sim_time}],
    )

    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{pkg_worlds}/models:{pkg_worlds}/custom_map'
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4", " -r", f" {world_path}"]
                    )
                ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "my_wheelchair"]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ]
    )

    delayed_joint_state_spawner = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=gz_spawn_entity,
        on_exit=[TimerAction(period=12.0, actions=[joint_state_spawner])],
    ))

    delayed_simple_velocity_spawner = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=joint_state_spawner,
        on_exit=[TimerAction(period=5.0, actions=[simple_velocity_controller_spawner])],
    ))

    return LaunchDescription([
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=share_root),
        AppendEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=share_root),
        lidar_sim_arg,
        imu_sim_arg,
        mock_hw_arg,
        robot_state_publisher_node,
        delayed_joint_state_spawner,
        delayed_simple_velocity_spawner,
        set_gz_resource_path,
        rviz2_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])