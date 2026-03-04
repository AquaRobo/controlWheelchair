"""
Pi-side launch file.
Starts: robot_state_publisher and the Gazebo bridge.
Run this on the Raspberry Pi.

Note: controller_manager is created by the ros2_control Gazebo plugin, which
runs inside Gazebo on the laptop.  For that reason the controller spawners live
in world_viz.launch.py, not here.

The Gazebo + RViz + spawners side runs on the laptop (world_viz.launch.py).
"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from utils.EnvParams import EnvParams

def generate_launch_description():
    use_sim_time = EnvParams().USE_SIM_TIME == 'true'
    use_lidar_sim = LaunchConfiguration('use_lidar_sim')
    lidar_sim_arg = DeclareLaunchArgument('use_lidar_sim', default_value='true')

    robot_description_pkg = get_package_share_directory('my_robot_description')
    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'my_wheelchair.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' use_lidar_sim:=', use_lidar_sim]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        lidar_sim_arg,
        robot_state_publisher_node,
    ])
