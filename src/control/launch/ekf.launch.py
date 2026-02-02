from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
import os

def generate_launch_description():
    ekf_params = os.path.join(get_package_share_directory('control'),'config','ekf_params.yaml')

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[{'use_sim_time': True}, ekf_params],
    )

    return LaunchDescription([
        robot_localization,
    ])