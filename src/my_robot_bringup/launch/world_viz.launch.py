"""
Laptop-side launch file.
Starts: RViz2.
Run this on your laptop while world_core.launch.py runs on the Pi.
Both machines must be on the same ROS_DOMAIN_ID / network.
"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from utils.EnvParams import EnvParams

def generate_launch_description():
    ## Environment parameters
    use_sim_time = EnvParams().USE_SIM_TIME == 'true'

    ## Paths
    robot_description_pkg = get_package_share_directory('my_robot_description')
    rviz_config_path = os.path.join(robot_description_pkg, 'rviz', 'wheelchair_config.rviz')
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        rviz2_node,
    ])
