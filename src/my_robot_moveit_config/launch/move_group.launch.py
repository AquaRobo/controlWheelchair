import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("my_robot_description", package_name="my_robot_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("my_robot_description"),
                "urdf",
                "arm_urdf_expanded.urdf",       # expanded output written by arm_gazebo.launch.py
            )
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([move_group_node])
