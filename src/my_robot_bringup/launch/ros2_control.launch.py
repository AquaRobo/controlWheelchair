from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare("my_robot_description").find("my_robot_description")
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "my_robot.urdf.xacro"])

    robot_description = Command([FindExecutable(name="xacro"), " ", urdf_file])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            remappings=[("robot_description", "/controller_manager/robot_description")],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "my_robot_controller.yaml"])
            ],
            output="screen",
        ),
    ])
