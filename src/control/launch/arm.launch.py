import os
from launch_ros.actions import Node
from launch import LaunchDescription
from utils.EnvParams import EnvParams
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = EnvParams().USE_SIM_TIME == 'true'

    bringup_pkg     = get_package_share_directory("my_robot_bringup")
    moveit_pkg      = get_package_share_directory("my_robot_moveit_config")

    # ── 1. Controllers (robot_state_publisher + ros2_control + spawners) ──
    arm_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "arm_core.launch.py")
        )
    )

    # ── 2. MoveIt move_group (delayed so controllers are up first) ────────
    move_group = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_pkg, "launch", "move_group.launch.py")
                )
            )
        ]
    )

    # ── 3. ArmCommander  ──────────────────────────────────────────────────
    arm_commander_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="control",
                executable="arm_commander_node",
                name="commander",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )


    # ── 5. CameraPoseNode ─────────────────────────────────────────────────
    camera_pose_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="control",
                executable="camera_pose_node",
                name="camera_pose_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )

    # ── 6. Steppers hardware node ─────────────────────────────────────────
    steppers_node = Node(
        package="control",
        executable="steppers_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        arm_core,
        move_group,
        arm_commander_node,
        camera_pose_node,
        steppers_node,
    ])