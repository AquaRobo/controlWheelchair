"""
Pi-side launch file.
Starts: robot_state_publisher, ros2_control_node and the controllers.
Run this on the Raspberry Pi.
"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from utils.EnvParams import EnvParams
from launch.substitutions import Command 
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler, TimerAction
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ## Environment parameters
    use_sim_time = EnvParams().USE_SIM_TIME == 'true'
    use_sim_time_str = 'true' if EnvParams().USE_SIM_TIME == 'true' else 'false'

    ## Paths
    robot_description_pkg = get_package_share_directory('my_robot_description')
    bringup_pkg = get_package_share_directory('my_robot_bringup')
    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'arm_urdf.xacro')
    controller_yaml = os.path.join(bringup_pkg, 'config', 'arm_controllers.yaml')

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' use_sim:=', use_sim_time_str,
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

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    ) 

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    delayed_joint_state_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[TimerAction(period=2.0, actions=[joint_state_spawner])],
        )
    )

    delayed_arm_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_spawner,
            on_start=[TimerAction(period=2.0, actions=[arm_controller_spawner])],
        )
    )

    delayed_gripper_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=arm_controller_spawner,
            on_start=[TimerAction(period=2.0, actions=[gripper_controller_spawner])],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        delayed_joint_state_spawner,
        delayed_arm_spawner,
        delayed_gripper_spawner,
    ])
