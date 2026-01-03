import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, SetEnvironmentVariable

def generate_launch_description():

    # Package paths
    robot_description_pkg = get_package_share_directory('my_robot_description')
    pkg_worlds = get_package_share_directory('gazebo_worlds')

    # Paths
    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'my_wheelchair.urdf.xacro')
    rviz_config_path = os.path.join(robot_description_pkg, 'rviz', 'wheelchair_config.rviz')
    world_path = os.path.join(pkg_worlds, 'worlds', 'house_turtlebot.world')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}], 
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
        parameters=[{'use_sim_time': True}],
    )

    set_gz_resource_path = SetEnvironmentVariable(
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
    event_handler=OnProcessStart(
        target_action=gz_spawn_entity,
        on_start=[TimerAction(period=8.0, actions=[joint_state_spawner])],
    ))

    delayed_simple_velocity_spawner = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=joint_state_spawner,
        on_start=[TimerAction(period=3.0, actions=[simple_velocity_controller_spawner])],
    ))

    return LaunchDescription([
        robot_state_publisher_node,
        delayed_joint_state_spawner,
        delayed_simple_velocity_spawner,
        set_gz_resource_path,
        rviz2_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])