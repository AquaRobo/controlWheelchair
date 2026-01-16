import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    mapping_params = os.path.join(get_package_share_directory('control'),'config','mapping.yaml')
    twist_mux_params = os.path.join(get_package_share_directory('control'),'config','twist_mux.yaml')

    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("control"), "launch"), "/joystick.launch.py"]),
    )
    
    wheelchair_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("my_robot_bringup"), "launch"), "/world.launch.py"]),
    )
    
    # Mapping Launch
    slam_async_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments=[('use_sim_time', 'true'),
                  ('slam_params_file', mapping_params)]
    )

    navigation_node = Node(
        package="control",
        executable="navigation_node",
        output="screen", 
        parameters=[{'use_sim_time': True}]
    )

    odom_node = Node(
        package="control",
        executable="odom_node",
        output="screen", 
        parameters=[{'use_sim_time': True}]
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    # Room_Identifier launch
    room_identifier_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("cv"), "launch"), "/roomIdentifier.launch.py"]),
    )

    # Room_Pose_Saver Node
    room_pose_saver_node = Node(
        package="control",
        executable="room_pose_saver_node",
        output="screen", 
        parameters=[{'use_sim_time': True}]
    )
    # Map Saver Node
    map_saver_node = Node(
        package="control",
        executable="map_saver_node",
        output="screen", 
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        joystick_launch,
        wheelchair_bringup,
        navigation_node,
        odom_node,
        slam_async_launch,
        twist_mux_node,
        room_pose_saver_node,
        map_saver_node,
        room_identifier_launch,
    ])