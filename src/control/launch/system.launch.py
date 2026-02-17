import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_share_path

def generate_launch_description():
    twist_mux_params = os.path.join(get_package_share_directory('control'),'config','twist_mux.yaml')
    localization_params = os.path.join(get_package_share_directory('control'),'config','localization_params.yaml')
    nav2_params = os.path.join(get_package_share_directory('control'),'config','nav2_params.yaml')
    ekf_params = os.path.join(get_package_share_directory('control'),'config','ekf_params.yaml')
    map_path = "./turtlebot_map.yaml"

    joystick_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("control"), "launch"), "/joystick.launch.py"]),
    )
    
    wheelchair_bringup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("my_robot_bringup"), "launch"), "/world.launch.py"]),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_path("nav2_bringup"), "launch"), "/localization_launch.py"]),
        launch_arguments=[('use_sim_time', 'true'),
                  ('map', map_path), ('params_file', localization_params)]
    )

    lifelong_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_path("control"), "launch"), "/lifelong_launch.py"]),
        launch_arguments=[('use_sim_time', 'true')]
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_path("nav2_bringup"), "launch"), "/navigation_launch.py"]),
        launch_arguments=[('use_sim_time', 'true'), 
                  ('params_file', nav2_params), ('map_subscribe_transient_local', 'true')]
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

    auto_nav_node = Node(
        package="control",
        executable="auto_nav_node",
        output="screen", 
        parameters=[{'use_sim_time': True}]
    )

    speach_recognizer_node = Node(
        package="cv",
        executable="speach_recognizer_node",
        output="screen", 
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        joystick_launch,
        wheelchair_bringup,
        navigation_node,
        odom_node,
        twist_mux_node,
        # localization_launch,
        # lifelong_slam_launch,
        # navigation_launch,
        # auto_nav_node,
        # speach_recognizer_node,
    ])