import os
from launch_ros.actions import Node
from launch import LaunchDescription
from utils.EnvParams import EnvParams
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_share_path

def generate_launch_description():
    use_sim_time = EnvParams().USE_SIM_TIME == 'true'
    use_sim_time_str = EnvParams().USE_SIM_TIME  # 'true' or 'false' string for launch args

    if EnvParams().LIDAR == "REALTIME":
        lidar_sim = 'false'
    else: 
        lidar_sim = 'true'

    if EnvParams().VISUALIZATION == "ON":
        launch_file = "world_viz.launch.py"
    else:
        launch_file = "world_core.launch.py"

    if EnvParams().USE_MOCK_HARDWARE == 'true':
        use_mock_hardware = 'true'
    else:
        use_mock_hardware = 'false'

    twist_mux_params = os.path.join(get_package_share_directory('control'),'config','twist_mux.yaml')
    nav2_params = os.path.join(get_package_share_directory('control'),'config','nav2_params.yaml')
    
    wheelchair_bringup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("my_robot_bringup"), "launch"), f"/{launch_file}"]),
                    launch_arguments={'use_lidar_sim': lidar_sim, 'use_mock_hardware': use_mock_hardware}.items()
    )

    lidar_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("control"), "launch"), "/lidar.launch.py"]),
                launch_arguments={'frame_id': 'lidar_r'}.items(),
                condition=IfCondition(PythonExpression([f"'{lidar_sim}' == 'false'"]))
    )

    lifelong_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_path("control"), "launch"), "/lifelong_launch.py"]),
        launch_arguments=[('use_sim_time', use_sim_time_str)]
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_path("nav2_bringup"), "launch"), "/navigation_launch.py"]),
        launch_arguments=[('use_sim_time', use_sim_time_str),
                  ('params_file', nav2_params), ('map_subscribe_transient_local', 'true')]
    )

    navigation_node = Node(
        package="control",
        executable="navigation_node",
        output="screen", 
        parameters=[{'use_sim_time': use_sim_time}]
    )

    odom_node = Node(
        package="control",
        executable="odom_node",
        output="screen", 
        parameters=[{'use_sim_time': use_sim_time}]
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    auto_nav_node = Node(
        package="control",
        executable="auto_nav_node",
        output="screen", 
        parameters=[{'use_sim_time': use_sim_time}]
    )

    speach_recognizer_node = Node(
        package="cv",
        executable="speach_recognizer_node",
        output="screen", 
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        wheelchair_bringup,
        lidar_launch,
        navigation_node,
        odom_node,
        twist_mux_node,
        # lifelong_slam_launch,
        # navigation_launch,
        # auto_nav_node,
        # speach_recognizer_node,
    ])