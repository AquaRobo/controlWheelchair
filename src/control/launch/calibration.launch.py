import os
from launch_ros.actions import Node
from launch import LaunchDescription
from utils.EnvParams import EnvParams
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = EnvParams().USE_SIM_TIME == 'true'
    use_sim_time_str = EnvParams().USE_SIM_TIME 

    lidar_sim = "false" if EnvParams().LIDAR == "REALTIME" else "true"
    imu_value = EnvParams().IMU
    imu_sim = "false" if imu_value in ("REALTIME", "PI") else "true"
    use_mock_hardware = "true" if EnvParams().USE_MOCK_HARDWARE == 'true' else "false"
    launch_file = "world_viz.launch.py" if EnvParams().VISUALIZATION == "ON" else "world_core.launch.py"

    twist_mux_params = os.path.join(get_package_share_directory('control'),'config','twist_mux.yaml')
    mapping_params = os.path.join(get_package_share_directory('control'),'config','mapping.yaml')
    robot_localization_params = os.path.join(get_package_share_directory('control'),'config','robot_localization.yaml')
    
    wheelchair_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("my_robot_bringup"), "launch"), f"/{launch_file}"]),
            launch_arguments={'use_lidar_sim': lidar_sim, 'use_imu_sim': imu_sim, 'use_mock_hardware': use_mock_hardware}.items()
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("control"), "launch"), "/lidar.launch.py"]),
            launch_arguments={'frame_id': 'lidar_r'}.items(),
            condition=IfCondition(PythonExpression([f"'{lidar_sim}' == 'false'"]))
    )

    slam_async_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments=[('use_sim_time', 'true'),
                  ('slam_params_file', mapping_params)]
    )

    room_identifier_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("cv"), "launch"), "/roomIdentifier.launch.py"]),
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
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression([f"'{use_sim_time_str}' == 'true'"]))
    )

    odom_hardware_node = Node(
        package="control",
        executable="odom_hardware_node",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression([f"'{use_sim_time_str}' == 'false'"]))
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[robot_localization_params, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry')]
    )

    imu_node = Node(
        package="control",
        executable="imu_node",
        output="screen", 
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression([f"'{imu_value}' == 'REALTIME'"]))
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    pi_imu_node = Node(
        package="control",
        executable="pi_imu_node",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression([f"'{imu_value}' == 'PI'"]))
    )

    hoverboard_node = Node(
        package="control",
        executable="hoverboard_node",
        output="screen", 
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    room_pose_saver_node = Node(
        package="control",
        executable="room_pose_saver_node",
        output="screen", 
        parameters=[{'use_sim_time': True}]
    )

    map_saver_node = Node(
        package="control",
        executable="map_saver_node",
        output="screen", 
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        wheelchair_bringup,
        navigation_node,
        odom_node,
        slam_async_launch,
        twist_mux_node,
        lidar_launch,
        # room_pose_saver_node,
        # map_saver_node,
        # room_identifier_launch,
        odom_hardware_node,
        robot_localization_node,
        imu_node,
        pi_imu_node,
        hoverboard_node
    ])