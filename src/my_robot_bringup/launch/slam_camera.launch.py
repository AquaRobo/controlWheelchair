import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # === Paths ===
    pkg_desc = get_package_share_path('my_robot_description')
    pkg_bringup = get_package_share_path('my_robot_bringup')
    pkg_worlds = get_package_share_path('gazebo_worlds')

    urdf_path = os.path.join(pkg_desc, 'urdf', 'my_wheelchair.urdf.xacro')
    rviz_config_path = os.path.join(pkg_desc, 'rviz', 'wheelchair_config.rviz')
    controllers_yaml = os.path.join(pkg_bringup, 'config', 'my_wheelchair_controller.yaml')
    world_path = os.path.join(pkg_worlds, 'worlds', 'small_house.world')
    slam_params = os.path.join(pkg_bringup, 'config', 'slam_params.yaml')

    # === Robot Description ===
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # === Gazebo resource path ===
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{pkg_worlds}/models:{pkg_worlds}/custom_map'
    )

    # === Launch argument: delay for diff drive controller spawn ===
    diff_drive_delay_arg = DeclareLaunchArgument(
        'diff_drive_spawn_delay',
        default_value='8.0',
        description='Delay before spawning diff_drive controller (seconds)'
    )

    # === Robot State Publisher ===
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # === ROS2 Control ===
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_yaml, {'robot_description': robot_description}],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    delayed_diff_drive_spawner = TimerAction(
        period=LaunchConfiguration('diff_drive_spawn_delay'),
        actions=[diff_drive_spawner]
    )

    # === Start Gazebo ===
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ]),
        launch_arguments={
            "gz_args": f"-v 4 -r {world_path}"
        }.items()
    )

    # === Spawn robot ===
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_wheelchair',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
    )

    # === Static transforms for camera ===
    # Added to match depthimage_to_laserscan expected frame
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_t'],
        output='screen'
    )

    static_tf_camera_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_t'],
        output='screen'
    )

    # === ROS-Gazebo bridge ===
    # Updated topic names to exactly match Gazebo topics
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/depth_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',  # matches Gazebo
        ],
        remappings=[('/imu', '/imu/out')],
        output='screen',
    )

    # === Depth image -> LaserScan conversion ===
    depth_to_scan = Node(
    package='depthimage_to_laserscan',
    executable='depthimage_to_laserscan_node',
    name='depthimage_to_laserscan',
    output='screen',
    remappings=[
        ('/depth_camera/points','depth' ),  # Depth image input
        ('scan', '/scan'),                     # LaserScan output
    ],
    parameters=[{
        'output_frame': 'base_link',
        'camera_info_topic': '/depth_camera/camera_info',
        'range_min': 0.1,
        'range_max': 10.0,
        'scan_height': 1,
        'scan_time': 0.033,
        'use_sim_time': True
    }]
)


    # === SLAM Toolbox ===
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
        remappings=[('/odom', '/diff_drive_controller/odom')]
    )

    # === RViz ===
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # === Launch Description ===
    return LaunchDescription([
        set_gz_resource_path,
        diff_drive_delay_arg,
        start_gazebo,
        robot_state_publisher,
        ros2_control_node,
        joint_state_spawner,
        delayed_diff_drive_spawner,
        spawn_entity,
        # Added static TFs
        static_tf_base_to_camera,
        static_tf_camera_to_optical,
        gz_ros2_bridge,
        depth_to_scan,
        slam_toolbox_node,
        rviz
    ])
