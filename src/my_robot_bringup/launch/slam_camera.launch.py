import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart

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

    delayed_joint_state_spawner = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=spawn_entity,
        on_start=[TimerAction(period=12.0, actions=[joint_state_spawner])],
    ))

    delayed_diff_drive_spawner = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=joint_state_spawner,
        on_start=[TimerAction(period=3.0, actions=[diff_drive_spawner])],
    ))

    # === ROS-Gazebo bridge ===
    # Updated topic names to exactly match Gazebo topics
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
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
            ('/depth_camera_info', '/rgbd_camera/camera_info'),
            ('/depth',  '/rgbd_camera/depth_image'),
        ],
        parameters=[{
            'output_frame': 'base_link',
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
        start_gazebo,
        robot_state_publisher,
        spawn_entity,
        delayed_joint_state_spawner,
        delayed_diff_drive_spawner,
        gz_ros2_bridge,
        depth_to_scan,
        slam_toolbox_node,
        rviz,
    ])
