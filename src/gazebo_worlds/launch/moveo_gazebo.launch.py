from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    # Launch arguments
    gz_args = LaunchConfiguration('gz_args')
    model_name = LaunchConfiguration('model_name')

    # Package directories
    moveo_gazebo_dir = get_package_share_directory('moveo_gazebo')
    robot_desc_dir = get_package_share_directory('my_robot_description')
    bringup_dir = get_package_share_directory('my_robot_bringup')

    # Gazebo resources
    models_path = os.path.join(moveo_gazebo_dir, 'models')
    gz_resource_path = f"{models_path}:{robot_desc_dir}:{os.path.join(robot_desc_dir, '..')}"
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    # Convert XACRO to URDF
    xacro_file = os.path.join(robot_desc_dir, 'urdf', 'my_robot_urdf.xacro')
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"XACRO file not found: {xacro_file}")
    urdf_file = os.path.join(robot_desc_dir, 'urdf', 'my_robot_urdf.urdf')
    # Convert and save URDF with use_sim parameter for Gazebo
    urdf_doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    urdf_doc.toxml()
    with open(urdf_file, 'w') as f:
        f.write(urdf_doc.toxml())

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_doc.toxml(),
            'use_sim_time': True
        }]
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # Spawn robot in Gazebo (from URDF file)
    spawn_entity = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=['-file', urdf_file, '-name', model_name, '-x', '0', '-y', '0', '-z', '0.1'],
    output='screen'
    )   


    # Controller spawners (delayed for safety)
    joint_state_broadcaster_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        )]
    )

    arm_controller_spawner = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
            output='screen'
        )]
    )

    gripper_controller_spawner = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller'],
            output='screen'
        )]
    )

    # Default world - use -r flag to run simulation immediately (not paused)
    default_world = os.path.join(moveo_gazebo_dir, 'worlds', 'pick_and_place_demo.world')

    return LaunchDescription([
        set_gz_resource_path,
        DeclareLaunchArgument(
            'gz_args',
            default_value=f'-r {default_world}',
            description='Gazebo arguments (world file or empty)'
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='my_robot',
            description='Robot model name in Gazebo'
        ),
        gazebo,
        robot_state_publisher_node,
        # Note: ros2_control_node is managed by Gazebo's gz_ros2_control plugin
        spawn_entity,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])
