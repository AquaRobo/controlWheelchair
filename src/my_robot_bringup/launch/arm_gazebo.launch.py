#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    TimerAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro


def launch_setup(context, *args, **kwargs):

    use_sim = LaunchConfiguration('use_sim').perform(context)
    use_camera = LaunchConfiguration('use_camera').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)

    robot_desc_dir = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(robot_desc_dir, 'urdf', 'arm_urdf.xacro')
    urdf_file = os.path.join(robot_desc_dir, 'urdf', 'arm_moveo_urdf.urdf')

    urdf_doc = xacro.process_file(
        xacro_file,
        mappings={
            'use_sim': use_sim,
            'use_camera': use_camera
        }
    )

    robot_description_content = urdf_doc.toxml()

    with open(urdf_file, 'w') as f:
        f.write(robot_description_content)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim == 'true'
        }]
    )

    nodes = [robot_state_publisher_node]

    # Spawn robot in Gazebo ONLY if simulation
    if use_sim == 'true':
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', urdf_file, '-name', model_name,
                       '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        )
        nodes.append(spawn_entity)

    return nodes


def generate_launch_description():

    use_sim = LaunchConfiguration('use_sim')
    use_camera = LaunchConfiguration('use_camera')
    gz_args = LaunchConfiguration('gz_args')

    moveo_gazebo_dir = get_package_share_directory('gazebo_worlds')
    robot_desc_dir = get_package_share_directory('my_robot_description')
    bringup_dir = get_package_share_directory('my_robot_bringup')

    models_path = os.path.join(moveo_gazebo_dir, 'models')
    gz_resource_path = f"{models_path}:{robot_desc_dir}:{os.path.join(robot_desc_dir, '..')}"

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    default_world = os.path.join(
        moveo_gazebo_dir,
        'worlds',
        'pick_and_place_demo.world'
    )

    # Gazebo (ONLY if simulation)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': gz_args}.items(),
        condition=IfCondition(use_sim)
    )



    # RGB bridge
    rgb_bridge = TimerAction(
        period=6.0,  # wait 6 seconds before starting
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/camera/image_raw/image@sensor_msgs/msg/Image[gz.msgs.Image]'],
            output='screen',
            condition=IfCondition(use_camera)
        )]
    )

    depth_bridge = TimerAction(
    period=6.0,
    actions=[Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw/depth_image@sensor_msgs/msg/Image[gz.msgs.Image]'],
        output='screen',
        condition=IfCondition(use_camera)
    )]
    )

    camera_info_bridge = TimerAction(
        period=6.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo]'],
            output='screen',
            condition=IfCondition(use_camera)
        )]
    )
        # Clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        condition=IfCondition(use_sim)
    )

    
    # Bringup (controllers + MoveIt)
    my_robot_launch = TimerAction(
        period=4.0 if use_sim else 0.0,
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        bringup_dir,
                        'launch',
                        'arm.launch.xml'
                    )
                ),
                launch_arguments={'use_sim': use_sim}.items()
            )
        ]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Run in simulation mode'
        ),

        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='Enable camera'
        ),

        DeclareLaunchArgument(
            'gz_args',
            default_value=f'-r {default_world}',
            description='Gazebo arguments'
        ),

        DeclareLaunchArgument(
            'model_name',
            default_value='my_robot',
            description='Robot model name'
        ),

        set_gz_resource_path,

        gazebo,

        OpaqueFunction(function=launch_setup),

        rgb_bridge,
        depth_bridge,
        camera_info_bridge,


        clock_bridge,

        my_robot_launch,
    ])