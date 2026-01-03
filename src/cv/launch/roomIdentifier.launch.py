import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    camera_streamer_params = os.path.join(get_package_share_directory('cv'),'config','camera_streamer.yaml')
    camera_streamer_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        parameters=[camera_streamer_params, {'use_sim_time': True}],
        output='screen'
    )

    room_identifier_node = Node(
        package='cv',
        executable='room_identifier_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        camera_streamer_node,
        room_identifier_node
    ])