import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    joystick_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("control"), "launch"), "/joystick.launch.py"]),
             )
    
    wheelchair_bringup = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("my_robot_bringup"), "launch"), "/my_wheelchair.launch.py"]),
             )
    
    return LaunchDescription([
        joystick_launch,
        wheelchair_bringup
    ])