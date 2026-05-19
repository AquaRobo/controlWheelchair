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
    use_sim_time_str = EnvParams().USE_SIM_TIME 

    launch_file = "world_viz.launch.py" if EnvParams().VISUALIZATION == "ON" else "arm_core.launch.py"
    
    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("my_robot_bringup"), "launch"), f"/{launch_file}"]),
           
    )

    arm_control_node = Node(
        package="control",
        executable="steppers_node",
        output="screen", 
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        arm_bringup,
        arm_control_node,
    ])