from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_link',
          'odom_frame_id':'odom',
          'odom_tf_linear_variance':0.001,
          'odom_tf_angular_variance':0.001,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'approx_sync':True,
          'sync_queue_size': 10,
          # RTAB-Map's internal parameters should be strings
          'RGBD/NeighborLinkRefining': 'true',    # Do odometry correction with consecutive laser scans
          'RGBD/ProximityBySpace':     'true',    # Local loop closure detection (using estimated position) with locations in WM
          'RGBD/ProximityByTime':      'false',   # Local loop closure detection with locations in STM
          'RGBD/ProximityPathMaxNeighbors': '10', # Do also proximity detection by space by merging close scans together.
          'Reg/Strategy':              '1',       # 0=Visual, 1=ICP, 2=Visual+ICP
          'Vis/MinInliers':            '12',      # 3D visual words minimum inliers to accept loop closure
          'RGBD/OptimizeFromGraphEnd': 'false',   # Optimize graph from initial node so /map -> /odom transform will be generated
          'RGBD/OptimizeMaxError':     '4',       # Reject any loop closure causing large errors (>3x link's covariance) in the map
          'Reg/Force3DoF':             'true',    # 2D SLAM
          'Grid/FromDepth':            'false',   # Create 2D occupancy grid from laser scan
          'Mem/STMSize':               '30',      # increased to 30 to avoid adding too many loop closures on just seen locations
          'RGBD/LocalRadius':          '5',       # limit length of proximity detections
          'Icp/CorrespondenceRatio':   '0.2',     # minimum scan overlap to accept loop closure
          'Icp/PM':                    'false',
          'Icp/PointToPlane':          'false',
          'Icp/MaxCorrespondenceDistance': '0.15',
          'Icp/VoxelSize':             '0.05'
    }
    
    remappings=[
         ('rgb/image',       '/rgbd_camera/image'),
         ('depth/image',     '/rgbd_camera/depth_image'),
         ('rgb/camera_info', '/rgbd_camera/camera_info'),
         ('scan',            '/scan')]
    
    # Nodes to launch
    rgbd_sync_node = Node(
            package='rtabmap_sync', 
            executable='rgbd_sync', 
            output='screen',
            parameters=[parameters,
              {'approx_sync_max_interval': 0.02}],
            remappings=remappings
    )
    # SLAM mode:
    slam_node = Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d'] # This will delete the previous database (~/.ros/rtabmap.db)
    ) 

    # Localization mode:
    localization_node =  Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings
    )

    # Visualization:
    rtabmap_viz_node = Node(
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            package='rtabmap_viz', 
            executable='rtabmap_viz', 
            output='screen',
            parameters=[parameters],
            remappings=remappings
    )
        
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='true',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        SetParameter(name='use_sim_time', value=True),

        rgbd_sync_node,
        slam_node,
        localization_node,
        rtabmap_viz_node,
    ])