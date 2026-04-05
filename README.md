# 🔁 Persistent Environment Setup (Add to .bashrc)
>``sudo nano ~/.bashrc``
## Add the following lines at the end of the bashrc
#### 2- Source ROS 2 Jazzy 
>``source /opt/ros/jazzy/setup.bash``
#### 3- Source the workspace (replace with the actual path to your Wheelchair/controlWheelchair/install/setup.bash) 
>``source /home/<linux root username>/Wheelchair/controlWheelchair/install/setup.bash``

#### 4- To run the simulation and rviz at same time:

> `` colcon build --symlink-install ``

> `` source ./install/setup.bash `` 

> `` ros2 launch my_robot_bringup arm_gazebo.launch.py use_sim:=true use_camera:=true``

#### 5- To run publisher:
> `` source ./install/setup.bash ``
> `` ros2 run control arm_pose_node  ``
---


