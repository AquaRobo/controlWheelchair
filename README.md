# 🔁 Persistent Environment Setup (Add to .bashrc)
>``sudo nano ~/.bashrc``
## Add the following lines at the end of the bashrc
#### 2- Source ROS 2 Jazzy 
>``source /opt/ros/jazzy/setup.bash``
#### 3- Source the workspace (replace with the actual path to your Wheelchair/controlWheelchair/install/setup.bash) 
>``source /home/<linux root username>/Wheelchair/controlWheelchair/install/setup.bash``

#### 4- To run the hardware lidar:

> Clone slamtec/rplidar_ros repo: ``git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git``

> `` cd rplidar_ros ``

> `` colcon build --symlink-install ``

> `` source ./install/setup.bash `` 

> `` source scripts/create_udev_rules.sh `` 
---
# How to run the work space
#### 1- Look for any "CHANGE" comment in the whole worksapce and follow the provided steps
#### 2- Build and source
>``colcon build``

>``source install/setup.bash``

#### 3- Calibration

>``ros2 launch control calibration.launch.py ``

#### 4- Launch the system 

> `` ros2 launch control system.launch.py ``

