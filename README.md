# 🔁 Persistent Environment Setup (Add to .bashrc)
>``sudo nano ~/.bashrc``
## Add the following lines at the end of the bashrc
#### 1- Set Python path for ASV (replace with the actual path to your ASV/src)
>``export PYTHONPATH=$PYTHONPATH:/home/<linux root username>/ASV/src``
#### 2- Source ROS 2 Humble 
>``source /opt/ros/humble/setup.bash``
#### 3- Source the workspace (replace with the actual path to your ASV/install/setup.bash) 
>``source /home/<linux root username>/ASV/install/setup.bash``
---
# How to run the work space
#### 1- Look for any "CHANGE" comment in the whole worksapce and follow the provided steps
#### 2- Build and source
>``colcon build``

>``source ASV/install/setup.bash``
