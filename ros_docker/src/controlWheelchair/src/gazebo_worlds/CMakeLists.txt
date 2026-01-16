cmake_minimum_required(VERSION 3.8)
project(gazebo_worlds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(xacro REQUIRED)
find_package(rviz2 REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(
  DIRECTORY models photos worlds 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
