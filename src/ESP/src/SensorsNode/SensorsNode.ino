#include "../sensors/headers/IMU.h"
#include "../sensors/simulation/IMUSIM.cpp"
#include <vector>
#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

IMU imu_sim;
std::vector<float> accelerations;
std::vector<float> gyroscope;
std::vector<float> quaternion;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t sensors_node;
rcl_timer_t timer;
constexpr uint32_t TIMER_TIMEOUT_MS = 5u;

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void pubIMU(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    accelerations = imu_sim.getAccelerations();
    gyroscope = imu_sim.getGyroscope();
    quaternion = imu_sim.getQuaternion();

    imu_msg.orientation.x = quaternion[0];
    imu_msg.orientation.y = quaternion[1];
    imu_msg.orientation.z = quaternion[2];
    imu_msg.orientation.w = quaternion[3];
    imu_msg.angular_velocity.x = gyroscope[0];
    imu_msg.angular_velocity.y = gyroscope[1];
    imu_msg.angular_velocity.z = gyroscope[2];
    imu_msg.linear_acceleration.x = accelerations[0];
    imu_msg.linear_acceleration.y = accelerations[1];
    imu_msg.linear_acceleration.z = accelerations[2];
    
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    }
}

void setup(){
    set_microros_transports();
    imu_sim.begin();
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&sensors_node, "sensors_node", "", &support);
    rclc_publisher_init_default(
    &imu_publisher,
    &sensors_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "IMU");
    rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(TIMER_TIMEOUT_MS),
    pubIMU);
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);
}

void loop(){
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(TIMER_TIMEOUT_MS)));
}

