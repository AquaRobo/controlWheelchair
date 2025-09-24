#include "../headers/IMU.h"

IMU::IMU(){
    this->quat_x = 0.0f;
    this->quat_y = 0.0f;
    this->quat_z = 0.0f;
    this->quat_w = 0.0f;
    this->accel_x = 0.0f;
    this->accel_y = 0.0f;
    this->accel_z = 0.0f;
    this->gyro_x = 0.0f;
    this->gyro_y = 0.0f;
    this->gyro_z = 0.0f;
}

void IMU::begin() {
    std::cout << "IMU simulation started." << std::endl;
}

std::vector<float> IMU::getAccelerations() {
    this->accel_x = rand() % 361 - 180;
    this->accel_y = rand() % 361 - 180;
    this->accel_z = rand() % 361 - 180;
    return {this->accel_x, this->accel_y, this->accel_z};
}

std::vector<float> IMU::getGyroscope() {
    this->gyro_x = rand() % 361 - 180;
    this->gyro_y = rand() % 361 - 180;
    this->gyro_z = rand() % 361 - 180;
    return {this->gyro_x, this->gyro_y, this->gyro_z};
}
std::vector<float> IMU::getQuaternion() {
    this->quat_x = rand() % 361 - 180;
    this->quat_y = rand() % 361 - 180;
    this->quat_z = rand() % 361 - 180;
    this->quat_w = rand() % 361 - 180;
    return {this->quat_x, this->quat_y, this->quat_z, this->quat_w};
}