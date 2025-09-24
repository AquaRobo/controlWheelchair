#include "../headers/IMU.h"

IMU::IMU(){
    this->roll = 0.0f;
    this->pitch = 0.0f;
    this->yaw = 0.0f;
}

void IMU::begin() {
    std::cout << "IMU simulation started." << std::endl;
}

std::vector<float> IMU::getRotations() {
    this->yaw = rand() % 361 - 180;
    this->pitch = rand() % 361 - 180;
    this->roll = rand() % 361 - 180;
    return {this->roll, this->pitch, this->yaw};
}