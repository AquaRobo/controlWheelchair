#include <iostream>
#include <vector>
#include "../sensors/headers/IMU.h"

IMU imuSim;

void testAccelerometer() {
    imuSim.begin();
    std::vector<float> accelerations = imuSim.getAccelerations();
    std::cout << "Accel X: " << accelerations[0] << ", Accel Y: " << accelerations[1] << ", Accel Z: " << accelerations[2] << std::endl;
}

void testGyroscope() {
    imuSim.begin();
    std::vector<float> gyroscope = imuSim.getGyroscope();
    std::cout << "Gyro X: " << gyroscope[0] << ", Gyro Y: " << gyroscope[1] << ", Gyro Z: " << gyroscope[2] << std::endl;
}

void testQuaternion() {
    imuSim.begin();
    std::vector<float> quaternion = imuSim.getQuaternion();
    std::cout << "Quat X: " << quaternion[0] << ", Quat Y: " << quaternion[1] << ", Quat Z: " << quaternion[2] << ", Quat W: " << quaternion[3] << std::endl;
}

int main() {
    testAccelerometer();
    testGyroscope();
    testQuaternion();
    return 0;
}
