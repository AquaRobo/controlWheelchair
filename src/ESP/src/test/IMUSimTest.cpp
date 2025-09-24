#include <iostream>
#include <vector>
#include "../sensors/headers/IMU.h"

IMU imuSim;

void testIMUSimulation() {
    imuSim.begin();
    while (true)  // Continuous loop to simulate ongoing IMU data retrieval
    {
        std::vector<float> rotations = imuSim.getRotations();
        std::cout << "Roll: " << rotations[0] << ", Pitch: " << rotations[1] << ", Yaw: " << rotations[2] << std::endl;
    }
}

int main() {
    testIMUSimulation();
    return 0;
}
