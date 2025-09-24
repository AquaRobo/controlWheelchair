#include <iostream>
#include <vector>
#include <gtest/gtest.h>
#include "../sensors/headers/IMU.h"

using namespace std;

TEST(IMUSimulationTest, INITIALIZATION) {
    IMU imuSim;
    imuSim.begin();
    SUCCEED(); // If begin() does not throw, the test passes
}

TEST(IMUSimulationTest, ACCELERATIONS) {
    IMU imuSim;
    imuSim.begin();
    vector<float> accelerations = imuSim.getAccelerations();

    // Check that the returned vector has exactly 3 elements
    ASSERT_EQ(accelerations.size(), 3);

    // Check that each acceleration value is within the expected range [-180, 180]
    for (const auto& acceleration : accelerations) {
        EXPECT_GE(acceleration, -180.0f);
        EXPECT_LE(acceleration, 180.0f);
    }
}

TEST(IMUSimulationTest, GYROSCOPE) {
    IMU imuSim;
    imuSim.begin();
    vector<float> gyroscope = imuSim.getGyroscope();

    // Check that the returned vector has exactly 3 elements
    ASSERT_EQ(gyroscope.size(), 3);

    // Check that each gyroscope value is within the expected range [-180, 180]
    for (const auto& gyro : gyroscope) {
        EXPECT_GE(gyro, -180.0f);
        EXPECT_LE(gyro, 180.0f);
    }
}

TEST(IMUSimulationTest, QUATERNION) {
    IMU imuSim;
    imuSim.begin();
    vector<float> quaternion = imuSim.getQuaternion();

    // Check that the returned vector has exactly 4 elements
    ASSERT_EQ(quaternion.size(), 4);

    // Check that each quaternion value is within the expected range [-180, 180]
    for (const auto& quat : quaternion) {
        EXPECT_GE(quat, -180.0f);
        EXPECT_LE(quat, 180.0f);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


