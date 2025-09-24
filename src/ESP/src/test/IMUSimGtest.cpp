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

TEST(IMUSimulationTest, ROTATIONS) {
    IMU imuSim;
    imuSim.begin();
    vector<float> rotations = imuSim.getRotations();
    
    // Check that the returned vector has exactly 3 elements
    ASSERT_EQ(rotations.size(), 3);
    
    // Check that each rotation value is within the expected range [-180, 180]
    for (const auto& rotation : rotations) {
        EXPECT_GE(rotation, -180.0f);
        EXPECT_LE(rotation, 180.0f);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


