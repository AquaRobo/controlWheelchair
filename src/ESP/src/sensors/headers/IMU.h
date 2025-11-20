#ifndef IMU_H
#define IMU_H

#include <vector>
#include <cstdint>
#include <iostream>
#include <stdlib.h>


class IMU {
    public:
        IMU();
        void begin();
        std::vector<float> getAccelerations();
        std::vector<float> getGyroscope();
        std::vector<float> getQuaternion();

    private:
        float accel_x;
        float accel_y;
        float accel_z;
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float quat_x;
        float quat_y;
        float quat_z;
        float quat_w;
};

#endif // IMU_H
