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
        std::vector<float> getRotations();

    private:
        float roll;
        float pitch;
        float yaw;
};

#endif // IMU_H
