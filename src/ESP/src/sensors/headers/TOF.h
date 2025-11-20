#ifndef TOF_H
#define TOF_H

#include <vector>

class TOF {
public:
    TOF();
    void begin();
    std::vector<float> getFrontDistances();
    std::vector<float> getLeftDistances();
    std::vector<float> getRightDistances();
    std::vector<float> getBackDistances();

private:
    float distance_front;
    float distance_left;
    float distance_right;
    float distance_back;
};

#endif // TOF_H