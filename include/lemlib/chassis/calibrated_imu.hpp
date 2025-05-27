#pragma once

#include "pros/imu.hpp"

namespace lemlib
{

class CalibratedImu
{
private:
    pros::Imu sensor;

    // units: deg/rev
    float errorPerRev;

public:
    CalibratedImu(pros::Imu sensor, float errorPerRev);

    void calibrate(float initalHeading);

    float getTotalRotation();

    float getYawRate();
};

}
