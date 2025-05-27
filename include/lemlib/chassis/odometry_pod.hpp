#pragma once

#include "pros/rotation.hpp"

namespace lemlib
{

class OdometryPod
{
private:
    pros::Rotation sensor;

    // units: in
    float wheelDiameter;
    // units: in
    float offset;

public:
    OdometryPod(pros::Rotation sensor, float wheelDiameter, float offset);

    void calibrate();

    float getDistance();

    float getOffset();
};

}