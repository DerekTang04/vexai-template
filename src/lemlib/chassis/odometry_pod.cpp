#include <cmath>
#include "odometry_pod.hpp"

lemlib::OdometryPod::OdometryPod(pros::Rotation sensor, float wheelDiameter, float offset) :
    sensor(sensor), wheelDiameter(wheelDiameter), offset(offset) {}

void lemlib::OdometryPod::calibrate()
{
    sensor.reset();
}

float lemlib::OdometryPod::getDistance()
{
    return sensor.get_position() / 36000.0 * M_PI * wheelDiameter;
}

float lemlib::OdometryPod::getOffset()
{
    return offset;
}
