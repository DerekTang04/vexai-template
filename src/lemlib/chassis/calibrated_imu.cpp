#include "calibrated_imu.hpp"

lemlib::CalibratedImu::CalibratedImu(pros::Imu sensor, float errorPerRev) :
    sensor(sensor), errorPerRev(errorPerRev) {}

void lemlib::CalibratedImu::calibrate(float initalHeading)
{
    sensor.reset(true);
    sensor.set_rotation(initalHeading * (1 - errorPerRev / 360));
}

float lemlib::CalibratedImu::getTotalRotation()
{
    return sensor.get_rotation() * 360 / (360 - errorPerRev);
}

float lemlib::CalibratedImu::getYawRate()
{
    return sensor.get_gyro_rate().z;
}
