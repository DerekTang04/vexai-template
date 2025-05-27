#include <cmath>
#include "calibrated_gps.hpp"
#include "lemlib/util.hpp"

pros::gps_position_s_t lemlib::CalibratedGps::transformOffsets(float heading)
{
    pros::gps_position_s_t transformed = {};
    
    float headingRad = degToRad(heading);

    transformed.x = yOffset * sin(headingRad) + xOffset * cos(headingRad);
    transformed.y = yOffset * cos(headingRad) - xOffset * sin(headingRad);
    
    return transformed;
}

lemlib::CalibratedGps::CalibratedGps(pros::Gps sensor, 
    float xOffset, float yOffset, float headingOffset, 
    float linearVelocityLimit, float angularVelocityLimit) :
    sensor(sensor), 
    xOffset(xOffset), yOffset(yOffset), headingOffset(headingOffset),
    linearVelocityLimit(linearVelocityLimit), angularVelocityLimit(angularVelocityLimit) {}

void lemlib::CalibratedGps::calibrate(float xInitial, float yInitial, float headingInitial)
{
    pros::gps_position_s_t offsetGlobal = transformOffsets(headingInitial);

    float heading = sanitizeAngle(headingInitial + headingOffset, false);

    sensor.set_position(
        INCHES_TO_METERS(xInitial + offsetGlobal.x), 
        INCHES_TO_METERS(yInitial + offsetGlobal.y), 
        heading
    );
}

lemlib::GpsPosition lemlib::CalibratedGps::getPosition(float robotHeading)
{
    pros::gps_position_s_t gpsReading = sensor.get_position();
    float rmsReading = sensor.get_error();
    
    float heading = sanitizeAngle(robotHeading, false);
    pros::gps_position_s_t offsetGlobal = transformOffsets(heading);
    pros::gps_position_s_t robotPos = {
        METERS_TO_INCHES(gpsReading.x) - offsetGlobal.x,
        METERS_TO_INCHES(gpsReading.y) - offsetGlobal.y
    };

    return {robotPos, METERS_TO_INCHES(rmsReading)};
}

float lemlib::CalibratedGps::getGain(float linearVelocity, float angularVelocity)
{
    float rawGain = 1 
        - pow(linearVelocity / linearVelocityLimit, 2)
        - pow(angularVelocity / angularVelocityLimit, 2);

    if(rawGain > 0)
    {
        return 0.03 * rawGain;
    }
    else
    {
        return 0;
    }
}
