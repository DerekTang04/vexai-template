#pragma once

#include "pros/gps.hpp"

#define INCHES_TO_METERS(INCHES) ( (INCHES) / 39.3701f )
#define METERS_TO_INCHES(METERS) ( (METERS) * 39.3701f )

namespace lemlib
{

typedef struct
{
    pros::gps_position_s_t pos;
    float rmsError; 
} GpsPosition;

class CalibratedGps
{   
private:
    pros::Gps sensor;

    // units: in
    float xOffset;
    // units: in
    float yOffset;
    // units: deg
    float headingOffset;
    // units: in/s
    float linearVelocityLimit;
    // units: deg/s
    float angularVelocityLimit;

    pros::gps_position_s_t transformOffsets(float heading);

public:
    CalibratedGps(pros::Gps gps, float xOffset, float yOffset, float headingOffset, float linearVelocityLimit, float angularVelocityLimit);

    void calibrate(float xInitial, float yInitial, float heading);

    GpsPosition getPosition(float heading);

    float getGain(float linearVelocity, float angularVelocity);
};

}