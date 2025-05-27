// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include <math.h>
#include "pros/rtos.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/chassis.hpp"

// tracking thread
static pros::Task *trackingTask = nullptr;

// global variables
static lemlib::OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr);
static lemlib::Pose odomPose(0, 0, 0);

// pose mutex
static pros::Mutex poseMutex;

void lemlib::setSensors(lemlib::OdomSensors sensors) {
    odomSensors = sensors;
}

lemlib::Pose lemlib::getPose(bool radians) {
    lemlib::Pose tmp(0, 0, 0);
    
    poseMutex.lock();
    tmp = radians ? odomPose : lemlib::Pose(odomPose.x, odomPose.y, radToDeg(odomPose.theta));
    poseMutex.unlock();

    return tmp;
}

void lemlib::setPose(lemlib::Pose pose, bool radians) {
    poseMutex.lock();
    odomPose = radians ? pose : lemlib::Pose(pose.x, pose.y, degToRad(pose.theta));
    poseMutex.unlock();
}

void lemlib::update(void *params) {
    float prevRotation   = degToRad(odomSensors.imu->getTotalRotation());
    float prevHorizontal = odomSensors.hPod->getDistance();
    float prevVertical   = odomSensors.vPod->getDistance();

    uint32_t prevRunTime = pros::millis();
    while(true)
    {
        /* get sensor data*/
        // get sensor readings for odometry
        float rotationDeg = odomSensors.imu->getTotalRotation();
        float rotation    = degToRad(rotationDeg);
        float horizontal  = odomSensors.hPod->getDistance();
        float vertical    = odomSensors.vPod->getDistance();

        // get sensor readings for gps fusion
        float gyroYawRate  = odomSensors.imu->getYawRate();
        GpsPosition gpsPos = odomSensors.gps->getPosition(rotationDeg);

        /* math for odometry */
        // find deltas
        float deltaRotation   = rotation - prevRotation;
        float deltaHorizontal = horizontal - prevHorizontal;
        float deltaVertical   = vertical - prevVertical;

        // calculate local displacement
        float localX = 0;
        float localY = 0;
        if(deltaRotation == 0)
        {
            localX = deltaHorizontal;
            localY = deltaVertical;
        }
        else
        {
            localX = 2 * sin(deltaRotation / 2) * (deltaHorizontal / deltaRotation + odomSensors.hPod->getOffset());
            localY = 2 * sin(deltaRotation / 2) * (deltaVertical / deltaRotation + odomSensors.vPod->getOffset());
        }

        // calculate global displacement
        float dispDir = prevRotation + deltaRotation / 2;
        float dispX = localY * sin(dispDir) + localX * cos(dispDir);
        float dispY = localY * cos(dispDir) - localX * sin(dispDir);

        /* math for gps fusion */
        // find velocities and gain
        float linearVelocity  = sqrt( pow(dispX / 0.01, 2) + pow(dispY / 0.01, 2) );
        float angularVelocity = fabs(gyroYawRate);
        float gain = odomSensors.gps->getGain(linearVelocity, angularVelocity);

        /* updating the pose */
        poseMutex.lock();

        // predict position with odometry
        odomPose.x += dispX;
        odomPose.y += dispY;

        // innovate position with gps
        if(gpsPos.rmsError < 1 && gain > 0)
        {
            odomPose.x = odomPose.x + gain * (gpsPos.pos.x - odomPose.x);
            odomPose.y = odomPose.y + gain * (gpsPos.pos.y - odomPose.y);
        }
        
        // update heading
        odomPose.theta = sanitizeAngle(rotation);
        
        poseMutex.unlock();

        /* misc chores */
        prevRotation   = rotation; 
        prevHorizontal = horizontal;
        prevVertical   = vertical;

        pros::Task::delay_until(&prevRunTime, 10);
    }
}

void lemlib::init() {
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task(update, nullptr, "Tracking Task");
    }
}
