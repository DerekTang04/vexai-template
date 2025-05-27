#pragma once

#include <deque>
#include "lemlib/pid.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "ring_mech/ring_mech_defs.hpp"

class RingMech {
private:
    // hardware     
    pros::Motor *roller;
    pros::Motor *conveyor;
    pros::Motor *arm;
    pros::Optical *optical;
    pros::adi::DigitalIn *limit;
    // arm pid
    lemlib::PID armController;
    // state management
    std::deque<IntakeRing_s> intakeState;
    RingColour_e armState;

    /* state tracking functions */
    OpticalChange_e getOpticalChange();

    ArmPosition_e getArmPosition();

    void updateState();

    /* utility functions */
    void moveArm();

    void antiJam();

    void colourSort(RingColour_e ringColour);

    /* motion functions */
    void executeMotion();

    MotionStatus_e hold1();

    MotionStatus_e hold2Arm();

    MotionStatus_e hold2();

    MotionStatus_e mogoLoad();

    MotionStatus_e score1();

    MotionStatus_e score2();

    MotionStatus_e unload();

public:
    // task
    pros::Task *task;
    // utility control
    ArmPosition_e armTargetPosition;
    RingColour_e sortOutColour;
    bool doAntiJam;
    bool doColourSort;
    // motion control
    Motion_e selectedMotion;
    MotionStatus_e motionStatus;

    RingMech(
        pros::Motor *roller, pros::Motor *conveyor, pros::Motor *arm, 
        pros::Optical *optical, pros::adi::DigitalIn* limit
    );

    void init();

    void mainLoop();
};
