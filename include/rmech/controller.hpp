#pragma once

#include <deque>
#include <memory>
#include "api.h"
#include "lemlib/pid.hpp"
#include "rmech/rmech_defs.hpp"

class RingMechController {
private:
    // hardware
    pros::Motor *roller;
    pros::Motor *conveyor;
    pros::Motor *arm;
    pros::Optical *optical;
    pros::adi::DigitalIn *limit;

    // arm pid controller
    lemlib::PID *armController;

    // task to update system
    std::unique_ptr<pros::Task> controllerTask;
    
    // state tracking
    std::deque<IntakeRing_s> intakeState;
    RingColour_e armState;
    bool conveyorStalled;
    bool armMoving;

    OpticalChange_e getOpticalChange();

    void getConveyorStalled();

    ArmPosition_e getArmPositon();

    void runAntiJam();

    void runColourSort(RingColour_e ringColour);

    void moveRoller();

    void moveConveyor();

    void moveArm();

public:
    RingColour_e sortOutColour;
    bool doAntiJam;
    bool doColourSort;
    int rollerTargetVoltage;
    int conveyorTargetVoltage;
    ArmPosition_e armTargetPosition;

    RingMechController(
        pros::Motor *roller, pros::Motor *conveyor, pros::Motor *arm, 
        pros::Optical *optical, pros::adi::DigitalIn *limit,
        lemlib::PID *armController
    );

    void init();

    std::deque<IntakeRing_s> getIntakeState();

    RingColour_e getArmState();
    
    bool isConveyorStalled();
    
    bool isArmMoving();

    // note: do not call directly
    void update();
};
