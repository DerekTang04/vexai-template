#include "rmech/controller.hpp"

static void updateEntry(void *params) {
    RingMechController *c = static_cast<RingMechController*>(params);
    
    uint32_t lastRunTime = pros::millis();
    while(true) {
        c->update();
        pros::Task::delay_until(&lastRunTime, 10);
    }
}

OpticalChange_e RingMechController::getOpticalChange() {
    static bool isDetected = false;

    if(!isDetected) {
        if(optical->get_proximity() > OPTICAL_PROXY_MAX) {
            isDetected = true;
            return DETECTED;
        }
    }
    else {
        if(optical->get_proximity() < OPTICAL_PROXY_MIN) {
            isDetected = false;
            return UNDETECTED;
        }
    }

    return NONE;
}

void RingMechController::getConveyorStalled() {
    static int stallCount = 0;

    if(conveyorStalled) {
        if(conveyor->get_current_draw() < CONVEYOR_STALL_CURRENT) {
            stallCount = 0;
            conveyorStalled = false;
        }
        return;
    }

    if(conveyor->get_current_draw() > CONVEYOR_STALL_CURRENT) {
        ++stallCount;
        
        if(stallCount > CONVEYOR_STALL_COUNT) {
            conveyorStalled = true;
        }
    }
    else {
        stallCount = 0;
    }
}

ArmPosition_e RingMechController::getArmPositon() {
    float pos = arm->get_position();

    if(std::abs(pos) < ARM_EXIT_ERR) {
        return HOME;
    }
    else if(std::abs(pos - ARM_LOAD_POS) < ARM_EXIT_ERR) {
        return LOAD;
    }
    else if(std::abs(pos - ARM_HOLD_POS) < ARM_EXIT_ERR) {
        return HOLD;
    }
    else if(std::abs(pos - ARM_NS_POS) < ARM_EXIT_ERR) {
        return NEUTRAL;
    }
    else if(std::abs(pos - ARM_AS_POS) < ARM_EXIT_ERR) {
        return ALLIANCE;
    }
    else {
        return TRANSITION;
    }
}

void RingMechController::runAntiJam() {
    if(conveyorStalled) {
        conveyor->move_voltage(-MOTOR_VOLTAGE_MAX);
        pros::delay(65);
        conveyor->move_voltage(std::clamp(conveyorTargetVoltage, -MOTOR_VOLTAGE_MAX, MOTOR_VOLTAGE_MAX));
    }
}

void RingMechController::runColourSort(RingColour_e ringColour) {
    if(ringColour == sortOutColour) {
        conveyor->move_voltage(-MOTOR_VOLTAGE_MAX);
        pros::delay(300);
        conveyor->move_voltage(std::clamp(conveyorTargetVoltage, -MOTOR_VOLTAGE_MAX, MOTOR_VOLTAGE_MAX));
    }
}

void RingMechController::moveRoller() {
    roller->move_voltage(std::clamp(rollerTargetVoltage, -MOTOR_VOLTAGE_MAX, MOTOR_VOLTAGE_MAX));
}

void RingMechController::moveConveyor() {
    roller->move_voltage(std::clamp(conveyorTargetVoltage, -MOTOR_VOLTAGE_MAX, MOTOR_VOLTAGE_MAX));
}

void RingMechController::moveArm() {
    int targetCounts = 0;
    switch(armTargetPosition) {
        case LOAD:
            targetCounts = ARM_LOAD_POS;
            break;
        case HOLD:
            targetCounts = ARM_HOLD_POS;
            break;
        case NEUTRAL:
            targetCounts = ARM_NS_POS;
            break;
        case ALLIANCE:
            targetCounts = ARM_AS_POS;
            break;
        default:
            ; // move to home, targetCounts initialized to 0
    }

    int error = targetCounts - arm->get_position();
    if(std::abs(error) > ARM_EXIT_ERR) {
        int output = armController->update(error);
        arm->move_voltage(std::clamp(output, -MOTOR_VOLTAGE_MAX, MOTOR_VOLTAGE_MAX));
        armMoving = true;
    }
    else {
        armController->reset();
        arm->brake();
        armMoving = false;
    }
}

RingMechController::RingMechController(
    pros::Motor *roller, pros::Motor *conveyor, pros::Motor *arm, 
    pros::Optical *optical, pros::adi::DigitalIn *limit,
    lemlib::PID *armController
) :
    roller(roller), conveyor(conveyor), arm(arm),
    optical(optical), limit(limit),
    armController(armController)
{
    controllerTask = nullptr;

    intakeState.clear();
    armState = NO_RING;
    conveyorStalled = false;
    armMoving = false;

    sortOutColour = BLUE;
    doAntiJam = true;
    doColourSort = true;
    rollerTargetVoltage = 0;
    conveyorTargetVoltage = 0;
    armTargetPosition = HOME;
}

void RingMechController::init() {
    // set up optical sensor
    optical->enable_gesture();
    optical->set_integration_time(OPTICAL_INT_TIME);
    optical->set_led_pwm(OPTICAL_LED_PWM);

    // set motor brake modes
    roller->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    conveyor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    arm->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // move arm to bottom position
    arm->move_voltage(-MOTOR_VOLTAGE_MAX/5);
    pros::delay(1000);
    while(std::abs(arm->get_actual_velocity()) > 5) {
        pros::delay(10);
    }
    arm->brake();

    // zero arm encoder
    arm->tare_position();

    // start task
    controllerTask = std::make_unique<pros::Task>(updateEntry, this, "Ring Mech Controller Task");
}

std::deque<IntakeRing_s> RingMechController::getIntakeState() {
    return intakeState;
}

RingColour_e RingMechController::getArmState() {
    return armState;
}

bool RingMechController::isConveyorStalled() {
    return conveyorStalled;
}

bool RingMechController::isArmMoving() {
    return armMoving;
}

void RingMechController::update() {
    /* handle changes at bottom (back of deque) */
    OpticalChange_e opDetect = getOpticalChange();
    pros::c::optical_direction_e_t gesture = optical->get_gesture();

    if(opDetect == DETECTED) {
        if(gesture == pros::c::UP) {
            float hue = optical->get_hue();
            if(hue > OPTICAL_BLUE_MIN && hue < OPTICAL_BLUE_MAX) {
                intakeState.push_back({BLUE, ROLLER});
            }
            else if(hue > OPTICAL_RED_MIN && hue < OPTICAL_RED_MAX) {
                intakeState.push_back({RED, ROLLER});
            }
            else {
               ; // ignore other hues
            }
        }
        else if(gesture == pros::c::DOWN) {
            if(!intakeState.empty()) {
                intakeState.back().position = ROLLER;
            }
        }
        else {
            ; // ignore other gestures
        }
    }
    else if(opDetect == UNDETECTED) {
        if(gesture == pros::c::UP) {
            if(!intakeState.empty()) {
                intakeState.back().position = CONVEYOR;
            }
        }
        else if(gesture == pros::c::DOWN) {
            if(!intakeState.empty()) {
                intakeState.pop_back();
            }
        }
        else {
            ; // ignore other gestures
        }
    }
    else {
        ; // no ring / ring idle
    }
    
    /* handle changes at top (front of deque) */
    getConveyorStalled();

    if(getArmPositon() != LOAD) {
        if(limit->get_new_press() && !intakeState.empty())
        {
            if(doColourSort) {
                runColourSort(intakeState.front().colour);
            }
            intakeState.pop_front();
        }
        else {
            if(doAntiJam) {
                runAntiJam();
            }
        }
    }
    else {
        if(limit->get_new_press() && !intakeState.empty())
        {
            armState = intakeState.front().colour;
            intakeState.pop_front();
        }
    }

    /* move motors with request voltages */
    moveRoller();
    moveConveyor();
    moveArm();
}