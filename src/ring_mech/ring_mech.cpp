#include "ring_mech/ring_mech.hpp"

static void mainLoopEntry(void *params) {
    RingMech *ringMech = static_cast<RingMech*>(params);
    ringMech->mainLoop();
}

void RingMech::executeMotion() {
    MotionStatus_e status = MotionStatus_e::DONE;

    switch(selectedMotion) {
        case Motion_e::HOLD_1:
            status = hold1();
            break;
        case Motion_e::HOLD_2_ARM:
            status = hold2Arm();
            break;
        case Motion_e::HOLD_2:
            status = hold2();
            break;
        case Motion_e::MOGO_LOAD:
            status = mogoLoad();
            break;
        case Motion_e::SCORE_1:
            status = score1();
            break;
        case Motion_e::SCORE_2:
            status = score2();
            break;
        case Motion_e::UNLOAD:
            status = unload();
            break;
        default:
            ; // do nothing, motion is done
    }

    motionStatus = status;
}

RingMech::RingMech(
    pros::Motor *roller, pros::Motor *conveyor, pros::Motor *arm, 
    pros::Optical *optical, pros::adi::DigitalIn* limit
) :
    roller(roller), conveyor(conveyor), arm(arm),
    optical(optical), limit(limit), 
    armController(lemlib::PID(ARM_KP, ARM_KI, ARM_KD))
{
    intakeState.clear();
    armState = RingColour_e::NONE;

    task = nullptr;

    armTargetPosition = ArmPosition_e::HOME;
    sortOutColour = RingColour_e::BLUE;
    doAntiJam = true;
    doColourSort = true;

    selectedMotion = Motion_e::IDLE;
    motionStatus = MotionStatus_e::DONE;
}

void RingMech::init() {
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
        ; // wait for arm settled
    }
    arm->brake();

    // zero arm encoder
    arm->tare_position();

    // start task
    task = new pros::Task(mainLoopEntry, this, "Ring Mech Task");
}

void RingMech::mainLoop() {
    uint32_t lastRunTime = pros::millis();
    while(true) {
        updateState();
        executeMotion();
        moveArm();

        pros::Task::delay_until(&lastRunTime, 10);
    }
}