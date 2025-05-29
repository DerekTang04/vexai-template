#pragma once

const int MOTOR_VOLTAGE_MAX = 12000;

const int OPTICAL_BLUE_MIN  = 80;
const int OPTICAL_BLUE_MAX  = 120;
const int OPTICAL_RED_MIN   = 10;
const int OPTICAL_RED_MAX   = 50;
const int OPTICAL_PROXY_MIN = 20;
const int OPTICAL_PROXY_MAX = 200;
const int OPTICAL_LED_PWM   = 50;
const int OPTICAL_INT_TIME  = 20;

const int CONVEYOR_STALL_CURRENT = 2450;
const int CONVEYOR_STALL_COUNT   = 25;

const float ARM_KP     = 1;
const float ARM_KI     = 0;
const float ARM_KD     = 1;
const int ARM_EXIT_ERR = 30;
const int ARM_LOAD_POS = 300;
const int ARM_HOLD_POS = 700;
const int ARM_NS_POS   = 1000;
const int ARM_AS_POS   = 1500;

enum Motion_e {
    IDLE = 0,
    HOLD_1,
    HOLD_2_ARM,
    HOLD_2,
    MOGO_LOAD,
    SCORE_1,
    SCORE_2,
    UNLOAD
};

enum OpticalChange_e {
    NONE = 0,
    DETECTED,
    UNDETECTED
};

enum ArmPosition_e {
    HOME = 0,
    LOAD,
    HOLD,
    NEUTRAL,
    ALLIANCE,
    TRANSITION
};

enum RingColour_e {
    NO_RING = 0,
    BLUE,
    RED
};

enum IntakePositon_e {
    ROLLER = 0,
    CONVEYOR
};

struct IntakeRing_s {
    RingColour_e colour;
    IntakePositon_e position;
};
