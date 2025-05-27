#include "ring_mech/ring_mech.hpp"

void RingMech::moveArm() {
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
        int output = armController.update(error);
        arm->move_voltage(output);
    }
    else {
        armController.reset();
        arm->brake();
    }
}

void RingMech::antiJam() {
    static int stallCount = 0;

    if(conveyor->get_current_draw() > CONVEYOR_STALL_CURRENT) {
        ++stallCount;
        
        if(stallCount > CONVEYOR_STALL_COUNT) {
            conveyor->move_voltage(-MOTOR_VOLTAGE_MAX);
            pros::delay(65);
            conveyor->move_voltage(MOTOR_VOLTAGE_MAX);    
            stallCount = 0;
        }
    }
    else {
        stallCount = 0;
    }
}

void RingMech::colourSort(RingColour_e ringColour) {
    if(ringColour == sortOutColour) {
        conveyor->move_voltage(-MOTOR_VOLTAGE_MAX);
        pros::delay(300);
        conveyor->move_voltage(MOTOR_VOLTAGE_MAX);
    }
}
