#include <cmath>
#include "ring_mech/ring_mech.hpp"

OpticalChange_e RingMech::getOpticalChange() {
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

ArmPosition_e RingMech::getArmPosition() {
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

void RingMech::updateState() {
    OpticalChange_e opDetect = getOpticalChange();
    pros::c::optical_direction_e_t gesture = optical->get_gesture();

    // handle changes at roller (back of deque)
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

    ArmPosition_e armPos = getArmPosition();

    if(armPos != LOAD && doAntiJam) {
        antiJam();
    }

    // handle changes at limit switch (front of deque)
    if(limit->get_new_press() && !intakeState.empty()) {
        IntakeRing_s firstRing = intakeState.front();
        
        if(armPos == LOAD) {
            armState = firstRing.colour;
        }
        else if(doColourSort) {
            colourSort(firstRing.colour);
        }
        else {
            ; // nothing else happens when limit is pressed 
        }

        intakeState.pop_front();
    }

    // handle arm state if scoring
    // note: assume that ring always exits when arm moves to these positions
    if(armPos == NEUTRAL || armPos == ALLIANCE) {
        armState = NO_RING;
    }
}
