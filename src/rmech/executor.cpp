#include "rmech/executor.hpp"

static void runMotionEntry(void *params) {
    RingMechExecutor *e = static_cast<RingMechExecutor*>(params);
    e->runMotion();
    while(true) {
        pros::delay(UINT32_MAX);
    }
}

RingMechExecutor::RingMechExecutor(RingMechController *controller) : controller(controller) {
    executorTask = nullptr;

    currentMotion = IDLE;
    motionDone = true;
}

void RingMechExecutor::init() {
    controller->init();
    requestMotion(IDLE);
}

void RingMechExecutor::requestMotion(Motion_e motion) {
    if(executorTask != nullptr) {
        executorTask->remove();
    }
    currentMotion = motion;
    executorTask = std::make_unique<pros::Task>(runMotionEntry, this, "Motion Task");
}

bool RingMechExecutor::isDone() {
    return motionDone;
}

void RingMechExecutor::runMotion() {
    motionDone = false;

    switch(currentMotion) {
        case HOLD_1:
            hold1();
            break;
        case HOLD_2_ARM:
            hold2Arm();
            break;
        case HOLD_2:
            hold2();
            break;
        case MOGO_LOAD:
            mogoLoad();
            break;
        case SCORE_1:
            score1();
            break;
        case SCORE_2:
            score2();
            break;
        case UNLOAD:
            unload();
            break;
        default:
            idle();
    }
    
    motionDone = true;
}