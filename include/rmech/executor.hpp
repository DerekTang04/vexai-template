#pragma once

#include <memory>
#include "api.h"
#include "rmech/controller.hpp"
#include "rmech/rmech_defs.hpp"

class RingMechExecutor {
private:
    // low level controller
    RingMechController *controller;

    // task to run macros
    std::unique_ptr<pros::Task> executorTask;
    
    // macro progress tracking
    Motion_e currentMotion;
    bool motionDone;

    void hold1();

    void hold2Arm();

    void hold2();

    void idle();

    void mogoLoad();

    void score1();

    void score2();

    void unload();

public:
    RingMechExecutor(RingMechController *controller);

    void init();

    void requestMotion(Motion_e motion);

    bool isDone();

    // note: do not call directly
    void runMotion();
};
