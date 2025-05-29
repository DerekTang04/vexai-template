#include "rmech/executor.hpp"

void RingMechExecutor::idle() {
    controller->rollerTargetVoltage = 0;
    controller->conveyorTargetVoltage = 0;
    controller->armTargetPosition = HOME;
}