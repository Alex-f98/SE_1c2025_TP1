// debug_utils.h
#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

#include "motor_control.h"
#include "mode_manager.h"


void getWheelVelocities(MotorController* leftController,
                        MotorController* rightController,
                        float* vl, float* vr);

void printStatus(ModeManager* manager,
                 MotorController* leftController,
                 MotorController* rightController,
                 bool emergencyStop);


void observedState(MotorController* leftController, MotorController* rightController);

#endif
