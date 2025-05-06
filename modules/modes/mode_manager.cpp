#include "mode_manager.h"


//Solo ejecuta la accion del estado actual.
static void _executeStateAction(ModeManager* manager) {
    switch (manager->currentMode) {
        case MODE_STOP:
            runStopMode(&manager->vel);
            break;
        case MODE_MANUAL:
            runManualMode(&manager->vel);
            break;
        case MODE_AUTOMATIC:
            runAutomaticMode(&manager->vel);
            break;
    }
}

void modeManagerInit(ModeManager* manager, OperationMode initialMode) {
    manager->currentMode = initialMode;
    runStopMode(&manager->vel);             //el struct robot_velocity_t ya esta creado en "manager".
}

void modeManagerRun(ModeManager* manager) {
    switch (manager->currentMode) {
        case MODE_STOP:
            if (isMoving(manager)== true) {
                manager->currentMode = MODE_MANUAL;
                //Para en un futuro agregar transicion automatica.
            }
            break;
        case MODE_MANUAL:
            //Para en un futuro agregar transicion automatica.    
            break;
        case MODE_AUTOMATIC:
            //Para en un futuro agregar transicion automatica.
            break;
    }
    _executeStateAction(manager);
}


void setMode(ModeManager* manager, OperationMode mode){
    manager->currentMode = mode;
}

void getMode(ModeManager* manager, OperationMode* mode){
    (*mode)      = manager->currentMode;
}

void getVelocity(ModeManager* manager, float* v, float* w){
    (*v)         = manager->vel.linealVelocity;
    (*w)         = manager->vel.angularVelocity;
}

bool isCloseLoop(ModeManager* manager){
    return manager->vel.closeLoop;

}

bool isMoving(ModeManager* manager){
    return (manager->vel.linealVelocity != 0.0f || manager->vel.angularVelocity != 0.0f);
}
//bool isMoving(ModeManager* manager) {
//    return (fabs(manager->vel.linealVelocity) > EPSILON || fabs(manager->vel.angularVelocity) > EPSILON);
//}

void info(ModeManager* manager, OperationMode* mode, float *v, float *w, bool *closeLoop){
    getMode(manager, mode);
    getVelocity(manager, v, w);
    (*closeLoop) =  isCloseLoop(manager);
}
