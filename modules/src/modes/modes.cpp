#include "modes.h"
#include "robot_config.h"
#include "utils/utils.h"



void runStopMode(robot_velocity_t* vel){   
    vel->linealVelocity  = 0.0f;
    vel->angularVelocity = 0.0f;
    vel->closeLoop       = CLOSED_LOOP_CONTROL;
    vel->stop            = ON;
}

bool runManualMode(robot_velocity_t* vel) {
    //Se modifican las velocidades del robot resspecto al centro de masas.
    char command = '\0';
    bool commandIssued = false; //Indica si se emitio un comando.

    if (isReadableUart()) {
        readUartCommand(&command);
        switch (command) {
            case 'w':
                vel->linealVelocity = _min(vel->linealVelocity + LINEAR_VELOCITY_STEP, MAX_LINEAR_VELOCITY);
                vel->stop = OFF;
                commandIssued = true;
                break;
            case 's':
                vel->linealVelocity = _max(vel->linealVelocity - LINEAR_VELOCITY_STEP, MIN_LINEAR_VELOCITY);
                vel->stop = OFF;
                commandIssued = true;
                break;
            case 'a':
                vel->angularVelocity = _min(vel->angularVelocity - YAW_VELOCITY_STEP, MIN_YAW_VELOCITY);
                vel->stop = OFF;
                commandIssued = true;
                break;
            case 'd':
                vel->angularVelocity = _min(vel->angularVelocity + YAW_VELOCITY_STEP, MAX_YAW_VELOCITY);
                vel->stop = OFF;
                commandIssued = true;
                break;
            case 'q':
                //Caso en donde cierro todo, por ahora paro motores en seco.
                runStopMode(vel);
                break;

            case 'c':
                vel->closeLoop = !(vel->closeLoop);
                char str[50];
                sprintf ( str, "CloseLoop: %d\r\n", vel->closeLoop );
                _printUart(str);
                break;
            default:
                availableCommands();
                break;
            }
    }
    return commandIssued;

}


void runAutomaticMode(robot_velocity_t* vel)
{
    // Implementar lógica de control autónomo aquí
    char command = '\0';

    if (isReadableUart()) {
        readUartCommand(&command);
        switch (command) {
            case 'q':
                //Caso en donde cierro todo, por ahora paro motores en seco.
                runStopMode(vel);
                break;

            case 'c':
                vel->closeLoop = !(vel->closeLoop);
                char str[50];
                sprintf ( str, "CloseLoop: %d\r\n", vel->closeLoop );
                _printUart(str);
                break;
            default:
                break;
        }
    }
    //Aca debe ir la logica para que el robot sea autonomo.
    if (!(vel->stop)){
        vel->linealVelocity  = (rand() / (float)RAND_MAX) * MAX_LINEAR_VELOCITY;
        vel->angularVelocity = (rand() / (float)RAND_MAX) * MAX_YAW_VELOCITY;
    }
}
