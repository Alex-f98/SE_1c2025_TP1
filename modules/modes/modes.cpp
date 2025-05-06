#include "modes.h"
#include "robot_config.h"
#include "utils/utils.h"



void runStopMode(robot_velocity_t* vel){              //ACA SE ROMPE, ENTRA A UNA VEL CON DIRECCION 0X0000 Y ROMPE TODO, ESTA MAL PASADO.
    vel->linealVelocity  = 0.0f;
    vel->angularVelocity = 0.0f;
    vel->closeLoop       = CLOSED_LOOP_CONTROL;
    vel->stop            = ON;
}

void runManualMode(robot_velocity_t* vel) {
     //v y w son globales.
    //vel.linealVelocityfloat v_L, v_R;
    char command = '\0';

    if (isReadableUart()) {
        readUartCommand(&command);
        switch (command) {
            case 'w':
                vel->linealVelocity = _min(vel->linealVelocity + V_STEP, V_MAX);
                break;
            case 's':
                vel->linealVelocity = _max(vel->linealVelocity - V_STEP, V_MIN);
                break;
            case 'a':
                vel->angularVelocity = _min(vel->angularVelocity - W_STEP, W_MIN);
                break;
            case 'd':
                vel->angularVelocity = _min(vel->angularVelocity + W_STEP, W_MAX);
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
    if (!(vel->stop)){
        vel->linealVelocity  = (rand() / (float)RAND_MAX) * V_MAX;
        vel->angularVelocity = (rand() / (float)RAND_MAX) * W_MAX;
    }
}
