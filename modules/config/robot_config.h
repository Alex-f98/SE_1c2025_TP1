#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

#include "arm_book_lib.h"

//=====[Mode Configuration]====================================
#define MODE_DEBUG             ON
#define TIME_DEBUG             1000
#define CURSOR_HOME            "\033[H"

//=====[Operation Modes]=======================================
#define MANUAL_MODE            0 //no se usa
#define AUTOMATIC_MODE         1 //no se usa

//=====[Control Flags]=========================================
#define CLOSED_LOOP_CONTROL    OFF

//=====[Control Parameters]====================================
#define KP                     2.0f //1.0f
#define KI                     0.05f //0.1f
#define KD                     0.01f
#define DT                     0.01f //PWM_STEP

//=====[Velocity Limits]=======================================
#define V_MAX                  1.0f
#define W_MAX                  1.0f
#define V_MIN                  0.0f
#define W_MIN                  0.0f
#define V_STEP                 0.1f
#define W_STEP                 0.1f

//=====[Robot Physical Parameters]=============================
#define WHEEL_DISTANCE         0.07f  //L
#define WHEEL_DIAMETER         0.069f //radio*2
#define MAX_LINEAR_VELOCITY  0.625f  // metros por segundo
#define MAX_ANGULAR_VELOCITY  8.93f  // radianes por segundo


//=====[Time Parameters ms]=======================================
#define TIME_INCREMENT_MS      10  //Tiermpo de incremento del sistema 
#define TMOTOR_UPDATE          1000 //Tiempo de actualizacion motor (hardware)
#define TENCODER_UPDATE        100  //Timpo de actualizacion de encoder (hardware)
#define PWM_PERIOD             0.1f
#define PWM_STEP               0.01f



//=====[PWM Duty Cycle Limits]=================================
#define DUTY_MIN               0.0f
#define DUTY_MAX               1.0f

#endif // _ROBOT_CONFIG_H_
