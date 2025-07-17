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

#define KC                     2.0f
#define TI                     0.0f
#define TD                     0.0f
#define TS                     0.02f //[s]:50hz tiempo de control PID, 

//=====[Wheel Velocity Limits]=======================================
#define W_MAX                  25.132f // (240RPM-6v) radianes por segundo
#define W_MIN                  0.0f    // radianes por segundo
#define W_STEP                 0.1f    // radianes por segundo

//=====[Robot Physical Parameters]=============================
#define RESOLUTION             20     // 20 ticks por revolución.
#define WHEEL_DISTANCE         0.07f  // L [m]
#define WHEEL_DIAMETER         0.069f // radio*2

/*
MAX_LINEAR_VELOCITY = ( W_MAX[rad/s] * (WHEEL_DIAMETER/2)[m]) = 25.132* 0.069/2 ~= 0.86
*/

#define MAX_LINEAR_VELOCITY    0.86f  // metros por segundo
#define MIN_LINEAR_VELOCITY    0.0f   // metros por segundo
#define LINEAR_VELOCITY_STEP   0.05f   // metros por segundo

/*
MAX_YAW_VELOCITY = W_MAX R/ L = 25.132* (0.069/2)/0.07 ~= 12.38 [rad/s] ...
*/
#define MAX_YAW_VELOCITY       12.38f   // radianes por segundo
#define MIN_YAW_VELOCITY       0.0f   // radianes por segundo
#define YAW_VELOCITY_STEP      0.1f   // radianes por segundo


//=====[Time Parameters s]=======================================
#define TIME_INCREMENT_MS      200 // 200ms=0.2s:[5hz]Tiermpo de incremento del sistema 
#define TMOTOR_UPDATE          0.2f //[s] Tiempo de actualizacion motor (hardware)
#define TENCODER_UPDATE        0.017f //[s]: 60hz  //Timpo de actualizacion de encoder (hardware)
#define PWM_PERIOD             0.1f //Periodo del PWM del motor
#define PWM_STEP               0.01f //Incremento del PWM del motor



//=====[PWM Duty Cycle Limits]=================================
#define DUTY_MIN               0.0f
#define DUTY_MAX               1.0f

#endif // _ROBOT_CONFIG_H_
