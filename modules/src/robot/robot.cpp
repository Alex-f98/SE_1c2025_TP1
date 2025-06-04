/**
 * @file robot.cpp
 * @brief Implementación del control principal del robot móvil diferencial.
 * 
 * Este archivo implementa el bucle principal, inicialización y lectura
 * de sensores, así como la lógica de control de motores y modos de operación.
 */

#include "robot.h"
#include "utils/debug.h"           ///< Herramientas para impresión por consola (debug)
#include "utils/utils.h"           ///< Funciones auxiliares de utilidad
#include "mbed.h"                  ///< Librería principal de Mbed
#include "arm_book_lib.h"          ///< Librería del libro de ARM para utilidades varias
#include "string.h"                ///< Funciones estándar de manipulación de cadenas
#include "encoder.h"               ///< Lectura de encoders de los motores
#include "motor_driver.h"          ///< Controladores de motores (PWM, dirección)
#include "emergency_button.h"      ///< Botón de parada de emergencia
#include "pin_out_default.h"       ///< Asignación de pines estándar
#include "robot_config.h"          ///< Parámetros del robot (distancias, radios, etc.)
#include "motor_control.h"         ///< Controladores PID de velocidad
#include "mode_manager.h"          ///< Manejo de modos automáticos/manuales
#include "modes.h"                 ///< Definiciones de los distintos modos
#include "kinematics.h"            ///< Modelo cinemático (diferencial, omnidireccional)

//=====[Variables globales]==================================================

/// @brief Estado del botón de parada de emergencia.
bool emergencyStop = OFF;

/// @brief Tiempo acumulado para debug (no usado aún).
int acumulatedTimeDebug = 0;

/// @brief Modo actual de operación.
OperationMode mode;

//=====[Objetos globales]====================================================

MotorController motorLeft(MOTOR_LEFT_PIN  , ENCODER_LEFT_PIN , KP, KI, KD, DT, TENCODER_UPDATE, PWM_PERIOD);
MotorController motorRight(MOTOR_RIGHT_PIN, ENCODER_RIGHT_PIN, KP, KI, KD, DT, TENCODER_UPDATE, PWM_PERIOD);

/// @brief Botón de emergencia.
DigitalIn buttonStop(BUTTON_STOP_PIN);

/// @brief Manejador del modo de operación.
ModeManager modeManager;


//=====[Funciones]===========================================================

/**
 * @brief Loop principal del robot.
 * 
 * Flujo de ejecución:
 * 1. Inicializa entradas/salidas.
 * 2. Inicializa cinemática.
 * 3. En bucle:
 *    - Verifica sensores
 *    - Actualiza modo
 *    - Obtiene velocidades deseadas
 *    - Calcula velocidades de rueda con modelo cinemático
 *    - Envía velocidades a motores (con o sin lazo cerrado)
 */
void robotLoop()
{
    inputsInit();
    outputsInit();

    Kinematics kin = {
        .type        = DIFFERENTIAL,
        .wheelRadius = WHEEL_DIAMETER/2.0,
        .wheelBase   = WHEEL_DISTANCE
    };

    Velocity vel = {
        .vx    = 0.0f,
        .vy    = 0.0f,
        .omega = 0.0f
    };

    WheelVelocities w;
    computeWheelVelocities(&kin, &vel, &w);

    while (true) {
        checkSensors();

        if (emergencyStop == ON){
            vel = {0.0f, 0.0f, 0.0f};
            setMode(&modeManager, MODE_STOP);
        }

        modeManagerRun(&modeManager);
        getVelocity(&modeManager, &(vel.vx), &(vel.omega));
        computeWheelVelocities(&kin, &vel, &w);

        motorLeft.enableClosedLoop(isCloseLoop(&modeManager));
        motorRight.enableClosedLoop(isCloseLoop(&modeManager));
        motorLeft.setTargetVelocity(w.w1);
        motorRight.setTargetVelocity(w.w2);

        //El update ahora se hacer con el ticker.
        //motorLeft.update();
        //motorRight.update();

        if (emergencyStop and isMoving(&modeManager))
            emergencyStop = OFF;

        delay(TIME_INCREMENT_MS);

        if (MODE_DEBUG == ON)
            //Seteo los parametros del control.
            float kc, ti, td, ts;
            kc = 0.0;
            ti = 0.0
            td = 0.0
            
            motorLeft.setMotorControlParameters(kc, ti, td, ts, W_MAX, W_MIN);
            motorRight.setMotorControlParameters(kc, ti, td, ts, W_MAX, W_MIN);
            //Imprimo variables.
            printStatus(&modeManager, &motorRight, &motorRight, emergencyStop);
    }
}


/**
 * @brief Inicializa entradas digitales.
 * 
 * Incluye:
 * - Botón de emergencia
 * - Modo inicial del manejador de modos
 */
void inputsInit()
{
    emergencyButtonInit();
    modeManagerInit(&modeManager, MODE_MANUAL);
}

/**
 * @brief Inicializa salidas digitales.
 * 
 * Actualmente no requiere configuración adicional, ya que los motores están
 * inicializados desde la declaración.
 */
void outputsInit()
{
    // Motores inicializados en declaración
}

/**
 * @brief Lee comandos por UART para alternar modo de operación.
 * 
 * Si se recibe el carácter 'm', alterna entre MODO_MANUAL y MODO_AUTOMATIC.
 * 
 * @param manager Puntero al gestor de modos.
 */
void checkMode(ModeManager* manager)
{
    char command = '\0';
    if (isReadableUart()){
        readUartCommand(&command);

        if (command == 'm') {
            getMode(manager, &mode);
            if (mode == MODE_MANUAL) {
                setMode(manager, MODE_AUTOMATIC);
            } else {
                setMode(manager, MODE_MANUAL);
            }
        }
    }
}

/**
 * @brief Verifica sensores críticos como botón de emergencia.
 * 
 * Si se detecta presión, activa el flag de parada de emergencia.
 */
void checkSensors()
{
    emergencyStop = emergencyButtonPressed();
}
