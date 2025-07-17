/**
 * @file motor_driver.h
 * @brief Módulo de control de bajo nivel para motores mediante PWM.
 *
 * Este módulo gestiona el estado de un motor mediante una máquina de estados finitos.
 * Se encarga de aplicar el PWM correspondiente según la velocidad deseada y el estado actual.
 *
 * ### Máquina de estados del motor:
 * @dot
 * digraph MotorStateMachine {
 *     rankdir=LR;
 *     MOTOR_OFF      -> MOTOR_STARTING [label="velocidad ≠ 0"];
 *     MOTOR_STARTING -> MOTOR_RUNNING  [label="PWM aplicado"];
 *     MOTOR_RUNNING  -> MOTOR_STOPPING [label="velocidad == 0"];
 *     MOTOR_STOPPING -> MOTOR_OFF      [label="PWM = 0"];
 * }
 * @enddot
 *
 * | Estado           | Descripción                        | Transición si...                       | Próximo estado   |
 * | ---------------- | ---------------------------------- | -------------------------------------- | ---------------- |
 * | `MOTOR_OFF`      | Motor detenido                     | Se llama a `motor_set_velocity(v ≠ 0)` | `MOTOR_STARTING` |
 * | `MOTOR_STARTING` | Se habilita la salida PWM, acelera | PWM aplicado correctamente             | `MOTOR_RUNNING`  |
 * | `MOTOR_RUNNING`  | Motor en funcionamiento estable    | Se llama a `motor_set_velocity(0)`     | `MOTOR_STOPPING` |
 * | `MOTOR_STOPPING` | Se baja el PWM a cero              | PWM llegó a 0                          | `MOTOR_OFF`      |
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "pin_out_default.h"
#include "robot_config.h"
#include "arm_book_lib.h"
#include "mbed.h"

/**
 * @enum MotorState
 * @brief Estados posibles del motor.
 */
typedef enum {
    MOTOR_OFF,      ///< Motor detenido
    MOTOR_STARTING, ///< Inicializando PWM para arrancar
    MOTOR_RUNNING,  ///< Motor girando normalmente
    MOTOR_STOPPING  ///< Disminuyendo PWM hasta detenerse
} MotorState;

/**
 * @struct MotorDriver
 * @brief Estructura que representa un motor controlado por PWM.
 */
struct MotorDriver {
    PwmOut pwm;          ///< Pin de salida PWM
    float dutyCycle;     ///< Ciclo de trabajo actual
    float dutyCycleMax;  ///< Maximo valor del ciclo de trabajo
    float dutyCycleMin;  ///< Minimo valor del ciclo de trabajo
    float velocity;      ///< Velocidad actual en m/s
    float wMax;          ///< Velocidad máxima permitida
    float wMin;          ///< Velocidad mínima permitida
    MotorState state;    ///< Estado actual del motor
    int elapsedMs;       ///< Tiempo transcurrido (para PWM manual si se usa)

    /**
     * @brief Constructor que inicializa el PWM y estado del motor.
     * @param pwmPin Pin físico para PWM.
     */
    MotorDriver(PinName pwmPin)
        : pwm(pwmPin)
    {
        pwm.period(PWM_PERIOD);
        pwm.write(0.0f);
        dutyCycle    = 0.0f;
        dutyCycleMax = DUTY_MAX;
        dutyCycleMin = DUTY_MIN;
        velocity     = 0.0f;
        wMax         = W_MAX;
        wMin         = W_MIN;
        state        = MOTOR_OFF;
        elapsedMs    = 0;
    }
};

/**
 * @brief Inicializa un objeto MotorDriver.
 * @param motor Puntero al objeto motor a inicializar.
 * @param pwmPin Pin físico de salida PWM.
 */
void motorInit(MotorDriver* motor, PinName pwmPin);

/**
 * @brief Actualiza el estado del motor según la velocidad deseada.
 * @param motor Puntero al motor.
 * @param velocity Velocidad angular deseada en rad/s.
 */
void motorUpdate(MotorDriver* motor, float* velocity);

/**
 * @brief Lee la velocidad angular actual almacenada en el motor.
 * @param motor Puntero al motor.
 * @param wlecture Puntero donde se almacena la velocidad actual.
 */
void motorRead(MotorDriver* motor, float* wlecture);

/**
 * @brief Detiene el motor de forma segura (cambia el estado).
 * @param motor Puntero al motor.
 */
void motorStop(MotorDriver* motor);

/**
 * @brief Define los parámetros del motor.
 */
void setMotorParameter(MotorDriver* motor, float wMaxIn, float wMinIn, float dutyCycleMaxOut, float dutyCycleMinOut);

/**
 * @brief Define el periodo del PWM del motor.
 * @param motor Puntero al motor.
 * @param period Periodo del PWM en segundos.
 */
void setMotorPeriod(MotorDriver* motor, float period);

#endif
