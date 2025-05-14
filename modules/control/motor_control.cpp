/**
 * @file motor_control.cpp
 * @brief Implementación del controlador de motor (lazo abierto/cerrado).
 */

#include "motor_control.h"

/**
 * @brief Constructor de MotorController. Inicializa motor, encoder y PID.
 */
MotorController::MotorController(PinName pwmPin, PinName encoderPin,
                                 float kp, float ki, float kd, float dt)
    : motor(pwmPin) 
    , pid(kp, ki, kd, dt)
    , encoder(encoderPin)
    , encoderPin(encoderPin)
{
    //motor = new MotorDriver();
    motorInit(&motor, pwmPin);
    referenceSpeed = 0.0f;
    controlClosedLoop = CLOSED_LOOP_CONTROL;
    pid.setSaturation(V_MIN, MAX_ANGULAR_VELOCITY);
}

/**
 * @brief Activa o desactiva el control en lazo cerrado.
 */
void MotorController::enableClosedLoop(bool enable)
{
    controlClosedLoop = enable;
    //if (controlClosedLoop)
    //    encoderInit(&encoder, encoderPin);
}

/**
 * @brief Define la velocidad deseada del motor.
 */
void MotorController::set_target_velocity(float reference)
{
    referenceSpeed = reference;
}

/**
 * @brief Devuelve la velocidad observada por el encoder.
 */
float MotorController::get_measured_velocity()
{
    if (!controlClosedLoop)
        return referenceSpeed;

    float observedVelocity = 0.0f;
    encoderUpdate(&encoder);
    encoderRead(&encoder, &observedVelocity);
    return observedVelocity;
}

/**
 * @brief Retorna la velocidad objetivo configurada.
 */
float MotorController::get_target_velocity(){ 
    return referenceSpeed; 
}

/**
 * @brief Ejecuta una iteración del controlador, ya sea abierto o cerrado.
 */
void MotorController::update()
{
    if (controlClosedLoop) {
        float observedVelocity = 0.0f;
        encoderUpdate(&encoder);
        encoderRead(&encoder, &observedVelocity);
        float controlOutput = pid.compute(referenceSpeed, observedVelocity);
        motorUpdate(&motor, &controlOutput);
    } else {
        motorUpdate(&motor, &referenceSpeed);
    }
}

/**
 * @brief Detiene el motor, reinicia el PID y la velocidad objetivo.
 */
void MotorController::stop()
{
    if (controlClosedLoop)
        pid.reset();
    motorStop(&motor);
    encoderReset(&encoder);
    referenceSpeed = 0.0f;
}
