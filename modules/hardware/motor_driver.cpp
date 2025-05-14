#include "motor_driver.h"
#include "pin_out_default.h"
#include "robot_config.h"
#include "utils.h"

/*
static MotorDriver motor = {
    .pwm       = nullptr,
    .dutyCycle = 0.0f,
    .velocity  = 0.0f,
    .vMax      = V_MAX,  // m/s
    .vMin      = V_MIN,
    .state     = MOTOR_OFF,
    .elapsedMs = 0
};
*/

//static PwmOut pwmAux(ENCODER_LEFT_PIN); // pin válido temporal para inicializar (rompia que el pin no tiene autoinicializacion, usar clases...no se si sea mejor).

/**
 * @brief Establece la velocidad deseada y actualiza el estado del motor.
 * @param motor Puntero al motor.
 * @param v Velocidad deseada.
 */
void _motorSetVelocity(MotorDriver* motor, float *v)
{
    motor->velocity = *v;
    if (*v == 0.0f)
        motor->state = MOTOR_STOPPING;
    else
        motor->state = MOTOR_STARTING;
}

/**
 * @brief Inicializa el motor con valores por defecto y configura el PWM.
 * @param motor Puntero al motor.
 * @param pwmPin Pin físico de salida PWM.
 */
void motorInit(MotorDriver* motor, PinName pwmPin)
{
    motor->pwm.period(PWM_PERIOD);
    motor->pwm.write(0.0f);


    motor->dutyCycle = 0.0f;
    motor->velocity = 0.0f;
    motor->vMax = MAX_ANGULAR_VELOCITY  ;
    motor->vMin = V_MIN;
    motor->state = MOTOR_OFF;
    motor->elapsedMs = 0;
}


/**
 * @brief Aplica lógica de máquina de estados para actualizar el motor.
 * @param motor Puntero al motor.
 * @param v Velocidad deseada.
 */
void motorUpdate(MotorDriver* motor, float *v)
{
    _motorSetVelocity(motor, v);
    switch (motor->state) {
        case MOTOR_OFF:
            // Nada que hacer
            break;

        case MOTOR_STARTING:
            motor->dutyCycle = motor->velocity / motor->vMax;
            motor->dutyCycle = _min(_max(motor->dutyCycle, DUTY_MIN), DUTY_MAX);
            motor->pwm.write(motor->dutyCycle);
            motor->state = MOTOR_RUNNING;
            break;

        case MOTOR_RUNNING:
            motor->dutyCycle = motor->velocity / motor->vMax;
            motor->dutyCycle = _min(_max(motor->dutyCycle, DUTY_MIN), DUTY_MAX);
            motor->pwm.write(motor->dutyCycle);
            motor->state = MOTOR_RUNNING;
            break;

        case MOTOR_STOPPING:
            motor->pwm.write(0.0f);
            motor->dutyCycle = 0.0f;
            motor->state = MOTOR_OFF;
            break;
    }
}

/**
 * @brief Devuelve la velocidad actual del motor.
 * @param motor Puntero al motor.
 * @param vlecture Puntero donde se guarda la velocidad actual.
 */
void motorRead(MotorDriver* motor, float* vlecture)
{
    *vlecture = motor->velocity;
}

/**
 * @brief Detiene el motor cambiando su velocidad objetivo a cero.
 * @param motor Puntero al motor.
 */
void motorStop(MotorDriver* motor )
{
    float vStop = 0.0f;
    _motorSetVelocity(motor, &vStop);
}

//ref: Chapter 8 | Advanced Time Management, Pulse-Width Modulation ... pag 410

//void _motorControlPWM(MotorDriver* motor)
//{
    /*
    Compara el tiempo actual con el duty cycle y enciende o apaga el motor:
        - Si el tiempo actual está dentro del duty cycle, enciende el motor.
        - Si el tiempo actual supera el duty cycle, apaga el motor.

    Compara el tiempo actual con 
    D_L * T_PWM y D_R*T_PWM
    donde:
    D_L = |V_L|/V_L{max}
    D_R = |V_R|/V_R{max}
    */

    // Control PWM
    //*motor = (*timeElapsed < duty * PWM_PERIOD);
    //if (motor->elapsedMs < motor->dutyCycle * PWM_PERIOD)
    //    *(motor->pin) = ON;
    //else
    //    *(motor->pin)= OFF;

//}





