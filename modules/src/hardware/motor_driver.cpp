#include "motor_driver.h"
#include "pin_out_default.h"
#include "robot_config.h"
#include "utils.h"


/**
 * @brief Establece la velocidad deseada y actualiza el estado del motor.
 * @param motor Puntero al motor.
 * @param w Velocidad angular deseada.
 */
void _motorSetVelocity(MotorDriver* motor, float *w)
{
    motor->velocity = *w;
    if (*w == 0.0f)
        motor->state = MOTOR_STOPPING;
    else
        motor->state = MOTOR_STARTING;
}

void setMotorPeriod(MotorDriver* motor, float period)
{
    motor->pwm.period(period);
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


    motor->dutyCycle    = 0.0f;
    motor->dutyCycleMax = DUTY_MAX;
    motor->dutyCycleMin = DUTY_MIN;
    motor->velocity     = 0.0f;
    motor->wMax         = W_MAX;
    motor->wMin         = W_MIN;
    motor->state        = MOTOR_OFF;
    motor->elapsedMs    = 0;
}


/**
 * @brief Aplica lógica de máquina de estados para actualizar el motor.
 * @param motor Puntero al motor.
 * @param w Velocidad angular deseada.
 */
void motorUpdate(MotorDriver* motor, float *w)
{
    _motorSetVelocity(motor, w);
    switch (motor->state) {
        case MOTOR_OFF:
            // Nada que hacer
            break;

        case MOTOR_STARTING:
            motor->dutyCycle = motor->velocity / motor->wMax;
            motor->dutyCycle = _min(_max(motor->dutyCycle, motor->dutyCycleMin), motor->dutyCycleMax);
            motor->pwm.write(motor->dutyCycle);
            motor->state = MOTOR_RUNNING;
            break;

        case MOTOR_RUNNING:
            motor->dutyCycle = motor->velocity / motor->wMax;
            motor->dutyCycle = _min(_max(motor->dutyCycle, motor->dutyCycleMin), motor->dutyCycleMax);
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
 * @brief Devuelve la velocidad angular actual del motor.
 * @param motor Puntero al motor.
 * @param wlecture Puntero donde se guarda la velocidad angular actual.
 */
void motorRead(MotorDriver* motor, float* wlecture)
{
    *wlecture = motor->velocity;
}

/**
 * @brief Detiene el motor cambiando su velocidad objetivo a cero.
 * @param motor Puntero al motor.
 */
void motorStop(MotorDriver* motor )
{
    float wStop = 0.0f;
    _motorSetVelocity(motor, &wStop);
}

void setMotorParameter(MotorDriver* motor, float wMaxIn, float wMinIn, float dutyCycleMaxOut, float dutyCycleMinOut)
{
    motor->wMax = wMaxIn;
    motor->wMin = wMinIn;
    motor->dutyCycleMax = dutyCycleMaxOut;
    motor->dutyCycleMin = dutyCycleMinOut;
}

//ref: Chapter 8 | Advanced Time Management, Pulse-Width Modulation ... pag 410





