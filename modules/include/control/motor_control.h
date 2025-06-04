/**
 * @file motor_control.h
 * @brief Módulo de control de motor en lazo abierto o cerrado con PID.
 * 
 * Este módulo encapsula el control de un motor mediante control en lazo abierto o cerrado.
 * Usa un encoder para la realimentación de velocidad y un PID para el ajuste dinámico.
 * Contiene una pequeña máquina de estados implícita según si se habilita control cerrado o no.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "mbed.h"
#include "pid.h"
#include "hardware/motor_driver.h"
#include "hardware/encoderVelocity.h"

/**
 * @class MotorController
 * @brief Controlador de motor que puede operar en lazo abierto o cerrado.
 */
class MotorController {
public:
    /**
     * @brief Constructor del controlador de motor.
     * @param pwmPin Pin para señal PWM.
     * @param encoderPin Pin conectado al encoder.
     * @param kc Constante de control del PID.
     * @param ti Constante de integración del PID.
     * @param td Constante de derivación del PID.
     * @param ts Periodo de muestreo del PID (en segundos).
     */
    MotorController(PinName pwmPin, PinName encoderPin,
                    float kc = KC , float ti = TI, float td = TD, float ts = TS,
                    float encoderInterval = TENCODER_UPDATE,
                    float motorPwmPeriod = PWM_PERIOD, float motorInterval = TMOTOR_UPDATE);
    ~MotorController();

    /**
     * @brief Habilita o deshabilita el control en lazo cerrado.
     * @param enable `true` para control cerrado, `false` para lazo abierto.
     */
    void enableClosedLoop(bool enable);

    /**
     * @brief Establece la velocidad deseada (setpoint).
     * @param reference Velocidad de referencia (rad/s).
     */
    void setTargetVelocity(float reference);
    
    /**
     * @brief Define los parámetros del controlador PID.
     */
    void setMotorControlParameters(float kc, float ti, float td, float ts, float wMaxIn, float wMinIn);
    
    /**
     * @brief Define los parámetros del encoder.
     */
    void setEncoderParameters(int resolution, float samplingInterval);
    
    /**
     * @brief Define los parámetros del motor.
     * @param wMaxIn Velocidad máxima del motor (rad/s).
     * @param wMinIn Velocidad mínima del motor (rad/s).
     * @param dutyCycleMaxOut Duty cycle máximo del motor (0.0f a 1.0f).
     * @param dutyCycleMinOut Duty cycle mínimo del motor (0.0f a 1.0f).
     * @param motorPwmPeriod Periodo del PWM del motor (en segundos).
     */
    void setMotorParameters(float wMaxIn, float wMinIn, float dutyCycleMaxOut, float dutyCycleMinOut, float motorPwmPeriod);

    /**
     * @brief Obtiene la velocidad medida por el encoder.
     * @return Velocidad actual (rad/s).
     */
    float getMeasuredVelocity();

    /**
     * @brief Obtiene la posición angular medida por el encoder.
     * @return Posición angular actual (rad).
     */
    float getAngularPosition();

    /**
     * @brief Obtiene la velocidad medida por el encoder.
     * @return Velocidad actual (rad/s).
     */
    int getMeasuredRPM();

    /**
     * @brief Devuelve la velocidad objetivo.
     * @return Velocidad de referencia actual.
     */
    float getTargetVelocity();

    /**
     * @brief Actualiza el estado del controlador (aplica control si está habilitado).
     */
    void controlUpdateDC();
    
    /**
     * @brief Actualiza el estado del motor (aplica control si está habilitado).
     */
    void motorUpdateDC();


    /**
     * @brief Detiene el motor y reinicia el PID.
     */
    void stop();

private:
    MotorDriver motor_;         ///< Instancia del driver de motor.
    PID pid_;                   ///< Controlador PID.
    EncoderVelocity* encoder_;   ///< Módulo de lectura de velocidad por encoder.
    PinName encoderPin_;        ///< Pin asignado al encoder.
    Ticker _cTicker;            ///< Ticker para controlar el periodo de muestreo del PID.
    Ticker _mTicker;            ///< Ticker para control el tiempo de actualizacion del motor.
    float controlOutput_;       ///< Velocidad de control.
    float referenceVelocity_;   ///< Velocidad deseada.
    bool controlClosedLoop_;    ///< Bandera de control cerrado habilitado.
    float kc_;
    float ti_;
    float td_;
    float ts_;
};

#endif // MOTOR_CONTROL_H
