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
#include "hardware/encoder.h"

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
     * @param kp Constante proporcional del PID.
     * @param ki Constante integral del PID.
     * @param kd Constante derivativa del PID.
     * @param dt Periodo de muestreo del PID (en segundos).
     */
    MotorController(PinName pwmPin, PinName encoderPin,
                    float kp, float ki, float kd, float dt);

    /**
     * @brief Habilita o deshabilita el control en lazo cerrado.
     * @param enable `true` para control cerrado, `false` para lazo abierto.
     */
    void enableClosedLoop(bool enable);

    /**
     * @brief Establece la velocidad deseada (setpoint).
     * @param reference Velocidad de referencia (rad/s).
     */
    void set_target_velocity(float reference);

    /**
     * @brief Obtiene la velocidad medida por el encoder.
     * @return Velocidad actual (rad/s).
     */
    float get_measured_velocity();

    /**
     * @brief Devuelve la velocidad objetivo.
     * @return Velocidad de referencia actual.
     */
    float get_target_velocity();

    /**
     * @brief Actualiza el estado del controlador (aplica control si está habilitado).
     */
    void update();

    /**
     * @brief Detiene el motor y reinicia el PID.
     */
    void stop();

private:
    MotorDriver motor;         ///< Instancia del driver de motor.
    PID pid;                   ///< Controlador PID.
    EncoderSpeeds encoder;     ///< Módulo de lectura de velocidad por encoder.
    PinName encoderPin;        ///< Pin asignado al encoder.
    float referenceSpeed;      ///< Velocidad deseada.
    bool controlClosedLoop;    ///< Bandera de control cerrado habilitado.
};

#endif // MOTOR_CONTROL_H
