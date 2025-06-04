/**
 * @file encoder.h
 * @brief Módulo para el manejo de un encoder óptico y estimación de velocidad angular.
 *
 * Este archivo define las estructuras y funciones necesarias para procesar señales
 * provenientes de un disco ranurado conectado a un motor, usando una máquina de
 * estados simple que evita conteos múltiples por rebote mecánico.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "pin_out_default.h"
#include "robot_config.h"
#include "arm_book_lib.h"

/// @brief Cantidad de ranuras (pulsos por vuelta) del disco del encoder.
#define DISC_TICS 20.0f


/**
 * @enum EncoderState
 * @brief Representa los estados posibles del encoder.
 *
 * Esta enumeración se utiliza en una máquina de estados finita
 * que permite evitar el rebote del encoder y contar los pulsos
 * de manera confiable.
 *
 * @dot
 * digraph EncoderStateDiagram {
 *     rankdir=LR;
 *     ENCODER_IDLE   -> ENCODER_ACTIVE [label="flanco ascendente (LOW → HIGH)"];
 *     ENCODER_ACTIVE -> ENCODER_WAIT   [label="se cuenta pulso"];
 *     ENCODER_WAIT   -> ENCODER_IDLE   [label="pin en LOW"];
 * }
 * @enddot
 */
typedef enum {
    ENCODER_IDLE,   ///< Espera flanco ascendente.
    ENCODER_ACTIVE, ///< Se detecta flanco y se cuenta el pulso.
    ENCODER_WAIT    ///< Espera que el pin vuelva a LOW.
} EncoderState;

/**
 * @struct EncoderSpeeds
 * @brief Estructura que representa el estado y datos del encoder.
 */
struct EncoderSpeeds {
    DigitalIn    pin;                    ///< Pin de entrada del encoder.
    EncoderState state;                  ///< Estado actual del encoder.
    float        speed;                  ///< Velocidad angular estimada (rad/s).
    int          lastState;              ///< Estado anterior del pin (para detectar flancos).
    int          pulseCount;             ///< Número de pulsos contados en el periodo.
    int          accumulatedTimeEncoder; ///< Tiempo acumulado desde último cálculo (ms).

    /**
     * @brief Constructor que inicializa los campos del encoder.
     * @param pinName Pin digital de entrada del encoder.
     */
    EncoderSpeeds(PinName pinName)
        : pin(pinName)
    {
        state                  = ENCODER_IDLE;
        speed                  = 0.0f;
        lastState              = 0;
        pulseCount             = 0;
        accumulatedTimeEncoder = 0;
    }
};

/**
 * @brief Inicializa los valores del encoder.
 * @param enc Puntero al encoder a inicializar.
 * @param pin Pin digital donde está conectado el encoder.
 */
void encoderInit(EncoderSpeeds* enc, PinName pin);

/**
 * @brief Actualiza el estado del encoder y cuenta pulsos.
 *
 * Esta función debe ser llamada periódicamente (cada TIME_INCREMENT_MS).
 * Internamente actualiza la máquina de estados y calcula la velocidad si se cumple el intervalo.
 *
 * @param enc Puntero al encoder que se desea actualizar.
 */
void encoderUpdate(EncoderSpeeds* enc);

/**
 * @brief Lee la velocidad angular estimada del encoder.
 * @param enc Puntero al encoder del cual se quiere obtener la velocidad.
 * @param observedSpeed Puntero donde se almacena el valor de la velocidad (rad/s).
 */
void encoderRead(EncoderSpeeds* enc, float* observedSpeed);

/**
 * @brief Reinicia los contadores y estado interno del encoder.
 * @param enc Puntero al encoder a reiniciar.
 */
void encoderReset(EncoderSpeeds* enc);









//========================================================
//
//                             Con interrupcciones
//ref:
// (de mbed)    https://os.mbed.com/users/aberk/code/QEI/docs/tip/QEI_8h_source.html
// (de diffbot) https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/diffbot_base/scripts/base_controller/lib/encoder/encoder_diffbot.h
// (Robotisim)  https://robotisim.com/course/robotics-development-fundamentals/
//========================================================


#endif // ENCODER_H
