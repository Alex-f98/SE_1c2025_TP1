/**
 * @file modes.h
 * @brief Módulo de modos de operación del robot móvil.
 * 
 * Este módulo define y gestiona los distintos modos de funcionamiento del robot:
 * - Manual
 * - Automático
 * - Parada (Stop)
 *
 * Cada modo controla la velocidad del robot de forma diferente. Las velocidades
 * se almacenan en una estructura común que se actualiza dinámicamente.
 *
 * @dot
 * digraph ModesStateMachine {
 *     node [shape = circle, style = filled, color = lightgrey];

 *     STOP     [label="STOP",     shape=doublecircle, color=red];
 *     MANUAL   [label="MANUAL"];
 *     AUTO     [label="AUTOMATICO"];

 *     STOP -> MANUAL   [label="comando manual (w/s/a/d)"];
 *     MANUAL -> STOP   [label="'q'"];
 *     MANUAL -> AUTO   [label="modo automatico externo"];
 *     AUTO -> STOP     [label="'q'"];
 *     AUTO -> MANUAL   [label="modo manual externo"];
 * }
 * @enddot
 */

#ifndef MODES_H
#define MODES_H

#include "mbed.h"
#include "arm_book_lib.h"

/**
 * @struct robot_velocity_t
 * @brief Estructura que representa el estado de velocidad del robot.
 *
 * Esta estructura encapsula las velocidades lineales y angulares del robot,
 * así como flags de parada y control en lazo cerrado.
 */
typedef struct {
    float linealVelocity;   ///< Velocidad lineal deseada en m/s.
    float angularVelocity;  ///< Velocidad angular deseada en rad/s.
    bool  stop;             ///< Flag para detener el robot inmediatamente.
    bool  closeLoop;        ///< Indica si se usa control en lazo cerrado.
} robot_velocity_t;

/**
 * @brief Ejecuta el modo manual del robot.
 *
 * Este modo permite controlar el robot mediante comandos UART:
 * - 'w': Aumenta velocidad lineal
 * - 's': Disminuye velocidad lineal
 * - 'a': Gira a la izquierda
 * - 'd': Gira a la derecha
 * - 'q': Detención de emergencia (modo STOP)
 * - 'c': Activa/Desactiva lazo cerrado
 *
 * @param vel Puntero a estructura de velocidades del robot.
 */
void runManualMode(robot_velocity_t* vel);

/**
 * @brief Ejecuta el modo automático del robot.
 *
 * En este modo el robot se mueve con velocidades aleatorias (modo demostración).
 * También se puede parar ('q') o cambiar el modo de control ('c') vía UART.
 *
 * @param vel Puntero a estructura de velocidades del robot.
 */
void runAutomaticMode(robot_velocity_t* vel);

/**
 * @brief Ejecuta el modo de parada del robot.
 *
 * Establece las velocidades en cero y activa el flag `stop`.
 * Este modo se activa desde otros modos con el comando `'q'`.
 *
 * @param vel Puntero a estructura de velocidades del robot.
 */
void runStopMode(robot_velocity_t* vel);

#endif // MODES_H
