/**
 * @file mode_manager.h
 * @brief Gestor de modos de operación para el robot móvil.
 * 
 * Este módulo controla el estado actual del robot (Manual, Automático o Stop)
 * y permite realizar transiciones controladas entre modos, así como acceder
 * a la velocidad y estado del sistema.
 * 
 * ### Máquina de Estados
 * 
 * @dot
 * digraph ModeManagerStateMachine {
 *     node [shape = circle, style = filled, color = lightgrey];
 *     STOP     [label="STOP",     shape=doublecircle, color=red];
 *     MANUAL   [label="MANUAL"];
 *     AUTO     [label="AUTOMATICO"];

 *     STOP -> MANUAL [label="vel ≠ 0"];
 *     STOP -> MANUAL [label="setMode(MANUAL)"];
 *     STOP -> AUTO   [label="setMode(AUTO)"];
 *     MANUAL -> STOP [label="setMode(STOP)"];
 *     MANUAL -> AUTO [label="setMode(AUTO)"];
 *     AUTO -> MANUAL [label="setMode(MANUAL)"];
 *     AUTO -> STOP   [label="setMode(STOP)"];
 * }
 * @enddot
 */

#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include "modes.h"

//=== Tipos de datos ===//

/**
 * @enum OperationMode
 * @brief Enumeración de modos operativos posibles.
 */
typedef enum {
    MODE_MANUAL,     ///< Control manual por comandos UART.
    MODE_AUTOMATIC,  ///< Movimiento aleatorio automático.
    MODE_STOP        ///< Parada de emergencia (velocidades 0).
} OperationMode;

/**
 * @struct ModeManager
 * @brief Estructura del gestor de modos del robot.
 */
typedef struct {
    OperationMode currentMode;  ///< Modo de operación actual.
    robot_velocity_t vel;       ///< Velocidades y estado de control.
} ModeManager;

//=== Funciones públicas ===//

/**
 * @brief Inicializa el gestor de modos con un modo inicial.
 * @param manager Puntero al gestor de modos.
 * @param initialMode Modo inicial (MANUAL, AUTOMATIC o STOP).
 */
void modeManagerInit(ModeManager* manager, OperationMode initialMode);

/**
 * @brief Ejecuta la lógica correspondiente al modo actual.
 * @param manager Puntero al gestor de modos.
 */
void modeManagerRun(ModeManager* manager);

/**
 * @brief Cambia el modo de operación del robot.
 * @param manager Puntero al gestor de modos.
 * @param mode Nuevo modo a establecer.
 */
void setMode(ModeManager* manager, OperationMode mode);

/**
 * @brief Obtiene el modo de operación actual.
 * @param manager Puntero al gestor de modos.
 * @param mode Puntero donde se almacena el modo actual.
 */
void getMode(ModeManager* manager, OperationMode* mode);

/**
 * @brief Obtiene las velocidades actuales del robot.
 * @param manager Puntero al gestor de modos.
 * @param v Puntero donde se almacena la velocidad lineal.
 * @param w Puntero donde se almacena la velocidad angular.
 */
void getVelocity(ModeManager* manager, float* v, float* w);

/**
 * @brief Verifica si el modo actual tiene control en lazo cerrado.
 * @param manager Puntero al gestor de modos.
 * @return `true` si el lazo cerrado está activado.
 */
bool isCloseLoop(ModeManager* manager);

/**
 * @brief Verifica si el robot se encuentra en movimiento.
 * @param manager Puntero al gestor de modos.
 * @return `true` si alguna velocidad es distinta de cero.
 */
bool isMoving(ModeManager* manager);

/**
 * @brief Devuelve toda la información relevante del modo.
 * @param manager Puntero al gestor de modos.
 * @param mode Puntero para recibir el modo actual.
 * @param v Puntero para recibir la velocidad lineal.
 * @param w Puntero para recibir la velocidad angular.
 * @param closeLoop Puntero para recibir el estado del lazo cerrado.
 */
void info(ModeManager* manager, OperationMode* mode, float *v, float *w, bool *closeLoop);

#endif // MODE_MANAGER_H
