/**
 * @file robot.h
 * @brief Funciones principales del robot móvil diferencial.
 *
 * Este módulo contiene la lógica de control de alto nivel del robot:
 * - Inicialización de entradas y salidas
 * - Bucle principal (`robotLoop`)
 * - Lectura de sensores
 * - Gestión de emergencia
 *
 * Se utiliza un modelo cinemático diferencial y controladores PID
 * individuales para cada motor.
 */

#ifndef ROBOT_H
#define ROBOT_H

#include "mbed.h"
#include "mode_manager.h"

/**
 * @brief Inicializa entradas digitales (botón de emergencia, modos).
 */
void inputsInit();

/**
 * @brief Inicializa salidas (motores).
 */
void outputsInit();

/**
 * @brief Verifica estado de sensores, incluyendo botón de parada.
 */
void checkSensors();

/**
 * @brief Loop principal del robot.
 *
 * Ejecuta:
 * - Lectura de sensores
 * - Gestión de modo (manual, automático, parada)
 * - Cálculo de velocidades por cinemática
 * - Control de velocidad de motores
 */
void robotLoop();

/**
 * @brief Verifica comandos de cambio de modo desde UART.
 *
 * Si se recibe 'm', alterna entre MODO_MANUAL y MODO_AUTOMATIC.
 *
 * @param manager Puntero al manejador de modo.
 */
void checkMode(ModeManager* manager);

#endif // ROBOT_H
