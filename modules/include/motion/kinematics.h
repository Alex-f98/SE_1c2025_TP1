/**
 * @file kinematics.h
 * @brief Módulo de cinemática para robots diferenciales y omnidireccionales.
 *
 * Este módulo permite calcular las velocidades angulares de las ruedas
 * a partir de velocidades lineales y angulares del cuerpo del robot.
 * 
 * Soporta:
 * - Robots diferenciales (2 ruedas motrices)
 * - Robots omnidireccionales (ej. 3 ruedas con configuración holonómica)
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "mbed.h"

/**
 * @enum RobotType
 * @brief Tipo de robot según su configuración de ruedas.
 */
typedef enum {
    DIFFERENTIAL,    /**< Robot diferencial (2 ruedas motrices). */
    OMNIDIRECTIONAL, /**< Robot omnidireccional (ej. 3 ruedas holonómicas). */
    SKIDSTEER        /**< Robot de 4 ruedas */
} RobotType;

/**
 * @struct Kinematics
 * @brief Estructura que describe la configuración física del robot.
 */
typedef struct {
    RobotType type;     ///< Tipo de robot

    // Común
    float wheelRadius;  ///< Radio de las ruedas [m]

    // Solo para robot diferencial
    float wheelBase;    ///< Distancia entre ruedas [m]

    // Solo para robot omnidireccional
    float lx;           ///< Distancia del centro al eje x de una rueda [m]
    float ly;           ///< Distancia del centro al eje y de una rueda [m]
} Kinematics;

/**
 * @struct Velocity
 * @brief Velocidades del cuerpo del robot (espacio cartesiano).
 */
typedef struct {
    float vx;     ///< Velocidad lineal en el eje X del robot [m/s]
    float vy;     ///< Velocidad lineal en el eje Y del robot [m/s] (solo para robots omnidireccionales)
    float omega;  ///< Velocidad angular del robot [rad/s]
} Velocity;

/**
 * @struct WheelVelocities
 * @brief Velocidades angulares de las ruedas [rad/s].
 * 
 * Para robots de 2 ruedas (diferenciales) solo se usan w1 y w2.
 * Para robots de 3 ruedas holonómicos se usan w1, w2 y w3.
 */
typedef struct {
    float w1; ///< Velocidad rueda 1
    float w2; ///< Velocidad rueda 2
    float w3; ///< Velocidad rueda 3
    float w4; ///< Velocidad rueda 4 (no usado en robots de 3 ruedas)
} WheelVelocities;

/**
 * @brief Calcula las velocidades angulares de las ruedas a partir de las velocidades del cuerpo.
 *
 * ### Para robots diferenciales:
 * Se usan las siguientes ecuaciones:
 * 
 * \\f[
 * V = \frac{V_R + V_L}{2}, \quad
 * \omega = \frac{V_R - V_L}{L}
 * \\f]
 * 
 * Despejando las velocidades de rueda:
 * 
 * \\f[
 * V_R = V + \frac{L}{2} \cdot \omega, \quad
 * V_L = V - \frac{L}{2} \cdot \omega
 * \\f]
 * 
 * Y finalmente, la velocidad angular de cada rueda:
 * 
 * \\f[
 * w_r = \frac{V_R}{r}, \quad
 * w_l = \frac{V_L}{r}
 * \\f]
 *
 * ### Para robots omnidireccionales (3 ruedas con ángulos 120°):
 * Se usa una formulación genérica:
 * 
 * \\f[
 * w_1 = \frac{v_x - v_y - (l_x + l_y)\omega}{r} \\
 * w_2 = \frac{v_x + v_y + (l_x + l_y)\omega}{r} \\
 * w_3 = \frac{v_x - v_y + (l_x + l_y)\omega}{r}
 * \\f]
 * 
 * @param kin Puntero a la configuración cinemática del robot.
 * @param vel Velocidad deseada del cuerpo del robot.
 * @param out Puntero donde se almacenan las velocidades de ruedas.
 */
void computeWheelVelocities(const Kinematics* kin, const Velocity* vel, WheelVelocities* out);

#endif
