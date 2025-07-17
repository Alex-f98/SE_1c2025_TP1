# `PID`

Esta implementación está basada en la famosa librería de PID de Brett Beauregard (Arduino) y ha sido adaptada para su uso en sistemas con restricciones de hardware.

[ref:PID](https://os.mbed.com/users/aberk/code/PID/docs/tip/PID_8h_source.html)

---

## Controlador PID

El controlador PID es una técnica de control **feedback** ampliamente utilizada en la industria para sistemas dinámicos. Ajusta la señal de control a partir del **error** entre la señal deseada (setpoint) y la señal medida (process variable).

## Ecuación PID Discreta

En el dominio continuo, la ley de control de un PID es:

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}
$$

donde:

- $ u(t) $: señal de control (salida del PID)
- $ e(t) = r(t) - y(t) $: error entre referencia y salida medida
- $ K_p $: ganancia proporcional
- $ K_i $: ganancia integral
- $ K_d $: ganancia derivativa

En su versión **discreta**, implementada en esta librería, se usa la siguiente forma aproximada:

$$
u[k] = \text{bias} + K_c \left( e[k] + \tau_I \sum_{i=0}^{k} e[i] - \tau_D \frac{y[k] - y[k-1]}{T_s} \right)
$$

donde:

- $ T_s $: intervalo de muestreo
- $ \tau_I = \frac{1}{K_i} \cdot T_s $: constante de integración
- $ \tau_D = K_d / T_s $: constante de derivación

Esta implementación también contempla:
- **Anti-windup** para evitar la acumulación excesiva del término integral cuando la salida está saturada.
- **Feedforward** opcional, usando `bias_` para mejorar la respuesta ante entradas conocidas.

---

## Estructura del Código

El archivo principal contiene la clase `PID`, con los siguientes métodos clave:

| Método | Descripción |
|--------|-------------|
| `PID(Kc, tauI, tauD, interval)` | Constructor. Inicializa los parámetros del PID. |
| `setInputLimits(min, max)` | Define límites de entrada (e.g., 0 a 3.3V). |
| `setOutputLimits(min, max)` | Define límites de salida. |
| `setTunings(Kc, tauI, tauD)` | Ajusta las ganancias del PID. |
| `setSetPoint(value)` | Define el setpoint deseado. |
| `setProcessValue(value)` | Ingresa el valor medido del sistema. |
| `compute()` | Realiza el cálculo PID y devuelve la salida de control. |
| `reset()` | Resetea errores acumulados y salidas anteriores. |
| `setMode(mode)` | Cambia entre modo automático o manual. |
| `setBias(bias)` | Activa control feedforward. |

---

## Detalles Matemáticos

La implementación utiliza una forma escalada para las entradas y salidas, normalizando en un rango de 0 a 1. Esto permite que el algoritmo funcione de forma genérica independientemente del rango físico de los sensores y actuadores.

### Cálculo del término derivativo

$$
\text{dMeas} = \frac{PV[k] - PV[k-1]}{T_s}
$$

### Condición anti-windup

El término integral (`accError_`) solo se actualiza si la salida no está saturada:

```cpp
if (!(prevControllerOutput_ >= 1 && error > 0) && !(prevControllerOutput_ <= 0 && error < 0)) {
    accError_ += error;
}
```

### Referencias

- [PID ](https://os.mbed.com/users/aberk/code/PID/docs/tip/PID_8h_source.html)
- [Control Guru](https://controlguru.com/table-of-contents/)

---
# `MOTOR CONTROL`


Este módulo gestiona el control de velocidad para un motor de corriente continua con soporte para lazo abierto y lazo cerrado mediante un controlador PID, y el encoder incremental para medir la velocidad angular y la posición angular.

## Descripción

El archivo `motor_control.cpp` contiene la implementación de la clase `MotorController`, que permite:

- Controlar un motor mediante señal PWM.
- Medir velocidad angular con un encoder incremental.
- Realizar control en lazo cerrado usando un PID configurable.
- Operar en lazo abierto si se desea una salida directa.

## Funcionalidades

- **Modo de control**: Lazo abierto / Lazo cerrado.
- **PID configurable**: Constantes `Kc`, `Ti`, `Td`, período de muestreo `Ts`.
- **Encoder incremental**: Medición de velocidad angular y posición.
- **Configuración flexible**: Límites de velocidad, resolución del encoder, parámetros del motor.
- **Funciones de control**: `setTargetVelocity`, `update`, `stop`.

## Estructura de la clase `MotorController`

### Constructor

```cpp
MotorController(PinName pwmPin, PinName encoderPin,
                float kc = KC , float ti = TI, float td = TD, float ts = TS,
                std::chrono::milliseconds encoderInterval = TENCODER_UPDATE,
                float motorPwmPeriod = PWM_PERIOD);
````

Inicializa el motor, encoder y controlador PID.

Todas las variables de entrada son opcionales y se usan las constantes por defecto definidas en el archivo `robot_config.h`.

---

### Métodos principales

* `void enableClosedLoop(bool enable)`
  Activa o desactiva el control en lazo cerrado.

* `void setTargetVelocity(double reference)`
  Establece la velocidad angular objetivo en rad/s.

* `void setMotorControlParameters(...)`
  Configura los parámetros del controlador PID (Kc, Ti, Td, Ts).

* `void setEncoderParameters(...)`
  Cambia la resolución del encoder y su periodo de muestreo.

* `void setMotorParameters(...)`
  Define los límites físicos del motor (velocidades, duty cycle) y periodo del PWM del motor.

* `double getMeasuredVelocity()`
  Retorna la velocidad observada (en lazo cerrado) o la referencia (en lazo abierto).

* `double getAngularPosition()`
  Devuelve la posición angular acumulada.

* `int getMeasuredRPM()`
  Velocidad medida en RPM.

* `void update()`
  Ejecuta una iteración del controlador cada `ts_` ms (es llamado por el ticker). Aplica PID si está en lazo cerrado.

* `void stop()`
  Detiene el motor y reinicia los estados internos del PID y encoder.


## Dependencias

* `motor.h`
* `encoder_velocity.h`
* `pid.h`
* Constantes como `KC`, `TI`, `TD`, `TS`, `W_MAX`, `W_MIN`, etc.

> Asegurarse de definir estas constantes o incluirlas desde un archivo `robot_config.h`.

---

## Ejemplo de uso

```cpp
#include "motor_control.h"
#include "robot_config.h"

MotorController motor(PB_6, PB_7); // PWM, encoder

int main() {
    motor.enableClosedLoop(true);
    motor.setTargetVelocity(5.0); // rad/s

    while (true) {
        motor.update();
        delay(10);
    }
}
```


##  **Nota**

* El método `update()` se  llama periódicamente por el ticker.
* El encoder debe estar correctamente conectado y configurado.
* El lazo cerrado requiere ajuste fino de PID para evitar oscilaciones o errores persistentes.



