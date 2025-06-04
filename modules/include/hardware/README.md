
# EncoderVelocity Module

Este módulo proporciona una interfaz orientada a objetos para medir **velocidad angular**, **posición angular** y **ticks acumulados** a partir de un encoder optico LM393 ( inspirado en el [mRotaryEncoder-os](https://os.mbed.com/users/charly/code/mRotaryEncoder-os/docs/tip/mRotaryEncoder_8cpp_source.html) ) conectado a un microcontrolador.

## Concepto

La clase `EncoderVelocity` encapsula un `EncoderCounter` y usa un `Ticker` interno para muestrear periódicamente los cambios de posición. A partir de los **ticks acumulados** y el **intervalo de muestreo**, se calcula la **velocidad angular** y **posición angular absoluta** en radianes.

---

## Estructura general

```cpp
EncoderVelocity::EncoderVelocity(PinName pin, int encoderResolution, float samplingInterval = 0.02, PullMode pullMode = PullUp);
```

### Parámetros:

* `pin`: Pin de entrada digital donde está conectado el encoder.
* `encoderResolution`: Número de **ticks por revolución** (TPR).
* `samplingInterval`: Intervalo de muestreo periódico para calcular la velocidad (default: 20 ms).
* `pullMode`: Modo de resistencia interna (PullUp/PullDown).

---

## Métodos

### `getRPM() → int`

Devuelve la **velocidad angular en RPM (revoluciones por minuto)**, calculada a partir de:

$$
\text{RPM} = \frac{\omega \cdot 60}{2\pi}
$$

Donde:

* $\omega$ es la velocidad angular en radianes/segundo.

---

### `getAngularVelocity() → double`

Devuelve la **velocidad angular instantánea** en radianes/segundo.

> ⚠️ Internamente almacena el último valor calculado por `update()` en `angularVelocity`, posteriorme se debe implementar filtro de ser necesario.

---

### `getAngularPosition() → double`

Devuelve la **posición angular acumulada** en radianes desde el último reset:

$$
\theta(t) = \sum_{i=0}^{t} \Delta \theta_i
$$

Cada $\Delta \theta$ se obtiene a partir de los ticks acumulados:

$$
\Delta \theta = \text{ticks} \times \frac{2\pi}{\text{resolución}}
$$

---

### `getTicks() → int`

Devuelve el **número total de ticks acumulados** desde el último reset.

---

### `resetVelocity()`

Resetea todos los contadores y pone en cero:

* Ticks acumulados
* Posición angular
* Velocidad angular
* Última cuenta

---

### `setResolution(int resolution)`

Permite cambiar dinámicamente la resolución del encoder (TPR).

---

### `setSamplingInterval(float samplingInterval)`

Permite ajustar el **intervalo de muestreo** del `Ticker`, y vuelve a configurar su llamada periódica.


---

### `ticks2angle() const → double`

Convierte un tick del encoder en **radianes**:

$$
\text{radianes por tick} = \frac{2\pi}{\text{resolución}}
$$

---

### `update()`

Este método se ejecuta periódicamente gracias al `Ticker`. Su función es:

1. Leer los ticks actuales del encoder.
2. Calcular los ticks nuevos desde la última lectura: `deltaTicks`.
3. Convertir `deltaTicks` a radianes: `deltaAngle`.
4. Acumular el ángulo total.
5. Calcular la velocidad angular:

$$
\omega = \frac{\Delta \theta}{\Delta t}
$$

Donde:

* $\Delta \theta = \text{deltaAngle}$
* $\Delta t = \text{samplingInterval}$

> Usa `CriticalSectionLock` para evitar condiciones de carrera con otras lecturas.

---

##  Dependencias

Este módulo depende de:

* `EncoderCounter` (otro módulo que cuenta ticks)
* `mbed::Ticker`
* `CriticalSectionLock`

---

## Ejemplo de uso

```cpp
#include "encoderVelocity.h"
#include "arm_book_lib.h"

EncoderVelocity encoder(D2, 20);  

int main() {
    while (true) {
        printf("RPM: %d\n", encoder.getRPM());
        printf("Posición: %.2f rad\n", encoder.getAngularPosition());
        sleep(100);
    }
}
```

---

## Notas

* La velocidad es una estimación basada en diferencias finitas. Puede fluctuar si el intervalo de muestreo es muy bajo.
* Si se usa un encoder cuadratura con dos canales, se puede adaptar `encoderVelocity` para lectura de A y B.
* Este módulo no implementa filtros, pero se puede extender para usar un filtro promedio o exponencial.

---

## Archivos involucrados

* `encoderVelocity.h`
* `encoderVelocity.cpp`
* (dependencia) `encoderCounter.h/cpp`

