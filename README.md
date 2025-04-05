# TP1: sistemas embebidos

## Titulo: Robot móvil tipo diferencial para búsqueda y localización de personas.

## Alumno: 

Brian Alex Fuentes Acuña

## Objetivo: 

Desarrollar un robot móvil tipo diferencial para implementación de algoritmos de búsqueda y localización mediante visión.

## Descripción:  

El robot debe poder funcionar en dos modos MANUAL y AUTOMÁTICO en donde el primero consiste en un robot que sigue las velocidades lineales [ $v(t)$ , $w(t)$ ] las cuales serán suministradas mediante UART a través de un computadora conectada a un Joystick.
El modo AUTOMÁTICO será comandado también por velocidades [ $v(t)$ , $w(t)$ ] pero de forma automática a través de la conexión UART  a la computadora (debido al peso de estos algoritmos).

El robot contará con una conexión a una cámara para visión (Cámara  monocular tipo Raspberry o Sensor Kinect) el mismo se usará para detectar a las personas y transmitir mediante I2C hacia la placa (la NUCLEO-F429ZI admite también DCMI).

Para esta primera etapa todos los sensores serán simulados mediante entradas digitales o analógicas.

Para la identificación de una persona  (MODO AUTOMÁTICO) se mandaran coordenadas en el espacio de la imagen (ej Size: 32x32 - > pose_yz: (14,20) ) mediante UART.

Para la enviar señales (MODO MANUAL) de velocidad lineal ( $v(t)$ ) y velocidad angular ( $w(t)$ ) se enviaran al micro. mediante UART.

Para simular los motores se utilizarán dos LEDs, los cuales se encenderán y apagarán proporcionalmente a la velocidad necesaria para cada rueda ($w_L(t)$, $w_R(t)$ $\in [W_{min}, W_{max}]$).

Ademas se implementará un boton digital de parada de emergencia.

## Periféricos a utilizar:

* **BUTTOM_STOP** : Entrada digital, fuerza la detención del robot llevándolo a un estado seguro.
* **M1** : Salida digital LED1 que emula un motor controlado por PWM.
* **M2** :  Salida digital LED2 que emula un motor controlado por PWM.
* **ENCODER_1** : Entrada digital, simula un encoder tomando las pulsaciones del LED1. 
* **ENCODER_1** : Entrada digital, simula un encoder tomando las pulsaciones del LED2. 
* **UART**:  Se utiliza para enviar datos de referencia o control al microprocesador.

## Plataforma de desarrollo: NUCLEO-F429ZI

## Esquematico:

![Esquematico tentativo](EsquematicoTP1SE.png)

---

## **Implementación**

Este código implementa el control de un **robot móvil diferencial**. 
Está diseñado para manejarse en **modo manual o automático**.

Nota: Inicialmente sin interrupciones ni modularización. 
Nota: La estructura se basa en la ejecución secuencial de eventos con tiempos de muestreo fijos.


# **1. Estructura General**
El programa sigue un **bucle principal (`main()`)** donde:
1. **Inicializa** entradas, salidas y el controlador PID.
2. **Verifica sensores y botón de emergencia** cada 100 ms.
3. **Maneja el modo de operación**:
   - En **modo manual**, lee comandos UART cada 100 ms para modificar velocidades (limitados).
   - En **modo automático**, de momento genera valores aleatorios de velocidad (limitados).
4. **Calcula las velocidades de las ruedas** según la cinemática diferencial.
5. **Aplica velocidades a los motores**, en lazo abierto o cerrado.

El control de tiempo se implementa de manera secuencial mediante **contadores de tiempo acumulados** (`accumulatedTimeX`), sin usar interrupciones.

---

# **2. Componentes Clave**

## **2.1. Control del Motor**
El sistema soporta:
- **Control en lazo abierto** (`_setMotorSpeedsOpenLoop()`): Convierte la velocidad en un **duty cycle fijo** proporcional a la velocidad deseada.
- **Control en lazo cerrado** (`_setMotorSpeedsCloseLoop()`): Usa **encoders** y un **PID** para ajustar el duty cycle y corregir errores.

La función principal para aplicar el control es:
```cpp
void setMotorSpeeds(float v_L, float v_R)
{
    if (closeLoop == true)
        _setMotorSpeedsCloseLoop(v_L, v_R);
    else
        _setMotorSpeedsOpenLoop(v_L, v_R);
}
```
Esto permite cambiar entre lazo abierto y cerrado **en tiempo de ejecución**.

### **Control PWM de los motores**
Los motores se controlan con **PWM por software**:
```cpp
void motorControlPWM(DigitalOut *motor, float duty, float *timeElapsed)
```
- Se usa **`timeElapsed`** para simular un ciclo PWM con comparación por software.
- **En futuras versiones**, esta función debe ser reemplazada por un **PWM por hardware**.

---

## **2.2. Cinemática Diferencial**
Para calcular las velocidades de las ruedas en función de la velocidad lineal $ V $ y angular $ W $, se usa:
$$
V_R = V + \frac{R}{2} W
$$
$$
V_L = V - \frac{R}{2} W
$$

Esto permite transformar \( V, W \) en velocidades de ruedas **izquierda y derecha**.

---

## **2.3. Controlador PID**
Se implementa un **control PID básico** con los términos:
$$
u = K_p e + K_i \int{e dt} + K_d \frac{de}{dt}
$$
Donde:
- $ e $ es el error ($ \text{setpoint} - \text{valor medido} $)
- **Integral y derivativa** se calculan de forma discreta.

Para evitar valores extremos, se usa **saturación de salida**:
```cpp
if (saturationEnabled){
    return max_(min_(outPut, outputMax), outputMin);
}
```
El PID se ejecuta en **cada ciclo PWM** (cada 10 ms).

---

## **2.4. Comunicación y Control Manual**
El robot se puede controlar manualmente vía **UART** con comandos como:
- `'v'` y `'s'`: Aumentar/disminuir velocidad lineal.
- `'a'` y `'d'`: Aumentar/disminuir velocidad angular.
- `'m'`: Cambiar entre **modo manual/automático**.
- `'c'`: Activar/desactivar **control en lazo cerrado**.
- `'q'`: Fuerza velocidades a cero **apago el robot**.

Los comandos se procesan en `manualMode()`, ejecutándose **cada 100 ms**.

---

## **2.5. Manejo de Sensores y Seguridad**
- **Botón de parada de emergencia** (`buttonStop`):
  - Se revisa **cada 100 ms** (`checkSensors()`).
  - Si se activa, se detienen los motores suavemente.

- **Encoders** (`encoderLeft` y `encoderRight`):
  - Son entradas digitales para medir velocidades de las ruedas.
  - Se usan en **control en lazo cerrado** (`readEncoders()`).
  - Se acumulan **Ticks cada 10ms** los cuales se usan para estimar la velocidad.
  - Se calcula la estimacion de velocidad sensada **cada 30 ms**.

---
# **3. Diagrama simple del codigo**

![DiagramaSimple](DiagramFlujo01.png)

# **4. Mejoras Futuras**

### ** Migración a Interrupciones**
- Actualmente, todo se ejecuta en un **bucle secuencial**, lo cual **no es eficiente**.
- Debe implementarse un **scheduler con interrupciones**, por ejemplo:
  - **Timers para muestreo PID y encoders**.
  - **Interrupción por UART para leer comandos**.

### ** Implementación de PWM por Hardware**
- La modulación PWM actual usa un **ciclo de comparación por software**.
- Se debe utilizar **PWM nativo** para **mejor precisión y eficiencia**.

### ** Integración de Sensores de Navegación**
- En **modo automático**, el robot solo usa velocidades aleatorias.
- Debe integrarse:
  - **Cámara** para localizar personas y de ser posible evitar obstáculos.
  - **Filtro de Kalman** para estimación de posición.

# **Para mejorar**:

**Implementar interrupciones para mejor eficiencia**  
**Usar PWM por hardware para control de motores**
**Modularizar codigo y mejorar la mantenibilidad**
**Agregar navegación autónoma basica**

---

## **Bibliografia**: 

https://ria.utn.edu.ar/server/api/core/bitstreams/1bbbbe5b-096e-4df5-8a84-a869f0a48766/content

