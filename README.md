## TP2: sistemas embebidos

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

| Componente       | Tipo             | Función                                     |
|------------------|------------------|---------------------------------------------|
| `BUTTON_STOP`    | Entrada digital  | Parada de emergencia                        |
| `M1` / `M2`      | pwm out          | Controla motor mediante pwm.                |
| `ENCODER_1/2`    | Entrada digital  | Cuenta ticks de un lm393                    |
| `UART`           | Comunicación     | Entrada de comandos de velocidad o posición |
## Plataforma de desarrollo: NUCLEO-F429ZI

## Esquematico:

![Esquematico tentativo](EsquematicoTP1SE.png)

---

## **Implementación**

Este código implementa el control de un **robot móvil diferencial**. 
Está diseñado para manejarse en **modo manual o automático**.


Nota: La estructura se basa en la ejecución secuencial de eventos con tiempos de muestreo fijos.
Nota: Los tiempos fijos se mantenienen ahora dentro de cada modulo.


# **1. Estructura General**
El programa sigue un **bucle principal (`main()`)** donde:
1. **Inicializa** entradas, salidas y el controlador PID.
2. **Verifica botón de emergencia** mediante interrupcción.
3. **Maneja el modo de operación**:
   - En **modo manual**, lee comandos UART cada 100 ms para modificar velocidades (limitados).
   - En **modo automático**, de momento genera valores aleatorios de velocidad (limitados).
4. **Calcula las velocidades de las ruedas** según la cinemática diferencial.
5. **Aplica velocidades a los motores**, en lazo abierto o cerrado.

El control de tiempo se implementa de manera secuencial mediante **contadores de tiempo acumulados** (`accumulatedTimeX`), sin usar interrupciones.

---

# **2. Componentes Clave**
Los header se necuentran en la carpeta **modules/include** y los respectivos archivos de implementacion en **modules/src**.
```
/robot_firmware/
├── main.cpp                       # Punto de entrada principal del firmware
├── README.md                      # Descripción general del proyecto
├── modules/
│   ├── config/                    # Configuraciones generales y pines
│   │   ├── pin_out_default.h      # Asignación de pines por defecto
│   │   └── robot_config.h         # Parámetros físicos y constantes del robot
│   │
│   ├── include/                   # Encabezados públicos de los módulos
│   │   ├── control/               # Interfaces de controladores (PID, motor)
│   │   │   ├── motor_control.h    # Controlador de motores (tiliza PID y encoder)
│   │   │   ├── pid.h              # Controlador PID
│   │   │   └── README.md          # Descripción del módulo de control
│   │   ├── hardware/              # Interfaces de hardware (drivers)
│   │   │   ├── emergency_button.h
│   │   │   ├── encoderCounter.h
│   │   │   ├── encoder.h          # Encoder(No se utiliza en esta versión)
│   │   │   ├── encoderVelocity.h  # Utiliza encoderCounter para medir velocidades
│   │   │   ├── motor_driver.h     # traduce velocidades a pwm
│   │   │   └── README.md          # Descripción del módulo de hardware
│   │   ├── modes/                 # Modos de operación del sistema
│   │   │   ├── mode_manager.h     # Gestor de modos
│   │   │   ├── modes.h            # Definición de modos
│   │   │   └── README.md          # Descripción del módulo de modos
│   │   ├── motion/                # Lógica de movimiento y cinemática
│   │   │   └── kinematics.h       # Cinemática (solo uso la diferencial)
│   │   ├── robot/                 # Definición y estado general del robot
│   │   │   └── robot.h            # Contiene el loop principal con la logica
│   │   └── utils/                 # Funciones auxiliares
│   │       ├── debug.h
│   │       └── utils.h
|   |
│   └── src/                       # Implementación de los módulos
│       ├── control/
│       │   ├── motor_control.cpp
│       │   └── pid.cpp
│       ├── hardware/
│       │   ├── emergency_button.cpp
│       │   ├── encoderCounter.cpp
│       │   ├── encoder.cpp
│       │   ├── encoderVelocity.cpp
│       │   └── motor_driver.cpp
│       ├── modes/
│       │   ├── mode_manager.cpp
│       │   └── modes.cpp
│       ├── motion/
│       │   └── kinematics.cpp
│       ├── robot/
│       │   └── robot.cpp
│       └── utils/
│           ├── debug.cpp
│           └── utils.cpp

```


## **2.1. Configuraciones**
- Contiene parametros fisicos y tiempos que se usan en la mayoria de los modulos.
- Contiene definiciones de pines de entrada y salida asi como los de UART.

## **2.2. Modulos de Harware**
- **emergency_button.cpp**: Implementa toda la logica de un boton de emergencia.
- **encoderCounter.cpp**: Este encoder esta centrado en contar ticks cada cierto mediante interrupciones.
- **encoderVelocity.cpp**: Implementa toda la logica de un encoder fisico, usa encoderCounter para medir velocidades cada cierto tiempo (configurable en robot_cofig.h).
- **encoder.cpp**: Implementa toda la logica de un encoder fisico, este encoder esta basado en contar ticks cada cierto tiempo (configurable en robot_cofig.h).
- **motor_driver.cpp**: Implementa un motor de continua, convierte velocidades de referencia en duty cycle para un pwm (usa pwmOut).

### **2.3. Modulo de control**
- pid.cpp: Implementa un control PID.
Se implementa un **control PID básico** con los términos:
$$
u = u_{Bias} + K_c(e + \frac{1}{T_i}\int{e dt} + T_d \frac{de}{dt})
$$
Donde:
- $ e $ es el error ($ \text{setpoint} - \text{valor medido} $)
- **Integral y derivativa** se calculan de forma discreta.

Tambien puede escribirse de la siguiente forma, segun [PID](https://os.mbed.com/cookbook/PID):

$$
CO = CO_{bias} + K_c(e(t) + \frac{1}{T_i}\int{e(t) dt} + T_d \frac{Pv}{dt})
$$

Donde:
- $ CO $ es la salida del controlador
- $ CO_{bias} $ es un bias optimo para el controlador
- $ K_c $ es la ganancia del controlador
- $ e(t) $ es el error ($ \text{Pv} - \text{valor medido} $)
- $ T_i $ es el tiempo de integración
- $ T_d $ es el tiempo de derivación
- $ Pv $ es la variable del proceso
- $ dt $ es la tasa de muestreo

mas información en el modulo [control](modules/include/control/README.md).

- **motor_control.cpp**: Se encarga del control a lazo abierto o cerrado(usando pid) del motor dc, usa el encoder para obtener velocidades observadas.

- **encoderVelocity.cpp**: Implementa toda la logica de un encoder fisico, usa encoderCounter para medir velocidades cada cierto tiempo (configurable en [robot_cofig.h](modules/config/robot_config.h)).
Tambien se puede encontrar mas informaciobn en el modulo [hardware](modules/include/hardware/README.md).

## **2.3. Cinemática Diferencial**
El modelo cinemático diferencial transforma velocidades lineales y angulares del cuerpo del robot a velocidades de rueda:

$$
v = \frac{v_R + v_L}{2}, \quad \omega = \frac{v_R - v_L}{L}
$$

Despejando:

$$
v_R = v + \frac{L}{2} \omega, \quad v_L = v - \frac{L}{2} \omega
$$

Para convertir las velocidades lineales a velocidades angulares de rueda ($ \omega_R, \omega_L $) se divide por el radio $ r $:

$$
\omega_R = \frac{v_R}{r}, \quad \omega_L = \frac{v_L}{r}
$$

Nota: de momento esta es la unica cinemática utilizada.
## **2.4. Gestor de modos de operacion**

El robot se puede controlar manualmente vía **UART** con comandos como:
- `'w'` y `'s'`: Aumentar/disminuir velocidad lineal.
- `'a'` y `'d'`: Aumentar/disminuir velocidad angular.
- `'m'`: Cambiar entre **modo manual/automático**.
- `'c'`: Activar/desactivar **control en lazo cerrado**.
- `'q'`: Fuerza velocidades a cero **apago el robot**.

Se defines los modos de operacion y las llamadas a otras funciones dentro de modes.cpp, luego se gestionan los modos de operacion a
traves de mode_manager.cpp.

---

## **2.5. Manejo de Sensores y Seguridad**
- **Botón de parada de emergencia** (`buttonStop`):
  - Se revisa **cada 10 ms** (`checkSensors()`), se activa el flag mediante interrupcciones.
  - Si se activa, se detienen los motores suavemente.

- **Encoders** (`encoderLeft` y `encoderRight`):
  - Son entradas digitales para medir velocidades de las ruedas.
  - Se usan en **control en lazo cerrado** (`readEncoders()`).
  - Se acumulan **Ticks cada vez que se detecta un flanco de subida** usando interrupciones.
  - Se calcula la estimacion de velocidad sensada **cada  0.017s**.

---

## **2.6. Manager de robot**
Recibe velocidades de referencia respecto del centro de masa del robot y aplica las velocidades necesarias para los motores, acciona segun su propia maquina de estados.
Mas informacion en el modulo [modes manager](modules/include/modes/README.md).


# **3. Calculo de tiempos**

Se sigue la siguiente regla basica para comenzar:

$$f_{observador} >= f_{control} >= 10 . f_{planta}$$

Donde:
- $f_{observador}$ es la frecuencia de muestreo del observador(encoder).
- $f_{control}$ es la frecuencia de muestreo del controlador(pid).
- $f_{planta}$ es la frecuencia de muestreo de la planta(motores).

Sabemos que los motores trabajan a 5v segun la informacion del mismo tiene velocidad maxima 150 rpm aprox.

entonces $\Delta Ticks = \frac{60 s/min}{TicksPorRev * VelMaxRPM } = \frac{60}{20 * 150} = \frac{1}{50} = 0.02s$

Entonces se elige:


$$
\begin{array}{|c|c|c|c|}
\hline
\textbf{Componente} & \textbf{Frecuencia} & \textbf{Período} & \textbf{Constante} \\
\hline
\text{Observador} & 50\,\text{Hz} & 20\,\text{ms} & \text{TENCODER\_UPDATE} \\
\hline
\text{Controlador} & 40\,\text{Hz} & 25\,\text{ms} & \text{TS} \\
\hline
\text{Planta} & 4\,\text{Hz} & 250\,\text{ms} & \text{TMOTOR\_UPDATE} \\
\hline
\end{array}
$$

Se puede encontrar las constantes en el archivo [robot_config.h](modules/config/robot_config.h).

# **4. Mejoras Futuras**

### **Migración a Interrupciones**

- Debe implementarse un **scheduler con interrupciones**, por ejemplo:
  - **Interrupción por UART para leer comandos**.


### **Integración de Sensores de Navegación**
- En **modo automático**, el robot solo usa velocidades aleatorias.
- Debe integrarse:
  - **Cámara** para localizar personas y de ser posible evitar obstáculos.
  - **Filtro de Kalman** para estimación de posición.

### **Actualizacion de hardware**
- Mejorar los metodos de validacion de velocidad (optical flow).
- Identificar la planta del motor DC.

# **Para mejorar**:
**Agregar navegación autónoma basica**

---

## **Video**

TP2: https://www.youtube.com/watch?v=Sc_idwAd8kU

TP3: [video demostración TP3](https://www.youtube.com/watch?si=rM97AdbaSwF0Z7xf&v=Ra4TXIUrl3o&feature=youtu.be)

---
## **Bibliografia**: 

https://ria.utn.edu.ar/server/api/core/bitstreams/1bbbbe5b-096e-4df5-8a84-a869f0a48766/content

