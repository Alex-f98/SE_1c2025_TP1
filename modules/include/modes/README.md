# Descripción
- `modes`: Define el comportamiento específico de cada modo (Manual, Automático y Stop).
- `mode_manager`: Administra la máquina de estados y gestiona las transiciones entre los modos.

---

# `modes`

Este módulo define el comportamiento de los tres modos de operación del robot:

### Modos disponibles

- **Manual**: Controlado por comandos UART (`w`, `s`, `a`, `d`, `c`, `q`).
- **Automático**: Movimiento autónomo aleatorio (modo demostración).
- **Stop**: Detiene inmediatamente al robot (activado por el comando `'q'`).

### Estructura principal

```c
typedef struct {
    float linealVelocity;   // Velocidad lineal (m/s)
    float angularVelocity;  // Velocidad angular (rad/s)
    bool stop;              // Bandera de parada inmediata
    bool closeLoop;         // Control en lazo cerrado habilitado
} robot_velocity_t;
````

### Funciones

* `runManualMode(robot_velocity_t* vel)`: Ejecuta la lógica de control manual por UART.
* `runAutomaticMode(robot_velocity_t* vel)`: Genera velocidades aleatorias y responde a comandos UART.
* `runStopMode(robot_velocity_t* vel)`: Detiene el robot y activa la bandera `stop`.

### Diagrama de estados

| Estado Actual | Evento                         | Nuevo Estado |
|---------------|--------------------------------|--------------|
| STOP          | Comando manual (`w/s/a/d`)     | MANUAL       |
| STOP          | Solicitud de modo automático   | AUTOMATICO   |
| MANUAL        | Comando `'q'`                  | STOP         |
| MANUAL        | Solicitud de modo automático   | AUTOMATICO   |
| AUTOMATICO    | Comando `'q'`                  | STOP         |
| AUTOMATICO    | Solicitud de modo manual       | MANUAL       |


---

##  `mode_manager`

Este módulo administra el modo actual del robot y ejecuta la lógica asociada. Funciona como una máquina de estados que responde a entradas del usuario y eventos internos.

### Enumeración de modos

```c
typedef enum {
    MODE_MANUAL,
    MODE_AUTOMATIC,
    MODE_STOP
} OperationMode;
```

### Estructura principal

```c
typedef struct {
    OperationMode currentMode;
    robot_velocity_t vel;
} ModeManager;
```

### Funciones públicas

* `modeManagerInit()`: Inicializa el gestor con el modo inicial.
* `modeManagerRun()`: Ejecuta la lógica correspondiente al modo activo.
* `setMode() / getMode()`: Cambia u obtiene el modo actual.
* `getVelocity()`: Devuelve las velocidades actuales del robot.
* `isCloseLoop()`: Indica si está activo el control en lazo cerrado.
* `isMoving()`: Verifica si el robot está en movimiento.
* `info()`: Devuelve toda la información relevante del estado del robot.

### Diagrama de estados del gestor


| Estado Actual | Condición / Evento      | Nuevo Estado |
|---------------|--------------------------|--------------|
| STOP          | vel ≠ 0                  | MANUAL       |
| STOP          | setMode(MANUAL)          | MANUAL       |
| STOP          | setMode(AUTO)            | AUTOMATICO   |
| MANUAL        | setMode(STOP)            | STOP         |
| MANUAL        | setMode(AUTO)            | AUTOMATICO   |
| AUTOMATICO    | setMode(MANUAL)          | MANUAL       |
| AUTOMATICO    | setMode(STOP)            | STOP         |



##  Comandos UART soportados

| Comando | Modo        | Acción                                     |
| ------- | ----------- | ------------------------------------------ |
| `w`     | Manual      | Aumentar velocidad lineal                  |
| `s`     | Manual      | Disminuir velocidad lineal                 |
| `a`     | Manual      | Giro antihorario (izquierda)               |
| `d`     | Manual      | Giro horario (derecha)                     |
| `c`     | Manual/Auto | Activar/desactivar control en lazo cerrado |
| `q`     | Manual/Auto | Activar modo STOP                          |



---

## TODO

* Implementar una estrategia de navegación autónoma real en `runAutomaticMode()`.
* Agregar modos adicionales.
* Soporte para control remoto.


