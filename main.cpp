/**
 * @file   main.cpp
 * @author Brian Fuentes (bfuentes@fi.uba.ar)
 * @brief  Código principal del robot móvil diferencial
 * @version 0.1
 * @date 2025-04-03
 * 
 * @copyright Copyright (c) 2025
 * 
 * @note El robot solo manejará velocidades positivas en esta etapa.
 * @note No se implementan modularizaciones ni interrupciones en esta versión inicial.
 * @note Por ahora hay tiempos hardcodeados para no tener que estar scrollenado al ajustar tiempos.
 * @ref https://ria.utn.edu.ar/server/api/core/bitstreams/1bbbbe5b-096e-4df5-8a84-a869f0a48766/content
 */

//=====[Libraries]=============================================================

#include "mbed.h"
#include "arm_book_lib.h"
#include "string.h"

//=====[Defines]===============================================================

/// @brief Definición de los modos de operación.
#define MANUAL_MODE                              0 ///< Modo manual
#define AUTOMATIC_MODE                           1 ///< Modo automático

/// @brief Límites y pasos de velocidad.
#define V_MAX                                    1.0f  ///< Velocidad máxima lineal (m/s)
#define W_MAX                                    1.0f  ///< Velocidad máxima angular (rad/s)
#define V_MIN                                    0.0f  ///< Velocidad mínima lineal (m/s)
#define W_MIN                                    0.0f  ///< Velocidad mínima angular (rad/s)
#define V_STEP                                   0.1f  ///< Incremento de velocidad lineal (m/s)
#define W_STEP                                   0.1f  ///< Incremento de velocidad angular (rad/s)

/// @brief Parámetros físicos del robot.
#define WHEEL_DISTANCE                           0.07f  ///< Distancia entre ruedas (m)
#define WHEEL_DIAMETER                           0.069f ///< Diámetro de la rueda (m)

/// @brief Configuración del control de tiempo.
#define TIME_INCREMENT_MS                        10   ///< Incremento de tiempo en milisegundos
#define PWM_PERIOD                               0.1  ///< Período total del "PWM" en segundos
#define PWM_STEP                                 0.01 ///< Paso de tiempo en cada iteración

/// @brief Límites de ciclo de trabajo del PWM.
#define DUTY_MIN                                 0.0f ///< Ciclo de trabajo mínimo
#define DUTY_MAX                                 1.0f ///< Ciclo de trabajo máximo

//=====[Declaration and initialization of public global objects]===============

/// @brief Entradas digitales para sensores y botones.
DigitalIn  buttonStop(BUTTON1); ///< Botón de parada de emergencia.
DigitalIn  encoderLeft(D0);     ///< Encoder de la rueda izquierda.
DigitalIn  encoderRight(D1);    ///< Encoder de la rueda derecha.

/// @brief Salidas digitales para control de motores.
DigitalOut motorLeft(D2);   ///< Control del motor izquierdo.
DigitalOut motorRight(D3);  ///< Control del motor derecho.

/// @brief Comunicación serie con la PC.
UnbufferedSerial uartUsb(USBTX, USBRX, 115200); ///< UART para comunicación USB.

/**
 * @class PID
 * @brief Implementación de un controlador PID básico.
 */
class PID {
private:
    float Kp, Ki, Kd;            ///< Coeficientes del controlador PID.
    float integral, prevError;   ///< Variables para el cálculo del control integral y derivativo.
    float dt;                    ///< Intervalo de muestreo en segundos.
    float outputMin, outputMax;  ///< Límites de la salida del PID.
    bool  saturationEnabled;     ///< Bandera para habilitar la saturación de la salida.

public:
    /**
     * @brief Constructor del controlador PID.
     * @param Kp Ganancia proporcional.
     * @param Ki Ganancia integral.
     * @param Kd Ganancia derivativa.
     * @param dt Intervalo de muestreo en segundos.
     */
    PID(float Kp, float Ki, float Kd, float dt) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->dt = dt;
        integral  = 0.0;
        prevError = 0.0;
        saturationEnabled = false;
    }

    /**
     * @brief Establece límites de salida para el PID.
     * @param min Valor mínimo de salida.
     * @param max Valor máximo de salida.
     */
    void setSaturation(float min, float max) {
        outputMin = min;
        outputMax = max;
        saturationEnabled = true;
    }

    

    /**
     * @brief Calcula la acción de control del PID.
     * @param setpoint Valor deseado.
     * @param measuredValue Valor medido.
     * @return Acción de control calculada.
     */
    float compute(float setpoint, float measuredValue) {
        float error = setpoint - measuredValue;

        // Componente proporcional
        float P = Kp * error;

        // Componente integral
        integral += error * dt;
        float I = Ki * integral;

        // Componente derivativa
        float derivative = (error - prevError) / dt;
        float D = Kd * derivative;

        prevError = error;

        float outPut = P + I + D;

        if (saturationEnabled){
            return max_(min_(outPut, outputMax), outputMin);
        }
        return outPut;
    }

    float min_(float a, float b) {
        return (a < b) ? a : b;
    }

    // Función para calcular el máximo entre dos valores
    float max_(float a, float b) {
        return (a > b) ? a : b;
}

};


//=====[Declaration and initialization of public global variables]===========
int   mode          = MANUAL_MODE; ///< Modo de operación del robot.
bool  emergencyStop = false;       ///< Estado de emergencia (true si activado).
float v             = 0.0f;        ///< Velocidad lineal del robot (m/s).
float w             = 0.0f;        ///< Velocidad angular del robot (rad/s).

// Variables del sistema de control
bool closeLoop = false;            ///< Activa o desactiva el control a lazo cerrado.

/// @brief Instancia del controlador PID para los motores.
/// @note Los valores de Kp, Ki y Kd pueden ajustarse para mejorar la respuesta.
PID pidController(1.0, 0.1, 0.01, PWM_STEP);


float leftSpeedMeasured  = 0.0f;  ///<  Velocidad estimada de la rueda izquierda
float rightSpeedMeasured = 0.0f;  ///<  Velocidad estimada de la rueda derecha
float leftDutyCyclePID   = 0.0f;  ///<  Duty cycle rueda izquierda calculada por PID (accion de control)
float rightDutyCyclePID  = 0.0f;  ///<  Duty cycle rueda derecha calculada por PID (accion de control)

int accumulatedTimePID     = 0;   ///<  Contador de tiempo PID
int accumulatedTimeEncoder = 0;   ///<  Contador de tiempo Encode
int accumulatedTimeMotor   = 0;   ///<  Contador de tiempo Motor
int accumulatedTimeStop    = 0;   ///<  Contador de tiempo BottonStop


//=====[Function Prototypes]===================================
/// @brief Inicialización de entradas digitales.
void inputsInit();

/// @brief Inicialización de salidas digitales.
void outputsInit();

/// @brief Verifica sensores y botón de emergencia.
void checkSensors();

/// @brief Modo de operación manual.
void manualMode();

/// @brief Modo de operación automática.
void automaticMode();

/// @brief Cambia entre modo manual y automático.
void checkMode();

/// @brief Calcula velocidades de las ruedas a partir de v y w.
void computeWheelSpeeds(float v, float w, float &v_L, float &v_R);

/// @brief Aplica velocidades a los motores.
void setMotorSpeeds(float v_L, float v_R);

/// @brief Control de motores en lazo cerrado.
void _setMotorSpeedsCloseLoop(float v_L, float v_R);

/// @brief Control de motores en lazo abierto.
void _setMotorSpeedsOpenLoop(float v_L, float v_R);

/// @brief Control PWM de los motores.
void motorControlPWM(DigitalOut *motor, float duty, float *timeElapsed);

/// @brief Lista de comandos disponibles para el usuario.
void availableCommands();

/// @brief Lee los encoders y calcula velocidades.
void readEncoders();


//=====[Utility Functions]=====================================================

/// @brief Calcula el mínimo entre dos valores.
float _min(float a, float b);

/// @brief Calcula el máximo entre dos valores.
float _max(float a, float b);

/// @brief Envia mensajes por UART.
void _printUart(const char* msg);


//=====[Main function, the program entry point after power on or reset]========

/**
 * @brief Función principal del programa.
 */
int main()
{
    inputsInit();
    outputsInit();
    pidController.setSaturation(DUTY_MIN, DUTY_MAX);
    
    while (true) {
        checkSensors();
        if (!emergencyStop) {
            if (mode == MANUAL_MODE)
                manualMode();
            else
                automaticMode();
        } 
        else {
            setMotorSpeeds(0.0f, 0.0f); //Asegurar una parada mas suave.
        }
        delay(TIME_INCREMENT_MS);
    }
}






//=====[Implementations of public functions]===================================
void inputsInit()
{
    //buttonStop.mode(PullDown); //En este caso ya tiene por default en la placa.
    encoderLeft.mode(PullDown);
    encoderRight.mode(PullDown);
}

void outputsInit()
{
    motorLeft  = OFF;
    motorRight = OFF;
}

void checkMode()
{
        // Verifica si se presionó 'm' y cambia entre modos
    char command = '\0';
    if (uartUsb.readable()) {
        uartUsb.read(&command, 1);

        if (command == 'm') { 
            if (mode == MANUAL_MODE) {
                mode = AUTOMATIC_MODE;
            } 
            else {
                mode = MANUAL_MODE;
            }
        }
    }

}

//Checkea boton de parada se checkea cada 100ms.
void checkSensors()
{
    accumulatedTimeStop = accumulatedTimeStop + TIME_INCREMENT_MS;
    if (accumulatedTimeStop > 100){ //HARDCODE
        if (buttonStop.read()) {
            emergencyStop = true;
        }
        accumulatedTimeStop = 0;
    }
}

//Setea velocidades manualmente por consola (cada 100ms).
void manualMode()
{
    //v y w son globales.
    float v_L, v_R;
    char command = '\0';

    accumulatedTimeMotor = accumulatedTimeMotor + TIME_INCREMENT_MS;
    if (accumulatedTimeMotor > 100){
        if (uartUsb.readable()) {
            uartUsb.read(&command, 1);
            switch (command) {
                case 'v':
                    v = _min(v + V_STEP, V_MAX);
                    break;
                case 's':
                    v = _max(v - V_STEP, V_MIN);
                    break;
                case 'a':
                    w = _min(w - W_STEP, W_MIN);
                    break;
                case 'd':
                    w = _min(w + W_STEP, W_MAX);
                    break;
                case 'q':
                //Caso en donde cierro todo, por ahora paro motores en seco.
                    v = 0.0f;
                    w = 0.0f;
                    break;
                case 'c':
                    closeLoop = !closeLoop;
                    char str[50];
                    sprintf ( str, "CloseLoop: %d\r\n", closeLoop );
                    _printUart(str);
                    break;
                default:
                    availableCommands();
                    break;
            }

        }
        accumulatedTimeMotor = 0;
    }
    computeWheelSpeeds(v, w, v_L, v_R);
    setMotorSpeeds(v_L, v_R);
}

//Modo automatico, si "q" entonces para el robot y pasa a mode=MANUAL_MODE=1.
void automaticMode()
{
    float v_L, v_R;

    // Implementar lógica de control autónomo aquí
    v = (rand() / (float)RAND_MAX) * V_MAX;
    w = (rand() / (float)RAND_MAX) * W_MAX;

    computeWheelSpeeds(v, w, v_L, v_R);
    setMotorSpeeds(v_L, v_R);
}

//Obtiene velocidades de cada rueda segun la velocidad lineal y angular.
void computeWheelSpeeds(float v, float w, float &v_L, float &v_R)
{
    /*
    R   = Distancia entre ruedas.
    V_R = Velocidad de la rueda derecha
    V_L = Velocidad de la rueda izquierda.
    V   = Velicidad Lineal del robot respecto a la terna de referencioa (CM).
    W   = Velocidad Angular del robot respecto a la terna de referencia (CM).  
    V = (V_R + V_L)/2
    W = (V_R - V_L)/R

    V_R = V + R*W/2
    V_L = V - R*W/2

    */
    v_L = v - (WHEEL_DISTANCE / 2) * w;
    v_R = v + (WHEEL_DISTANCE / 2) * w;

    //Asegurar que esten dentro del rango

}

// Setea velocidad de motor izquierdo y derecho
// si "closeLoop == true" activa el control a lazo cerrado.
void setMotorSpeeds(float v_L, float v_R)
{
    if (closeLoop == true)
        _setMotorSpeedsCloseLoop(v_L, v_R);
    else
        _setMotorSpeedsOpenLoop(v_L, v_R);
}

void _setMotorSpeedsOpenLoop(float v_L, float v_R) 
{
    static float leftDutyCycle  = 0.0f;
    static float rightDutyCycle = 0.0f;
    static float timeElapsed    = 0.0f;

    timeElapsed += PWM_STEP;
    if (timeElapsed >= PWM_PERIOD) {
        timeElapsed = 0.0f;
    }

    // Convertir velocidad en duty cycle (sin feedback)
    leftDutyCycle  = _min(_max(abs(v_L) / V_MAX, DUTY_MIN), DUTY_MAX);
    rightDutyCycle = _min(_max(abs(v_R) / V_MAX, DUTY_MIN), DUTY_MAX);

    motorControlPWM(&motorLeft , leftDutyCycle , &timeElapsed);  
    motorControlPWM(&motorRight, rightDutyCycle, &timeElapsed); 
}

// Setea velocidades, controlando las mismas mediante un PID
// accion de control es el DutyCytcle.
void _setMotorSpeedsCloseLoop(float v_L, float v_R) 
{
    readEncoders(); // Leer los encoders antes de calcular el PID
    // ****SpeedMeasured sera calculado cada 30 ms

    static float leftDutyCyclePID  = 0.0f;
    static float rightDutyCyclePID = 0.0f;
    static float timeElapsed       = 0.0f;

    // Usar PID para calcular el duty cycle corregido (cada 10ms, por cada llamada)
    leftDutyCyclePID  = pidController.compute(v_L, leftSpeedMeasured);
    rightDutyCyclePID = pidController.compute(v_R, rightSpeedMeasured);

    //timeElapsed += PWM_STEP;
    timeElapsed += PWM_STEP; //ACA LUEGO USAR UN VERDADERO PWM, va a funcionar por ser PWM_STEP=TIME_..
    if (timeElapsed >= PWM_PERIOD) {
        timeElapsed = 0.0f;
    }

    //Duty ya fue limitado en PID pero en la sig. func. tambien, se lo deja por completitud, mejorar mas adelante
    motorControlPWM(&motorLeft , leftDutyCyclePID , &timeElapsed);  
    motorControlPWM(&motorRight, rightDutyCyclePID, &timeElapsed); 
}


//Funcion para controlar el motor mediante DutyCycle.
void motorControlPWM(DigitalOut *motor, float duty, float *timeElapsed)
{
    /*
    Compara el tiempo actual con el duty cycle y enciende o apaga el motor:
        - Si el tiempo actual está dentro del duty cycle, enciende el motor.
        - Si el tiempo actual supera el duty cycle, apaga el motor.

    Compara el tiempo actual con 
    D_L * T_PWM y D_R*T_PWM
    donde:
    D_L = |V_L|/V_L{max}
    D_R = |V_R|/V_R{max}
    */

    // Asegurar que el duty cycle está entre 0 y 1.
    duty = _min(_max(duty, DUTY_MAX), DUTY_MAX);
    // Control PWM
    //*motor = (*timeElapsed < duty * PWM_PERIOD);
    if (*timeElapsed < duty * PWM_PERIOD)
        *motor = ON;
    else
        *motor = OFF;

}
//Calcula velocidades cada 30ms
void readEncoders() {
    static int lastStateLeft   = 0;
    static int lastStateRight  = 0;
    static int pulseCountLeft  = 0;
    static int pulseCountRight = 0;
    
    int currentStateLeft  = encoderLeft.read();
    int currentStateRight = encoderRight.read();

    // Solo se incrementa el contador si el encoder cambia de OFF a ON (FlancoAscendente)
    if (currentStateLeft == ON && lastStateLeft == OFF) {
        pulseCountLeft++;
    }
    if (currentStateRight == ON && lastStateRight == OFF) {
        pulseCountRight++;
    }

    lastStateLeft  = currentStateLeft;
    lastStateRight = currentStateRight;

    accumulatedTimeEncoder = accumulatedTimeEncoder + TIME_INCREMENT_MS;
    if (accumulatedTimeEncoder > 30){
        // Calcular velocidad en RPM (suponiendo 20 pulsos por vuelta)
        float wheelPerimeter = WHEEL_DIAMETER * 3.1416;
        leftSpeedMeasured  = (pulseCountLeft  / 20.0f) * wheelPerimeter/ PWM_PERIOD;  // m/s
        rightSpeedMeasured = (pulseCountRight / 20.0f) * wheelPerimeter / PWM_PERIOD; // m/s
        
        pulseCountLeft  = 0; // Resetear el conteo de pulsos cada ciclo
        pulseCountRight = 0;

        accumulatedTimeEncoder = 0;
    }
}


void availableCommands()
{
    /*
    Basado en el siguiente caso ejemplo
    potentiometerReading = potentiometer.read();
    sprintf ( str, "Potentiometer: %.2f\r\n", potentiometerReading );
    stringLength = strlen(str);
    uartUsb.write( str, stringLength );
    */
    _printUart("Available commands:\r\n");
    _printUart("Press 'v' to increase speed\r\n");
    _printUart("Press 's' to decrease speed\r\n");
    _printUart("Press 'd' to decrease rotation\r\n");
    _printUart("Press 'a' to increase rotation\r\n");
    _printUart("Press 'q' to stop\r\n");
}

void _printUart(const char* msg) {
    uartUsb.write(msg, strlen(msg));
}

float _min(float a, float b) {
    return (a < b) ? a : b;
}

// Función para calcular el máximo entre dos valores
float _max(float a, float b) {
    return (a > b) ? a : b;
}
