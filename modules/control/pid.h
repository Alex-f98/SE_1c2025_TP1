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
    PID(float Kp, float Ki, float Kd, float dt);

    /**
     * @brief Establece límites de salida para el PID.
     * @param min Valor mínimo de salida.
     * @param max Valor máximo de salida.
     */
    void setSaturation(float min, float max);
    

    /**
     * @brief Calcula la acción de control del PID.
     * @param setpoint Valor deseado.
     * @param measuredValue Valor medido.
     * @return Acción de control calculada.
     */
    float compute(float setpoint, float measuredValue);

    void reset();

};
