#include "utils.h"
#include "pid.h"

PID::PID(float Kp, float Ki, float Kd, float dt) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->dt = dt;
    integral  = 0.0;
    prevError = 0.0;
    saturationEnabled = false;
}



void PID::setSaturation(float min, float max) {
    outputMin = min;
    outputMax = max;
    saturationEnabled = true;
}


float PID::compute(float setpoint, float measuredValue) {
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
        return _max(_min(outPut, outputMax), outputMin);
    }
    return outPut;
}

void PID::reset() {
    integral  = 0.0f;
    prevError = 0.0f;
}

