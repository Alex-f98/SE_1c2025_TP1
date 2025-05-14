// kinematics.cpp
#include "kinematics.h"

void computeWheelVelocities(const Kinematics* kin, const Velocity* vel, WheelVelocities* out) {
    float r = kin->wheelRadius;

    if (kin->type == DIFFERENTIAL) {
        /*
        r = radio de las ruedas.
        L   = Distancia entre ruedas.
        V_R = Velocidad de la rueda derecha
        V_L = Velocidad de la rueda izquierda.
        V   = Velicidad Lineal del robot respecto a la terna de referencioa (CM).
        W   = Velocidad Angular del robot respecto a la terna de referencia (CM).  
        V = (V_R + V_L)/2
        W = (V_R - V_L)/L

        V_R = V + L*W/2
        V_L = V - L*W/2

        w_r = V_R/r
        w_l = V_L/r

        */
        float L = kin->wheelBase;
        out->w1 = (vel->vx - vel->omega * L / 2) / r; // izquierda
        out->w2 = (vel->vx + vel->omega * L / 2) / r; // derecha
        out->w3 = 0.0f;
        out->w4 = 0.0f;
    }

    else if (kin->type == OMNIDIRECTIONAL) {
        float lx = kin->lx;
        float ly = kin->ly;

        // Para un robot de 3 ruedas Mecanum o holonómico (simplificado)
        out->w1 = (vel->vx - vel->vy - (lx + ly) * vel->omega) / r;
        out->w2 = (vel->vx + vel->vy + (lx + ly) * vel->omega) / r;
        out->w3 = (vel->vx - vel->vy + (lx + ly) * vel->omega) / r;
        out->w4 = 0; // No usada si solo hay 3 ruedas
    }
}



/*
int main() {
    #include <cstdio>
    #include "robot_config.h"

    Kinematics kin = {
        .type        = DIFFERENTIAL,
        .wheelRadius = WHEEL_DIAMETER/2.0,
        .wheelBase   = WHEEL_DISTANCE
    };

    Velocity vel = {
        .vx    = 0.2f,
        .vy    = 0.0f,
        .omega = 1.0f
    };

    WheelVelocities w;
    computeWheelVelocities(&kin, &vel, &w);

    printf("Wheel 1: %.2f, Wheel 2: %.2f\n", w.w1, w.w2);
}
*/