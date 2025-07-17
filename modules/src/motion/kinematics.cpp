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
        //ref: https://blog.csdn.net/weixin_45929038/article/details/122632351
        float lx = kin->lx;
        float ly = kin->ly;
        float sqrt3_2 = 0.8660254037844386;
        
        out->w1 = (0   + vel->vy +lx  * vel->omega) / r;
        out->w2 = (-sqrt(3)*vel->vx / 2 - vel->vy/2 + ly*vel->omega) / r;
        out->w3 = (sqrt(3)*vel->vx / 2 - vel->vy/2 - ly*vel->omega) / r;
        out->w4 = 0; // No usada si solo hay 3 ruedas
    }
    else if  (kin->type == SKIDSTEER){
        //TODO
        
    }
}
