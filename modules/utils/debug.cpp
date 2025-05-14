#include "debug.h"
#include "mbed.h"
#include "robot_config.h"
#include "utils.h"
static float acumulatedTimeDebug = 0.0f;

void getWheelVelocities(MotorController* leftController,
                        MotorController* rightController,
                        float* vl, float* vr)
{
    *vl = leftController->get_measured_velocity();
    *vr = rightController->get_measured_velocity();
}



void printStatus(ModeManager* manager,
                 MotorController* leftController,
                 MotorController* rightController,
                 bool emergencyStop)
{
    acumulatedTimeDebug += TIME_INCREMENT_MS;

    if (acumulatedTimeDebug > TIME_DEBUG) {
        float v = 0, w = 0;
        float wl = 0, wr = 0;
        float wl_ref = 0, wr_ref = 0;
        float pwm_l = 0, pwm_r = 0;
        bool closeLoop = false;
        OperationMode mode;
        //velocidades medidas del encoder
        getWheelVelocities(leftController, rightController, &wl, &wr);

        wl_ref = leftController->get_target_velocity();
        wr_ref = rightController->get_target_velocity();

        float err_l = wl_ref - wl;
        float err_r = wr_ref - wr;

        pwm_l = wl_ref/MAX_ANGULAR_VELOCITY;
        pwm_r = wr_ref/MAX_ANGULAR_VELOCITY;

        //velocidades que se le envia al manager
        info(manager, &mode, &v, &w, &closeLoop);

        // Formatear valores a enteros con 2 decimales
        int wl_int = (int)wl, wl_dec = (int)((wl - wl_int) * 100);
        int wr_int = (int)wr, wr_dec = (int)((wr - wr_int) * 100);

        int wl_intr = (int)wl_ref, wl_decr = (int)((wl_ref - wl_intr) * 100);
        int wr_intr = (int)wr_ref, wr_decr = (int)((wr_ref - wr_intr) * 100);

        int pwm_lint = (int)pwm_l, pwm_ldec = (int)((pwm_l - pwm_lint) * 100);
        int pwm_rint = (int)pwm_r, pwm_rdec = (int)((pwm_r - pwm_rint) * 100);

        int err_lint = (int)err_l, err_ldec = (int)((err_l - err_lint) * 100);
        int err_rint = (int)err_r, err_rdec = (int)((err_r - err_rint) * 100);

        int v_int = (int)v, v_dec = (int)((v - v_int) * 100);
        int w_int = (int)w, w_dec = (int)((w - w_int) * 100);
        
        int vmax_int = (int)V_MAX, vmax_dec = (int)((V_MAX - vmax_int) * 100);
        int wmax_int = (int)MAX_ANGULAR_VELOCITY, wmax_dec = (int)((MAX_ANGULAR_VELOCITY - wmax_int) * 100);

        char buffer[512];
        snprintf(buffer, sizeof(buffer),
            CURSOR_HOME
            "+------------------------+\r\n"
            "|  State of Robot v1     |\r\n"
            "|------------------------|\r\n"
            "| Close loop     | %3s  |\r\n"
            "| MODE           | %-10s |\r\n"
            "| STOP_BUTTON    | %s       |\r\n"
            "| w_izq_ref      | %d.%02d rad/s |\r\n"
            "| w_der_ref      | %d.%02d rad/s |\r\n"
            "| w_izq_obs      | %d.%02d rad/s |\r\n"
            "| w_der_obs      | %d.%02d rad/s |\r\n"
            "| pwm_l          | %d.%02d     |\r\n"
            "| pwm_r          | %d.%02d     |\r\n"
            "| Vel_lin        | %d.%02d m/s |\r\n"
            "| Vel_ang        | %d.%02d rad/s |\r\n"
            "| V_max          | %d.%02d     |\r\n"
            "| W_max          | %d.%02d     |\r\n"
            "+------------------------+\r\n",
            closeLoop ? "ON" : "OFF",
            mode == MODE_MANUAL ? "MANUAL" :
            mode == MODE_AUTOMATIC ? "AUTOMATIC" : "STOP",
            emergencyStop ? "ON" : "OFF",
            wl_intr, wl_decr,
            wr_intr, wr_decr,
            wl_int, wl_dec,
            wr_int, wr_dec,
            err_lint, err_ldec,
            err_rint, err_rdec,
            pwm_lint, pwm_ldec,
            pwm_rint, pwm_rdec,
            v_int, v_dec,
            w_int, w_dec,
            vmax_int, vmax_dec,
            wmax_int, wmax_dec
        );

        _printUart(buffer);
        acumulatedTimeDebug = 0;
    }
}

void observedState(MotorController* leftController, MotorController* rightController){
    /*
        float leftSpeedMeasured   Velocidad estimada de la rueda izquierda
        float rightSpeedMeasured
    */
    acumulatedTimeDebug = acumulatedTimeDebug + TIME_DEBUG;
    if (acumulatedTimeDebug > TIME_DEBUG){
        char str[50];
        
        float vl = 0, vr = 0;
        getWheelVelocities(leftController, rightController, &vl, &vr);
        sprintf ( str, "V_L_OBS: %f\r\n", vl ); _printUart(str);
        sprintf ( str, "V_R_OBS: %f\r\n", vr ); _printUart(str);
        acumulatedTimeDebug = 0;
    }
    
}
