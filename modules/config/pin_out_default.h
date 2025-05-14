#ifndef _PIN_OUT_DEFAULT_H_
#define _PIN_OUT_DEFAULT_H_

#include "mbed.h"

//=====[Digital Inputs]========================================
#define BUTTON_STOP_PIN        BUTTON1
#define ENCODER_LEFT_PIN       D0
#define ENCODER_RIGHT_PIN      D1

//=====[Digital Outputs]=======================================
#define MOTOR_LEFT_PIN         PA_0 //D2
#define MOTOR_RIGHT_PIN        PB_4 //D3

//=====[Serial Communication]==================================
#define USBTX_PIN              USBTX
#define USBRX_PIN              USBRX
#define USB_BAUD_RATE          115200

#endif // _PIN_OUT_DEFAULT_H_
