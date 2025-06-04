#include "mbed.h"
#include "encoder.h"

//=====[Declaration of private defines]========================================
#define TENCODER_UPDATE_ 100

//=====[Declaration of private data types]=====================================

//=====[Declaration and initialization of public global objects]===============

//=====[Declaration of external public global variables]=======================

//=====[Declaration and initialization of public global variables]=============

//=====[Declaration and initialization of private global variables]============

//=====[Declarations (prototypes) of private functions]========================

//=====[Implementations of public functions]===================================


//void encoderInit(EncoderSpeeds* encoder, DigitalIn* pin)
void encoderInit(EncoderSpeeds* enc, PinName pin)
{  //lo dejo por completitud
    //new (&enc->pin) DigitalIn(pin); // placement new para inicializar `pin` in-place
    enc->state = ENCODER_IDLE;
    enc->speed = 0.0f;
    enc->lastState = 0;
    enc->pulseCount = 0;
    enc->accumulatedTimeEncoder = 0;
}

void encoderUpdate(EncoderSpeeds* enc) 
{
    int currentState = enc->pin.read();

    switch (enc->state) {
        case ENCODER_IDLE:
            if (currentState == ON && enc->lastState == OFF) {
                enc->state = ENCODER_ACTIVE;
            }
            break;

        case ENCODER_ACTIVE:
            enc->pulseCount++;
            enc->state = ENCODER_WAIT;
            break;

        case ENCODER_WAIT:
            if (currentState == OFF) {
                enc->state = ENCODER_IDLE;
            }
            break;
    }
    enc->lastState = currentState;
    enc->accumulatedTimeEncoder += TIME_INCREMENT_MS;
     // Si pasó suficiente tiempo, calcula la velocidad angular
    if (enc->accumulatedTimeEncoder >= TENCODER_UPDATE_) {
        float wheelPerimeter = WHEEL_DIAMETER * 3.1416f;
        //enc->speed = (enc->pulseCount / DISC_TICS) * wheelPerimeter / (enc->accumulatedTimeEncoder/1000.0f); //m/s
        float revolutions = enc->pulseCount / DISC_TICS;
        enc->speed = (2.0f * 3.1416f * revolutions) / (enc->accumulatedTimeEncoder / 1000.0f); // rad/s
        enc->pulseCount = 0;
        enc->accumulatedTimeEncoder = 0;
    }
}
//sug: CriticalSectionLock  para que no joda con interrupcciones

//EncoderSpeeds encodersRead(){ //solo son 1 float, no es mucho(rapido).
//    return speedMeasured;
//}
void encoderRead(EncoderSpeeds* enc, float* observedSpeed) {
     *observedSpeed = enc->speed;
}

void encoderReset(EncoderSpeeds* enc){
    // Reinicia el estado completo del encoder
    enc->state      = ENCODER_IDLE;
    enc->speed      = 0.0f;
    enc->lastState  = 0;
    enc->pulseCount = 0;
    enc->accumulatedTimeEncoder = 0;
}

