#ifndef ENCODER_COUNTER_H
#define ENCODER_COUNTER_H

#include "mbed.h"
#include "pin_out_default.h"
#include "robot_config.h"
#include "arm_book_lib.h"

class EncoderCounter {
public:
    /** \brief Contructor encoder optico, cuenta pulsos mediante interrupcciones.
    * 
    * \param pin Pin del encoder fisico.
    * \param encoderResolution Numero de ticks para una revolucion completa (en este caso seran 20 ticks por revolucion donde el disco esta luego del reductor)
    * \ref https://os.mbed.com/users/charly/code/mRotaryEncoder-os/docs/tip/mRotaryEncoder_8cpp_source.html
    */
    EncoderCounter(PinName pin, int EncoderResolution = 20,  PinMode pullMode = PullUp);
    ~EncoderCounter();
    
    // Conteo básico
    long getCount() const;           // Obtener conteo actual
    long getAndResetCount();         // Obtener y resetear (atomic)
    void resetCount();               // Resetear a cero
    void CallbackOnPulse();
    
private:
    InterruptIn*  _pinEncoder;
    PinMode       _pinMode;
    volatile long _pulseCount;
    volatile long _lastPulseCount;
    long          _encoderResolution;
};
#endif