#include "encoderCounter.h"

// Constructor - inicializa todas las variables miembro
EncoderCounter::EncoderCounter(PinName pin, int encoderResolution, PinMode pullMode) 
    :
    _pinMode(pullMode), 
    _pulseCount(0),  _lastPulseCount(0),  
    _encoderResolution(encoderResolution)
{
    //Default _pinMode = PullUp
    _pinEncoder = new InterruptIn(pin, _pinMode);    
    _pinEncoder->rise(callback(this, &EncoderCounter::CallbackOnPulse)); // callback en flanco de subida

    //conviene usar ticker para debounce?
    //es mejor un cap 100uf en la entrada digital?
}

EncoderCounter::~EncoderCounter() {
    delete _pinEncoder;
}

// Obtener conteo actual (solo lectura->const)
long EncoderCounter::getCount() const {
    return _pulseCount;
}

// Obtener conteo y resetear de forma atómica
long EncoderCounter::getAndResetCount() {
    __disable_irq();
    _lastPulseCount = _pulseCount; //para que quiero lastCOunt?=??
    _pulseCount     = 0; 
    __enable_irq();
    return _lastPulseCount;
}

// Resetear contador a cero
void EncoderCounter::resetCount() {
    __disable_irq();
    _lastPulseCount = _pulseCount;
    _pulseCount = 0;
    __enable_irq();
}

// Método callback que será llamado desde la ISR. 
void EncoderCounter::CallbackOnPulse() {
    _pulseCount++;
}
