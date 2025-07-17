#include "encoderVelocity.h"

#define M_PI 3.14159265

//https://os.mbed.com/docs/mbed-os/v6.16/mbed-os-api-doxy/classmbed_1_1_ticker.html#aecc275cbe328728bbee4d153e5b6b5b5
EncoderVelocity::EncoderVelocity(PinName pin, int encoderResolution, float samplingInterval, PinMode pullMode)
    : _encoderResolution(encoderResolution),
      _samplingInterval(samplingInterval),
      _lastCount(0),
      _ticks(0),
      _angularPosition(0.0),
      _angularVelocity(0.0),
      _lastAngularVelocity(0.0)
{
    _encoder = new EncoderCounter(pin, encoderResolution, pullMode);
    //_ticker.attach(callback(this, &EncoderVelocity::update), _samplingInterval);
    _ticker.attach(callback(this, &EncoderVelocity::update), _samplingInterval);

}

EncoderVelocity::~EncoderVelocity() {
    _ticker.detach();
    delete _encoder;
}

int EncoderVelocity::getRPM() const {
    // velocidad en RPM (revoluciones por minuto)
    // w[rad/s]*60 [s/min]/(2π [rad/rev])
    return int((_angularVelocity * 60.0) / (2.0 * M_PI));
}

double EncoderVelocity::getAngularVelocity() const {
    //return_lastAngularVelocity*(1-alpha) + _angularVelocity*alpha) 
    return _angularVelocity;
}

double EncoderVelocity::getAngularPosition() const {
    return _angularPosition;
}

int EncoderVelocity::getTicks() const {
    return _ticks;
}

void EncoderVelocity::setResolution(int resolution) {
    _encoderResolution = resolution;
}

void EncoderVelocity::setSamplingInterval(float samplingInterval) {
    _samplingInterval = samplingInterval;
    _ticker.detach();
    _ticker.attach(callback(this, &EncoderVelocity::update), _samplingInterval);
}

void EncoderVelocity::resetVelocity() {
    CriticalSectionLock lock;
    _encoder->resetCount();
    _lastCount           = _encoder->getCount();
    _angularVelocity     = 0.0;
    _angularPosition     = 0.0;
    _lastAngularVelocity = 0.0;
    _ticks               = 0;
}

double EncoderVelocity::ticks2angle(int ticks) const {
    // Cada tick equivale a 2π / resolution radianes
    // Los ticks son de modulo _encoderResolution
    ticks = ticks % _encoderResolution;
    return double(ticks) * (2.0 * M_PI) / double(_encoderResolution);
}

// Se actualiza con ticker.
void EncoderVelocity::update() {
    CriticalSectionLock lock;
    _lastAngularVelocity  = _angularVelocity;
    int currentCount      = _encoder->getCount();
    int deltaTicks        = currentCount - _lastCount;
    _lastCount            = currentCount;

    _ticks               += deltaTicks;

    // Actualiza la posición y velocidad angular
    double deltaAngle     = ticks2angle(deltaTicks);
    _angularPosition     += deltaAngle;
    _angularVelocity      = deltaAngle / _samplingInterval;
    
}
