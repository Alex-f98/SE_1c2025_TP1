#ifndef ENCODER_VELOCITY_H
#define ENCODER_VELOCITY_H

#include "mbed.h"
#include "encoderCounter.h"
#include "robot_config.h"
class EncoderVelocity {
public:
    EncoderVelocity(PinName pin, int encoderResolution = 20, float samplingInterval = TENCODER_UPDATE, PinMode pullMode = PullUp); // en ms
    ~EncoderVelocity();

    int getRPM() const;
    double getAngularPosition() const;
    double getAngularVelocity() const;
    int getTicks() const;
    void setResolution(int resolution);
    void setSamplingInterval(float samplingInterval);
    void resetVelocity();    

private:
    double ticks2angle(int ticks) const;
    void update(); // función llamada por el ticker

    EncoderCounter* _encoder;
    Ticker _ticker;

    int    _lastCount;
    float  _samplingInterval;
    int    _encoderResolution;
    int    _ticks;
    double _angularPosition;
    double _angularVelocity;
    double _lastAngularVelocity;

};

#endif
