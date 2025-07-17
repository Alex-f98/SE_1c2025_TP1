/**
 * @file motor_control.cpp
 * @brief Implementación del controlador de motor (lazo abierto/cerrado).
 */

#include "motor_control.h"


/**
 * @brief Constructor de MotorController. Inicializa motor, encoder y PID.
 */
MotorController::MotorController(PinName pwmPin, PinName encoderPin,
                                 float kc , float ti, float td, float ts,
                                 float encoderInterval,
                                 float motorPwmPeriod, float motorInterval)
    : motor_(pwmPin) 
    , encoderPin_(encoderPin)
    , pid_(kc, ti, td, ts), kc_(kc), ti_(ti), td_(td), ts_(ts)
{
    
    motorInit(&motor_, pwmPin);
    setMotorPeriod(&motor_, motorPwmPeriod);
    _mTicker.attach(callback(this, &MotorController::motorUpdateDC), motorInterval); //deprecado(es un lio usar crono,typedef...).



    referenceVelocity_ = 0.0f;
    controlClosedLoop_ = CLOSED_LOOP_CONTROL;
    
    encoder_ = new EncoderVelocity(encoderPin_, RESOLUTION, encoderInterval);

    //pid_.setTunings(kc_, ti_, td_); //Kc, Ti, Td, RATE);
    //pid_.setInterval(ts_);
    pid_.setInputLimits(W_MIN, W_MAX);
    pid_.setOutputLimits(W_MIN, W_MAX);
    pid_.setMode(AUTO_MODE);
    pid_.setBias(referenceVelocity_);


    //TODO: no activar si no hay se usa!
    _cTicker.attach(callback(this, &MotorController::controlUpdateDC), ts_); //deprecado(es un lio usar crono,typedef...).
}

MotorController::~MotorController() {
    _cTicker.detach();
    delete encoder_;
}

/**
 * @brief Activa o desactiva el control en lazo cerrado.
 */
void MotorController::enableClosedLoop(bool enable)
{
    controlClosedLoop_ = enable;
    //if (controlClosedLoop)
    //    encoderInit(&encoder, encoderPin);
}

/**
 * @brief Define la velocidad deseada del motor.
 */
void MotorController::setTargetVelocity(float reference)
{
    referenceVelocity_ = reference;
}

/**
 * @brief Define los parámetros del controlador PID.
 */
void MotorController::setMotorControlParameters(float kc, float ti, float td, float ts, float wMaxIn, float wMinIn)
{
    _cTicker.detach();
    kc_ = kc;
    ti_ = ti;
    td_ = td;
    ts_ = ts;
    pid_.setTunings(kc_, ti_, td_);
    pid_.setInterval(ts_);
    pid_.setInputLimits(wMinIn, wMaxIn);
    _cTicker.attach(callback(this, &MotorController::controlUpdateDC), ts_);
}

/**
 * @brief Define los parámetros del encoder.
 */
void MotorController::setEncoderParameters(int resolution, float samplingInterval)
{
    encoder_->setResolution(resolution);
    encoder_->setSamplingInterval(samplingInterval);
}

/**
 * @brief Define los parámetros del motor.
 */
void MotorController::setMotorParameters(float wMaxIn, float wMinIn, float dutyCycleMaxOut, float dutyCycleMinOut, float motorPwmPeriod)
{ 
    pid_.setOutputLimits(wMinIn, wMaxIn);
    setMotorParameter(&motor_, wMaxIn, wMinIn, dutyCycleMaxOut, dutyCycleMinOut);
    setMotorPeriod(&motor_, motorPwmPeriod);
}

/**
 * @brief Devuelve la velocidad observada por el encoder.
 */
float MotorController::getMeasuredVelocity()
{
    if (!controlClosedLoop_)
        return referenceVelocity_;

    return encoder_->getAngularVelocity();
}

/**
 * @brief Devuelve la posición angular observada por el encoder.
 */
float MotorController::getAngularPosition(){
    return encoder_->getAngularPosition();
}

/**
 * @brief Retorna la velocidad objetivo configurada.
 */
float MotorController::getTargetVelocity(){ 
    return referenceVelocity_; 
}

int MotorController::getMeasuredRPM(){
    return encoder_->getRPM();
}
/**
 * @brief Ejecuta una iteración del controlador, ya sea abierto o cerrado.
 *          es llamado por el ticker.
 */
void MotorController::controlUpdateDC()
{
    if (controlClosedLoop_) {
        float observedVelocity = encoder_->getAngularVelocity(); //fobs> fcontrol.
        pid_.setSetPoint(referenceVelocity_);
        pid_.setProcessValue(observedVelocity); //pid_.setProcessValue((encoder_->getAngularVelocity())
        controlOutput_ = pid_.compute();
        //motorUpdate(&motor_, &controlOutput);
    } 
    //else {
    //    motorUpdate(&motor_, &referenceVelocity_);
    //}
}

void MotorController::motorUpdateDC()
{
    if (controlClosedLoop_) {
        motorUpdate(&motor_, &controlOutput_);
    } 
    else {
        motorUpdate(&motor_, &referenceVelocity_);
    }
}

/**
 * @brief Detiene el motor, reinicia el PID y la velocidad objetivo.
 */
void MotorController::stop()
{
    if (controlClosedLoop_)
        pid_.reset();
    motorStop(&motor_);
    _cTicker.detach();
    encoder_->resetVelocity();
    referenceVelocity_ = 0.0f;
    _mTicker.detach();

}
