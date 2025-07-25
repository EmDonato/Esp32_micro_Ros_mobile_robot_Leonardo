#include "PID.h"
#include <Arduino.h>

PID::PID(float Kp, float Ki, float Kd,
         float dt,
         float integratorMin, float integratorMax,
         int outputMin, int outputMax)
  : _Kp(Kp), _Ki(Ki), _Kd(Kd),
    _dt(dt > 0.0001f ? dt : 0.001f),
    _integrator(0), _prevError(0), _prevMeas(0), _dFiltered(0),
    _dFilterAlpha(0.1f),
    _intMin(integratorMin), _intMax(integratorMax),
    _outMin(outputMin), _outMax(outputMax),
    _deadZone(2200),
    error(0), output(0),
    _ffA(0.077f), _ffB(-85.62f),
    _useFeedforward(true) {}

int PID::compute(float setpoint, float measurement) {
  error = setpoint - measurement;

  float P = _Kp * error;

  float D = 0;
  if (_dt > 0) {
    float dMeas = (measurement - _prevMeas) / _dt;
    _dFiltered = _dFilterAlpha * dMeas + (1.0f - _dFilterAlpha) * _dFiltered;
    D = -_Kd * _dFiltered;
  }

  float I_temp = _integrator + _Ki * error * _dt;
  float u_un = P + I_temp + D;

  if (u_un > _outMin && u_un < _outMax) {
    _integrator = I_temp;
  }
  _integrator = constrain(_integrator, _intMin, _intMax);

  float pid_control = P + _integrator + D;
  float ff_control = _useFeedforward ? (_ffA * setpoint + _ffB) : 0.0f;

  int raw = (int)round(pid_control + ff_control);

  if (raw > 0) {
    raw += _deadZone;
  } else if (raw < 0) {
    raw -= _deadZone;
  }

  output = constrain(raw, _outMin, _outMax);
  _prevMeas = measurement;

  return output;
}

void PID::reset() {
  _integrator = 0;
  _prevError = 0;
  _prevMeas = 0;
  _dFiltered = 0;
  error = 0;
  output = 0;
}

void PID::setTunings(float Kp, float Ki, float Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
}

void PID::setWindupLimits(float integratorMin, float integratorMax) {
  _intMin = integratorMin;
  _intMax = integratorMax;
}

void PID::setOutputLimits(int outputMin, int outputMax) {
  _outMin = outputMin;
  _outMax = outputMax;
}

void PID::setInterval(float dt) {
  _dt = (dt > 0.0001f) ? dt : 0.001f;
}

void PID::setDeadZone(int dz) {
  _deadZone = abs(dz);
}

void PID::setDerivativeFilterAlpha(float alpha) {
  _dFilterAlpha = constrain(alpha, 0.0f, 1.0f);
}

void PID::setFeedforwardParams(float a, float b) {
  _ffA = a;
  _ffB = b;
}

void PID::enableFeedforward(bool enable) {
  _useFeedforward = enable;
}

bool PID::isSaturated() const {
  return output == _outMin || output == _outMax;
}

