#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
public:
  PID(float Kp, float Ki, float Kd,
      float dt,
      float integratorMin, float integratorMax,
      int outputMin, int outputMax);

  int compute(float setpoint, float measurement);

  void reset();

  void setTunings(float Kp, float Ki, float Kd);
  void setWindupLimits(float integratorMin, float integratorMax);
  void setOutputLimits(int outputMin, int outputMax);
  void setInterval(float dt);
  void setDeadZone(int dz);
  void setDerivativeFilterAlpha(float alpha);

  void setFeedforwardParams(float a, float b);
  void enableFeedforward(bool enable);

  bool isSaturated() const;

  float error;
  int output;

private:
  float _Kp, _Ki, _Kd;
  float _dt;
  float _integrator;
  float _prevError;
  float _prevMeas;
  float _dFiltered;
  float _dFilterAlpha;
  float _intMin, _intMax;
  int _outMin, _outMax;
  int _deadZone;

  // Feedforward
  float _ffA = 0.077f;
  float _ffB = -85.62f;
  bool _useFeedforward = true;
};

#endif // PID_H

