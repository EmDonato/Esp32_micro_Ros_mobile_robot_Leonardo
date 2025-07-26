#include "MotorDriver.h"

// Constructor for 1 motor
L298N::L298N(int channel0, int frequency, int resolution, int numb_motor)
    : _channel0(channel0), _channel1(-1), _frequency(frequency), _resolution(resolution) {
    this->numb_motor = numb_motor;
}

// Constructor for 2 motors
L298N::L298N(int channel0, int channel1, int frequency, int resolution, int numb_motor)
    : _channel0(channel0), _channel1(channel1), _frequency(frequency), _resolution(resolution) {
    this->numb_motor = numb_motor;
}

// Initialization for 1 motor
void L298N::init(int enA, int in1, int in2) {
    _enA = enA;
    _in1 = in1;
    _in2 = in2;

    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_enA, OUTPUT);

    ledcSetup(_channel0, _frequency, _resolution);
    ledcAttachPin(_enA, _channel0);
    ledcWrite(_channel0, 0);
}

// Initialization for 2 motors
void L298N::init(int enA, int in1, int in2, int in3, int in4, int enB) {
    _enA = enA;
    _in1 = in1;
    _in2 = in2;
    _in3 = in3;
    _in4 = in4;
    _enB = enB;

    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_in3, OUTPUT);
    pinMode(_in4, OUTPUT);
    pinMode(_enA, OUTPUT);
    pinMode(_enB, OUTPUT);

    ledcSetup(_channel0, _frequency, _resolution);
    ledcAttachPin(_enA, _channel0);

    if (_channel1 != -1) {
        ledcSetup(_channel1, _frequency, _resolution);
        ledcAttachPin(_enB, _channel1);
    }

    ledcWrite(_channel0, 0);
    if (_channel1 != -1) {
        ledcWrite(_channel1, 0);
    }
}

// Move motor 0 forward
void L298N::forward(int speed_pwm) {
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
    ledcWrite(_channel0, speed_pwm);
}

// Move motor 0 backward
void L298N::backward(int speed_pwm) {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
    ledcWrite(_channel0, speed_pwm);
}

// Stop all motors
void L298N::stop() {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);

    if (numb_motor == 2) {
        digitalWrite(_in3, LOW);
        digitalWrite(_in4, LOW);
    }
}

// Move selected motor forward
void L298N::forward(int speed_pwm, int motorID) {
    if (!isValidMotorID(motorID)) return;

    if (motorID == 0) {
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, LOW);
        ledcWrite(_channel0, speed_pwm);
    } else if (motorID == 1) {
        digitalWrite(_in3, LOW);
        digitalWrite(_in4, HIGH);
        ledcWrite(_channel1, speed_pwm);
    }
}

// Move selected motor backward
void L298N::backward(int speed_pwm, int motorID) {
    if (!isValidMotorID(motorID)) return;

    if (motorID == 0) {
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, HIGH);
        ledcWrite(_channel0, speed_pwm);
    } else if (motorID == 1) {
        digitalWrite(_in3, HIGH);
        digitalWrite(_in4, LOW);
        ledcWrite(_channel1, speed_pwm);
    }
}

// Validate motor ID
bool L298N::isValidMotorID(int motorID) {
    return (motorID == 0 || motorID == 1);
}



