#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>

// Abstract base class for motor drivers
class MotorDriver {

protected:
    int numb_motor;  // Number of motors

public:
    MotorDriver() : numb_motor(0) {}

    // Pure virtual methods to be implemented by derived classes

    virtual void stop() = 0;
    virtual void forward(int speed_pwm) = 0;
    virtual void backward(int speed_pwm) = 0;
    virtual ~MotorDriver() {}  // Virtual destructor for safe inheritance
};

// Derived class L298N that implements MotorDriver
class L298N : public MotorDriver {

private:
    int _channel0;        // PWM channel0
    int _channel1;        // PWM channel1
    int _frequency;       // PWM frequency
    int _resolution;      // PWM resolution
    int _enA;             // Enable pin A
    int _in1;             // IN1 pin
    int _in2;             // IN2 pin
    int _in3;             // IN3 pin
    int _in4;             // IN4 pin
    int _enB;             // Enable pin B

public:
    // Constructor for 1 motor setup (3 pins)
    L298N(int channel0, int frequency, int resolution, int numb_motor);

    // Constructor for 2 motors setup (6 pins)
    L298N(int channel0, int channel1, int frequency, int resolution, int numb_motor);

    // Overloaded init method for 6 pins (2 motors)
    void init(int enA, int in1, int in2, int in3, int in4, int enB);

    // Overloaded init method for 3 pins (1 motor)
    void init(int enA, int in1, int in2);

    // Move the motor forward with specified PWM speed
    void forward(int speed_pwm);

    // Move the motor backward with specified PWM speed
    void backward(int speed_pwm);

    // Move a specific motor forward with specified PWM speed
    void forward(int speed_pwm, int motorID);

    // Move a specific motor backward with specified PWM speed
    void backward(int speed_pwm, int motorID);

    // Stop the motors
    void stop();

private:
    // Check if motorID is valid (0 or 1)
    bool isValidMotorID(int motorID);
};

#endif


