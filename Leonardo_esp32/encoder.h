#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <stdint.h>

/**
 * @file Encoder.hpp
 * @brief Interface for a quadrature wheel encoder.
 *
 * Manages ticks, direction, and RPM calculation with optional gear ratio compensation.
 */
class Encoder {
public:
    /**
     * @brief Constructor.
     * @param _pinA    GPIO pin connected to encoder channel A.
     * @param _pinB    GPIO pin connected to encoder channel B.
     * @param _ppr     Pulses per revolution of the encoder shaft.
     * @param _gearRatio Gear reduction ratio between motor and encoder (default: 1.0).
     *
     * Initializes tick count and direction.
     */
    Encoder(int _pinA, int _pinB, float _ppr, float _gearRatio = 1.0f)
        : pinA(_pinA), pinB(_pinB), ppr(_ppr), gearRatio(_gearRatio),
          tickCount(0), direction(0) {}

    /**
     * @brief Initializes encoder pins and attaches ISR.
     * @param isr Pointer to the interrupt service routine for encoder pulses.
     *
     * Configures both channels as inputs and attaches the provided ISR
     * to handle pinA transitions.
     */
    void begin(void (*isr)());

    /**
     * @brief ISR handler for encoder ticks.
     *
     * Reads pins A and B to determine rotation direction, updates tick count and direction.
     * Marked IRAM_ATTR for placement in IRAM for low-latency execution.
     */
    void IRAM_ATTR handleInterrupt();

    /**
     * @brief Computes output shaft RPM and resets the tick counter.
     * @param dt_ms Time interval since last call in milliseconds.
     * @return Calculated RPM of the encoder output shaft (after gear ratio).
     */
    float getOutputRPMandReset(unsigned long dt_ms);

    /**
     * @brief Computes motor shaft RPM (accounting for gear ratio) and resets ticks.
     * @param dt_ms Time interval since last call in milliseconds.
     * @return Calculated RPM of the motor shaft.
     */
    float getMotorRPMandReset(unsigned long dt_ms);

    /**
     * @brief Returns the last detected rotation direction.
     * @return +1 for forward, -1 for reverse, 0 if stationary.
     */
    int getDirection() const;

private:
    volatile long tickCount;    /**< Count of encoder ticks (quadrature). */
    volatile int direction;     /**< Last tick direction (+1 or -1). */
    const int pinA;             /**< Encoder channel A pin. */
    const int pinB;             /**< Encoder channel B pin. */
    const float ppr;            /**< Pulses per revolution. */
    const float gearRatio;      /**< Gear ratio (motor:encoder). */
};

#endif // ENCODER_HPP
