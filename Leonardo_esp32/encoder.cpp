#include "encoder.h"
#include "Variable.hpp"

/**
 * @file Encoder.cpp
 * @brief Implementation for Encoder class handling quadrature encoder input.
 */

/**
 * @brief Interrupt Service Routine handler for encoder ticks.
 *
 * Reads encoder channel states to determine tick direction, increments or decrements
 * the tick count, and updates direction accordingly.
 * Marked with IRAM_ATTR for placement in IRAM for lower latency.
 */
void IRAM_ATTR Encoder::handleInterrupt() {
  int stateA = digitalRead(pinA);
  int stateB = digitalRead(pinB);

  if (stateA == stateB) {
    tickCount++;
    direction = +1;
  } else {
    tickCount--;
    direction = -1;
  }
}

/**
 * @brief Initializes encoder pins and attaches ISR.
 *
 * Sets pinMode for encoder channels and attaches the provided ISR
 * to handle CHANGE events on channel A.
 *
 * @param isr Function pointer to the interrupt service routine.
 */
void Encoder::begin(void (*isr)()) {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA), isr, CHANGE);
}

/**
 * @brief Calculates output shaft RPM and resets tick count.
 *
 * Disables interrupts momentarily to safely copy and reset the tick count,
 * then computes RPM using the formula:
 *     RPM = (ticks * 60000) / (ppr * dt_ms)
 *
 * @param dt_ms Time interval in milliseconds over which ticks were counted.
 * @return RPM of the encoder output shaft in revolutions per minute.
 */
float Encoder::getOutputRPMandReset(unsigned long dt_ms) {
  noInterrupts();
  long ticks = tickCount;
  tickCount = 0;
  interrupts();

  return (ticks * 60000.0f) / (ppr * dt_ms);
}

/**
 * @brief Calculates motor shaft RPM considering gear ratio and resets ticks.
 *
 * Calls getOutputRPMandReset and scales by gearRatio to report motor RPM.
 *
 * @param dt_ms Time interval in milliseconds over which ticks were counted.
 * @return RPM of the motor shaft in revolutions per minute.
 */
float Encoder::getMotorRPMandReset(unsigned long dt_ms) {
  return getOutputRPMandReset(dt_ms) * gearRatio;
}

/**
 * @brief Returns the last detected direction of rotation.
 *
 * @return +1 if last tick was forward, -1 if reverse, 0 if stationary.
 */
int Encoder::getDirection() const {
  return direction;
}
