#include "Variable.hpp"
#include "ISR.hpp"

/**
 * @brief ISR for handling left encoder interrupts.
 *
 * This interrupt service routine is triggered on a change in the left encoder signal.
 * It delegates processing to the encoder_L object which updates counts and state.
 *
 * Attributes:
 * - IRAM_ATTR: Places the routine into IRAM for faster execution.
 */
void IRAM_ATTR encoderISR_L() {
    encoder_L.handleInterrupt();  // Process left encoder tick
}

/**
 * @brief ISR for handling right encoder interrupts.
 *
 * This interrupt service routine is triggered on a change in the right encoder signal.
 * It delegates processing to the encoder_R object which updates counts and state.
 *
 * Attributes:
 * - IRAM_ATTR: Places the routine into IRAM for faster execution.
 */
void IRAM_ATTR encoderISR_R() {
    encoder_R.handleInterrupt();  // Process right encoder tick
}
