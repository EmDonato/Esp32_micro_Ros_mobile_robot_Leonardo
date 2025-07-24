#ifndef ISR_HPP
#define ISR_HPP

/**
 * @file ISR.hpp
 * @brief Declarations for encoder interrupt service routines.
 *
 * These functions handle encoder pulse interrupts and are placed in IRAM
 * for minimal latency and fast execution.
 */

/**
 * @brief Interrupt Service Routine for the left wheel encoder.
 *
 * This function is invoked on each pulse from the left encoder. It is
 * marked with IRAM_ATTR to ensure placement in instruction RAM for
 * faster response.
 */
void IRAM_ATTR encoderISR_L();

/**
 * @brief Interrupt Service Routine for the right wheel encoder.
 *
 * This function is invoked on each pulse from the right encoder. It is
 * marked with IRAM_ATTR to ensure placement in instruction RAM for
 * faster response.
 */
void IRAM_ATTR encoderISR_R();

#endif // ISR_HPP
