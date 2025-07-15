#ifndef CONTROL_TOOLS_H
#define CONTROL_TOOLS_H

#include "Variable.hpp"
#include <cmath>
#include <utility>

/**
 * @file ControlTools.hpp
 * @brief Utility functions for control-related calculations.
 */

/**
 * @brief Computes wheel RPM values from angular velocities.
 *
 * @param omega_left  Left wheel angular velocity in radians per second (rad/s).
 * @param omega_right Right wheel angular velocity in radians per second (rad/s).
 * @return A pair of floats: {rpm_left, rpm_right} in revolutions per minute (RPM).
 *
 * Uses the relation: RPM = (ω * 60) / (2π).
 */
inline std::pair<float, float> computeRPM(float omega_left, float omega_right) {
    const float factor = 60.0f / (2.0f * M_PI);
    float rpm_left  = omega_left  * factor;
    float rpm_right = omega_right * factor;
    return std::make_pair(rpm_left, rpm_right);
}

/**
 * @brief Normalizes an angle to the range [-π, +π].
 *
 * @param angle Input angle in radians.
 * @return Equivalent angle normalized to [-π, +π].
 *
 * Repeatedly adds or subtracts 2π until the angle lies within the desired range.
 */
inline float normalize_angle(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/**
 * @brief Computes the sign of a floating-point number.
 *
 * @param x Input value.
 * @return +1 if x > 0, -1 if x < 0, 0 if x == 0.
 */
inline int sign(float x) {
    return (x > 0.0f) - (x < 0.0f);
}

/**
 * @brief Performs a complementary filter between encoder and IMU angles.
 *
 * @param theta_enc_raw Raw heading from encoders in radians.
 * @param theta_imu     Heading integrated from IMU gyroscope in radians.
 * @return Filtered heading in radians.
 *
 * Blends the IMU-derived angle and encoder raw angle using a constant
 * weighting factor alpha (0.98). The formula is:
 *     theta_filtered = alpha * theta_imu + (1 - alpha) * theta_enc_raw;
 */
inline float complementaryFilter(float theta_enc_raw, float theta_imu) {
    const float alpha = 0.98f;
    return alpha * theta_imu + (1.0f - alpha) * theta_enc_raw;
}

#endif // CONTROL_TOOLS_H
