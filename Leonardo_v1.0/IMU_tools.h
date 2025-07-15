#ifndef IMU_TOOLS_H
#define IMU_TOOLS_H

#include <Arduino.h> // Ensure Serial is available
#include <stdint.h>

// Global variable to store the gyro Z-axis bias
extern float gyro_z_bias;

/**
 * @brief Calibrates the Z-axis gyroscope bias.
 *
 * Samples the gyroscope multiple times while the robot is stationary to
 * determine the average offset (bias) on the Z-axis. The bias is then stored
 * in the global variable gyro_z_bias.
 *
 * @note The robot must remain completely still during calibration.
 * @note This function blocks for approximately samples * 5 ms.
 */
inline void calibrateGyroZ() {
    const int samples = 500;       /**< Number of samples for calibration */
    int16_t gz = 0;                /**< Raw gyro reading for Z-axis */
    long sum = 0;                  /**< Accumulator for raw readings */

    Serial.println("Gyroscope calibration... keep the robot stationary");

    // Collect samples to compute average bias
    for (int i = 0; i < samples; ++i) {
        gz = imu.getRotationZ();         // Read raw Z-axis gyro value
        sum += gz;
        delay(5);                        // 5 ms delay between samples
    }

    // Compute and store the bias as the average of all samples
    gyro_z_bias = static_cast<float>(sum) / samples;

    Serial.print("Calculated Z-axis bias: ");
    Serial.println(gyro_z_bias);
}

#endif // IMU_TOOLS_H
