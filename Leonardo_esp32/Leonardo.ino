#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Variable.hpp"
#include "ISR.hpp"
#include "micro_ROS_fun.hpp"
#include "IMU_tools.h"

/**
 * @file main.cpp
 * @brief Main entry point: initializes hardware, IMU, encoders, PID, and micro-ROS, then spins executor.
 */

/**
 * @brief Arduino setup function.
 *
 * - Initializes Serial port for debugging
 * - Configures and stops motor driver
 * - Initializes IMU over I2C, checks connection, and calibrates gyro
 * - Attaches encoder ISRs
 * - Configures PID controllers (reset, deadzone, feedforward)
 * - Initializes and starts micro-ROS support
 */
void setup() {
    // Serial debug interface
    Serial.begin(115200);

    // Motor driver initialization and stop
    motorDriver.init(pwmPinA, inPinA, inPinB, inPinC, inPinD, pwmPinB);
    motorDriver.stop();

    // IMU initialization over I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    imu.initialize();
    if (!imu.testConnection()) {
        Serial.println("ERROR: IMU not detected!");
        while (true) {
            ; // Halt on IMU failure
        }
    }
    calibrateGyroZ();
    Serial.println("IMU ready.");

    // Attach encoder interrupt service routines
    encoder_L.begin(encoderISR_L);
    encoder_R.begin(encoderISR_R);

    // Configure PID controllers
    pid_R.reset();
    pid_R.setDeadZone(0);
    pid_L.reset();
    pid_L.setDeadZone(0);
    pid_R.setFeedforwardParams(20.88f, 500.85f);
    pid_L.setFeedforwardParams(20.88f, 500.85f);
    pid_L.enableFeedforward(1);
    pid_R.enableFeedforward(1);

    // Initialize odometry message headers
    nav_msgs__msg__Odometry__init(&odom_msg);
    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");

    // Start micro-ROS and report status
    init_uRos_code = initMicroROS();
    Serial.print("micro-ROS init returned: ");
    Serial.println(init_uRos_code);
}

/**
 * @brief Arduino loop function.
 *
 * Spins the micro-ROS executor to handle callbacks and timers.
 */
void loop() {
    // Process pending ROS tasks (max 10 ms)
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
