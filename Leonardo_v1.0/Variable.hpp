#ifndef VARIABLE_HPP
#define VARIABLE_HPP

#include "encoder.h"
#include "MotorDriver.h"
#include "PID.h"
#include "UnicycleOdometry.hpp"
#include <MPU6050.h> // imu
//micro_ROS
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/vector3.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>

#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/int32.h>
#include "UnicycleOdometry.hpp"
#include <stdint.h>
#include "EncoderPoseEstimator.hpp"

/**
 * @file Variable.hpp
 * @brief Global variable declarations and hardware abstractions.
 *
 * Defines constants for sensors, actuators, and external frameworks,
 * and declares extern variables for shared objects and state.
 */

// Covariance settings for odometry
#define HIGH_COV        1e6f  /**< High covariance to indicate low confidence */
#define MAX_SAMPLES     50    /**< Maximum samples for calibration or filtering */

// Motor and encoder parameters
#define ENCODER_PPR     (497 * 2) /**< Pulses per revolution of Hall encoder */
#define WHEEL_RADIUS    0.034f    /**< Wheel radius in meters */
#define WHEEL_SEPARATION 0.20f    /**< Distance between wheels in meters */
#define PID_DT          100.0f    /**< PID control loop period in ms */
#define DEFAULT_DT      0.01f     /**< Default time step in seconds */

#define INT_MIN_PID     -2000.0f  /**< PID integrator minimum limit */
#define INT_MAX_PID      2000.0f  /**< PID integrator maximum limit */

#define OUT_MIN         0         /**< Minimum PWM output */
#define OUT_MAX         4095      /**< Maximum PWM output */
#define DEADZONE        2048      /**< PWM deadzone threshold */



typedef double float64_t;      /**< Double precision alias */

// Hardware pin assignments
extern const int encoderPinA_L;
extern const int encoderPinB_L;
extern const int encoderPinA_R;
extern const int encoderPinB_R;
extern const int pwmPinA;
extern const int inPinA;
extern const int inPinB;
extern const int inPinC;
extern const int inPinD;
extern const int pwmPinB;
extern const int SDA_PIN;      /**< I2C SDA pin for IMU */
extern const int SCL_PIN;      /**< I2C SCL pin for IMU */

// LEDC PWM channels and settings
extern const int pwmChannel0;
extern const int pwmChannel1;
extern const int pwmFreq;
extern const int pwmResolution;

// Motor driver instance
extern L298N motorDriver;

// Encoder instances and synchronization primitives
extern portMUX_TYPE encoderMux_L;
extern portMUX_TYPE encoderMux_R;
extern Encoder encoder_L;
extern Encoder encoder_R;

// micro-ROS communication objects
extern rcl_publisher_t encoder_pub;
extern rcl_subscription_t control_sub;
extern rcl_node_t enc_node;
extern rcl_node_t ctrl_node;
extern rcl_timer_t timer_odmtry;
extern rcl_timer_t timer_IMU;
extern rcl_timer_t timer_ctrl;
extern rcl_timer_t timer_enc;
extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern int init_uRos_code;

// Odometry and pose estimation
extern UnicycleOdometry odometry;
extern EncoderPoseEstimator Enc_est;

// ROS message objects
extern geometry_msgs__msg__Vector3 control_msg;
extern nav_msgs__msg__Odometry odom_msg;

// Data buffers
extern float data_array[5];

// Wi-Fi credentials for micro-ROS transport
extern char wifiNetName[];
extern char wifiPassword[];
extern char personalIP[];

// Control parameters
extern volatile float w_L_ref;
extern volatile float w_R_ref;
extern volatile float rpm_ref_L;
extern volatile float rpm_ref_R;
extern volatile float rpm_L;
extern volatile float rpm_R;
extern float ki;
extern float kp;
extern float kd;
extern PID pid_L;
extern PID pid_R;

// IMU state
extern volatile float Theta_IMU; /**< Integrated yaw from IMU */
extern float gyro_z_bias;        /**< Gyroscope Z-axis bias */
extern float bias_correction_gain; /**< Bias correction filter gain */
extern portMUX_TYPE timerMux;    /**< Mutex for ISR timing */
extern MPU6050 imu;              /**< IMU sensor instance */

// Odometry filter state
extern volatile float theta_filtered;

// Filter coefficients
extern float alpha_filter_c;
extern float alpha;              /**< Low-pass filter coefficient for IMU */
extern float dt;                 /**< Control loop time step [s] */

#endif // VARIABLE_HPP

