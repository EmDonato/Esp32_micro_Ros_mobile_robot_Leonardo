#ifndef MICRO_ROS_FUN_HPP
#define MICRO_ROS_FUN_HPP

#include <rcl/error_handling.h>
/** Odometry update interval in milliseconds */
#define ODOMETRY_TIME_MS 50u
/** Odometry update interval in seconds */
#define ODOMETRY_TIME_S 0.05f

/**
 * @brief Check return code of an rcl call; on failure, print error and halt.
 * @param fn Expression returning rcl_ret_t
 */
#define RCCHECK(fn)                                   \
  do {                                                \
    rcl_ret_t temp_rc = (fn);                        \
    if (temp_rc != RCL_RET_OK) {                      \
      Serial.print("Error: ");                       \
      Serial.println(temp_rc);                       \
      error_loop();                                  \
    }                                                 \
  } while (0)

/**
 * @brief Soft error check for an rcl call; on failure, print warning and wait.
 * @param fn Expression returning rcl_ret_t
 */
#define RCSOFTCHECK(fn)                              \
  do {                                                \
    rcl_ret_t temp_rc = (fn);                        \
    if (temp_rc != RCL_RET_OK) {                      \
      Serial.print("Warning: ");                     \
      Serial.println(temp_rc);                       \
      delay(1000);                                   \
    }                                                 \
  } while (0)

/**
 * @brief Initialize micro-ROS (transport, nodes, pubs/subs, timers, executor).
 * @return 0 on success; negative error code on failure.
 */
int initMicroROS();

/**
 * @brief Clean up micro-ROS resources.
 * @return 0 on success; negative error code on failure.
 */
int micro_ros_cleanup();

/** 
 * @brief Enter an infinite loop on fatal error.
 */
void error_loop();

/**
 * @brief Callback for velocity control subscription.
 * @param msgin Pointer to incoming message (geometry_msgs::msg::Vector3).
 */
void subscription_callback_control(const void *msgin);

/**
 * @brief Timer callback: compute and publish odometry.
 * @param timer Pointer to the rcl_timer_t instance.
 * @param last_call_time Timestamp of last invocation.
 */
void timer_callback_odometry(rcl_timer_t *timer, int64_t last_call_time);

// void timer_callback_encoder(rcl_timer_t *timer, int64_t last_call_time);

/**
 * @brief Timer callback: run PID control loop.
 * @param timer Pointer to the rcl_timer_t instance.
 * @param last_call_time Timestamp of last invocation.
 */
void timer_callback_ctrl_PID(rcl_timer_t *timer, int64_t last_call_time);

/**
 * @brief Timer callback: integrate IMU data.
 * @param timer Pointer to the rcl_timer_t instance.
 * @param last_call_time Timestamp of last invocation.
 */
void timer_callback_IMU(rcl_timer_t *timer, int64_t last_call_time);

/** 
 * @brief Initialize odometry message fields (frames, covariance, etc.).
 */
void init_odom_msg();

/**
 * @brief Convert yaw angle to a ROS quaternion.
 * @param yaw Yaw in radians.
 * @return geometry_msgs::msg::Quaternion representing that yaw.
 */
static geometry_msgs__msg__Quaternion yaw_to_quat(float yaw);

/**
 * @brief Timer callback: mark LiDAR scan readiness.
 * @param timer Pointer to the rcl_timer_t instance.
 * @param last_call_time Timestamp of last invocation.
 */
void scan_timer_callback(rcl_timer_t *timer, int64_t last_call_time);

/** 
 * @brief Clear the LiDAR scan buffer in preparation for a new scan.
 */
void resetScanBuffer();

/**
 * @brief Check if LiDAR scan buffer is full.
 * @return true if scan complete, false otherwise.
 */
bool isScanComplete();

/**
 * @brief Timer callback: read LiDAR data into buffer.
 * @param timer Pointer to the rcl_timer_t instance.
 * @param last_call_time Timestamp of last invocation.
 */
void lidar_read_timer_callback(rcl_timer_t *timer, int64_t last_call_time);

/**
 * @brief Get current system time as a ROS Time message.
 * @return builtin_interfaces::msg::Time with current timestamp.
 */
builtin_interfaces__msg__Time get_now();

#endif // MICRO_ROS_FUN_HPP
