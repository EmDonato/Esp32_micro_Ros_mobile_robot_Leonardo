#include "Variable.hpp"
#include "micro_ROS_fun.hpp"
#include "control_tools.h"

const float GYRO_VAR   = 1e-4f;  // rad^2/s^2 
const float ACCEL_VAR  = 4e-2f;  // m^2/s^4
const float G_TO_MS2  = 9.80665f;
const float DEG2RAD   = 0.01745329252f;

/**
 * @brief Initializes micro-ROS entities including nodes, publishers, subscribers, and timers.
 *
 * This function sets up the micro-ROS transport over Wi-Fi, initializes required ROS 2 messages,
 * configures the allocator and support structure, and sequentially creates nodes, publishers,
 * subscribers, timers, and the executor. The debug node and publisher are initialized first,
 * enabling debug messages to be published for the remainder of the initialization process.
 * 
 * @return int Returns 0 on success, or a negative error code corresponding to the failing step.
 */
int initMicroROS() {

  // Configure micro-ROS Wi-Fi transport
  set_microros_wifi_transports(wifiNetName, wifiPassword, personalIP, 8888);
  delay(1500);
  Serial.println("Wi-Fi transport setup complete");

  // Initialize message structures
  geometry_msgs__msg__Twist__init(&cmd_vel_msg);
  imu_msg_init(&imu_msg, "imu_link");
  nav_msgs__msg__Odometry__init(&odom_msg);
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");
  std_msgs__msg__String__init(&debug_msg);

  // Configure covariance matrices for odometry message
  const float BIG = 1e6f;

  // Pose covariance
  odom_msg.pose.covariance[0]  = 5e-5f;   // x
  odom_msg.pose.covariance[7]  = 5e-5f;   // y
  odom_msg.pose.covariance[14] = BIG;     // z
  odom_msg.pose.covariance[21] = BIG;     // roll
  odom_msg.pose.covariance[28] = BIG;     // pitch
  odom_msg.pose.covariance[35] = 1.95e-2f; // yaw

  // Twist covariance
  odom_msg.twist.covariance[0]  = 2.5e-5f; // vx
  odom_msg.twist.covariance[7]  = 2.5e-5f; // vy
  odom_msg.twist.covariance[14] = BIG;     // vz
  odom_msg.twist.covariance[21] = BIG;     // roll rate
  odom_msg.twist.covariance[28] = BIG;     // pitch rate
  odom_msg.twist.covariance[35] = 9e-4f;   // yaw rate

  // Get the default memory allocator
  allocator = rcl_get_default_allocator();
  delay(1500);

  // Initialize micro-ROS support
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return -1;
  Serial.println("Support initialized");
  delay(1500);

  // Initialize debug node and publisher FIRST (only after support is ready)
  if (rclc_node_init_default(&debug_node, "debugNode", "", &support) != RCL_RET_OK) return -2;
  if (rclc_publisher_init_default(
        &debug_pub,
        &debug_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "debug_log") != RCL_RET_OK) return -4;
  delay(1000);

  // Now we can publish debug messages
  publish_debug("[INFO] Debug node and publisher initialized");

  // Initialize remaining nodes
  if (rclc_node_init_default(&enc_node, "encoderNode", "", &support) != RCL_RET_OK) {
    publish_debug("[ERROR] Failed to initialize encoderNode");
    return -21;
  }
  publish_debug("[INFO] Encoder node initialized");

  if (rclc_node_init_default(&ctrl_node, "controlNode", "", &support) != RCL_RET_OK) {
    publish_debug("[ERROR] Failed to initialize controlNode");
    return -3;
  }
  publish_debug("[INFO] Control node initialized");

  if (rclc_node_init_default(&imu_node, "IMUNode", "", &support) != RCL_RET_OK) {
    publish_debug("[ERROR] Failed to initialize IMU node");
    return -31;
  }
  publish_debug("[INFO] IMU node initialized");
  delay(1000);

  // Initialize publishers
  if (rclc_publisher_init_default(
        &encoder_pub,
        &enc_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom") != RCL_RET_OK) {
    publish_debug("[ERROR] Failed to initialize odom publisher");
    return -4;
  }
  publish_debug("[INFO] Odometry publisher initialized");
  delay(1000);

  if (rclc_publisher_init_best_effort(
        &imu_pub,
        &imu_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data") != RCL_RET_OK) {
    publish_debug("[ERROR] Failed to initialize IMU publisher");
    return -41;
  }
  publish_debug("[INFO] IMU publisher initialized");
  delay(1000);

  // Initialize subscriber
  if (rclc_subscription_init_best_effort(
        &control_sub,
        &ctrl_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel") != RCL_RET_OK) {
    publish_debug("[ERROR] Failed to initialize cmd_vel subscriber");
    return -5;
  }
  publish_debug("[INFO] cmd_vel subscriber initialized");

  // Initialize timers
  if (rclc_timer_init_default(&timer_ctrl, &support, RCL_MS_TO_NS(50), timer_callback_ctrl_PID) != RCL_RET_OK) {
    publish_debug("[ERROR] Failed to initialize control timer");
    return -62;
  }
  publish_debug("[INFO] Control timer initialized");
  delay(1500);

  if (rclc_timer_init_default(&timer_odmtry, &support, RCL_MS_TO_NS(ODOMETRY_TIME_MS), timer_callback_odometry) != RCL_RET_OK) {
    publish_debug("[ERROR] Failed to initialize odometry timer");
    return -61;
  }
  publish_debug("[INFO] Odometry timer initialized");
  delay(1500);

  if (rclc_timer_init_default(&timer_IMU, &support, RCL_MS_TO_NS(10), timer_callback_IMU) != RCL_RET_OK) {
    publish_debug("[ERROR] Failed to initialize IMU timer");
    return -63;
  }
  publish_debug("[INFO] IMU timer initialized");
  delay(2000);

  // Initialize executor with all handles (4 timers + 1 subscriber)
  if (rclc_executor_init(&executor, &support.context, 6, &allocator) != RCL_RET_OK) {
    publish_debug("[ERROR] Failed to initialize executor");
    return -7;
  }
  publish_debug("[INFO] Executor initialized");
  delay(500);

  // Add all entities to executor
  if (rclc_executor_add_timer(&executor, &timer_odmtry) != RCL_RET_OK) return -8;
  if (rclc_executor_add_subscription(&executor, &control_sub, &control_msg, &subscription_callback_control, ON_NEW_DATA) != RCL_RET_OK) return -9;
  if (rclc_executor_add_timer(&executor, &timer_ctrl) != RCL_RET_OK) return -10;
  if (rclc_executor_add_timer(&executor, &timer_IMU) != RCL_RET_OK) return -11;

  publish_debug("[INFO] All timers and subscribers added to executor");
  Serial.println("[INFO] micro-ROS init complete");
  delay(1000);
  return 0;
}


void error_loop() {
  while (1) {
    Serial.println("[ERROR] Entering error loop...");
    Serial.print("Heap disponibile: ");
    delay(1000);
  }
}


void subscription_callback_control(const void *msgin) {
  
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  auto angular_velocity = compute_wheel_speeds(
        msg->linear.x, msg->angular.z
    );
  w_L_ref = angular_velocity.first;
  w_R_ref = angular_velocity.second;
  std::pair<float, float> rpms = computeRPM(w_L_ref, w_R_ref);
  Serial.println(w_L_ref);
  Serial.println(w_R_ref);
  rpm_ref_L = rpms.first;
  rpm_ref_R = rpms.second;  
}

void timer_callback_IMU(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  static float prev_gyro_z = 0.0;
  int16_t gz_raw = imu.getRotationZ(); 
  float gz = (gz_raw - gyro_z_bias) / 131.0 * DEG_TO_RAD;  // rad/s (con scala ±250°/s)
  //float gz_filtered = alpha * prev_gyro_z + (1-alpha) * gz;

  portENTER_CRITICAL_ISR(&timerMux);
  Theta_IMU += gz * 0.01;  // integrazione con Δt = 10 ms
 //odometry.setTheta(theta);
  Theta_IMU = normalize_angle(Theta_IMU);
  portEXIT_CRITICAL_ISR(&timerMux);

  prev_gyro_z = gz;
  int16_t ax, ay, az, gx, gy;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz_raw);

  // Scala (assumi ±2g e ±250°/s)
  float ax_ms2 = (ax / 16384.0f) * G_TO_MS2;
  float ay_ms2 = (ay / 16384.0f) * G_TO_MS2;
  float az_ms2 = (az / 16384.0f) * G_TO_MS2;

  float gx_rads = (gx / 131.0f) * DEG2RAD;
  float gy_rads = (gy / 131.0f) * DEG2RAD;
  float gz_rads = (gz_raw / 131.0f) * DEG2RAD;

  imu_msg_fill(&imu_msg, ax_ms2, ay_ms2, az_ms2, gx_rads, gy_rads, gz_rads);
  RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
}

void timer_callback_odometry(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(timer);
  if (timer == NULL) return;
  rpm_L = encoder_L.getOutputRPMandReset(50);
  rpm_R = encoder_R.getOutputRPMandReset(50);
  Enc_est.update(rpm_L, rpm_R, ODOMETRY_TIME_S );

  uint32_t ms = millis();
  odom_msg.header.stamp.sec = ms / 1000;
  odom_msg.header.stamp.nanosec = (ms % 1000) * 1000000;

  float theta = complementaryFilter(Enc_est.getThetaEncoder(), normalize_angle(Theta_IMU));
 // float theta = Enc_est.getThetaEncoder();

  odometry.updateFromFusion(Enc_est.getLinearVelocity(),
                            Enc_est.getAngularVelocity(),
                            theta,
                            ODOMETRY_TIME_S);
  float x = odometry.getX();
  float y = odometry.getY();

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
//  odom_msg.twist.twist.linear.x  = odometry.getLinearVelocity();

  odom_msg.twist.twist.linear.x  = odometry.getLinearVelocity();
  odom_msg.twist.twist.linear.y  = 0;
  odom_msg.twist.twist.linear.z  = theta *  180.0 / PI;

  odom_msg.twist.twist.angular.z = odometry.getAngularVelocity();
  float half_yaw = theta * 0.5f;
  odom_msg.pose.pose.orientation.x = 0.0f;
  odom_msg.pose.pose.orientation.y = 0.0f;
  odom_msg.pose.pose.orientation.z = sinf(half_yaw);
  odom_msg.pose.pose.orientation.w = cosf(half_yaw);

  // --- 5) Pubblica ---
  RCSOFTCHECK(rcl_publish(&encoder_pub, &odom_msg, NULL));
}

/**
 * @brief PID control timer callback: update motor outputs.
 *
 * Reads reference and measured wheel velocities, computes control commands via PID,
 * and applies forward/backward drive accordingly. Uses a named threshold constant
 * to avoid macro name conflicts.
 */
void timer_callback_ctrl_PID(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(timer);
    RCLC_UNUSED(last_call_time);

    // Threshold to ignore very small velocity commands
    constexpr float EPS_THRESHOLD = 0.01f;

    // Left wheel control
    if (std::abs(w_L_ref) > EPS_THRESHOLD) {
        float uL = std::abs(pid_L.compute(rpm_ref_L, rpm_L));
        if (w_L_ref > 0) {
            motorDriver.forward(uL, 0);
        } else {
            motorDriver.backward(uL, 0);
        }
    } else {
        motorDriver.forward(0, 0);
    }

    // Right wheel control
    if (std::abs(w_R_ref) > EPS_THRESHOLD) {
        float uR = std::abs(pid_R.compute(rpm_ref_R, rpm_R));
        if (w_R_ref > 0) {
            motorDriver.forward(uR, 1);
        } else {
            motorDriver.backward(uR, 1);
        }
    } else {
        motorDriver.forward(0, 1);
    }
}


void init_odom_msg(void) {
  // 1) Inizializza la struct
  nav_msgs__msg__Odometry__init(&odom_msg);

  // 2) Frame names
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id,   "odom");
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id,    "base_link");

  // 3) Covarianze alte per segnalare "non affidabile"
  for (size_t i = 0; i < 36; ++i) {
    odom_msg.pose.covariance[i]  = (i % 7 == 0) ? HIGH_COV : 0.0f;
    odom_msg.twist.covariance[i] = (i % 7 == 0) ? HIGH_COV : 0.0f;
  }

  // 4) Posa iniziale zero + quaternion identità
  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = 0.0;
  odom_msg.pose.pose.orientation.w = 1.0;

  // 5) Twist iniziale zero
  odom_msg.twist.twist.linear.x  = 0.0;
  odom_msg.twist.twist.linear.y  = 0.0;
  odom_msg.twist.twist.linear.z  = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;
}

static geometry_msgs__msg__Quaternion yaw_to_quat(float yaw)
{
  geometry_msgs__msg__Quaternion q;
  q.x = 0.0f;
  q.y = 0.0f;
  q.z = sinf(yaw * 0.5f);
  q.w = cosf(yaw * 0.5f);
  return q;
}


builtin_interfaces__msg__Time get_now() {
  int64_t ms = rmw_uros_epoch_millis();
  builtin_interfaces__msg__Time t;
  t.sec     = ms / 1000;
  t.nanosec = (ms % 1000) * 1000000;
  return t;
}


void imu_msg_init(sensor_msgs__msg__Imu *msg, const char *frame_id)
{
  sensor_msgs__msg__Imu__init(msg);

  // frame_id
  msg->header.frame_id.data     = (char*)frame_id;
  msg->header.frame_id.size     = strlen(frame_id);
  msg->header.frame_id.capacity = msg->header.frame_id.size + 1;

  for (int i = 0; i < 9; i++) msg->orientation_covariance[i] = -1.0;

  // Gyro covariance
  for (int i = 0; i < 9; i++) msg->angular_velocity_covariance[i] = 0.0;
  msg->angular_velocity_covariance[0] = GYRO_VAR;
  msg->angular_velocity_covariance[4] = GYRO_VAR;
  msg->angular_velocity_covariance[8] = GYRO_VAR;

  // Accel covariance
  for (int i = 0; i < 9; i++) msg->linear_acceleration_covariance[i] = 0.0;
  msg->linear_acceleration_covariance[0] = ACCEL_VAR;
  msg->linear_acceleration_covariance[4] = ACCEL_VAR;
  msg->linear_acceleration_covariance[8] = ACCEL_VAR;

  msg->orientation.w = 0.0f;
  msg->orientation.x = 0.0f;
  msg->orientation.y = 0.0f;
  msg->orientation.z = 0.0f;
}

void imu_msg_fill(sensor_msgs__msg__Imu *msg,
                  float ax_ms2, float ay_ms2, float az_ms2,
                  float gx_rads, float gy_rads, float gz_rads)
{
  uint32_t ms = millis();
  msg->header.stamp.sec     = ms / 1000;
  msg->header.stamp.nanosec = (ms % 1000) * 1000000;

  // Angular velocity
  msg->angular_velocity.x = gx_rads;
  msg->angular_velocity.y = gy_rads;
  msg->angular_velocity.z = gz_rads;

  // Linear acceleration
  msg->linear_acceleration.x = ax_ms2;
  msg->linear_acceleration.y = ay_ms2;
  msg->linear_acceleration.z = az_ms2;
}

void publish_debug(const char* msg)
{
    debug_msg.data.data = (char*)msg;
    debug_msg.data.size = strlen(msg);
    debug_msg.data.capacity = debug_msg.data.size + 1;
    rcl_publish(&debug_pub, &debug_msg, NULL);
}
