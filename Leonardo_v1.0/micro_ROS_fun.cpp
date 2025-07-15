#include "Variable.hpp"
#include "micro_ROS_fun.hpp"
#include "control_tools.h"



int initMicroROS() {
  // Imposta il trasporto Wi-Fi per micro-ROS
  //memset(&odom_msg, 0, sizeof(odom_msg));
  set_microros_wifi_transports(wifiNetName, wifiPassword, personalIP, 8888);
  //set_microros_wifi_transports("AutomazioneTesisti", "nicosia456", "192.168.0.120", 8888); // tesisti
  delay(1500);
  Serial.println("Wi-Fi transport setup complete");

  // Ottieni l'allocatore predefinito
  allocator = rcl_get_default_allocator();
  delay(1500);

  // Inizializza supporto
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return -1;
  Serial.println("Support initialized");
  delay(1500);

  // Inizializza nodi
  if (rclc_node_init_default(&enc_node, "encoderNode", "", &support) != RCL_RET_OK) return -2;
  if (rclc_node_init_default(&ctrl_node, "controlNode", "", &support) != RCL_RET_OK) return -3;
  delay(1000);

  // Inizializza publisher
  if (rclc_publisher_init_default(
        &encoder_pub,
        &enc_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom") != RCL_RET_OK) return -4;
  delay(1000);


  // Inizializza subscriber
  
  if (rclc_subscription_init_best_effort(
        &control_sub,
        &ctrl_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "velocity_ctrl") != RCL_RET_OK) return -5;

  // Inizializza timer


  if (rclc_timer_init_default(
        &timer_ctrl,
        &support,
        RCL_MS_TO_NS(50),  // es. 50ms
        timer_callback_ctrl_PID) != RCL_RET_OK) return -62;
  delay(1500);
  if (rclc_timer_init_default(
        &timer_odmtry,
        &support,
        RCL_MS_TO_NS(ODOMETRY_TIME_MS),  // es. 100ms = 10 Hz
        timer_callback_odometry) != RCL_RET_OK) return -61;
  delay(1500);


  if (rclc_timer_init_default(
        &timer_IMU,
        &support,
        RCL_MS_TO_NS(10),  // es. 50ms
        timer_callback_IMU) != RCL_RET_OK) return -63;
  //delay(1500);

  delay(2000);





  nav_msgs__msg__Odometry__init(&odom_msg);
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");
  for (int i = 0; i < 36; ++i) {
    odom_msg.pose.covariance[i]  = 0.0f;
    odom_msg.twist.covariance[i] = 0.0f;
  }

  // Pose: varianze in x, y e yaw
  // usiamo ~1 m² per la posizione e 0.3 rad² (~17°²) per l'orientamento
  odom_msg.pose.covariance[0*6 + 0] = 1.0f;   // var(x)
  odom_msg.pose.covariance[1*6 + 1] = 1.0f;   // var(y)
  odom_msg.pose.covariance[5*6 + 5] = 0.3f;   // var(yaw)

  // Twist: varianze su vx e ωz
  // usiamo ~0.1 (m/s)² e 0.05 (rad/s)²
  odom_msg.twist.covariance[0*6 + 0] = 0.1f;  // var(vx)
  odom_msg.twist.covariance[5*6 + 5] = 0.05f; // var(ωz)
  // prepare LaserScan message


  // Inizializza executor
  if (rclc_executor_init(&executor, &support.context, 6, &allocator) != RCL_RET_OK) return -7;
    delay(500);

  if (rclc_executor_add_timer(&executor, &timer_odmtry) != RCL_RET_OK) return -8;
  if (rclc_executor_add_subscription(&executor, &control_sub, &control_msg, &subscription_callback_control, ON_NEW_DATA) != RCL_RET_OK) return -9;
  if (rclc_executor_add_timer(&executor, &timer_ctrl) != RCL_RET_OK) return -10;
  if (rclc_executor_add_timer(&executor, &timer_IMU) != RCL_RET_OK) return -11;


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
  
  const geometry_msgs__msg__Vector3 *msg = (const geometry_msgs__msg__Vector3 *)msgin;
  //noInterrupts();
  //Serial.println("[DEBUG] Subscription callback triggered");
  //Serial.println(msg->x);
  //Serial.println(msg->y);
  w_L_ref = msg->x;
  w_R_ref = msg->y;
  std::pair<float, float> rpms = computeRPM(msg->x, msg->y);
  //Serial.println(rpm_ref_L);
  //Serial.println(rpm_ref_R);
  rpm_ref_L = rpms.first;
  rpm_ref_R = rpms.second;
  //interrupts();
  
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




