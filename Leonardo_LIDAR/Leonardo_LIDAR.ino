#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <builtin_interfaces/msg/time.h>
#include <std_msgs/msg/string.h>
#include "LD06forArduino.h"

#define RESOLUTION 360
#define RX_PIN 16
#define TX_PIN 17
#define BAUDRATE 230400


// Wi-Fi config
char wifiNetName[]  = "FASTWEB-USA6DG";
char wifiPassword[] = "26KCXAYRSU";
char personalIP[]   = "192.168.1.56";

// micro-ROS entities
rclc_support_t lidar_support;
rcl_node_t lidar_node;
rcl_allocator_t lidar_allocator;
rcl_publisher_t lidar_scan_pub;
rcl_publisher_t lidar_debug_pub;
rcl_subscription_t lidar_time_sub;
rclc_executor_t lidar_executor;

// LaserScan and LD06
LD06forArduino lidar;
sensor_msgs__msg__LaserScan lidar_scan_msg;
std_msgs__msg__String lidar_debug_msg;
builtin_interfaces__msg__Time lidar_time_msg;

float lidar_ranges[RESOLUTION];
float lidar_intensities[RESOLUTION];
bool lidar_angle_collected[RESOLUTION];
int lidar_count_received = 0;

// Time sync
int64_t lidar_time_offset_us = 0;
bool lidar_time_synced = false;
bool lidar_scan_published_once = false;

/**
 * @brief Reset buffers for 360° scan
 */
void resetLidarScanBuffer() {
  for (int i = 0; i < RESOLUTION; ++i) {
    lidar_ranges[i] = NAN;
    lidar_intensities[i] = 0.0f;
    lidar_angle_collected[i] = false;
  }
  lidar_count_received = 0;
}

/**
 * @brief Check if full scan has been received
 */
bool isLidarScanComplete() {
  return lidar_count_received >= RESOLUTION;
}

/**
 * @brief Return time in ROS format adjusted with offset
 */
builtin_interfaces__msg__Time lidar_get_sync_time() {
  builtin_interfaces__msg__Time t;
  int64_t now_us = rmw_uros_epoch_millis() * 1000;
  int64_t synced_us = now_us + lidar_time_offset_us;
  t.sec = synced_us / 1000000;
  t.nanosec = (synced_us % 1000000) * 1000;
  return t;
}

/**
 * @brief Publish a debug message on /lidar/debug_log
 */
void lidar_publish_debug(const char * msg) {
  lidar_debug_msg.data.data = (char *)msg;
  lidar_debug_msg.data.size = strlen(msg);
  lidar_debug_msg.data.capacity = lidar_debug_msg.data.size + 1;
  rcl_publish(&lidar_debug_pub, &lidar_debug_msg, NULL);
}

/**
 * @brief Callback for receiving synchronized time from PC
 */
void lidar_time_callback(const void * msgin) {
  const builtin_interfaces__msg__Time * t_msg = (const builtin_interfaces__msg__Time *)msgin;
  int64_t received_sec = t_msg->sec;
  int64_t received_nsec = t_msg->nanosec;
  int64_t received_us = received_sec * 1000000LL + received_nsec / 1000LL;
  int64_t now_us = rmw_uros_epoch_millis() * 1000;
  lidar_time_offset_us = received_us - now_us;
  lidar_time_synced = true;

  lidar_publish_debug("[INFO] Lidar time sync received and offset updated");
}

/**
 * @brief micro-ROS + LD06 setup
 */
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("LD06 + micro-ROS + time sync + debug");

  // micro-ROS transport over Wi-Fi
  set_microros_wifi_transports(wifiNetName, wifiPassword, personalIP, 8888);
  delay(2000);

  lidar_allocator = rcl_get_default_allocator();
  rclc_support_init(&lidar_support, 0, NULL, &lidar_allocator);
  rclc_node_init_default(&lidar_node, "lidar_node", "", &lidar_support);

  // Publisher: /lidar/scan
  rclc_publisher_init_default(
    &lidar_scan_pub,
    &lidar_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "scan"
  );

  // Publisher: /lidar/debug_log
  rclc_publisher_init_default(
    &lidar_debug_pub,
    &lidar_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "lidar/debug_log"
  );

  // Subscriber: /lidar/time_sync
  rclc_subscription_init_default(
    &lidar_time_sub,
    &lidar_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(builtin_interfaces, msg, Time),
    "time_sync"
  );

  // Executor for time callback
  rclc_executor_init(&lidar_executor, &lidar_support.context, 1, &lidar_allocator);
rclc_executor_add_subscription(&lidar_executor, &lidar_time_sub, &lidar_time_msg, &lidar_time_callback, ON_NEW_DATA);

  // Init LaserScan message
  lidar_scan_msg.header.frame_id.data = (char*)"base_laser";
  lidar_scan_msg.header.frame_id.size = strlen("base_laser");
  lidar_scan_msg.header.frame_id.capacity = lidar_scan_msg.header.frame_id.size + 1;
  lidar_scan_msg.angle_min       = 0.0f;
  lidar_scan_msg.angle_max       = 2.0f * M_PI;
  lidar_scan_msg.angle_increment = (2.0f * M_PI) / RESOLUTION;
  lidar_scan_msg.scan_time       = 1.0f / 10.0f;
  lidar_scan_msg.time_increment  = lidar_scan_msg.scan_time / RESOLUTION;
  lidar_scan_msg.range_min       = 0.02f;
  lidar_scan_msg.range_max       = 12.0f;
  lidar_scan_msg.ranges.data     = lidar_ranges;
  lidar_scan_msg.ranges.size     = RESOLUTION;
  lidar_scan_msg.ranges.capacity = RESOLUTION;
  lidar_scan_msg.intensities.data = lidar_intensities;
  lidar_scan_msg.intensities.size = RESOLUTION;
  lidar_scan_msg.intensities.capacity = RESOLUTION;

  // Init LiDAR
  lidar.Init(RX_PIN);
  resetLidarScanBuffer();
}

/**
 * @brief Main loop: wait for sync, acquire and publish scan
 */
void loop() {
  rclc_executor_spin_some(&lidar_executor, RCL_MS_TO_NS(1));
  // Wait for time sync
  if (!lidar_time_synced) {
    Serial.println("[WARN] Waiting for lidar time sync...");
    return;
  }

  lidar.read_lidar_data();

  if (!lidar.angles.empty()) {
    for (size_t i = 0; i < lidar.angles.size(); ++i) {
      int deg = ((int)round(lidar.angles[i])) % 360;
      if (!lidar_angle_collected[deg]) {
        float dist_m = lidar.distances[i] / 1000.0f;

        if (dist_m < lidar_scan_msg.range_min || dist_m > lidar_scan_msg.range_max)
          lidar_ranges[deg] = std::numeric_limits<float>::infinity();
        else
          lidar_ranges[deg] = dist_m;

        lidar_intensities[deg] = lidar.confidences[i];
        lidar_angle_collected[deg] = true;
        lidar_count_received++;
      }
    }

    if (isLidarScanComplete()) {
      lidar_scan_msg.header.stamp = lidar_get_sync_time();
      rcl_publish(&lidar_scan_pub, &lidar_scan_msg, NULL);

      if (!lidar_scan_published_once) {
        lidar_publish_debug("[INFO] First LaserScan published after sync");
        lidar_scan_published_once = true;
      }

      resetLidarScanBuffer();
    }
  }
}
