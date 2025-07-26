# LD06 LiDAR + ESP32 (Secondary Node) + micro-ROS Humble

This project enables the integration of the LD06 LiDAR sensor with an **ESP32** microcontroller as a **dedicated LiDAR node**, using the **micro-ROS Humble** stack to publish `sensor_msgs/msg/LaserScan` over ROS 2.

---

## 🛠️ Hardware Components

* **ESP32** 
* **LD06 LiDAR** (UART interface, 230400 baud)
* **micro-ROS Agent** running on a host computer or Raspberry Pi

---

## 📦 Software Components

* **Arduino framework for ESP32**
* **micro-ROS Arduino client** for ROS 2 Humble

---
## 🚀 Main Node Overview (`Leonardo_LIDAR.ino`)

This sketch implements:

1. **Wi-Fi transport** for micro-ROS over Ethernet/Wi-Fi.
2. **Publishers**:

   * `/scan` (`sensor_msgs/msg/LaserScan`): LiDAR data
   * `/lidar/debug_log` (`std_msgs/msg/String`): debug messages
3. **Subscriber**:

   * `/time_sync` (`builtin_interfaces/msg/Time`): external timestamp for time synchronization
4. **Executor** to handle time sync callback.
5. **Scan buffer management** and **synchronized publishing**.

### Key Functions

#### `resetLidarScanBuffer()`

Clears `ranges[]`, `intensities[]`, `lidar_angle_collected[]`, and resets `lidar_count_received`.

#### `isLidarScanComplete()`

Returns `true` when `lidar_count_received >= RESOLUTION` (360 points collected).

#### `lidar_get_sync_time()`

Computes ROS time by:

* Calling `rmw_uros_epoch_millis()` to get current microcontroller time (μs)
* Adding `lidar_time_offset_us` (computed from last `/time_sync` message)
* Returning adjusted `builtin_interfaces__msg__Time` (sec, nanosec)

#### `lidar_time_callback(const void * msgin)`

* Receives a `builtin_interfaces/msg/Time` from the PC
* Computes offset: `received_us - now_us`
* Sets `lidar_time_synced = true`
* Publishes a debug log on `/lidar/debug_log`

#### `lidar_publish_debug(const char * msg)`

Publishes a string message to the `/lidar/debug_log` topic.

---

## 🔄 `setup()` & `loop()` Flow

```cpp
void setup() {
  // Serial & Wi-Fi init
  Serial.begin(115200);
  set_microros_wifi_transports(...);

  // micro-ROS: support, node, publishers, subscriber, executor
  rclc_support_init(...);
  rclc_node_init_default(..., "lidar_node", ...);
  rclc_publisher_init_default(..., "scan");
  rclc_publisher_init_default(..., "lidar/debug_log");
  rclc_subscription_init_default(..., "time_sync");
  rclc_executor_init(...);
  rclc_executor_add_subscription(..., lidar_time_callback);

  // LaserScan msg initialization (frame_id, angle params, buffers)
  lidar_scan_msg.header.frame_id = "base_laser";
  lidar_scan_msg.angle_min = 0.0f;
  lidar_scan_msg.angle_max = 2.0f * M_PI;
  lidar_scan_msg.angle_increment = 2.0f * M_PI / RESOLUTION;
  lidar_scan_msg.scan_time = 1.0f / 10.0f;
  lidar_scan_msg.range_min = 0.02f;
  lidar_scan_msg.range_max = 12.0f;
  lidar_scan_msg.ranges.data = lidar_ranges;
  lidar_scan_msg.intensities.data = lidar_intensities;

  // LiDAR init & buffer reset
  lidar.Init(RX_PIN);
  resetLidarScanBuffer();
}

void loop() {
  // Handle incoming time_sync messages
  rclc_executor_spin_some(&lidar_executor, RCL_MS_TO_NS(1));

  // Wait until first time sync is received
  if (!lidar_time_synced) {
    Serial.println("[WARN] Waiting for lidar time sync...");
    return;
  }

  // Read and parse LiDAR data
  lidar.read_lidar_data();

  if (!lidar.angles.empty()) {
    // Accumulate scan points
    for (...) {
      // convert mm to m, validate range
      // store ranges[deg], intensities[deg]
    }

    // Publish when full 360° scan complete
    if (isLidarScanComplete()) {
      lidar_scan_msg.header.stamp = lidar_get_sync_time();
      rcl_publish(&lidar_scan_pub, &lidar_scan_msg, NULL);
      resetLidarScanBuffer();

      // Periodic debug every 30s
      if (!lidar_scan_published_once ||
          (rmw_uros_epoch_millis() / 1000) % 30 == 0) {
        lidar_publish_debug("[INFO] LiDAR scan published");
        lidar_scan_published_once = true;
      }
    }
  }
}
```

* **Time synchronization** ensures all scans carry a timestamp coherent with the companion PC.
* A new time sync is accepted whenever a `/time_sync` message arrives (recommended every 30 s).

---

## 📡 ROS 2 Topics

* **/scan** (`sensor_msgs/msg/LaserScan`) — LiDAR full 360° scan at \~10 Hz
* **/lidar/debug\_log** (`std_msgs/msg/String`) — Status and debug messages
* **/time\_sync** (`builtin_interfaces/msg/Time`) — External timestamp for time alignment

---

## ✅ Status

✅ Fully functional LiDAR node with time-synced publishing
🚧 Ensure companion PC publishes `/time_sync` at least every 30 s
📤 Ready for integration with mapping, SLAM, or obstacle avoidance stacks

---

## 🙏 Acknowledgements

Special thanks to [**Inoue Minoru (henjin0)**](https://github.com/henjin0/Lidar_LD06_for_Arduino) for the LD06 decoding library.

*This ESP32 node cleanly separates LiDAR data processing and synchronized ROS 2 publishing, facilitating modular integration into navigation systems.*

