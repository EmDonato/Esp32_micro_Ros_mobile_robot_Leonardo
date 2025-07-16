# Differential-Drive ESP32 Robot with micro-ROS Humble

This project implements a differential-drive robot using an ESP32 microcontroller and integrates it with micro-ROS Humble for ROS 2 communication. The system fuses wheel encoder data and IMU measurements to perform odometry and publishes it over ROS 2, while also subscribing to control commands for closed-loop velocity control. Actually it's only for teleoperation application, so it's implemented only joystick navigation. Stay tuned for the autonoumus navigation.

## Table of Contents

1. [Hardware Setup](#hardware-setup)
2. [Software Architecture](#software-architecture)

   * [Leonardo_v1.0.ino](#Leonardo-v10ino)
   * [Variable.hpp / Variable.cpp](#variablehpp--variablecpp)
   * [ISR.hpp / ISR\_Handlers](#isrhpp--isr_handlers)
   * [EncoderPoseEstimator](#encoderposeestimator)
   * [UnicycleOdometry](#unicycleodometry)
   * [ControlTools](#controltools)
   * [IMU\_Tools](#imu_tools)
   * [micro\_ROS\_fun.hpp / micro\_ROS\_fun.cpp](#micro_ros_funhpp--micro_ros_funcpp)
3. [Workflow](#workflow)
4. [ROS 2 Integration](#ros-2-integration)
5. [PID Control](#pid-control)
6. [Calibration](#calibration)
7. [License](#license)

## Hardware Setup

* **Microcontroller**: ESP32
* **Wheel Encoders**: Magnetic encoders connected to GPIO pins (motor 12v dc 25ga-370, 130 rpm)
* **IMU**: MPU6050 connected via I2C (SDA, SCL pins)
* **Motor Driver**: L298N controlled via PWM and digital direction pins (findable here: https://github.com/EmDonato/motorDriver.git)
* **LiDAR**: LD06 (handled in a separate module https://github.com/EmDonato/ld06_esp32_microRos.git; requires an additional ESP32 when integrated)
* **Battery**: 12v nimh battery
* **DC-DC converter**: XL4015 STEP DOWN


## Software Architecture

### Leonardo_v1.0.ino

* **setup()**: Initializes Serial, motor driver, IMU (with calibration), encoders (attach ISRs), PID controllers, and micro-ROS.
* **loop()**: Spins the micro-ROS executor to handle timers and subscriptions.

### Variable.hpp / Variable.cpp

* **Global Constants**: Hardware pin definitions, encoder PPR, wheel geometry, PID limits, etc.
* **Extern Objects**: `motorDriver`, `encoder_L`, `encoder_R`, `imu`, `odom_msg`, `executor`, etc.

### ISR.hpp / ISR\_Handlers

* **encoderISR\_L / encoderISR\_R**: Interrupt service routines for left/right encoders, updating tick counts and direction.

### EncoderPoseEstimator

* **Purpose**: Convert wheel RPM to linear and angular velocity and integrate heading.
* **Key Method**: `update(rpm_left, rpm_right, dt)` computes velocities and updates `theta_enc`.

### UnicycleOdometry

* **Purpose**: Integrate fused linear and angular velocities to track `(x, y, theta)` pose.
* **Key Method**: `updateFromFusion(v, omega, theta, dt)` updates position based on current heading.

### ControlTools

* **computeRPM**: Convert angular velocity (rad/s) to RPM.
* **normalize\_angle**: Wrap angles to `[-π, π]`.
* **complementaryFilter**: Fuse encoder and IMU yaw estimates.

### IMU\_Tools

* **calibrateGyroZ()**: Compute gyroscope Z-axis bias by sampling while stationary.

### micro\_ROS\_fun.hpp / micro\_ROS\_fun.cpp

* **initMicroROS()**: Configures Wi-Fi transport, ROS nodes, publishers, subscribers, timers, and executor.
* **Error Macros**: `RCCHECK`, `RCSOFTCHECK` for robust ROS calls.
* **Callbacks**:

  * `subscription_callback_control`: Updates wheel velocity references on command.
  * `timer_callback_IMU`: Integrates gyro data to update IMU yaw.
  * `timer_callback_odometry`: Reads encoders, fuses yaw, updates odometry, and publishes.
  * `timer_callback_ctrl_PID`: Runs PID loops to drive motors.
  * `init_odom_msg`: Initializes odometry message fields.
  * `yaw_to_quat`: Utility to convert yaw to quaternion.
  * `get_now`: Retrieves current time for ROS stamps.

## Workflow

1. **Startup**: ESP32 boots, runs `setup()`, initializes all hardware and micro-ROS.
2. **Calibration**: Gyro bias is calculated by `calibrateGyroZ()`.
3. **Periodic Tasks**:

   * **IMU Timer**: Updates IMU-based yaw every 10 ms.
   * **Odometry Timer**: Every 50 ms, encoder RPMs are read, heading fused, and odometry published.
   * **Control Timer**: Every 50 ms, PID computes motor commands based on velocity references.
4. **Control**: Velocity commands published to `/velocity_ctrl` topic are received and processed.
5. **Execution**: `loop()` continuously spins the ROS executor to service callbacks.
## micro-ROS Agent & Arduino Library Setup

To enable communication between your ESP32 robot and the ROS 2 network, follow these steps (refer to the official guide: [https://github.com/micro-ROS/micro\_ros\_arduino](https://github.com/micro-ROS/micro_ros_arduino)):

1. **Install and Run the micro-ROS Agent**

   * Ensure ROS 2 Humble is installed on your host (PC or Raspberry Pi).
   * Build and run the agent for UDP transport:

     ```bash
     ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
     ```

2. **Configure the Arduino micro-ROS Library**

   * Update your workspace’s `colcon.meta` to increase RMW limits:

     ```json
     {
       "rmw_microxrcedds": {
         "cmake-args": [
           "-DRMW_UXRCE_MAX_NODES=5",
           "-DRMW_UXRCE_MAX_PUBLISHERS=10",
           "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=5",
           "-DRMW_UXRCE_MAX_SERVICES=1",
           "-DRMW_UXRCE_MAX_CLIENTS=1",
           "-DRMW_UXRCE_MAX_HISTORY=4",
           "-DRMW_UXRCE_TRANSPORT=custom"
         ]
       }
     }
     ```
   * Rebuild the `rmw_microxrcedds` package:

     ```bash
     colcon build --merge-install --packages-select rmw_microxrcedds
     ```

3. **Verify Connectivity**

   * Be sure to change the wifi parameters with your own parameters (micro_ros_fun.cpp)


This configuration lets your ESP32 publish odometry and subscribe to velocity commands over ROS 2 using micro-ROS.

## Detailed ROS 2 Integration

### Message Types

* **nav\_msgs/Odometry**: publishes robot pose (position + orientation) and velocity.
* **geometry\_msgs/Vector3**: subscribes to control input (left and right wheel angular velocities).

### Published Topics

* **/odom** (`nav_msgs/Odometry`)

  * **Description**: Robot's pose and velocity relative to the `odom` frame.
  * **Message Fields**:

    * `header.stamp`: Timestamp of measurement (built from `millis()`).
    * `header.frame_id`: "odom".
    * `child_frame_id`: "base\_link".
    * `pose.pose.position`: X, Y coordinates.
    * `pose.pose.orientation`: Quaternion from fused yaw.
    * `twist.twist.linear.x`: Linear velocity.
    * `twist.twist.angular.z`: Angular velocity.

### Subscribed Topics

* **/velocity\_ctrl** (`geometry_msgs/Vector3`)

  * **Description**: Target angular velocities for left (x) and right (y) wheels.
  * **Usage**: Received values are stored in `w_L_ref` and `w_R_ref`, converted to RPM, and fed into PID controllers.

### Additional Communication

* **Teleoperation Input**

  * **Source**: A separate teleop node publishes raw velocity commands (e.g., `geometry_msgs/Twist`).
  * **Processing**: These commands are parsed by a custom node/script into separate wheel angular velocities and republished on `/velocity_ctrl`. This allows integration with keyboard or joystick teleoperation.


## PID Control library
findable here: https://github.com/EmDonato/PID_control.git 
* **Controllers**: Two independent PID loops for left and right wheels
* **Feedforward**: Enabled to improve dynamic response
* **Deadzone**: Zero threshold prevents jitter around zero velocity

## Calibration

* **Gyroscope**: `calibrateGyroZ()` averages 500 samples to compute bias

## License

Released under the MIT License.
