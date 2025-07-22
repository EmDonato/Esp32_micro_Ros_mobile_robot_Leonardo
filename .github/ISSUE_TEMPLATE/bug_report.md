---
name: Bug report
about: Create a report to help us improve
title: ''
labels: bug
assignees: EmDonato

---

**Description**
A clear and concise description of the issue.
Example: SLAM data stops updating after a few minutes when running micro-ROS on ESP32.

**Steps to Reproduce**

1. Flash the ESP32 with the provided firmware \[link if applicable].
2. Launch the SLAM node on ROS 2 using \[launch file name].
3. Start the robot and monitor relevant topics (e.g., `/scan`, `/odom`, `/tf`).
4. After a certain amount of time, the SLAM process stops updating, and topics freeze.

**Expected Behavior**
The mobile robot should continuously publish sensor data (e.g., `/scan`, `/odom`), with SLAM performing real-time map updates without interruption.

**Actual Behavior**
After a few minutes of operation, sensor topics are no longer published and the SLAM node stops updating the map. Restarting the micro-ROS agent temporarily restores functionality, but the issue reappears.

**Environment**

* Hardware: ESP32 (model: e.g., ESP32-WROOM-32, ESP32-S3)
* ROS 2 Distribution: Humble / Iron / Rolling (specify)
* micro-ROS Version: \[version or commit hash]
* micro-ROS Agent Version: \[version or commit hash]
* SLAM Package: \[e.g., slam\_toolbox, cartographer\_ros]
* Operating System: Ubuntu 22.04 (or specify)
* Communication Interface: UART / Wi-Fi / Ethernet
* Other relevant environment details: \[e.g., Wi-Fi router model, firmware customization]

**Logs / Error Messages**

```
[micro_ros_agent] [WARN] serial error: protocol error
[slam_toolbox] TF transform timeout between scan and map
```

**Additional Context**

* The issue is reproducible on multiple ESP32 boards with the same configuration.
* The problem only appears when SLAM is active; without SLAM, the system remains stable.
* Restarting the agent clears the issue temporarily, but the problem returns after several minutes.
* \[Optional] Link to video, screenshots, or repository: \[link]
