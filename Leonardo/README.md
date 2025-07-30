# Mobile Robot ROS2 Teleoperation and SLAM (Companion PC code)

This program runs on the companion PC and contains all ROS 2 packages needed to control the Leonardo differential‑drive robot:

Teleoperation control and command multiplexing (twist_mux)

Static and odometry TF broadcasters

Time synchronization with the ESP32 micro‑ROS nodes

SLAM Toolbox mapping and RViz2 visualization
---

## Repository Structure

```

leonardo/
├── build/                   # CMake build artifacts (generated)
├── install/                 # Install space (generated)
├── log/                     # Colcon and ROS2 logs (generated)
├── config/                  # some yalm file
├── src/                     # Source files and package structure
│   ├── leonardo/            # Teleoperation control node package
│   │   ├── CMakeLists.txt   # Build instructions
│   │   ├── package.xml      # Package metadata
│   │   ├── launch/          # Launch files
│   │   │   └── teleop.launch.py
│   │   ├── src/             # Implementation source files
│   │   │   └── control_teleop.cpp
│   │   ├── urdf/             # for RVIZ2
│   │   │   └── unicycle.urdf
│   │   └── include/         # Public headers
│   │       └── leonardo/
│   │           └── control_teleop.hpp
│   ├── odom_to_tf_cpp/      # Odometry to TF broadcaster package
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src/
│   │       └── odom_to_tf_node.cpp
│   ├── laser_tf_cpp/        # Static TF broadcaster package
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src/
│   │       └── static_tf_base_laser.cpp
│   ├── time_sync_publisher/        # node for time sync
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src/
│   │       └── time_sync_publisher.cpp
│   ├── cmd_vel_normalizer/        # node for normalize cmd_vel form nav2
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src/
│   │       └── cmd_vel_normalizer.cpp
│   └── mio_slam_toolbox/    # SLAM Toolbox config and integration
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── config/
│           └── slam_toolbox_config.yaml


```

---

## Environment and Dependencies

* **ROS2 Distribution**:  Humble 
* **micro-ROS Agent**: Installed under `~/Desktop/agent`
* **External Packages**:

  * `joy` and `teleop_twist_joy`
  * `slam_toolbox`
  * `tf2_ros`, `geometry_msgs`, `nav_msgs`

Install system dependencies and ROS2 packages:

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-joy ros-${ROS_DISTRO}-teleop-twist-joy \
                 ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-tf2-ros
```

---

## Building the Workspace

```bash
# From workspace root:
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
source install/setup.bash
```

---

## Launching the System

### Teleoperation and TF Broadcasters

```bash
ros2 launch leonardo launch_leonardo.py
```

# Mobile Robot ROS 2 Teleoperation, twist\_mux, Time Sync & SLAM

This repository contains the ROS 2 packages for controlling a differential‑drive mobile robot with micro‑ROS integration, teleoperation, command arbitration (`twist_mux`), time synchronization with a companion PC, and SLAM Toolbox mapping. All components now reside within the \`\` workspace for streamlined development.

---

## 📂 Repository Structure

```
leonardo/
├── build/                   # CMake build artifacts (generated)
├── install/                 # Install space (generated)
├── log/                     # Colcon and ROS 2 logs (generated)
├── src/                     # Source packages
│   ├── leonardo_ctrl/       # Teleoperation & command mux package
│   │   ├── launch/          # Launch files
│   │   │   ├── teleop_launch.py
│   │   │   └── twist_mux.launch.py
│   │   ├── config/          # YAML configs (joy, twist_mux)
│   │   │   ├── joy.yaml
│   │   │   └── twist_mux.yaml
│   │   ├── src/             # Node implementations
│   │   │   ├── control_teleop.cpp
│   │   │   └── twist_mux_node.cpp
│   │   └── include/         # Public headers
│   │       └── leonardo_ctrl/
│   │           └── teleop.hpp
│   ├── odom_to_tf_cpp/      # Odometry → TF broadcaster package
│   │   └── src/             # Implementation
│   │       └── odom_to_tf_node.cpp
│   ├── static_tf_broadcaster/ # Static TF base → laser
│   │   └── src/
│   │       └── static_tf_base_laser.cpp
│   ├── time_sync/           # Companion PC time sync subscriber
│   │   ├── launch/
│   │   │   └── time_sync.launch.py
│   │   ├── src/
│   │   │   └── time_sync_node.cpp
│   │   └── config/
│   │       └── time_sync.yaml
│   ├── mio_slam_toolbox/    # SLAM Toolbox integration & config
│   │   ├── launch/
│   │   │   └── slam_toolbox_async.launch.py
│   │   └── config/
│   │       └── mapper_params_online_async.yaml
│   └── rviz_config/         # RViz2 visualization files
│       ├── model.urdf       # Robot URDF model
│       └── leonardo.rviz    # RViz2 display configuration
└── README.md                # Project overview (this file)
```

---

## 🔧 Environment & Dependencies

* **ROS 2 Distribution**: Humble
* **micro‑ROS Agent**: installed under `~/Desktop/agent`
* **Required Packages**:

  * `joy`, `teleop_twist_joy`
  * `twist_mux`
  * `tf2_ros`, `geometry_msgs`, `nav_msgs`
  * `slam_toolbox`

```bash
sudo apt update
sudo apt install \
  ros-${ROS_DISTRO}-joy \
  ros-${ROS_DISTRO}-teleop-twist-joy \
  ros-${ROS_DISTRO}-twist-mux \
  ros-${ROS_DISTRO}-slam-toolbox \
  ros-${ROS_DISTRO}-tf2-ros
```

---

## 🏗️ Building the Workspace

```bash
# From workspace root:
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Launching the System

### Teleoperation and TF Broadcasters

Run the combined teleoperation, command arbitration, transform broadcasters, and micro‑ROS agent sequence:

```bash
ros2 launch leonardo_ctrl teleop_launch.py
```

**This single launch file (`teleop_launch.py`) handles:**

1. **Joystick Input** (`joy_node`)

   * Reads game controller events via `/dev/input/js1`
   * Configurable `deadzone`, `autorepeat_rate`, and `sticky_buttons`
2. **Teleoperation Mapping** (`teleop_twist_joy`)

   * Converts joystick axes/buttons into `/cmd_vel_joy` (`geometry_msgs/Twist`)
   * Requires holding an enable button (index 5) to send commands
   * Supports scaling of linear (x, y) and angular (yaw) velocities
3. **Command Arbitration** (`twist_mux`)

   * Merges multiple velocity sources (`/cmd_vel_joy`, `/cmd_vel_auto`, etc.) into `/cmd_vel_mux`
   * Parameterized via `twist_mux.yaml` for priority and timeout
4. **Control Teleop** (`control_teleop`)

   * Translates the final `/cmd_vel_mux` into individual wheel speed commands and publishes on `/velocity_ctrl`
5. **Static & Odometry TF Broadcasters**

   * `odom_to_tf_node`: broadcasts `odom` → `base_link`
   * `static_tf_base_laser`: broadcasts fixed `base_link` → `base_laser`
6. **micro‑ROS Agent**

   * Executes the agent in-process via UDP6 on port 8888
   * Bridges ROS 2 topics to embedded micro‑ROS nodes
7. **Time Synchronization Publisher** (`time_sync_publisher`)

   * Periodically publishes ROS time on `/time_sync`
   * Ensures all sensor and scan messages use a coherent timestamp
8. **Robot State Publisher**

   * Loads the `unicycle.urdf` model to broadcast joint states and base transforms

```python
# Example snippet from teleop_launch.py:
ld.add_action(Node(
    package='joy', executable='joy_node', name='game_controller',
    parameters=[{'device_id':1, 'deadzone':0.05, 'autorepeat_rate':0.0}],
    output='screen'
))

ld.add_action(Node(
    package='teleop_twist_joy', executable='teleop_node',
    remappings=[('cmd_vel','/cmd_vel_joy')], output='screen'
))

ld.add_action(Node(
    package='twist_mux', executable='twist_mux',
    parameters=[['config/twist_mux.yaml']],
    remappings=[('cmd_vel_out','/cmd_vel_mux')],
    output='screen'
))
```

By combining all these actions in one file, you simplify startup and ensure consistent configuration across teleoperation, TF publishers, arbitration, and micro‑ROS integration.

---

### SLAM Toolbox Mapping

```bash
ros2 launch slam_toolbox online_async_launch.py   slam_params_file:=Leonardo/src/mio_slam_toolbox/config/mapper_params_online_async.yaml
```

* **mode**: `mapping` to build a 2D occupancy grid.
* **scan\_topic**: defaults to `/scan` for LiDAR input.
* **odom\_frame**, **map\_frame**, **base\_frame**: frame conventions.

Adjust `slam_toolbox_config.yaml` parameters for resolution, loop closure, and optimization.

### RViz2 Visualization

```bash
rviz2
```

Use an RViz2 config that displays:

* TF frames (`odom`, `base_link`, `base_laser`, `map`)
* Laser scans on `/scan`
* SLAM map (`/map` topic)
* Odometry path or footprint visualization

---

## Docker Container 

STILL IN PROGRESS

---
## 🔄 Workflow

1. Start micro‑ROS agent on PC: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
2. Launch teleop & twist\_mux (Sect. 1)
3. Launch time synchronization (Sect. 2)
4. Bring up TF broadcasters (Sect. 3)
5. Start SLAM Toolbox mapping (Sect. 4)
6. Visualize in RViz2 (Sect. 5)

---

## ✅ Status

* Teleoperation & twist\_mux: ✅
* Time sync node: ✅ (must run companion PC publisher every 30 s)
* TF broadcasters: ✅
* SLAM Toolbox mapping: ✅
* RViz2 config & URDF: ✅

---

## Support

For issues, contributions, or questions, please open an issue on the GitHub repository.
