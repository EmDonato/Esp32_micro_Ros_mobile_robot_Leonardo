# Mobile Robot ROS2 Teleoperation and SLAM

This repository contains the ROS2 components for controlling a differential‑drive mobile robot with micro-ROS integration https://github.com/EmDonato/Esp32_micro_Ros_mobile_robot_Leonardo.git. It provides teleoperation, static and dynamic TF broadcasters, odometry-to-TF conversion, and SLAM Toolbox mapping, together with launch files and configuration for Rviz2 visualization.

---

## Repository Structure

```

leonardo/
├── build/                   # CMake build artifacts (generated)
├── install/                 # Install space (generated)
├── log/                     # Colcon and ROS2 logs (generated)
├── src/                     # Source files and package structure
│   ├── leonardo/            # Teleoperation control node package
│   │   ├── CMakeLists.txt   # Build instructions
│   │   ├── package.xml      # Package metadata
│   │   ├── launch/          # Launch files
│   │   │   └── teleop.launch.py
│   │   ├── src/             # Implementation source files
│   │   │   └── control_teleop.cpp
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

This launch file will:

1. Start the **joy\_node** to read your game controller.
2. Start **teleop\_twist\_joy** to convert joystick axes into `/cmd_vel` Twist messages.
3. Launch your **control\_teleop** node to compute individual wheel speeds and publish on `/velocity_ctrl`.
4. Start **odom\_to\_tf\_node** to broadcast odometry as TF (`odom` → `base_link`).
5. Start **static\_tf\_base\_laser** to publish the fixed transform (`base_link` → `base_laser`).
6. Run the **micro-ROS agent** (UDP6 on port 8888) to bridge ROS2 and embedded devices.

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

## Docker Container (Optional)

A pre-built Docker container is available if you prefer an isolated environment. You can find it at:

```
<INSERT-DOCKER-CONTAINER-URL-HERE>
```

To run:

```bash
docker pull your-docker-repo/mobile-ros2:latest
docker run --rm -it \
  --network host \
  -v $(pwd):/workspace \
  mobile-ros2:latest /bin/bash
```

---

## Support

For issues, contributions, or questions, please open an issue on the GitHub repository.
