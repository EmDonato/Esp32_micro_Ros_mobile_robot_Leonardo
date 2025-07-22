---
name: Feature request
about: Suggest an idea for this project
title: ''
labels: ''
assignees: ''

---

**Description**
A clear and concise description of the feature you would like to see implemented.
Example: Support for SLAM map saving directly from ESP32 running micro-ROS without relying on an external ROS 2 node.

**Use Case / Motivation**
Explain why this feature would be useful.
Example: This feature would allow lightweight mobile robots to save SLAM maps or intermediate data directly on the microcontroller, reducing the dependency on a full ROS 2 environment and enabling more autonomous operation in network-constrained scenarios.

**Proposed Solution**
Describe how you envision the feature could be implemented, or leave it open for discussion.
Example: An optional micro-ROS service or action could be provided to trigger a map save, with the data written to local storage (e.g., SD card) via a custom micro-ROS publisher.

**Alternatives Considered**
List any alternative approaches you have considered and why they might not be sufficient.
Example: Using a ROS 2 node to handle map saving is currently possible but limits the robot’s autonomy in cases where the connection to ROS 2 is unstable or unavailable.

**Additional Context**
Provide any additional context, links to similar projects, or references that could be useful.
Example: Reference to similar functionality in [slam\_toolbox](https://github.com/SteveMacenski/slam_toolbox) where map serialization is triggered via a ROS 2 service.
