#ifndef CONTROL_TELEOP_HPP
#define CONTROL_TELEOP_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <utility>

/// Calcola le velocità angolari (rad/s) delle ruote [sinistra, destra]
std::pair<float, float> compute_wheel_speeds(float v_norm, float w_norm);

class ControlTeleop : public rclcpp::Node
{
public:
  ControlTeleop();

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

#endif // CONTROL_TELEOP_HPP
