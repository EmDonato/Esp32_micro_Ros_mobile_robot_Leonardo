/**
 * @file static_tf_base_laser.cpp
 * @brief Broadcasts a static transform between base_link and base_laser frames
 *
 * This node publishes a one-time static transform from the robot's base_link to
 * the laser scanner frame (base_laser), enabling accurate sensor data visualization
 * and frame lookup in TF2.
 */

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>

/**
 * @class StaticTfPublisher
 * @brief ROS2 node that publishes a static transform for the laser scanner
 */
class StaticTfPublisher : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Static Tf Publisher node
   *
   * Initializes the static transform broadcaster and publishes the transform once.
   */
  StaticTfPublisher()
  : Node("static_tf_base_laser")
  {
    // Create static transform broadcaster
    static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Prepare the transform message
    geometry_msgs::msg::TransformStamped t;

    // Use current time; static transform remains constant afterwards
    t.header.stamp = this->now();
    t.header.frame_id = "base_link";     ///< Parent frame
    t.child_frame_id  = "base_laser";    ///< Child frame

    // Translation: laser position relative to base_link
    t.transform.translation.x = 0.095;    ///< 9.5 cm forward
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.18;     ///< 18 cm above base

    // Orientation: roll, pitch, yaw in radians
    double roll  = M_PI;                  ///< 180° rotation around X
    double pitch = 0.0;
    double yaw   = -M_PI_2;               ///< -90° rotation around Z

    // Convert RPY to quaternion
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Broadcast the transform once
    static_broadcaster_->sendTransform(t);

    RCLCPP_INFO(this->get_logger(),
      "Published static transform %s → %s",
      t.header.frame_id.c_str(),
      t.child_frame_id.c_str());
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;  ///< Static transform broadcaster
};

/**
 * @brief Main entry point: initialize ROS, run the static transform publisher node
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticTfPublisher>();
  // Keep node alive (no need to resend static transform)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

