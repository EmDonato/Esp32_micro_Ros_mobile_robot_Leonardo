/**
 * @file odom_to_tf_node.cpp
 * @brief Converts odometry messages into TF2 transforms for frame broadcasting
 *
 * This node subscribes to the /odom topic for nav_msgs::msg::Odometry messages,
 * extracts the robot's pose, and publishes it on the /tf topic as a TransformStamped.
 * Ensures that odom_frame and base_frame are available in TF2 for other nodes.
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

/**
 * @class OdomToTfNode
 * @brief ROS2 node that broadcasts transforms from odometry
 *
 * Listens to Odometry messages and re-publishes the pose as a TF2 transform.
 */
class OdomToTfNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new OdomToTfNode
   *
   * Initializes the TransformBroadcaster and subscription to /odom.
   */
  OdomToTfNode()
  : Node("odom_to_tf_node")
  {
    // Create TF2 broadcaster for dynamic transforms
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscribe to the odometry topic
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::QoS(10),
      std::bind(&OdomToTfNode::odomCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "OdomToTfNode initialized, listening to /odom");
  }

private:
  /**
   * @brief Callback for incoming Odometry messages
   *
   * Extracts the pose and stamps it into a TransformStamped message,
   * then broadcasts it via TF2 using PC time.
   *
   * @param msg Shared pointer to the received Odometry message
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Use local clock time instead of microcontroller's timestamp
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = msg->header.frame_id;       ///< Typically "odom"
    t.child_frame_id = msg->child_frame_id;         ///< Typically "base_link"

    // Copy translation from odometry pose
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    // Copy rotation from odometry pose
    t.transform.rotation = msg->pose.pose.orientation;

    // Broadcast the transform
    tf_broadcaster_->sendTransform(t);

    RCLCPP_DEBUG(this->get_logger(), "Broadcasted transform %s -> %s at time %u.%u",
      t.header.frame_id.c_str(), t.child_frame_id.c_str(),
      t.header.stamp.sec, t.header.stamp.nanosec);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  ///< Subscription to /odom topic
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;      ///< TF2 transform broadcaster
};

/**
 * @brief Main function: initialize ROS2 and spin the OdomToTfNode
 *
 * @param argc Argument count
 * @param argv Argument values
 * @return int Exit code
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomToTfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

