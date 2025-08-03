#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define WHEEL_SEPARATION 0.20f
#define V_MAX 0.216f
#define W_MAX 2.136f

class CmdVelNormalizer : public rclcpp::Node {
public:
  CmdVelNormalizer()
  : Node("cmd_vel_normalizer")
  {
    // Publisher on a new topic for normalized commands
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel_nav2_norm", 10);

    // Subscription to Nav2's cmd_vel output ( published on /cmd_vel )
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10,
      std::bind(&CmdVelNormalizer::callback, this, std::placeholders::_1));
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    geometry_msgs::msg::Twist norm_msg;

    // Normalize to [-1, +1] based on your V_MAX and W_MAX
    norm_msg.linear.x  = msg->linear.x  / V_MAX;
    norm_msg.angular.z = msg->angular.z / W_MAX;

    // Zero out unused axes
    norm_msg.linear.y   = 0.0;
    norm_msg.linear.z   = 0.0;
    norm_msg.angular.x  = 0.0;
    norm_msg.angular.y  = 0.0;

    pub_->publish(norm_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelNormalizer>());
  rclcpp::shutdown();
  return 0;
}

