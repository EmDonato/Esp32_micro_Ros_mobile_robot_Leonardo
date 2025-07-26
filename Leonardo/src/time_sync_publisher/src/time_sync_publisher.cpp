#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;

class TimeSyncPublisher : public rclcpp::Node
{
public:
  TimeSyncPublisher()
  : Node("time_sync_publisher")
  {
    publisher_ = this->create_publisher<builtin_interfaces::msg::Time>("time_sync", 10);

    timer_ = this->create_wall_timer(
      30s,  // Periodo: 30 secondi
      std::bind(&TimeSyncPublisher::publish_time, this)
    );

    RCLCPP_INFO(this->get_logger(), "Time sync publisher started. Publishing every 30 seconds.");
  }

private:
  void publish_time()
  {
    auto now = this->now();  // Ora corrente del sistema ROS (Clock del PC)
    builtin_interfaces::msg::Time msg;
    msg.sec = now.seconds();       // sec e nanosec sono separati
    msg.nanosec = now.nanoseconds() % 1000000000;

    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Published time: %d.%09u", msg.sec, msg.nanosec);
  }

  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
  
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeSyncPublisher>());
  rclcpp::shutdown();
  return 0;
}

