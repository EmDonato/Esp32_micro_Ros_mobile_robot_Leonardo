/**
 * @file control_teleop.cpp
 * @brief Teleoperation node converting velocity commands into individual wheel speeds
 *
 * This node subscribes to geometry_msgs::msg::Twist messages on the "cmd_vel" topic,
 * computes the corresponding left and right wheel angular velocities, and publishes
 * them as a geometry_msgs::msg::Vector3 on the "velocity_ctrl" topic.
 */

#include "leonardo/control_teleop.hpp"
#include <algorithm>  // For std::clamp
using namespace std::chrono_literals;

namespace leonardo
{
/**
 * @brief Compute individual wheel angular speeds from normalized velocity commands.
 *
 * Denormalizes the input commands, applies limits, and converts linear wheel speeds
 * into angular velocities.
 *
 * @param v_norm Normalized linear velocity in range [-1, 1]
 * @param w_norm Normalized angular velocity in range [-1, 1]
 * @return std::pair<float, float> First element is left wheel angular speed (rad/s),
 *         second element is right wheel angular speed (rad/s).
 */
std::pair<float, float> compute_wheel_speeds(float v_norm, float w_norm)
{
    // Robot kinematics constants
    constexpr float WHEEL_RADIUS   = 0.0346f;               ///< Wheel radius [m]
    constexpr float BASE_WIDTH     = 0.20f;                 ///< Distance between wheels [m]
    constexpr float V_MAX          = 0.362f;                ///< Max linear speed [m/s]
    constexpr float W_MAX          = 2.0f * V_MAX / BASE_WIDTH; ///< Max angular speed [rad/s]

    // Prevent excessive spinning when stationary
    if (std::abs(v_norm) < 1e-2f) {
        w_norm *= 0.8f;
    }

    // Scale normalized inputs to physical units
    float v = std::clamp(v_norm, -1.0f, 1.0f) * V_MAX;
    float w = std::clamp(w_norm, -1.0f, 1.0f) * W_MAX;

    // Linear speeds for each wheel
    float v_right = v + (BASE_WIDTH / 2.0f) * w;
    float v_left  = v - (BASE_WIDTH / 2.0f) * w;

    // Ensure wheels do not exceed linear speed limit
    float max_speed = std::max(std::abs(v_right), std::abs(v_left));
    if (max_speed > V_MAX) {
        float scale = V_MAX / max_speed;
        v_right *= scale;
        v_left  *= scale;
    }

    // Convert linear speed [m/s] to angular speed [rad/s]
    float omega_right = v_right / WHEEL_RADIUS;
    float omega_left  = v_left  / WHEEL_RADIUS;

    return {omega_left, omega_right};
}
}  // namespace leonardo

/**
 * @class ControlTeleop
 * @brief ROS2 node for teleoperation control of differential-drive robot.
 */
ControlTeleop::ControlTeleop()
: Node("control_teleop")
{
    // Configure QoS: best-effort, no durability
    rclcpp::QoS qos_settings(rclcpp::KeepLast(1));
    qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Publisher: output wheel speeds
    publisher_ = create_publisher<geometry_msgs::msg::Vector3>("velocity_ctrl", qos_settings);

    // Subscriber: input velocity commands
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_joy", 10,
        std::bind(&ControlTeleop::twist_callback, this, std::placeholders::_1)
    );
}

/**
 * @brief Callback processing incoming Twist messages.
 *
 * Converts linear.x and angular.z from the Twist message into wheel angular velocities,
 * logs the result, and publishes on the velocity_ctrl topic.
 *
 * @param msg Shared pointer to incoming Twist message
 */
void ControlTeleop::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    auto [omega_left, omega_right] = leonardo::compute_wheel_speeds(
        msg->linear.x, msg->angular.z
    );

    geometry_msgs::msg::Vector3 output;
    output.x = omega_left;
    output.y = omega_right;
    output.z = 0.0f;

    RCLCPP_INFO(
        get_logger(),
        "[ControlTeleop] cmd_vel -> v=%.2f m/s, w=%.2f rad/s produced \u03C9_left=%.2f rad/s, \u03C9_right=%.2f rad/s",
        msg->linear.x, msg->angular.z, omega_left, omega_right
    );

    publisher_->publish(output);
}

/**
 * @brief Program entry point.
 *
 * Initializes the ROS2 client library, creates the ControlTeleop node, and spins until shutdown.
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlTeleop>());
    rclcpp::shutdown();
    return 0;
}

