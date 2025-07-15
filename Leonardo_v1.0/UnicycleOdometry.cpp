#include "UnicycleOdometry.hpp"
#include <math.h>

/**
 * @file UnicycleOdometry.cpp
 * @brief Implementation of the UnicycleOdometry class for pose tracking.
 *
 * Integrates linear and angular velocities to update the robot's X, Y, and theta.
 */

/**
 * @brief Constructor.
 * @param wheel_radius Radius of the wheels in meters (unused in this class but stored for potential use).
 * @param wheel_separation Distance between wheels in meters (unused in this class but stored for potential use).
 *
 * Initializes position (x, y) and orientation (theta) to zero,
 * as well as stored velocity values.
 */
UnicycleOdometry::UnicycleOdometry(float wheel_radius, float wheel_separation)
    : x_(0.0f),
      y_(0.0f),
      theta_(0.0f),
      R_(wheel_radius),  // Wheel radius [m]
      L_(wheel_separation),  // Wheel separation [m]
      v_(0.0f),         // Linear velocity [m/s]
      omega_(0.0f)      // Angular velocity [rad/s]
{
    // No additional initialization required
}

/**
 * @brief Updates odometry using fused velocity and heading estimates.
 * @param v     Linear velocity input [m/s].
 * @param omega Angular velocity input [rad/s].
 * @param theta Absolute orientation of the robot [rad].
 * @param dt    Time interval since the last update [s].
 *
 * Sets internal velocity and heading states, then integrates
 * to update Cartesian coordinates x and y.
 */
void UnicycleOdometry::updateFromFusion(float v, float omega, float theta, float dt) {
    // Update stored state
    theta_ = theta;
    v_ = v;
    omega_ = omega;

    // Integrate motion in the world frame
    x_ += v_ * cosf(theta_) * dt;
    y_ += v_ * sinf(theta_) * dt;
}

/** @brief Returns the current X position.
 *  @return X coordinate [m].
 */
float UnicycleOdometry::getX() const {
    return x_;
}

/** @brief Returns the current Y position.
 *  @return Y coordinate [m].
 */
float UnicycleOdometry::getY() const {
    return y_;
}

/** @brief Returns the current orientation (theta).
 *  @return Yaw angle [rad].
 */
float UnicycleOdometry::getTheta() const {
    return theta_;
}

/** @brief Manually set the current orientation.
 *  @param tht New yaw angle [rad].
 */
void UnicycleOdometry::setTheta(float tht) {
    theta_ = tht;
}

/** @brief Returns the last stored linear velocity.
 *  @return Linear velocity [m/s].
 */
float UnicycleOdometry::getLinearVelocity() const {
    return v_;
}

/** @brief Returns the last stored angular velocity.
 *  @return Angular velocity [rad/s].
 */
float UnicycleOdometry::getAngularVelocity() const {
    return omega_;
}
