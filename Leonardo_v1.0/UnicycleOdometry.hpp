#ifndef UNICYCLE_ODOMETRY_HPP
#define UNICYCLE_ODOMETRY_HPP

#include <stdint.h>

/**
 * @file UnicycleOdometry.hpp
 * @brief Declaration of the UnicycleOdometry class for tracking robot pose.
 *
 * This class maintains the robot's x, y position and orientation (theta)
 * by integrating linear and angular velocity inputs over time.
 */

/**
 * @typedef float64_t
 * @brief Alias for double precision floating point.
 */
typedef double float64_t;

/**
 * @class UnicycleOdometry
 * @brief Computes and stores odometry for a differential-drive robot.
 *
 * Uses fused velocity (v), angular rate (omega), and heading (theta) inputs
 * to update Cartesian pose coordinates and provides accessors for state.
 */
class UnicycleOdometry {
public:
    /**
     * @brief Constructor.
     * @param wheel_radius Radius of the robot wheels in meters.
     * @param wheel_separation Distance between the two wheels in meters.
     *
     * Initializes the pose (x, y, theta) and motion state (v, omega) to zero.
     */
    UnicycleOdometry(float wheel_radius, float wheel_separation);

    /**
     * @brief Updates odometry from fused motion data.
     * @param v     Linear velocity input in meters per second (m/s).
     * @param omega Angular velocity input in radians per second (rad/s).
     * @param theta Absolute orientation of the robot in radians (rad).
     * @param dt    Time delta since last update in seconds (s).
     *
     * Sets internal velocity and heading, then integrates to update x and y.
     */
    void updateFromFusion(float v, float omega, float theta, float dt);

    /**
     * @brief Gets the current X position.
     * @return X coordinate in meters.
     */
    float getX() const;

    /**
     * @brief Gets the current Y position.
     * @return Y coordinate in meters.
     */
    float getY() const;

    /**
     * @brief Gets the current orientation angle (theta).
     * @return Orientation in radians (rad).
     */
    float getTheta() const;

    /**
     * @brief Sets the current orientation angle.
     * @param theta New orientation in radians (rad).
     */
    void setTheta(float theta);

    /**
     * @brief Gets the last stored linear velocity.
     * @return Linear velocity in meters per second (m/s).
     */
    float getLinearVelocity() const;

    /**
     * @brief Gets the last stored angular velocity.
     * @return Angular velocity in radians per second (rad/s).
     */
    float getAngularVelocity() const;

private:
    float x_;       /**< Position X coordinate [m] */
    float y_;       /**< Position Y coordinate [m] */
    float theta_;   /**< Orientation angle (yaw) [rad] */
    float R_;       /**< Wheel radius [m] */
    float L_;       /**< Distance between wheels [m] */
    float v_;       /**< Linear velocity [m/s] */
    float omega_;   /**< Angular velocity [rad/s] */
};

#endif // UNICYCLE_ODOMETRY_HPP

