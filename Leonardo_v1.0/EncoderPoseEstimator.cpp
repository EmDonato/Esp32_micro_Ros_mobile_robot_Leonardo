#include "EncoderPoseEstimator.hpp"
#include <math.h>

// Class: EncoderPoseEstimator
// Purpose: Estimate robot pose (orientation and velocities) using wheel encoder readings.

EncoderPoseEstimator::EncoderPoseEstimator(float wheel_radius, float wheel_separation)
    : R_(wheel_radius),     // Wheel radius [m]
      L_(wheel_separation), // Distance between wheels [m]
      theta_enc_(0.0f),     // Estimated heading [rad]
      v_(0.0f),             // Linear velocity [m/s]
      omega_(0.0f)          // Angular velocity [rad/s]
{
    // No additional initialization needed
}

// Method: update
// Inputs:  rpm_l - left wheel speed [RPM] (signed)
//          rpm_r - right wheel speed [RPM] (signed)
//          dt    - elapsed time since last update [s]
// Behavior:
//  - Converts RPM to linear wheel velocities
//  - Computes chassis linear and angular velocities
//  - Integrates angular velocity to update heading
//  - Wraps heading to [-π, +π]
void EncoderPoseEstimator::update(float rpm_l, float rpm_r, float dt) {
    // Convert RPM to rad/s and then to linear speed: v = ω * R
    float v_l = rpm_l * 2 * M_PI / 60.0f * R_;  // Left wheel linear speed [m/s]
    float v_r = rpm_r * 2 * M_PI / 60.0f * R_;  // Right wheel linear speed [m/s]

    // Compute forward (linear) velocity as the average of both wheels
    v_ = 0.5f * (v_r + v_l);

    // Compute angular velocity: difference in wheel speeds over wheel separation
    omega_ = (v_r - v_l) / L_;  // Positive ω means rotation to the left

    // Integrate angular velocity to estimate change in heading
    theta_enc_ += omega_ * dt;

    // Normalize heading to the range [-π, +π]
    if (theta_enc_ >  M_PI) {
        theta_enc_ -= 2 * M_PI;
    } else if (theta_enc_ < -M_PI) {
        theta_enc_ += 2 * M_PI;
    }
}

// Accessors:
// - getThetaEncoder(): Returns current heading estimate [rad]
// - getLinearVelocity(): Returns current linear velocity [m/s]
// - getAngularVelocity(): Returns current angular velocity [rad/s]

float EncoderPoseEstimator::getThetaEncoder() const {
    return theta_enc_;
}

float EncoderPoseEstimator::getLinearVelocity() const {
    return v_;
}

float EncoderPoseEstimator::getAngularVelocity() const {
    return omega_;
}
