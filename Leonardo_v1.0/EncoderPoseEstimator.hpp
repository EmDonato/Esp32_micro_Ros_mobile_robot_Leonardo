#ifndef ENCODERPOSEESTIMATOR_H
#define ENCODERPOSEESTIMATOR_H

/**
 * @class EncoderPoseEstimator
 * @brief Estimates the robot's orientation and velocities using wheel encoder data.
 *
 * Provides methods to update yaw (theta), linear velocity, and angular velocity
 * based on wheel RPM readings.
 */
class EncoderPoseEstimator {
public:
    /**
     * @brief Constructor.
     * @param wheel_radius Radius of the wheels in meters.
     * @param wheel_separation Distance between the two wheels in meters.
     */
    EncoderPoseEstimator(float wheel_radius, float wheel_separation);

    /**
     * @brief Updates the pose estimate using wheel RPMs.
     * @param rpm_left  Left wheel speed in RPM (signed).
     * @param rpm_right Right wheel speed in RPM (signed).
     * @param dt         Time elapsed since the last update, in seconds.
     *
     * Converts RPM to linear wheel speeds, computes the average linear velocity
     * and differential angular velocity, then integrates to update the heading.
     */
    void update(float rpm_left, float rpm_right, float dt);

    /**
     * @brief Returns the current estimated yaw angle (theta).
     * @return Estimated yaw in radians within the range [-π, +π].
     */
    float getThetaEncoder() const;

    /**
     * @brief Returns the current estimated linear velocity of the robot.
     * @return Linear velocity in meters per second.
     */
    float getLinearVelocity() const;

    /**
     * @brief Returns the current estimated angular velocity of the robot.
     * @return Angular velocity in radians per second.
     */
    float getAngularVelocity() const;

    // TODO: Enable this if manual correction of the estimated heading is required.
    // void setCorrectedTheta(float corrected_theta);

private:
    float R_;         /**< Wheel radius [m] */
    float L_;         /**< Distance between wheels [m] */
    float theta_enc_; /**< Estimated yaw angle from encoders [rad] */
    float v_;         /**< Estimated linear velocity [m/s] */
    float omega_;     /**< Estimated angular velocity [rad/s] */
};

#endif // ENCODERPOSEESTIMATOR_H
