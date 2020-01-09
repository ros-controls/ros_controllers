/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Exobotic
 *  Copyright (c) 2017, Irstea
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Exobotic nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef SWERVE_CONTROLLER_ODOMETRY_H_
#define SWERVE_CONTROLLER_ODOMETRY_H_

#include <ros/time.h>
#include <boost/function.hpp>

namespace swerve_controller
{

/**
* \brief The Odometry class handles odometry readings
* (2D pose and velocity with related timestamp)
*/
class Odometry
{
    public:
    /// Integration function, used to integrate the odometry:
    typedef boost::function<void(double, double)> IntegrationFunction;

    /**
     * \brief Constructor
     * Timestamp will get the current time value
     * Value will be set to zero
     * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
     */
    explicit Odometry(size_t velocity_rolling_window_size = 10);

    /**
     * \brief Initialize the odometry
     * \param time Current time
     */
    void init(const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest wheels and steerings position
     * \param lf_speed front left wheel vehicle speed [rad/s]
     * \param rf_speed front right wheel vehicle speed [rad/s]
     * \param lh_speed rear left wheel vehicle speed [rad/s]
     * \param rh_speed rear right wheel vehicle speed [rad/s]
     * \param lf_steering front left steering position [rad]
     * \param rf_steering front right steering position [rad]
     * \param lh_steering rear left steering position [rad]
     * \param rh_steering rear right steering position [rad]
     * \param time Current time
     * \return true if the odometry is actually updated
     */
    bool update(const double& lf_speed, const double& rf_speed, const double& lh_speed,
                const double& rh_speed, const double& lf_steering, const double& rf_steering,
                const double& lh_steering, const double& rh_steering, const ros::Time &time);

    /**
     * \brief heading getter
     * \return heading [rad]
     */
    double getHeading() const
    {
      return heading_;
    }

    /**
     * \brief x position getter
     * \return x position [m]
     */
    double getX() const
    {
      return x_;
    }

    /**
     * \brief y position getter
     * \return y position [m]
     */
    double getY() const
    {
      return y_;
    }

    /**
     * \brief linear velocity getter along X on the robot base link frame
     * \return linear velocity [m/s]
     */
    double getLinearX() const
    {
      return linear_x_;
    }

    /**
     * \brief linear velocity getter along Y on the robot base link frame
     * \return linear velocity [m/s]
     */
    double getLinearY() const
    {
      return linear_y_;
    }

    /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
    double getAngular() const
    {
      return angular_;
    }

    /**
     * \brief Sets the wheel parameters: radius and separation
     * \param steering_track          Seperation between left and right steering joints [m]
     * \param wheel_radius            Wheel radius [m]
     * \param wheel_base              Wheel base [m]
     */
    void setWheelParams(double steering_track, double wheel_radius, double wheel_base);

    /**
     * \brief Velocity rolling window size setter
     * \param velocity_rolling_window_size Velocity rolling window size
     */
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

    private:
    /**
     * \brief Integrates the velocities (linear on x and y and angular)
     * \param linear_x  Linear  velocity along x of the robot frame  [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param linear_y  Linear  velocity along y of the robot frame   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateXY(double linear_x, double linear_y, double angular);

    /**
     * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateRungeKutta2(double linear, double angular);

    /**
     * \brief Integrates the velocities (linear and angular) using exact method
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateExact(double linear, double angular);

    /**
     *  \brief Reset linear and angular accumulators
     */
    void resetAccumulators();

    // Current timestamp:
    ros::Time last_update_timestamp_;

    // Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    // Current velocity:
    double linear_x_, linear_y_;  // [m/s]
    double angular_;  // [rad/s]

    // Wheel kinematic parameters [m]:
    double steering_track_;
    double wheel_steering_y_offset_;
    double wheel_radius_;
    double wheel_base_;
};

}  // namespace swerve_controller

#endif  // SWERVE_CONTROLLER_ODOMETRY_H_
