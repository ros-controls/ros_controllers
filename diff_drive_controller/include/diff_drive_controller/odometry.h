/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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
 *   * Neither the name of the PAL Robotics nor the names of its
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

/*
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <ros/time.h>
#include <Eigen/Core>
#include <sophus/se2.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

#include <diff_drive_controller/integrate_function.h>

namespace diff_drive_controller
{
  namespace bacc = boost::accumulators;

  /**
   * \brief The Odometry class handles odometry readings
   * (2D pose and velocity with related timestamp)
   *
   * The odometry covariance is computed according with the model presented in:
   *
   * [Siegwart, 2004]:
   *   Roland Siegwart, Illah R. Nourbakhsh
   *   Introduction to Autonomous Mobile Robots
   *   1st Edition, 2004
   *
   * Section:
   *   5.2.4 'An error model for odometric position estimation' (pp. 186-191)
   *
   * Although the twist covariance doesn't appear explicitly, the implementation
   * here is based on the same covariance model used for the pose covariance.
   */
  class Odometry
  {
  public:
    /// SO(2) and SE(2) Lie Groups:
    typedef Sophus::SE2d SE2;

    /// Covariance matrices:
    typedef Eigen::Matrix3d Covariance;

    typedef Covariance PoseCovariance;
    typedef Covariance TwistCovariance;

    typedef Eigen::Matrix2d MeasCovariance;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Default diagonal value to initialize the covariance on the constructor:
    static const double DEFAULT_MINIMUM_TWIST_COVARIANCE;
    static const double DEFAULT_POSE_COVARIANCE;

    /**
     * \brief Constructor
     * Timestamp will get the current time value
     * Value will be set to zero
     * \param[in] velocity_rolling_window_size Rolling window size used to
     *                                         compute the velocity mean
     */
    explicit Odometry(const size_t velocity_rolling_window_size = 10);

    /**
     * \brief Initialize the odometry
     * \param[in] time Current time
     */
    void init(const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest wheels position, i.e. in
     * close loop
     * \param[in] left_position  Left  wheel position [rad]
     * \param[in] right_position Right wheel position [rad]
     * \param[in] left_velocity  Left  wheel velocity [rad/s]
     * \param[in] right_velocity Right wheel velocity [rad/s]
     * \param[in] time           Current time
     * \return true if the odometry is actually updated
     */
    bool updateCloseLoop(
        const double left_position, const double right_position,
        const double left_velocity, const double right_velocity,
        const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest velocity command, i.e. in
     * open loop
     * \param[in] linear  Linear  velocity [m/s]
     * \param[in] angular Angular velocity [rad/s]
     * \param[in] time    Current time
     * \return true if the odometry is actually updated
     */
    bool updateOpenLoop(const double linear, const double angular,
        const ros::Time &time);

    /**
     * \brief Heading getter
     * \return Heading [rad]
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
     * \brief x velocity getter
     * \return x velocity [m/s]
     */
    double getVx() const
    {
      return v_x_;
    }

    /**
     * \brief y velocity getter
     * \return y velocity [m/s]
     */
    double getVy() const
    {
      return v_y_;
    }

    /**
     * \brief yaw velocity getter
     * \return yaw velocity [rad/s]
     */
    double getVyaw() const
    {
      return v_yaw_;
    }

    /**
     * \brief Pose covariance getter
     * \return Pose covariance
     */
    const PoseCovariance& getPoseCovariance() const
    {
      return pose_covariance_;
    }

    /**
     * \brief Twist covariance getter
     * \return Twist covariance
     */
    const TwistCovariance& getTwistCovariance() const
    {
      return twist_covariance_;
    }

    /**
     * \brief Minimum twist covariance getter
     * \return Minimum twist covariance
     */
    const TwistCovariance& getMinimumTwistCovariance() const
    {
      return minimum_twist_covariance_;
    }

    /**
     * \brief Pose covariance setter
     * \param[in] pose_covariance Pose covariance
     */
    void setPoseCovariance(const PoseCovariance& pose_covariance)
    {
      pose_covariance_ = pose_covariance;
    }

    /**
     * \brief Minimum twist covariance setter
     * \param[in] twist_covariance Twist covariance
     */
    void setMinimumTwistCovariance(const TwistCovariance& twist_covariance)
    {
      minimum_twist_covariance_ = twist_covariance;
    }

    /**
     * \brief Sets the wheel parameters: radius and separation
     * \param[in] wheel_separation   Seperation between
     *                               left and right wheels [m]
     * \param[in] left_wheel_radius  Left  wheel radius [m]
     * \param[in] right_wheel_radius Right wheel radius [m]
     */
    void setWheelParams(const double wheel_separation,
        const double left_wheel_radius, const double right_wheel_radius);

    /**
     * \brief Sets the Measurement Covariance Model parameters: k_l and k_r
     * \param[in] k_l Left  wheel velocity multiplier
     * \param[in] k_r Right wheel velocity multiplier
     */
    void setMeasCovarianceParams(const double k_l, const double k_r);

    /**
     * \brief Velocity rolling window size setter
     * \param[in] velocity_rolling_window_size Velocity rolling window size
     */
    void setVelocityRollingWindowSize(
        const size_t velocity_rolling_window_size);

  private:
    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    /**
     * \brief Updates the odometry class with latest velocity command and wheel
     * velocities
     * \param[in] dp_l  Left  wheel position increment [rad]
     * \param[in] dp_r  Right wheel position increment [rad]
     * \param[in] v_l   Left  wheel velocity [rad/s]
     * \param[in] v_r   Right wheel velocity [rad/s]
     * \param[in] time  Current time
     * \return true if the odometry is actually updated
     */
    bool update(const double dp_l, const double dp_r,
        const double v_l, const double v_r, const ros::Time& time);

    /**
     * \brief Update the odometry twist with the previous and current odometry
     * pose
     * \param[in] p0   Previous odometry pose
     * \param[in] p1   Current  odometry pose
     * \param[in] v_l  Left  wheel velocity [rad/s]
     * \param[in] v_r  Right wheel velocity [rad/s]
     * \param[in] time Current time
     * \return true if the odometry twist is actually updated
     */
    bool updateTwist(const SE2& p0, const SE2& p1,
        const double v_l, const double v_r, const ros::Time& time);

    /**
     * \brief Update the measurement covariance
     * \param[in] dp_l Left  wheel position increment [rad]
     * \param[in] dp_r Right wheel position increment [rad]
     */
    void updateMeasCovariance(const double dp_l, const double dp_r);

    /**
     * \brief Reset linear and angular accumulators
     */
    void resetAccumulators();

    /// Current timestamp:
    ros::Time timestamp_;

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    /// Current velocity:
    double v_x_;   //   [m/s]
    double v_y_;   //   [m/s]
    double v_yaw_; // [rad/s]

    /// Pose covariance:
    PoseCovariance pose_covariance_;

    /// Twist (and minimum twist) covariance:
    TwistCovariance twist_covariance_;
    TwistCovariance minimum_twist_covariance_;

    /// Measurement covariance:
    MeasCovariance meas_covariance_;

    /// Wheel kinematic parameters [m]:
    double wheel_separation_;
    double left_wheel_radius_;
    double right_wheel_radius_;

    /// Measurement Covariance Model parameters:
    double k_l_;
    double k_r_;

    /// Previous wheel position/state [rad]:
    double left_position_previous_;
    double right_position_previous_;

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc v_x_acc_;
    RollingMeanAcc v_y_acc_;
    RollingMeanAcc v_yaw_acc_;

    /// Integration funcion, used to integrate the odometry:
    boost::shared_ptr<IntegrateFunction> integrate_fun_;
  };
}

#endif /* ODOMETRY_H_ */
