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

    /**
     * \brief Constructor
     * Timestamp will get the current time value
     * Value will be set to zero
     * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
     */
    explicit Odometry(size_t velocity_rolling_window_size = 10);

    /**
     * \brief Initialize the odometry
     * \param time [in] Current time
     */
    void init(const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest wheels position, i.e. in
     * close loop
     * \param left_pos  [in] Left  wheel position [rad]
     * \param right_pos [in] Right wheel position [rad]
     * \param time      [in] Current time
     * \return true if the odometry is actually updated
     */
    bool updateCloseLoop(double left_pos, double right_pos, const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest wheels velocity, i.e. in
     * close loop
     * \param left_vel  [in] Left  wheel velocity [rad/s]
     * \param right_vel [in] Right wheel velocity [rad/s]
     * \param time      [in] Current time
     * \return true if the odometry is actually updated
     */
    bool updateCloseLoopFromVelocity(double left_pos, double right_pos, const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest velocity command, i.e. in
     * open loop
     * \param linear  [in] Linear  velocity [m/s]
     * \param angular [in] Angular velocity [rad/s]
     * \param time    [in] Current time
     * \return true if the odometry is actually updated
     */
    bool updateOpenLoop(double linear, double angular, const ros::Time &time);

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
     * \brief pose covariance getter
     * \return pose covariance
     */
    const PoseCovariance& getPoseCovariance() const
    {
      return pose_covariance_;
    }

    /**
     * \brief twist covariance getter
     * \return twist covariance
     */
    const TwistCovariance& getTwistCovariance() const
    {
      return twist_covariance_;
    }

    /**
     * \brief minimum twist covariance getter
     * \return minimum twist covariance
     */
    const TwistCovariance& getMinimumTwistCovariance() const
    {
      return minimum_twist_covariance_;
    }

    /**
     * \brief pose covariance setter
     * \param pose_covariance [in] pose covariance
     */
    void setPoseCovariance(const PoseCovariance& pose_covariance)
    {
      pose_covariance_ = pose_covariance;
    }

    /**
     * \brief minimum twist covariance setter
     * \param twist_covariance [in] twist covariance
     */
    void setMinimumTwistCovariance(const TwistCovariance& twist_covariance)
    {
      minimum_twist_covariance_ = twist_covariance;
    }

    /**
     * \brief Sets the wheel parameters: radius and separation
     * \param wheel_separation   [in] Seperation between
     *                                left and right wheels [m]
     * \param left_wheel_radius  [in] Left  wheel radius [m]
     * \param right_wheel_radius [in] Right wheel radius [m]
     */
    void setWheelParams(double wheel_separation,
        double left_wheel_radius, double right_wheel_radius);

    /**
     * \brief Sets the Measurement Covariance Model parameters: k_l and k_r
     * \param k_l [in] Left  wheel velocity multiplier
     * \param k_r [in] Right wheel velocity multiplier
     */
    void setMeasCovarianceParams(double k_l, double k_r);

    /**
     * \brief Velocity rolling window size setter
     * \param[in] velocity_rolling_window_size Velocity rolling window size
     */
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  private:

  public:
    /// Default diagonal value to initialize the covariance on the constructor:
    static const double DEFAULT_MINIMUM_TWIST_COVARIANCE;
    static const double DEFAULT_POSE_COVARIANCE;

  private:
    /**
     * \brief Updates the odometry class with latest velocity command and wheel
     * velocities
     * \param v_l  [in] Left  wheel velocity displacement [rad]
     * \param v_r  [in] Right wheel velocity displacement [rad]
     * \param time [in] Current time
     * \return true if the odometry is actually updated
     */
    bool update(double v_l, double v_r, const ros::Time& time);

    /**
     * \brief Update the odometry twist with the previous and current odometry
     * pose
     * \param p0   [in] Previous odometry pose
     * \param p1   [in] Current  odometry pose
     * \param v_l  [in] Left  wheel velocity displacement [rad]
     * \param v_r  [in] Right wheel velocity displacement [rad]
     * \param time [in] Current time
     * \return true if the odometry twist is actually updated
     */
    bool updateTwist(const SE2& p0, const SE2& p1,
        const double v_l, const double v_r, const ros::Time& time);

    /**
     * \brief Update the measurement covariance
     * \param v_l [in] Left  wheel velocity displacement [rad]
     * \param v_r [in] Right wheel velocity displacement [rad]
     */
    void updateMeasCovariance(double v_l, double v_r);


  private:
    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

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
    double left_wheel_old_pos_;
    double right_wheel_old_pos_;

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
