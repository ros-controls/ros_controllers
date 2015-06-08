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

#include <diff_drive_controller/odometry.h>

#include <Eigen/Core>

#include <boost/bind.hpp>

namespace diff_drive_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , v_x_(0.0)
  , v_y_(0.0)
  , v_yaw_(0.0)
  , wheel_separation_(0.0)
  , wheel_radius_(0.0)
  , left_wheel_old_pos_(0.0)
  , right_wheel_old_pos_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , v_x_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , v_y_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , v_yaw_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , integrate_fun_(boost::bind(&Odometry::integrateExact, this, _1, _2))
  {
  }

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators:
    v_x_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    v_y_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    v_yaw_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);

    // Reset timestamp:
    timestamp_ = time;
  }

  bool Odometry::update(double left_pos, double right_pos, const ros::Time &time)
  {
    /// Get current wheel joint positions:
    const double left_wheel_cur_pos  = left_pos  * left_wheel_radius_;
    const double right_wheel_cur_pos = right_pos * right_wheel_radius_;

    /// Estimate velocity of wheels using old and current position:
    const double left_wheel_est_vel  = left_wheel_cur_pos  - left_wheel_old_pos_;
    const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

    /// Update old position with current:
    left_wheel_old_pos_  = left_wheel_cur_pos;
    right_wheel_old_pos_ = right_wheel_cur_pos;

    /// Compute linear and angular diff:
    const double linear  = (right_wheel_est_vel + left_wheel_est_vel) * 0.5 ;
    const double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation_;

    /// Safe current state:
    const SE2 p0(SE2::Scalar(heading_), SE2::Point(x_, y_));

    /// Integrate odometry:
    integrate_fun_(linear, angular);

    /// Update twist:
    const SE2 p1(SE2::Scalar(heading_), SE2::Point(x_, y_));

    return updateTwist(p0, p1, time);
  }

  bool Odometry::updateOpenLoop(double linear, double angular, const ros::Time &time)
  {
    /// Safe current state:
    const SE2 p0(SE2::Scalar(heading_), SE2::Point(x_, y_));

    /// Integrate odometry:
    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;
    integrate_fun_(linear * dt, angular * dt);

    /// Update twist:
    const SE2 p1(SE2::Scalar(heading_), SE2::Point(x_, y_));

    return updateTwist(p0, p1, time);
  }

  bool Odometry::updateTwist(const SE2& p0, const SE2& p1, const ros::Time& time)
  {
    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if(dt < 0.0001)
      return false; // Interval too small to integrate with

    timestamp_ = time;

    /// Compute relative transformation:
    const SE2 p = p0.inverse() * p1;

    /// Retrieve rotation and translation:
    /// Note that we don't use the log from SE(2) because we didn't use exp
    /// to create p0 and p1.
    /// So instead of:
    ///
    ///   const SE2::Tangent v = p.log();
    ///
    /// we use the following:
    const SE2::ConstTranslationReference t = p.translation();

    v_x_   = t[0];
    v_y_   = t[1];
    v_yaw_ = p.so2().log();

    /// Estimate speeds using a rolling mean to filter them out:
    v_x_acc_(v_x_/dt);
    v_y_acc_(v_y_/dt);
    v_yaw_acc_(v_yaw_/dt);

    v_x_   = bacc::rolling_mean(v_x_acc_);
    v_y_   = bacc::rolling_mean(v_y_acc_);
    v_yaw_ = bacc::rolling_mean(v_yaw_acc_);

    return true;
  }

  void Odometry::setWheelParams(double wheel_separation,
      double left_wheel_radius, double right_wheel_radius)
  {
    wheel_separation_   = wheel_separation;
    left_wheel_radius_  = left_wheel_radius;
    right_wheel_radius_ = right_wheel_radius;
  }

  void Odometry::integrateRungeKutta2(double linear, double angular)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_       += linear * cos(direction);
    y_       += linear * sin(direction);
    heading_ += angular;
  }

  /**
   * \brief Other possible integration method provided by the class
   * \param linear
   * \param angular
   */
  void Odometry::integrateExact(double linear, double angular)
  {
    if(fabs(angular) < 10e-3)
      integrateRungeKutta2(linear, angular);
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = heading_;
      const double r = linear/angular;
      heading_ += angular;
      x_       +=  r * (sin(heading_) - sin(heading_old));
      y_       += -r * (cos(heading_) - cos(heading_old));
    }
  }

} // namespace diff_drive_controller
