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

#include <diff_drive_controller/autodiff_integrate_function.h>

#include <diff_drive_controller/direct_kinematics_integrate_functor.h>
#include <diff_drive_controller/runge_kutta_2_integrate_functor.h>
#include <diff_drive_controller/exact_integrate_functor.h>

#include <Eigen/Core>

namespace diff_drive_controller
{
  namespace bacc = boost::accumulators;

  const double Odometry::DEFAULT_MINIMUM_TWIST_COVARIANCE = 1e-9;
  const double Odometry::DEFAULT_POSE_COVARIANCE = 1e-6;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , timestamp_twist_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , v_x_(0.0)
  , v_y_(0.0)
  , v_yaw_(0.0)
  , d_x_(0.0)
  , d_y_(0.0)
  , d_yaw_(0.0)
  , wheel_separation_(0.0)
  , left_wheel_radius_(0.0)
  , right_wheel_radius_(0.0)
  , k_l_(1.0)
  , k_r_(1.0)
  , left_position_previous_(0.0)
  , right_position_previous_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , v_x_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , v_y_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , v_yaw_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , integrate_fun_(
      new AutoDiffIntegrateFunction<DirectKinematicsIntegrateFunctor,
                                    ExactIntegrateFunctor>(
      new DirectKinematicsIntegrateFunctor<ExactIntegrateFunctor>(
      new ExactIntegrateFunctor)))
  {
    meas_covariance_.setZero();

    minimum_twist_covariance_.setIdentity();
    minimum_twist_covariance_ *= DEFAULT_MINIMUM_TWIST_COVARIANCE;

    twist_covariance_ = minimum_twist_covariance_;

    pose_covariance_.setIdentity();
    pose_covariance_ *= DEFAULT_POSE_COVARIANCE;

    incremental_pose_covariance_.setZero();
  }

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = timestamp_twist_ = time;
  }

  bool Odometry::updateCloseLoop(
      const double left_position, const double right_position,
      const double left_velocity, const double right_velocity,
      const ros::Time &time)
  {
    /// Estimate wheels position increment using previous and current position:
    const double left_position_increment  = left_position  - left_position_previous_;
    const double right_position_increment = right_position - right_position_previous_;

    /// Update previous position with current:
    left_position_previous_  = left_position;
    right_position_previous_ = right_position;

    /// Update pose and twist:
    return update(left_position_increment, right_position_increment,
        left_velocity, right_velocity, time);
  }

  bool Odometry::updateOpenLoop(const double linear, const double angular,
      const ros::Time& time)
  {
    /// Compute wheel velocities, i.e. Inverse Kinematics:
    // @todo we should expose a method to compute this:
    // void inverseKinematics(const double linear, const double angular,
    //                        double& left, double &right);
    // note that the input/output can be velocity or relative position
    // (incremental position/displacement)
    //
    // this method would be use here and to 'Compute wheels velocities' in
    // diff_drive_controller; here is displacement, there is velocity
    //double v_l, v_r;
    //inverseKinematics(linear, angular, v_l, v_r);
    const double v_l = (linear - angular * wheel_separation_ / 2.0) / left_wheel_radius_;
    const double v_r = (linear + angular * wheel_separation_ / 2.0) / right_wheel_radius_;

    /// Compute time step:
    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;

    /// Update pose and twist:
    return update(v_l * dt, v_r * dt, v_l, v_r, time);
  }

  bool Odometry::update(
      const double dp_l, const double dp_r,
      const double v_l, const double v_r,
      const ros::Time& time)
  {
    /// Integrate odometry pose:
    IntegrateFunction::PoseJacobian J_pose;
    IntegrateFunction::MeasJacobian J_meas;
    (*integrate_fun_)(x_, y_, heading_, dp_l, dp_r, J_pose, J_meas);

    /// Update Measurement Covariance with the wheel joint position increments:
    updateMeasCovariance(dp_l, dp_r);

    /// Update pose covariance:
    pose_covariance_ = J_pose * pose_covariance_ * J_pose.transpose() +
                       J_meas * meas_covariance_ * J_meas.transpose();

    /// Update incremental pose:
    // @todo in principle there's no need to use v_l, v_r at all!
    //updateIncrementalPose(v_l * dt, v_r * dt);
    updateIncrementalPose(dp_l, dp_r);

    return true;
  }

  void Odometry::updateIncrementalPose(const double dp_l, const double dp_r)
  {
    /// Integrate incremental odometry pose:
    IntegrateFunction::PoseJacobian J_pose;
    IntegrateFunction::MeasJacobian J_meas;
    (*integrate_fun_)(d_x_, d_y_, d_yaw_, dp_l, dp_r, J_pose, J_meas);

    /// Update Measurement Covariance with the wheel joint position increments:
    updateMeasCovariance(dp_l, dp_r);

    /// Update incremental pose covariance:
    incremental_pose_covariance_ =
        J_pose * incremental_pose_covariance_ * J_pose.transpose() +
        J_meas * meas_covariance_ * J_meas.transpose();
  }

  bool Odometry::updateTwist(const ros::Time& time)
  {
    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_twist_).toSec();
    if (dt < 0.0001)
    {
      return false;
    }

    timestamp_twist_ = time;

    /// Estimate speeds using a rolling mean to filter them out:
    const double f = 1.0 / dt;

    v_x_acc_(d_x_ * f);
    v_y_acc_(d_y_ * f);
    v_yaw_acc_(d_yaw_ * f);

    v_x_   = bacc::rolling_mean(v_x_acc_);
    v_y_   = bacc::rolling_mean(v_y_acc_);
    v_yaw_ = bacc::rolling_mean(v_yaw_acc_);

    /// Update twist covariance:
    IntegrateFunction::PoseJacobian J_twist =
       IntegrateFunction::PoseJacobian::Identity() * f;
    twist_covariance_ = J_twist * incremental_pose_covariance_ * J_twist.transpose();

    /// Add minimum (diagonal) covariance to avoid ill-conditioned covariance
    /// matrices, i.e. with a very large condition number, which would make
    /// inverse or Cholesky decomposition fail on many algorithms:
    twist_covariance_ += minimum_twist_covariance_;

    /// Reset incremental pose and its covariance:
    d_x_ = d_y_ = d_yaw_ = 0.0;
    incremental_pose_covariance_.setZero();

    return true;
  }

  void Odometry::updateMeasCovariance(double v_l, double v_r)
  {
    /// Compute Measurement Covariance Model:
    // @todo This can be extended to support lateral slippage
    // k_s_ * [see/find Olson notes]
    meas_covariance_.diagonal() << k_l_ * std::abs(v_l),
                                   k_r_ * std::abs(v_r);
  }

  void Odometry::setWheelParams(const double wheel_separation,
      const double left_wheel_radius, const double right_wheel_radius)
  {
    wheel_separation_   = wheel_separation;
    left_wheel_radius_  = left_wheel_radius;
    right_wheel_radius_ = right_wheel_radius;

    integrate_fun_->setWheelParams(wheel_separation,
        left_wheel_radius, right_wheel_radius);
  }

  void Odometry::setVelocityRollingWindowSize(
      const size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  void Odometry::setMeasCovarianceParams(const double k_l, const double k_r)
  {
    k_l_ = k_l;
    k_r_ = k_r;
  }

  void Odometry::resetAccumulators()
  {
    v_x_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    v_y_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    v_yaw_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }

} // namespace diff_drive_controller
