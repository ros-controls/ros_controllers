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
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , v_x_(0.0)
  , v_y_(0.0)
  , v_yaw_(0.0)
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

    // There's no need to initialize the twist covariance because it's updated
    // from scratch on each cycle, but it's safer to initialize it anyway:
    twist_covariance_ = minimum_twist_covariance_;

    pose_covariance_.setIdentity();
    pose_covariance_ *= DEFAULT_POSE_COVARIANCE;
  }

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
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

    /// Update pose and twist:
    return update(v_l * dt, v_r * dt, v_l, v_r, time);
  }

  bool Odometry::update(
      const double dp_l, const double dp_r,
      const double v_l, const double v_r,
      const ros::Time& time)
  {
    /// Safe current state:
    const SE2 p0(SE2::Scalar(heading_), SE2::Point(x_, y_));

    /// Integrate odometry pose:
    IntegrateFunction::PoseJacobian J_pose;
    IntegrateFunction::MeasJacobian J_meas;
    (*integrate_fun_)(x_, y_, heading_, dp_l, dp_r, J_pose, J_meas);

    /// Update Measurement Covariance with the wheel joint position increments:
    updateMeasCovariance(dp_l, dp_r);

    /// Update pose covariance:
    pose_covariance_ = J_pose * pose_covariance_ * J_pose.transpose() +
                       J_meas * meas_covariance_ * J_meas.transpose();

    /// Safe new state:
    const SE2 p1(SE2::Scalar(heading_), SE2::Point(x_, y_));

    /// Update twist:
    return updateTwist(p0, p1, v_l, v_r, time);
  }

  bool Odometry::updateTwist(const SE2& p0, const SE2& p1,
      double v_l, double v_r, const ros::Time& time)
  {
    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if (dt < 0.0001)
      return false;

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

    /// Integrate odometry twist:
    /// Note that this is done this way because it isn't trivial to compute the
    /// Jacobians for the relative transformation between p0 and p1
    const double dp_l = v_l * dt;
    const double dp_r = v_r * dt;

    IntegrateFunction::PoseJacobian J_dummy;
    IntegrateFunction::MeasJacobian J_meas;
    double x = 0.0, y = 0.0, yaw = 0.0;
    (*integrate_fun_)(x, y, yaw, dp_l, dp_r, J_dummy, J_meas);

    /// Include the Jacobian of dividing by dt, which is equivalent to divide
    /// all the elements of the other Jacobian by dt:
    J_meas /= dt;

    /// Update Measurement Covariance with the wheel joint velocites:
    updateMeasCovariance(dp_l, dp_r);

    /// Update twist covariance:
    twist_covariance_ = J_meas * meas_covariance_ * J_meas.transpose();

    /// Add minimum (diagonal) covariance to avoid ill-conditioned covariance
    /// matrices, i.e. with a very large condition number, which would make
    /// inverse or Cholesky decomposition fail on many algorithms:
    twist_covariance_ += minimum_twist_covariance_;

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
