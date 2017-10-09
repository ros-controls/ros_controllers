/*********************************************************************
 * Software License Agreement (BSD License)
 *
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
 *   * Neither the name of Irstea nor the names of its
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

#include <four_wheel_steering_controller/odometry.h>

#include <boost/bind.hpp>

namespace four_wheel_steering_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : last_update_timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_(0.0)
  , linear_x_(0.0)
  , linear_y_(0.0)
  , angular_(0.0)
  , steering_track_(0.0)
  , wheel_steering_y_offset_(0.0)
  , wheel_radius_(0.0)
  , wheel_base_(0.0)
  , wheel_old_pos_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_accel_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , linear_jerk_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , front_steer_vel_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , rear_steer_vel_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  {
  }

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    last_update_timestamp_ = time;
  }

  bool Odometry::update(const double &fl_speed, const double &fr_speed,
                        const double &rl_speed, const double &rr_speed,
                        double front_steering, double rear_steering, const ros::Time &time)
  {
    const double front_tmp = cos(front_steering)*(tan(front_steering)-tan(rear_steering))/wheel_base_;
    const double front_left_tmp = front_tmp/sqrt(1-steering_track_*front_tmp*cos(front_steering)
                                               +pow(steering_track_*front_tmp/2,2));
    const double front_right_tmp = front_tmp/sqrt(1+steering_track_*front_tmp*cos(front_steering)
                                                +pow(steering_track_*front_tmp/2,2));
    const double fl_speed_tmp = fl_speed * (1/(1-wheel_steering_y_offset_*front_left_tmp));
    const double fr_speed_tmp = fr_speed * (1/(1-wheel_steering_y_offset_*front_right_tmp));
    const double front_linear_speed = wheel_radius_ * copysign(1.0, fl_speed_tmp+fr_speed_tmp)*
        sqrt((pow(fl_speed,2)+pow(fr_speed,2))/(2+pow(steering_track_*front_tmp,2)/2.0));

    const double rear_tmp = cos(rear_steering)*(tan(front_steering)-tan(rear_steering))/wheel_base_;
    const double rear_left_tmp = rear_tmp/sqrt(1-steering_track_*rear_tmp*cos(rear_steering)
                                               +pow(steering_track_*rear_tmp/2,2));
    const double rear_right_tmp = rear_tmp/sqrt(1+steering_track_*rear_tmp*cos(rear_steering)
                                                +pow(steering_track_*rear_tmp/2,2));
    const double rl_speed_tmp = rl_speed * (1/(1-wheel_steering_y_offset_*rear_left_tmp));
    const double rr_speed_tmp = rr_speed * (1/(1-wheel_steering_y_offset_*rear_right_tmp));
    const double rear_linear_speed = wheel_radius_ * copysign(1.0, rl_speed_tmp+rr_speed_tmp)*
        sqrt((pow(rl_speed_tmp,2)+pow(rr_speed_tmp,2))/(2+pow(steering_track_*rear_tmp,2)/2.0));

    angular_ = (front_linear_speed*front_tmp + rear_linear_speed*rear_tmp)/2.0;

    linear_x_ = (front_linear_speed*cos(front_steering) + rear_linear_speed*cos(rear_steering))/2.0;
    linear_y_ = (front_linear_speed*sin(front_steering) - wheel_base_*angular_/2.0
                + rear_linear_speed*sin(rear_steering) + wheel_base_*angular_/2.0)/2.0;
    linear_ =  copysign(1.0, rear_linear_speed)*sqrt(pow(linear_x_,2)+pow(linear_y_,2));

    /// Compute x, y and heading using velocity
    const double dt = (time - last_update_timestamp_).toSec();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    last_update_timestamp_ = time;
    /// Integrate odometry:
    integrateXY(linear_x_*dt, linear_y_*dt, angular_*dt);

    linear_accel_acc_((linear_vel_prev_ - linear_)/dt);
    linear_vel_prev_ = linear_;
    linear_jerk_acc_((linear_accel_prev_ - bacc::rolling_mean(linear_accel_acc_))/dt);
    linear_accel_prev_ = bacc::rolling_mean(linear_accel_acc_);
    front_steer_vel_acc_((front_steer_vel_prev_ - front_steering)/dt);
    front_steer_vel_prev_ = front_steering;
    rear_steer_vel_acc_((rear_steer_vel_prev_ - rear_steering)/dt);
    rear_steer_vel_prev_ = rear_steering;
    return true;
  }

  void Odometry::setWheelParams(double steering_track, double wheel_steering_y_offset, double wheel_radius, double wheel_base)
  {
    steering_track_   = steering_track;
    wheel_steering_y_offset_ = wheel_steering_y_offset;
    wheel_radius_     = wheel_radius;
    wheel_base_       = wheel_base;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  void Odometry::integrateXY(double linear_x, double linear_y, double angular)
  {
    const double delta_x = linear_x*cos(heading_) - linear_y*sin(heading_);
    const double delta_y = linear_x*sin(heading_) + linear_y*cos(heading_);

    x_ += delta_x;
    y_ += delta_y;
    heading_ += angular;
  }

  void Odometry::integrateRungeKutta2(double linear, double angular)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_       += linear * cos(direction);
    y_       += linear * sin(direction);
    heading_ += angular;
  }

  void Odometry::integrateExact(double linear, double angular)
  {
    if (fabs(angular) < 1e-6)
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

  void Odometry::resetAccumulators()
  {
    linear_accel_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    linear_jerk_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    front_steer_vel_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    rear_steer_vel_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }

} // namespace four_wheel_steering_controller
