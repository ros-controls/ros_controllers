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
 *   * Neither the name of the Willow Garage nor the names of its
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
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <ros/time.h>
#include <angles/angles.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/bind.hpp>

namespace diff_drive_controller
{
  namespace bacc = boost::accumulators;

  /**
   * @brief The Odometry class handles odometry readings
   *  (2D oriented position with related timestamp)
   */
  class Odometry
  {
  public:
    typedef boost::function<void(double, double)> IntegrationFunction;

    /**
      * Timestamp will get the current time value
      * Value will be set to zero
      */
    Odometry(size_t velocity_rolling_window_size = 10)
      : timestamp_(0),
        x_(0),
        y_(0),
        heading_(0),
        wheel_separation_(0.0),
        wheel_radius_(0.0),
        left_wheel_old_pos_(0.0),
        right_wheel_old_pos_(0.0),
        linear_acc_(RollingWindow::window_size = velocity_rolling_window_size),
        angular_acc_(RollingWindow::window_size = velocity_rolling_window_size),
        integrate_fun_(boost::bind(&Odometry::integrationExact, this, _1, _2))
    { }

    /**
     * @brief update the odometry class with latest wheels position
     * @param left_pos
     * @param right_pos
     * @param time
     * @return
     */
    bool update(double left_pos, double right_pos, const ros::Time &time)
    {
      // get current wheel joint positions
      const double left_wheel_cur_pos  = left_pos  * wheel_radius_;
      const double right_wheel_cur_pos = right_pos * wheel_radius_;

      // estimate velocity of wheels using old and current position
      const double left_wheel_est_vel  = left_wheel_cur_pos  - left_wheel_old_pos_;
      const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

      // update old position with current
      left_wheel_old_pos_  = left_wheel_cur_pos;
      right_wheel_old_pos_ = right_wheel_cur_pos;

      // compute linear and angular diff
      const double linear  = (right_wheel_est_vel + left_wheel_est_vel) * 0.5 ;
      const double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation_;

      // integrate
      integrate_fun_(linear, angular);

      // can not estimate speed with very small time intervals
      const double dt = (time - timestamp_).toSec();
      if(dt < 0.0001)
        return false; // interval too small to integrate with

      timestamp_ = time;

      // estimate speeds: use a rolling mean to filter them out
      linear_acc_(linear/dt);
      angular_acc_(angular/dt);

      return true;
    }

    double getHeading() const
    {
      return heading_;
    }

    double getX() const
    {
      return x_;
    }

    double getY() const
    {
      return y_;
    }

    ros::Time getTimestamp() const
    {
      return timestamp_;
    }

    double getLinearEstimated() const
    {
      return bacc::rolling_mean(linear_acc_);
    }

    double getAngularEstimated() const
    {
      return bacc::rolling_mean(angular_acc_);
    }

    void setWheelParams(double wheel_separation, double wheel_radius)
    {
      wheel_separation_ = wheel_separation;
      wheel_radius_     = wheel_radius;
    }

  private:

    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    /**
     * @brief Function to update the odometry based on the velocities of the robot
     * @param linear : linear velocity m/s * DT (linear desplacement) computed by encoders
     * @param angular   : angular velocity rad/s * DT (angular desplacement) computed by encoders
     * @param time  : timestamp of the measured velocities
     *
     */
    void integrationByRungeKutta(double linear, double angular)
    {
      double direction = heading_ + angular*0.5;

      /// RUNGE-KUTTA 2nd ORDER INTEGRATION
      x_       += linear * cos(direction);
      y_       += linear * sin(direction);
      heading_ += angular;

      /// Normalization of angle between -Pi and Pi
      heading_ = angles::normalize_angle(heading_);
    }

    /**
     * @brief Other possible integration method provided by the class
     * @param linear
     * @param angular
     */
    void integrationExact(double linear, double angular)
    {
      if(fabs(angular) < 10e-3)
        integrationByRungeKutta(linear, angular);
      else
      {
        ///EXACT INTEGRATION (should solve problems when angular is zero)
        double thetaOld = heading_;
        heading_ += angular;
        x_ +=   (linear/angular)*(sin(heading_) - sin(thetaOld));
        y_ +=  -(linear/angular)*(cos(heading_) - cos(thetaOld));
      }
    }

    ros::Time timestamp_;

    // current position
    double x_;
    double y_;
    double heading_;

    // kinematic parameters
    double wheel_separation_;
    double wheel_radius_;

    // state at n-1
    double left_wheel_old_pos_;
    double right_wheel_old_pos_;

    // velocity accumulators
    RollingMeanAcc linear_acc_;
    RollingMeanAcc angular_acc_;

    IntegrationFunction integrate_fun_;
  };
}
#endif /* ODOMETRY_H_ */
