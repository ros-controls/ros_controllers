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

#include <swerve_controller/odometry.h>

#include <boost/bind.hpp>

namespace swerve_controller
{

    Odometry::Odometry(size_t velocity_rolling_window_size)
    : last_update_timestamp_(0.0)
    , x_(0.0)
    , y_(0.0)
    , heading_(0.0)
    , linear_x_(0.0)
    , linear_y_(0.0)
    , angular_(0.0)
    , steering_track_(0.0)
    , wheel_radius_(0.0)
    , wheel_base_(0.0)
    {
    }

    void Odometry::init(const ros::Time& time)
    {
        // Reset accumulators and timestamp:
        last_update_timestamp_ = time;
    }

    bool Odometry::update(const double &lf_speed, const double &rf_speed,
                          const double &lh_speed, const double &rh_speed,
                          const double &lf_steering, const double &rf_steering,
                          const double &lh_steering, const double &rh_steering,
                          const ros::Time &time)
    {
        // Compute velocity vectors in X-Y for each wheel
        const double lf_b = sin(lf_steering) * lf_speed * wheel_radius_;
        const double lf_d = cos(lf_steering) * lf_speed * wheel_radius_;
        const double rf_b = sin(rf_steering) * rf_speed * wheel_radius_;
        const double rf_c = cos(rf_steering) * rf_speed * wheel_radius_;
        const double lh_a = sin(lh_steering) * lh_speed * wheel_radius_;
        const double lh_d = cos(lh_steering) * lh_speed * wheel_radius_;
        const double rh_a = sin(rh_steering) * rh_speed * wheel_radius_;
        const double rh_c = cos(rh_steering) * rh_speed * wheel_radius_;

        // Compute robot velocities components
        const double a = (rh_a + lh_a) / 2.0;
        const double b = (rf_b + lf_b) / 2.0;
        const double c = (rf_c + rh_c) / 2.0;
        const double d = (lf_d + lh_d) / 2.0;

        // Average angular speed
        const double angular_1 = (b - a) / wheel_base_;
        const double angular_2 = (c - d) / steering_track_;
        angular_ = (angular_1 + angular_2) / 2.0;

        // Average linear speed
        const double linear_x_1 = angular_ * (wheel_base_ / 2.0) + c;
        const double linear_x_2 = -angular_ * (wheel_base_ / 2.0) + d;
        const double linear_y_1 = angular_ * (steering_track_ / 2.0) + a;
        const double linear_y_2 = -angular_ * (steering_track_ / 2.0) + b;

        linear_x_ = (linear_x_1 + linear_x_2) / 2.0;
        linear_y_ = (linear_y_1 + linear_y_2) / 2.0;

        // Avoid too small intervals
        const double dt = (time - last_update_timestamp_).toSec();
        if (dt < 0.0001)
            return false;

        // Integrate odometry
        last_update_timestamp_ = time;
        integrateXY(linear_x_*dt, linear_y_*dt, angular_*dt);

        return true;
    }

    void Odometry::setWheelParams(double steering_track, double wheel_radius, double wheel_base)
    {
        steering_track_   = steering_track;
        wheel_radius_     = wheel_radius;
        wheel_base_       = wheel_base;
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

}  // namespace swerve_controller
