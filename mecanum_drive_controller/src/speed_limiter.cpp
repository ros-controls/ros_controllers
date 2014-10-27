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
 * Author: Enrique Fernández
 */

#include <algorithm>

#include <mecanum_drive_controller/speed_limiter.h>

template<typename T>
T clamp(T x, T min, T max)
{
  return std::min(std::max(min, x), max);
}

namespace mecanum_drive_controller
{

  SpeedLimiter::SpeedLimiter(
    bool has_velocity_limits,
    bool has_acceleration_limits,
    double min_velocity,
    double max_velocity,
    double min_acceleration,
    double max_acceleration
  )
  : has_velocity_limits(has_velocity_limits),
    has_acceleration_limits(has_acceleration_limits),
    min_velocity(min_velocity),
    max_velocity(max_velocity),
    min_acceleration(min_acceleration),
    max_acceleration(max_acceleration)
  {
  }

  void SpeedLimiter::limit(double& v, double v0, double dt)
  {
    limit_velocity(v);
    limit_acceleration(v, v0, dt);
  }

  void SpeedLimiter::limit_velocity(double& v)
  {
    if (has_velocity_limits)
    {
      v = clamp(v, min_velocity, max_velocity);
    }
  }

  void SpeedLimiter::limit_acceleration(double& v, double v0, double dt)
  {
    if (has_acceleration_limits)
    {
      double dv = v - v0;

      const double dv_min = min_acceleration * dt;
      const double dv_max = max_acceleration * dt;

      dv = clamp(dv, dv_min, dv_max);

      v = v0 + dv;
    }
  }

} // namespace mecanum_drive_controller
