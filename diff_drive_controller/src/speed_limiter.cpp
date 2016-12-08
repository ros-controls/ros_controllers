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

#include <diff_drive_controller/speed_limiter.h>

template<typename T>
T clamp(T x, T min, T max)
{
  return std::min(std::max(min, x), max);
}

namespace diff_drive_controller
{

  SpeedLimiter::SpeedLimiter(
    bool has_velocity_limits,
    bool has_acceleration_limits,
    bool has_jerk_limits,
    double min_velocity,
    double max_velocity,
    double min_acceleration,
    double max_acceleration,
    double min_jerk,
    double max_jerk,
    double min_deceleration,
    double max_deceleration
  )
  : has_velocity_limits(has_velocity_limits)
  , has_acceleration_limits(has_acceleration_limits)
  , has_jerk_limits(has_jerk_limits)
  , min_velocity(min_velocity)
  , max_velocity(max_velocity)
  , min_acceleration(min_acceleration)
  , max_acceleration(max_acceleration)
  , min_jerk(min_jerk)
  , max_jerk(max_jerk)
  , min_deceleration(min_deceleration)
  , max_deceleration(max_deceleration)
  {
  }

  double SpeedLimiter::limit(double& v, double v0, double v1, double dt)
  {
    const double tmp = v;

    limit_jerk(v, v0, v1, dt);
    limit_acceleration(v, v0, dt);
    limit_velocity(v);

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  double SpeedLimiter::limit_velocity(double& v)
  {
    const double tmp = v;

    if (has_velocity_limits)
    {
      v = clamp(v, min_velocity, max_velocity);
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  double SpeedLimiter::limit_acceleration(double& v, double v0, double dt)
  {
    const double tmp = v;

    if (has_acceleration_limits)
    {
      double dv_min, dv_max;

      if (v0 >= 0.0 && v >= 0.0)
      {
        // Moving in positive direction
        dv_min = min_deceleration * dt;
        dv_max = max_acceleration * dt;
      }
      else if (v0 <= 0.0 && v <= 0.0)
      {
        // Moving in the negative direction
        dv_min = min_acceleration * dt;
        dv_max = max_deceleration * dt;
      }
      else
      {
        // Transitioning from positive to negative velocity
        if (v0 > 0)
        {
          dv_max = v0;
          dv_min = min_deceleration * dt;
          if (v0 + dv_min < 0)
          {
            dv_min = min_acceleration * (dt - v0 / min_deceleration);
          }
        }
        // Transitioning from negative to positive velocity
        else // if (v0 < 0)
        {
          dv_min = v0;
          dv_max = max_deceleration * dt;
          if (v0 + dv_max > 0)
          {
            dv_max = max_acceleration * (dt - v0 / max_deceleration);
          }
        }
      }

      const double dv = clamp(v - v0, dv_min, dv_max);

      v = v0 + dv;
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

  double SpeedLimiter::limit_jerk(double& v, double v0, double v1, double dt)
  {
    const double tmp = v;

    if (has_jerk_limits)
    {
      const double dv  = v  - v0;
      const double dv0 = v0 - v1;

      const double dt2 = 2. * dt * dt;

      const double da_min = min_jerk * dt2;
      const double da_max = max_jerk * dt2;

      const double da = clamp(dv - dv0, da_min, da_max);

      v = v0 + dv0 + da;
    }

    return tmp != 0.0 ? v / tmp : 1.0;
  }

}  // namespace diff_drive_controller
