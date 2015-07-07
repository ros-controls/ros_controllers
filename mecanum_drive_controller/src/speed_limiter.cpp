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
