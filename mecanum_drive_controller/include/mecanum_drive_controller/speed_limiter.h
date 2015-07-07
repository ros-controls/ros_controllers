#ifndef SPEED_LIMITER_H
#define SPEED_LIMITER_H

namespace mecanum_drive_controller
{

  class SpeedLimiter
  {
  public:

    /**
     * \brief Constructor
     * \param [in] has_velocity_limits     if true, applies velocity limits
     * \param [in] has_acceleration_limits if true, applies acceleration limits
     * \param [in] min_velocity Minimum velocity [m/s], usually <= 0
     * \param [in] max_velocity Maximum velocity [m/s], usually >= 0
     * \param [in] min_acceleration Minimum acceleration [m/s^2], usually <= 0
     * \param [in] max_acceleration Maximum acceleration [m/s^2], usually >= 0
     */
    SpeedLimiter(
      bool has_velocity_limits = false,
      bool has_acceleration_limits = false,
      double min_velocity = 0.0,
      double max_velocity = 0.0,
      double min_acceleration = 0.0,
      double max_acceleration = 0.0
    );

    /**
     * \brief Limit the velocity and acceleration
     * \param [in, out] v  Velocity [m/s]
     * \param [in]      v0 Previous velocity [m/s]
     * \param [in]      dt Time step [s]
     */
    void limit(double& v, double v0, double dt);

    /**
     * \brief Limit the velocity
     * \param [in, out] v Velocity [m/s]
     */
    void limit_velocity(double& v);

    /**
     * \brief Limit the acceleration
     * \param [in, out] v  Velocity [m/s]
     * \param [in]      v0 Previous velocity [m/s]
     * \param [in]      dt Time step [s]
     */
    void limit_acceleration(double& v, double v0, double dt);

  public:
    // Enable/Disable velocity/acceleration limits:
    bool has_velocity_limits;
    bool has_acceleration_limits;

    // Velocity limits:
    double min_velocity;
    double max_velocity;

    // Acceleration limits:
    double min_acceleration;
    double max_acceleration;
  };

} // namespace mecanum_drive_controller

#endif // SPEED_LIMITER_H
