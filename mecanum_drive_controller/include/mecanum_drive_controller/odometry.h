#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

#define PLANAR_POINT_DIM 3

namespace mecanum_drive_controller
{
namespace bacc = boost::accumulators;

/// \brief The Odometry class handles odometry readings
/// (2D pose and velocity with related timestamp)
class Odometry
{
public:
  /// Integration function, used to integrate the odometry:
  typedef boost::function<void(double, double, double)> IntegrationFunction;

  /// \brief Constructor
  /// Timestamp will get the current time value
  /// Value will be set to zero
  /// \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
  Odometry(size_t velocity_rolling_window_size = 10);

  /// \brief Initialize the odometry
  /// \param time Current time
  void init(const ros::Time& time, double base_frame_offset[PLANAR_POINT_DIM]);

  /// \brief Updates the odometry class with latest wheels position
  /// \param wheel0_vel  Wheel velocity [rad/s]
  /// \param wheel1_vel  Wheel velocity [rad/s]
  /// \param wheel2_vel  Wheel velocity [rad/s]
  /// \param wheel3_vel  Wheel velocity [rad/s]
  /// \param time      Current time
  /// \return true if the odometry is actually updated
  bool update(double wheel0_vel, double wheel1_vel, double wheel2_vel, double wheel3_vel, const ros::Time& time);

  /// \return position (x component) [m]
  double getX() const
  {
    return px_b_b0_;
  }
  /// \return position (y component) [m]
  double getY() const
  {
    return py_b_b0_;
  }
  /// \return orientation (z component) [m]
  double getRz() const
  {
    return rz_b_b0_;
  }
  /// \return body velocity of the base frame (linear x component) [m/s]
  double getVx() const
  {
    return vx_Ob_b_b0_b_;
  }
  /// \return body velocity of the base frame (linear y component) [m/s]
  double getVy() const
  {
    return vy_Ob_b_b0_b_;
  }
  /// \return body velocity of the base frame (angular z component) [m/s]
  double getWz() const
  {
    return wz_b_b0_b_;
  }

  /// \brief Sets the wheels parameters: mecanum geometric param and radius
  /// \param wheels_k       Wheels geometric param (used in mecanum wheels' ik) [m]
  /// \param wheels_radius  Wheels radius [m]
  void setWheelsParams(double wheels_k, double wheels_radius);

private:
  /// Rolling mean accumulator and window:
  typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
  typedef bacc::tag::rolling_window RollingWindow;

  /// Current timestamp:
  ros::Time timestamp_;

  /// Reference frame (wrt to center frame).
  double base_frame_offset_[PLANAR_POINT_DIM];

  /// Current pose:
  double px_b_b0_;  // [m]
  double py_b_b0_;  // [m]
  double rz_b_b0_;  // [rad]

  /// Current velocity.
  /// \note The indices meaning is the following:
  ///    b : base frame
  ///    c : center frame
  ///    Ob: origin of the base frame
  ///    Oc: origin of the center frame
  ///    b0: initial position if the base frame
  ///    c0: initial position of the center frame
  double vx_Ob_b_b0_b_;  // [m/s]
  double vy_Ob_b_b0_b_;  // [m/s]
  double wz_b_b0_b_;     // [rad/s]

  /// Wheels kinematic parameters [m]:
  double wheels_k_;
  double wheels_radius_;
};

}  // namespace mecanum_drive_controller

#endif /* ODOMETRY_H_ */
