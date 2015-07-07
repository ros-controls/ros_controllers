#include <mecanum_drive_controller/odometry.h>

#include <tf/transform_datatypes.h>

#include <boost/bind.hpp>

namespace mecanum_drive_controller
{

namespace bacc = boost::accumulators;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0)
, px_b_b0(0.0)
, py_b_b0(0.0)
, rz_b_b0(0.0)
, vx_Oc_c_c0_c_(0.0)
, vy_Oc_c_c0_c_(0.0)
, wz_c_c0_c_(0.0)
, vx_Ob_b_b0_b_(0.0)
, vy_Ob_b_b0_b_(0.0)
, wz_b_b0_b_(0.0)
, wheels_k_(0.0)
, wheels_radius_(0.0)
, velocity_rolling_window_size_(velocity_rolling_window_size)
, linearX_acc_(RollingWindow::window_size = velocity_rolling_window_size)
, linearY_acc_(RollingWindow::window_size = velocity_rolling_window_size)
, angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Odometry::init(const ros::Time& time, double base_frame_offset[PLANAR_POINT_DIM])
{
  // Reset accumulators:
  linearX_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  linearY_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);

  // Reset timestamp:
  timestamp_ = time;

  // Base frame offset (wrt to center frame).
  base_frame_offset_[0] = base_frame_offset[0];
  base_frame_offset_[1] = base_frame_offset[1];
  base_frame_offset_[2] = base_frame_offset[2];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Odometry::update(double wheel0_vel, double wheel1_vel, double wheel2_vel, double wheel3_vel, const ros::Time &time)
{
  /// We cannot estimate the speed with very small time intervals:
  const double dt = (time - timestamp_).toSec();
  if (dt < 0.0001)
    return false; // Interval too small to integrate with

  timestamp_ = time;

  /// Compute FK (i.e. compute mobile robot's body twist out of its wheels velocities):
  /// NOTE: we use the IK of the mecanum wheels which we invert using a pseudo-inverse.
  /// NOTE: the mecanum IK gives the body speed at the center frame, we then offset this velocity at the base frame.
  /// NOTE: in the diff drive the velocity is filtered out, but we prefer to return it raw and let the user perform
  ///       post-processing at will. We prefer this way of doing as filtering introduces delay (which makes it
  ///       difficult to interpret and compare behavior curves).
  vx_Oc_c_c0_c_ = 0.25 * wheels_radius_              * ( wheel0_vel + wheel1_vel + wheel2_vel + wheel3_vel);
  vy_Oc_c_c0_c_ = 0.25 * wheels_radius_              * (-wheel0_vel + wheel1_vel - wheel2_vel + wheel3_vel);
  wz_c_c0_c_    = 0.25 * wheels_radius_  / wheels_k_ * (-wheel0_vel - wheel1_vel + wheel2_vel + wheel3_vel);

  tf::Matrix3x3 R_c_b         = tf::Matrix3x3(tf::createQuaternionFromYaw(-base_frame_offset_[2]));
  tf::Vector3   v_Oc_c_c0_b   = R_c_b * tf::Vector3(vx_Oc_c_c0_c_, vy_Oc_c_c0_c_, 0.0);
  tf::Vector3   Oc_b          = R_c_b * tf::Vector3(-base_frame_offset_[0], -base_frame_offset_[1], 0.0);

  vx_Ob_b_b0_b_ = v_Oc_c_c0_b.x() + Oc_b.y() * wz_c_c0_c_;
  vy_Ob_b_b0_b_ = v_Oc_c_c0_b.y() - Oc_b.x() * wz_c_c0_c_;
  wz_b_b0_b_    = wz_c_c0_c_;

  /// Integration.
  /// NOTE: the position is expressed in the odometry frame (frame b0), unlike the twist which is expressed in the body
  ///       frame (frame b).
  rz_b_b0 += wz_b_b0_b_ * dt;

  tf::Matrix3x3 R_b_b0 = tf::Matrix3x3(tf::createQuaternionFromYaw(rz_b_b0));
  tf::Vector3 vx_Ob_b_b0_b0 = R_b_b0 * tf::Vector3(vx_Ob_b_b0_b_, vy_Ob_b_b0_b_, 0.0);

  px_b_b0 += vx_Ob_b_b0_b0.x() * dt;
  py_b_b0 += vx_Ob_b_b0_b0.y() * dt;

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Odometry::setWheelsParams(double wheels_k, double wheels_radius)
{
  wheels_k_ = wheels_k;

  wheels_radius_ = wheels_radius;
}

} // namespace mecanum_drive_controller
