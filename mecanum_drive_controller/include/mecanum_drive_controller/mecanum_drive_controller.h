#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <mecanum_drive_controller/odometry.h>

namespace mecanum_drive_controller
{

// Check file README.md for restrictions and notes.
class MecanumDriveController
    : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  MecanumDriveController();

  /**
   * \brief Initialize controller
   * \param hw            Velocity joint interface for the wheels
   * \param root_nh       Node handle at root namespace
   * \param controller_nh Node handle inside the controller namespace
   */
  bool init(hardware_interface::VelocityJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh);

  /**
   * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
   * \param time   Current time
   * \param period Time since the last called to update
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Starts controller
   * \param time Current time
   */
  void starting(const ros::Time& time);

  /**
   * \brief Stops controller
   * \param time Current time
   */
  void stopping(const ros::Time& time);

private:
  std::string name_;

  /// Hardware handles:
  hardware_interface::JointHandle wheel0_jointHandle_;
  hardware_interface::JointHandle wheel1_jointHandle_;
  hardware_interface::JointHandle wheel2_jointHandle_;
  hardware_interface::JointHandle wheel3_jointHandle_;

  /// Velocity command related:
  struct Command
  {
    double vx_Ob_b_b0_b;
    double vy_Ob_b_b0_b;
    double wz_b_b0_b;
    ros::Time stamp;

    Command() : vx_Ob_b_b0_b(0.0), vy_Ob_b_b0_b(0.0), wz_b_b0_b(0.0), stamp(0.0) {}
  };
  realtime_tools::RealtimeBuffer<Command> command_rt_buffer_;
  Command command_;
  ros::Subscriber command_sub_;

  /// Odometry related:
  Odometry odometry_;

  boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_pub_;

  geometry_msgs::TransformStamped odom_frame_;

  ros::Duration publish_period_;
  ros::Time last_state_publish_time_;

  /// Wheel radius (assuming it's the same for the left and right wheels):
  bool use_realigned_roller_joints_;
  double wheels_k_; // wheels geometric param used in mecanum wheels' ik
  double wheels_radius_;

  /// Timeout to consider cmd_vel commands old:
  double cmd_vel_timeout_;

  /// Frame to use for the robot base:
  std::string   base_frame_id_;
  double        base_frame_offset_[PLANAR_POINT_DIM];

  /// Whether to publish odometry to tf or not:
  bool enable_odom_tf_;

  /// Number of wheel joints:
  size_t wheel_joints_size_;

private:
  /**
   * \brief Brakes the wheels, i.e. sets the velocity to 0
   */
  void brake();

  /**
   * \brief Velocity command callback
   * \param command Velocity command message (twist)
   */
  void commandCb(const geometry_msgs::Twist& command);

  /**
   * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and separation
   * \param root_nh Root node handle
   * \param wheel0_name Name of wheel0 joint
   * \param wheel1_name Name of wheel1 joint
   * \param wheel2_name Name of wheel2 joint
   * \param wheel3_name Name of wheel3 joint
   */
  bool setWheelParamsFromUrdf(ros::NodeHandle& root_nh,
                             const std::string& wheel0_name,
                             const std::string& wheel1_name,
                             const std::string& wheel2_name,
                             const std::string& wheel3_name);

  /**
   * \brief Get the radius of a given wheel
   * \param       model         urdf model used
   * \param       wheel_link    link of the wheel from which to get the radius
   * \param[out]  wheels_radius radius of the wheel read from the urdf
   */
  bool getWheelRadius(const boost::shared_ptr<urdf::ModelInterface> model, const boost::shared_ptr<const urdf::Link>& wheel_link, double& wheel_radius);

  /**
   * \brief Sets the odometry publishing fields
   * \param root_nh Root node handle
   * \param controller_nh Node handle inside the controller namespace
   */
  void setupRtPublishersMsg(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

};

PLUGINLIB_EXPORT_CLASS(mecanum_drive_controller::MecanumDriveController, controller_interface::ControllerBase)

} // namespace mecanum_drive_controller
