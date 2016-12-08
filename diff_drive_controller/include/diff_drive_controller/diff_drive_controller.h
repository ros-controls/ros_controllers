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

#ifndef DIFF_DRIVE_CONTROLLER_H
#define DIFF_DRIVE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <diff_drive_controller/DiffDriveControllerState.h>
#include <tf/tfMessage.h>

#include <dynamic_reconfigure/server.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <diff_drive_controller/odometry.h>
#include <diff_drive_controller/speed_limiter.h>
#include <diff_drive_controller/DiffDriveControllerConfig.h>

#include <boost/timer/timer.hpp>

#include <vector>
#include <string>

namespace diff_drive_controller
{

  /**
   * This class makes some assumptions on the model of the robot:
   *  - the rotation axes of wheels are collinear
   *  - the wheels are identical in radius
   * Additional assumptions to not duplicate information readily available in
   * the URDF:
   *  - the wheels have the same parent frame
   *  - a wheel collision geometry is a cylinder in the urdf
   *  - a wheel joint frame center's vertical projection on the floor must lie
   *  within the contact patch
   */
  class DiffDriveController
      : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    DiffDriveController();

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
     * \brief Updates controller, i.e. computes the odometry and sets the new
     * velocity commands
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
    void stopping(const ros::Time& /*time*/);

  private:
    std::string name_;

    /// Odometry related:
    ros::Duration publish_period_;
    ros::Time last_odom_publish_time_;
    ros::Time last_odom_tf_publish_time_;
    bool open_loop_;

    bool pose_from_joint_position_;
    bool twist_from_joint_position_;

    bool use_position_;
    bool use_velocity_;

    /// Hardware handles:
    std::vector<hardware_interface::JointHandle> left_wheel_joints_;
    std::vector<hardware_interface::JointHandle> right_wheel_joints_;

    /// Velocity command related:
    struct Commands
    {
      double lin;
      double ang;
      ros::Time stamp;

      Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
    };
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
    ros::Subscriber sub_command_;

    /// Odometry related:
    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
    Odometry odometry_;

    boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> > cmd_vel_limited_pub_;

    boost::shared_ptr<realtime_tools::RealtimePublisher<DiffDriveControllerState> > state_pub_;

    std::vector<double> left_positions_;
    std::vector<double> right_positions_;

    std::vector<double> left_velocities_;
    std::vector<double> right_velocities_;

    std::vector<double> left_positions_estimated_;
    std::vector<double> right_positions_estimated_;

    std::vector<double> left_velocities_estimated_;
    std::vector<double> right_velocities_estimated_;

    std::vector<double> left_positions_previous_;
    std::vector<double> right_positions_previous_;

    std::vector<double> left_velocities_previous_;
    std::vector<double> right_velocities_previous_;

    std::vector<double> left_velocities_estimated_previous_;
    std::vector<double> right_velocities_estimated_previous_;

    double left_velocity_average_previous_;
    double right_velocity_average_previous_;

    double left_velocity_estimated_average_previous_;
    double right_velocity_estimated_average_previous_;

    double left_velocity_desired_previous_;
    double right_velocity_desired_previous_;

    double left_velocity_limited_previous_;
    double right_velocity_limited_previous_;

    /// Dynamic reconfigure server related:
    typedef dynamic_reconfigure::Server<DiffDriveControllerConfig> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> cfg_server_;

    static const DiffDriveControllerConfig config_default_;

    /// Timing related:
    boost::timer::cpu_timer cpu_timer_;

    struct DynamicParams
    {
      bool pose_from_joint_position;
      bool twist_from_joint_position;

      double wheel_separation_multiplier;
      double left_wheel_radius_multiplier;
      double right_wheel_radius_multiplier;

      double k_l;
      double k_r;

      double wheel_resolution;

      bool publish_state;
      bool publish_cmd_vel_limited;

      double control_frequency_desired;

      DynamicParams()
        : pose_from_joint_position(true)
        , twist_from_joint_position(false)
        , wheel_separation_multiplier(1.0)
        , left_wheel_radius_multiplier(1.0)
        , right_wheel_radius_multiplier(1.0)
        , k_l(0.01)
        , k_r(0.01)
        , wheel_resolution(0.0)
        , publish_state(false)
        , publish_cmd_vel_limited(false)
        , control_frequency_desired(0.0)
      {}
    };
    realtime_tools::RealtimeBuffer<DynamicParams> dynamic_params_;
    DynamicParams dynamic_params_struct_;

    /// Wheel separation, wrt the midpoint of the wheel width:
    double wheel_separation_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_;

    /// Wheel separation and radius calibration multipliers:
    double wheel_separation_multiplier_;
    double left_wheel_radius_multiplier_;
    double right_wheel_radius_multiplier_;

    /// Measurement Covariance Model multipliers:
    double k_l_;
    double k_r_;

    double wheel_resolution_;  // [rad]

    /// Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

    /// Frame to use for the robot base:
    std::string base_frame_id_;

    /// Whether to publish odometry to tf or not:
    bool enable_odom_tf_;

    /// Number of wheel joints:
    size_t wheel_joints_size_;

    /// Speed limiters:
    Commands last1_cmd_;
    Commands last0_cmd_;
    SpeedLimiter limiter_lin_;
    SpeedLimiter limiter_ang_;

    /// Publish limited velocity command:
    /// Note that the realtime_tools::RealtimePublisher doesn't provide any
    /// method to obtain the number of subscribers because getNumSubscribers()
    /// isn't RT-safe, so we cannot implement a lazy publisher with:
    /// cmd_vel_limited_pub_->getNumSubscribers() > 0
    bool publish_cmd_vel_limited_;

    /// Publish joint trajectory controller state:
    /// Note that the realtime_tools::RealtimePublisher doesn't provide any
    /// method to obtain the number of subscribers because getNumSubscribers()
    /// isn't RT-safe, so we cannot implement a lazy publisher with:
    /// state_pub_->getNumSubscribers() > 0
    bool publish_state_;

    /// Desired control frequency [Hz] (and corresponding period [s]):
    /// This can be used when the actual/real control frequency/period has too
    /// much jitter.
    /// If !(control_frequency_desired_ > 0.0), the actual/real
    /// frequency/period provided on the update hook (method) is used, which is
    /// the default and preferred behaviour in general.
    double control_frequency_desired_;
    double control_period_desired_;

  private:
    /**
     * \brief Brakes the wheels, i.e. sets the velocity to 0
     */
    void brake();

    /**
     * \brief Velocity command callback
     * \param command Velocity command message (twist)
     */
    void cmdVelCallback(const geometry_msgs::Twist& command);

    /**
     * \brief Reconfigure callback
     * \param [in, out] config Input new desired configuration and
     *                         output applied configuration
     * \param [in]      level  Reconfigure level
     */
    void reconfigureCallback(DiffDriveControllerConfig& config, uint32_t level);

    /**
     * \brief Get the wheel names from a wheel param
     * \param [in]  controller_nh Controller node handler
     * \param [in]  wheel_param   Param name
     * \param [out] wheel_names   Vector with the whel names
     * \return true if the wheel_param is available and the wheel_names are
     *        retrieved successfully from the param server; false otherwise
     */
    bool getWheelNames(ros::NodeHandle& controller_nh,
                       const std::string& wheel_param,
                       std::vector<std::string>& wheel_names);

    /**
     * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and
     * separation
     * \param root_nh Root node handle
     * \param left_wheel_name Name of the left wheel joint
     * \param right_wheel_name Name of the right wheel joint
     */
    bool setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
                               const std::string& left_wheel_name,
                               const std::string& right_wheel_name,
                               bool lookup_wheel_separation,
                               bool lookup_wheel_radius);

    /**
     * \brief Sets the odometry publishing fields
     * \param root_nh Root node handle
     * \param controller_nh Node handle inside the controller namespace
     */
    void setOdomPubFields(
        ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  };

  PLUGINLIB_EXPORT_CLASS(diff_drive_controller::DiffDriveController,
      controller_interface::ControllerBase);
}  // namespace diff_drive_controller

#endif // DIFF_DRIVE_CONTROLLER_H
