/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Irstea
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

#pragma once


#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>

#include <nav_msgs/Odometry.h>
#include <four_wheel_steering_msgs/FourWheelSteeringStamped.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <four_wheel_steering_controller/odometry.h>
#include <four_wheel_steering_controller/speed_limiter.h>

namespace four_wheel_steering_controller{

  /**
   * This class makes some assumptions on the model of the robot:
   *  - the rotation axes of wheels are collinear
   *  - the wheels are identical in radius
   * Additional assumptions to not duplicate information readily available in the URDF:
   *  - the wheels have the same parent frame
   *  - a wheel collision geometry is a cylinder in the urdf
   *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
   */
  class FourWheelSteeringController
      : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
                                                              hardware_interface::PositionJointInterface>
  {
  public:
    FourWheelSteeringController();

    /**
     * \brief Initialize controller
     * \param robot_hw      Velocity and position joint interface for the wheels
     * \param root_nh       Node handle at root namespace
     * \param controller_nh Node handle inside the controller namespace
     */
    bool init(hardware_interface::RobotHW* robot_hw,
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
    void stopping(const ros::Time& /*time*/);

  private:
    std::string name_;

    /// Odometry related:
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;
    bool open_loop_;

    /// Hardware handles:
    std::vector<hardware_interface::JointHandle> front_wheel_joints_;
    std::vector<hardware_interface::JointHandle> rear_wheel_joints_;
    std::vector<hardware_interface::JointHandle> front_steering_joints_;
    std::vector<hardware_interface::JointHandle> rear_steering_joints_;

    /// Velocity command related:
    struct Command
    {
      ros::Time stamp;

      Command() : stamp(0.0) {}
    };
    struct CommandTwist : Command
    {
      double lin_x;
      double lin_y;
      double ang;

      CommandTwist() : lin_x(0.0), lin_y(0.0), ang(0.0) {}
    };
    struct Command4ws : Command
    {
      double lin;
      double front_steering;
      double rear_steering;

      Command4ws() : lin(0.0), front_steering(0.0), rear_steering(0.0) {}
    };
    realtime_tools::RealtimeBuffer<CommandTwist> command_twist_;
    CommandTwist command_struct_twist_;
    ros::Subscriber sub_command_;

    /// FourWheelSteering command related:
    realtime_tools::RealtimeBuffer<Command4ws> command_four_wheel_steering_;
    Command4ws command_struct_four_wheel_steering_;
    ros::Subscriber sub_command_four_wheel_steering_;

    /// Odometry related:
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<four_wheel_steering_msgs::FourWheelSteeringStamped> > odom_4ws_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
    Odometry odometry_;

    /// Wheel separation (or track), distance between left and right wheels (from the midpoint of the wheel width):
    double track_;
    /// Distance between a wheel joint (from the midpoint of the wheel width) and the associated steering joint:
    /// We consider that the distance is the same for every wheel
    double wheel_steering_y_offset_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_;

    /// Wheel base (distance between front and rear wheel):
    double wheel_base_;

    /// Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

    /// Frame to use for the robot base:
    std::string base_frame_id_;

    /// Whether to publish odometry to tf or not:
    bool enable_odom_tf_;

    /// Whether the control is make with four_wheel_steering msg or twist msg:
    bool enable_twist_cmd_;

    /// Speed limiters:
    CommandTwist last1_cmd_;
    CommandTwist last0_cmd_;
    SpeedLimiter limiter_lin_;
    SpeedLimiter limiter_ang_;

  private:

    /**
     * \brief Update and publish odometry
     * \param time   Current time
     */
    void updateOdometry(const ros::Time &time);
    /**
     * \brief Compute and publish command
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void updateCommand(const ros::Time& time, const ros::Duration& period);

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
     * \brief Velocity and steering command callback
     * \param command Velocity and steering command message (4ws)
     */
    void cmdFourWheelSteeringCallback(const four_wheel_steering_msgs::FourWheelSteering &command);

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
     * \brief Sets the odometry publishing fields
     * \param root_nh Root node handle
     * \param controller_nh Node handle inside the controller namespace
     */
    void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  };

  PLUGINLIB_EXPORT_CLASS(four_wheel_steering_controller::FourWheelSteeringController, controller_interface::ControllerBase);
} // namespace four_wheel_steering_controller
