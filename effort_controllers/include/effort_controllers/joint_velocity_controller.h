/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#ifndef EFFORT_CONTROLLERS__JOINT_VELOCITY_CONTROLLER_H
#define EFFORT_CONTROLLERS__JOINT_VELOCITY_CONTROLLER_H

/**
   @class effort_controllers::JointVelocityController
   @brief Joint Velocity Controller

   This class controls velocity using a pid loop.

   @section ROS ROS interface

   @param type Must be "effort_controllers::JointVelocityController"
   @param joint Name of the joint to control.
   @param pid Contains the gains for the PID loop around velocity.  See: control_toolbox::Pid

   Subscribes to:

   - @b command (std_msgs::Float64) : The joint velocity to achieve.

   Publishes:

   - @b state (control_msgs::JointControllerState) :
     Current state of the controller, including pid error and gains.

*/

#include <ros/node_handle.h>
#include <boost/thread/condition.hpp>
#include <boost/scoped_ptr.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_publisher.h>

namespace effort_controllers
{

class JointVelocityController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:

  JointVelocityController();
  ~JointVelocityController();

  bool init(hardware_interface::EffortJointInterface *robot, const std::string &joint_name, const control_toolbox::Pid &pid);

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */  
  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

  /*!
   * \brief Give set velocity of the joint for next update: revolute (angle) and prismatic (velocity)
   *
   * \param double pos Velocity command to issue
   */
  void setCommand(double cmd);

  /*!
   * \brief Get latest velocity command to the joint: revolute (angle) and prismatic (velocity).
   */
  void getCommand(double & cmd);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Get the PID parameters
   */
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

  /**
   * \brief Get the PID parameters
   */
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

  /**
   * \brief Print debug info to console
   */
  void printDebug();

  /**
   * \brief Get the PID parameters
   */
  void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);

  /**
   * \brief Get the name of the joint this controller uses
   */
  std::string getJointName();
  
  hardware_interface::JointHandle joint_;
  double command_;                                /**< Last commanded velocity. */

private:
  int loop_count_;
  control_toolbox::Pid pid_controller_;           /**< Internal PID controller. */

  //friend class JointVelocityControllerNode; // what is this for??

  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      control_msgs::JointControllerState> > controller_state_publisher_ ;

  ros::Subscriber sub_command_;

  /**
   * \brief Callback from /command subscriber for setpoint
   */
  void setCommandCB(const std_msgs::Float64ConstPtr& msg);
};

} // namespace

#endif
