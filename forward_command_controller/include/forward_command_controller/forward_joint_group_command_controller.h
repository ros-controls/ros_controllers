/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
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

#pragma once


#include <vector>
#include <string>

#include <ros/node_handle.h>
#include <realtime_tools/realtime_buffer.h>

// actionlib
#include <actionlib/server/action_server.h>

// ROS messages
#include <control_msgs/JointGroupCommandAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64MultiArray.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>


namespace forward_command_controller
{

/**
 * \brief Forward command controller for a set of joints.
 *
 * This class forwards the command signal down to a set of joints.
 * Command signal and joint hardware interface are of the same type, e.g. effort commands for an effort-controlled
 * joint.
 *
 * \tparam T Type implementing the JointCommandInterface.
 *
 * \section ROS interface
 *
 * \param type hardware interface type.
 * \param joints Names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64MultiArray) : The joint commands to apply.
 */
template <class HardwareInterface>
class ForwardJointGroupCommandController: public controller_interface::Controller<HardwareInterface>
{
public:
  ForwardJointGroupCommandController() {}
  ~ForwardJointGroupCommandController() {sub_command_.shutdown();}

  bool init(HardwareInterface* hw, ros::NodeHandle &n);

  void starting(const ros::Time& /*time*/);
  void stopping(const ros::Time& /*time*/) {preemptActiveGoal();}
  void update(const ros::Time& time, const ros::Duration& /*period*/)
  {
    std::vector<double> & commands = *commands_buffer_.readFromRT();
    for(unsigned int i=0; i<n_joints_; i++)
    {  joints_[i].setCommand(commands[i]); }
    setActionFeedback(time);
  }

protected:
  typedef actionlib::ActionServer<control_msgs::JointGroupCommandAction>                      ActionServer;
  typedef std::shared_ptr<ActionServer>                                                       ActionServerPtr;
  typedef ActionServer::GoalHandle                                                            GoalHandle;
  typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::JointGroupCommandAction>     RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;

  realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
  std::vector<double> default_commands_; // Defaults to 0 on init, can override in starting()
  unsigned int n_joints_;

  std::vector< std::string >                     joint_names_;         ///< Controlled joint names.
  std::vector< hardware_interface::JointHandle > joints_;              ///< Handle to controlled joint.
  std::string                                    name_;               ///< Controller name.
  RealtimeGoalHandlePtr                          rt_active_goal_;     ///< Currently active action goal, if any.

  // ROS API
  ros::NodeHandle    controller_nh_;
  ros::Subscriber    sub_command_;
  ActionServerPtr    action_server_;
  ros::Timer         goal_handle_timer_;
  ros::Timer         goal_duration_timer_;
  ros::Duration      action_monitor_period_;

  // Callback to call setGoal. Override with specific command
  void goalCB(GoalHandle gh);
  void setGoal(GoalHandle gh, std::vector<double> command);

  // General callbacks
  void cancelCB(GoalHandle gh);
  void timeoutCB(const ros::TimerEvent& event);
  void preemptActiveGoal();
  void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);

  // Defaults to no change in default command
  void updateDefaultCommand();

private:
  /**
   * @brief Updates the pre-allocated feedback of the current active goal (if any)
   * based on the current state values.
   *
   * @note This function is NOT thread safe but intended to be used in the
   * update-function.
   */
  void setActionFeedback(const ros::Time& time);
};

} // namespace

#include <forward_command_controller/forward_joint_group_command_controller_impl.h>
