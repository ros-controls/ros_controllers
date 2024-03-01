///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021, Personal Robotics Lab
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Personal Robotics Lab nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Ethan Kroll Gordon

#pragma once


namespace forward_command_controller
{

namespace internal
{
// TODO: create a utils file?
/**
 * \return The map between \p t1 indices (implicitly encoded in return vector indices) to \t2 indices.
 * If \p t1 is <tt>"{C, B}"</tt> and \p t2 is <tt>"{A, B, C, D}"</tt>, the associated mapping vector is
 * <tt>"{2, 1}"</tt>.
 */
template <class T>
inline std::vector<unsigned int> mapping(const T& t1, const T& t2)
{
  typedef unsigned int SizeType;

  // t1 must be a subset of t2
  if (t1.size() > t2.size()) {return std::vector<SizeType>();}

  std::vector<SizeType> mapping_vector(t1.size()); // Return value
  for (typename T::const_iterator t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    typename T::const_iterator t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it) {return std::vector<SizeType>();}
    else
    {
      const SizeType t1_dist = std::distance(t1.begin(), t1_it);
      const SizeType t2_dist = std::distance(t2.begin(), t2_it);
      mapping_vector[t1_dist] = t2_dist;
    }
  }
  return mapping_vector;
}

inline std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}

} // namespace

template <class HardwareInterface>
bool ForwardJointGroupCommandController<HardwareInterface>::init(HardwareInterface* hw,
                                                                    ros::NodeHandle&   n)
{
    // Cache controller node handle
    controller_nh_ = n;

    // Controller name
    name_ = internal::getLeafNamespace(controller_nh_);

    // Action status checking update rate
    double action_monitor_rate = 20.0;
    controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
    action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
    ROS_DEBUG_STREAM_NAMED(name_, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");

    // Initialize controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    n_joints_ = joint_names_.size();

    if(n_joints_ == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }
    for(unsigned int i=0; i<n_joints_; i++)
    {
      try
      {
        joints_.push_back(hw->getHandle(joint_names_[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }
    }

    commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
    default_commands_.resize(n_joints_);

    // ROS API: Subscribed topics
    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &ForwardJointGroupCommandController::commandCB, this);

    // ROS API: Action interface
    action_server_.reset(new ActionServer(controller_nh_, "joint_group_command",
                                        boost::bind(&ForwardJointGroupCommandController::goalCB,   this, _1),
                                        boost::bind(&ForwardJointGroupCommandController::cancelCB, this, _1),
                                        false));
    action_server_->start();
    return true;
}

template <class HardwareInterface>
void ForwardJointGroupCommandController<HardwareInterface>::preemptActiveGoal()
{
    RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

    // Cancel any goal timeout
    goal_duration_timer_.stop();

    // Cancels the currently active goal
    if (current_active_goal)
    {
      // Marks the current goal as canceled
      rt_active_goal_.reset();
      current_active_goal->gh_.setCanceled();
    }
}

template <class HardwareInterface>
void ForwardJointGroupCommandController<HardwareInterface>::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  // Preconditions
  if (!this->isRunning())
  {
    ROS_ERROR_STREAM_NAMED(name_, "Can't accept new commands. Controller is not running.");
    return;
  }

  if (!msg)
  {
    ROS_WARN_STREAM_NAMED(name_, "Received null-pointer message, skipping.");
    return;
  }

  if(msg->data.size()!=n_joints_)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
    return;
  }

  commands_buffer_.writeFromNonRT(msg->data);
  preemptActiveGoal();
}

template <class HardwareInterface>
void ForwardJointGroupCommandController<HardwareInterface>::setGoal(GoalHandle gh,
  std::vector<double> command)
{
  ROS_DEBUG_STREAM_NAMED(name_,"Received new action goal");
  control_msgs::JointGroupCommandResult result;

  // Preconditions
  if (!this->isRunning())
  {
    result.error_string = "Can't accept new action goals. Controller is not running.";
    ROS_ERROR_STREAM_NAMED(name_, result.error_string);
    result.error_code = control_msgs::JointGroupCommandResult::INVALID_GOAL;
    gh.setRejected(result);
    return;
  }


  if (gh.getGoal()->joint_names.size() != command.size()) {
    result.error_string = "Size of command must match size of joint_names.";
    ROS_ERROR_STREAM_NAMED(name_, result.error_string);
    result.error_code = control_msgs::JointGroupCommandResult::INVALID_GOAL;
    gh.setRejected(result);
    return;
  }

  // Goal should specify valid controller joints (they can be ordered differently). Reject if this is not the case
  using internal::mapping;
  std::vector<unsigned int> mapping_vector = mapping(gh.getGoal()->joint_names, joint_names_);

  if (mapping_vector.size() != gh.getGoal()->joint_names.size())
  {
    result.error_string = "Joints on incoming goal don't match the controller joints.";
    ROS_ERROR_STREAM_NAMED(name_, result.error_string);
    result.error_code = control_msgs::JointGroupCommandResult::INVALID_JOINTS;
    gh.setRejected(result);
    return;
  }

  // update new command
  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
  std::vector< double > new_commands = default_commands_;
  for(int i = 0; i < mapping_vector.size(); i++) {
    new_commands[mapping_vector[i]] = command[i];
  }
  rt_goal->preallocated_feedback_->joint_names = joint_names_;
  commands_buffer_.writeFromNonRT(new_commands);

  // Accept new goal
  preemptActiveGoal();
  gh.setAccepted();
  rt_active_goal_ = rt_goal;

  // Setup goal status checking timer
  goal_handle_timer_ = controller_nh_.createTimer(action_monitor_period_,
                                                    &RealtimeGoalHandle::runNonRealtime,
                                                    rt_goal);
  goal_handle_timer_.start();

  // Setup goal timeout
  if (gh.getGoal()->command.time_from_start > ros::Duration()) {
    goal_duration_timer_ = controller_nh_.createTimer(gh.getGoal()->command.time_from_start,
                                                    &ForwardJointGroupCommandController::timeoutCB,
                                                    this,
                                                    true);
    goal_duration_timer_.start();
  }
}

template <class HardwareInterface>
void ForwardJointGroupCommandController<HardwareInterface>::timeoutCB(const ros::TimerEvent& event)
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Check that timeout refers to currently active goal (if any)
  if (current_active_goal) {
    ROS_DEBUG_NAMED(name_, "Active action goal reached requested timeout.");

    // Give sub-classes option to update default_commands_
    updateDefaultCommand();
    commands_buffer_.writeFromNonRT(default_commands_);

    // Marks the current goal as succeeded
    rt_active_goal_.reset();
    current_active_goal->gh_.setSucceeded();
  }
}

template <class HardwareInterface>
void ForwardJointGroupCommandController<HardwareInterface>::cancelCB(GoalHandle gh)
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Check that cancel request refers to currently active goal
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    ROS_DEBUG_NAMED(name_, "Canceling active action goal because cancel callback recieved from actionlib.");

    // Give sub-classes option to update default_commands_
    updateDefaultCommand();
    commands_buffer_.writeFromNonRT(default_commands_);

    preemptActiveGoal();
  }
}

template <class HardwareInterface>
void ForwardJointGroupCommandController<HardwareInterface>::setActionFeedback(const ros::Time& time)
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
  if (!current_active_goal)
  {
    return;
  }

  current_active_goal->preallocated_feedback_->header.stamp = time;
  current_active_goal->preallocated_feedback_->desired = current_active_goal->gh_.getGoal()->command;
  current_active_goal->preallocated_feedback_->actual.positions.clear();
  current_active_goal->preallocated_feedback_->actual.velocities.clear();
  current_active_goal->preallocated_feedback_->actual.effort.clear();
  for (auto j : joints_)
  {
    current_active_goal->preallocated_feedback_->actual.positions.push_back(j.getPosition());
    current_active_goal->preallocated_feedback_->actual.velocities.push_back(j.getVelocity());
    current_active_goal->preallocated_feedback_->actual.effort.push_back(j.getEffort());
  }

  current_active_goal->setFeedback( current_active_goal->preallocated_feedback_ );
}

} // namespace
