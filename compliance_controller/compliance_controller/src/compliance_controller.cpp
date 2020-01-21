///////////////////////////////////////////////////////////////////////////////
//      Title     : compliance_controller.cpp
//      Project   : compliance_controller
//      Created   : 12/4/2018
//      Author    : Andy Zelenak
//
// BSD 3-Clause License
//
// Copyright (c) 2018, Los Alamos National Security, LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#include <compliance_controller/compliance_controller.h>

namespace compliance_controller
{
template <class SegmentImpl, class HardwareInterface>
void ComplianceController<SegmentImpl, HardwareInterface>::update(const ros::Time& time, const ros::Duration& period)
{
  // When updating, the real-time velocity controller takes priority over the trajectory controller.
  // The real-time velocity controller takes over as soon as a trajectory completes, and a new real-time velocity
  // command can interrupt a trajectory at any time.
  // The member variable allow_trajectory_execution_ determines which controller gets updated.

  bool stale_compliance_command =
      ((ros::Time::now() - last_compliance_adjustment_stamp_) > compliance_command_timeout_);

  // If trajectory execution is not active
  if (!TrajOrJogController::allow_trajectory_execution_)
  {
    std::vector<double>& command = *TrajOrJogController::commands_buffer_.readFromRT();
    if (command.size() > 0)
    {
      for (unsigned int i = 0; i < JointTrajectoryController::joints_.size(); ++i)
      {
        // Add the compliance adjustment if:
        // it has been received recently,
        // robot is not near a joint limit,
        // compliance is enabled
        if (!stale_compliance_command && !near_joint_limit_ && compliance_enabled_)
        {
          JointTrajectoryController::joints_[i].setCommand(command[i] + compliance_velocity_adjustment_.data[i]);
        }
        else
        {
          JointTrajectoryController::joints_[i].setCommand(command[i]);
        }
      }
    }
    else
    {
      for (unsigned int i = 0; i < JointTrajectoryController::joints_.size(); ++i)
      {
        JointTrajectoryController::joints_[i].setCommand(0);
      }
    }
  }
  // If trajectory execution is allowed
  else
  {
    // Update the base class, JointTrajectoryController
    if (!stale_compliance_command && !near_joint_limit_ && compliance_enabled_)
    {
      ComplianceController::updateJointTrajControllerWithCompliace(time, period);
    }
    // Regular update, without adding a compliance adjustment
    else
    {
      JointTrajectoryController::update(time, period);
    }

    // Back to real-time velocity control if the trajectory is complete
    if (JointTrajectoryController::rt_active_goal_ == NULL)
    {
      ComplianceController::activateVelocityStreaming();
    }
  }
}

template <class SegmentImpl, class HardwareInterface>
bool ComplianceController<SegmentImpl, HardwareInterface>::init(HardwareInterface* hw, ros::NodeHandle& root_nh,
                                                                ros::NodeHandle& controller_nh)
{
  // Initialize the parent class first
  TrajOrJogController::init(hw, root_nh, controller_nh);

  compliance_adjustment_sub_ =
      controller_nh_.subscribe(controller_nh_.getNamespace() + "compliance_controller/compliance_velocity_adjustment",
                               1, &ComplianceController::complianceAdjustmentCallback, this);
  toggle_compliance_service_ =
      controller_nh_.advertiseService(controller_nh_.getNamespace() + "compliance_controller/toggle_compliance",
                                      &ComplianceController::toggleCompliance, this);
  enable_compliance_service_ =
      controller_nh_.advertiseService(controller_nh_.getNamespace() + "compliance_controller/enable_compliance",
                                      &ComplianceController::enableCompliance, this);
  disable_compliance_service_ =
      controller_nh_.advertiseService(controller_nh_.getNamespace() + "compliance_controller/disable_compliance",
                                      &ComplianceController::disableCompliance, this);
  last_compliance_adjustment_stamp_ = ros::Time::now();
  last_trajectory_update_time_ = ros::Time(0);
  compliance_velocity_adjustment_.data.resize(TrajOrJogController::n_joints_, 0);
  integrated_traj_position_adjustment_.resize(TrajOrJogController::n_joints_, 0);

  // Read controller parameters
  XmlRpc::XmlRpcValue timeout;
  if (!controller_nh.getParam("compliance_command_timeout", timeout))
  {
    ROS_ERROR_STREAM(
        "Could not find 'compliance_command_timeout' parameter (namespace: " << controller_nh.getNamespace() << ").");
    return false;
  }
  compliance_command_timeout_ = ros::Duration(static_cast<double>(timeout));

  return true;
}

// Flag that the velocity-streaming controller should be active
template <class SegmentImpl, class HardwareInterface>
void ComplianceController<SegmentImpl, HardwareInterface>::activateVelocityStreaming()
{
  TrajOrJogController::allow_trajectory_execution_ = false;
  last_trajectory_update_time_ = ros::Time(0);
  compliance_velocity_adjustment_.data.resize(TrajOrJogController::n_joints_, 0);
  integrated_traj_position_adjustment_.resize(TrajOrJogController::n_joints_, 0);

  // Reset velocity commands to zero
  TrajOrJogController::commands_buffer_.readFromRT()->assign(TrajOrJogController::n_joints_, 0.0);
}

// Update the JointTrajectoryController like usual, but add the compliance command
template <class SegmentImpl, class HardwareInterface>
void ComplianceController<SegmentImpl, HardwareInterface>::updateJointTrajControllerWithCompliace(
    const ros::Time& time, const ros::Duration& period)
{
  // Get currently followed trajectory
  typename JointTrajectoryController::TrajectoryPtr curr_traj_ptr;
  JointTrajectoryController::curr_trajectory_box_.get(curr_traj_ptr);
  typename JointTrajectoryController::Trajectory& curr_traj = *curr_traj_ptr;

  // Update time data
  typename JointTrajectoryController::TimeData time_data;
  time_data.time = time;      // Cache current time
  time_data.period = period;  // Cache current control period
  time_data.uptime = JointTrajectoryController::time_data_.readFromRT()->uptime + period;  // Update controller uptime
  JointTrajectoryController::time_data_.writeFromNonRT(
      time_data);  // TODO: Grrr, we need a lock-free data structure here!

  // NOTE: It is very important to execute the two above code blocks in the specified sequence: first get current
  // trajectory, then update time data. Hopefully the following paragraph sheds a bit of light on the rationale.
  // The non-rt thread responsible for processing new commands enqueues trajectories that can start at the _next_
  // control cycle (eg. zero start time) or later (eg. when we explicitly request a start time in the future).
  // If we reverse the order of the two blocks above, and update the time data first; it's possible that by the time we
  // fetch the currently followed trajectory, it has been updated by the non-rt thread with something that starts in the
  // next control cycle, leaving the current cycle without a valid trajectory.

  typedef joint_trajectory_controller::JointTrajectorySegment<SegmentImpl> Segment;
  typedef typename Segment::Scalar Scalar;

  // Update current state and state error
  for (unsigned int i = 0; i < JointTrajectoryController::joints_.size(); ++i)
  {
    JointTrajectoryController::current_state_.position[i] = JointTrajectoryController::joints_[i].getPosition();
    JointTrajectoryController::current_state_.velocity[i] = JointTrajectoryController::joints_[i].getVelocity();
    // There's no acceleration data available in a joint handle

    typename JointTrajectoryController::TrajectoryPerJoint::const_iterator segment_it =
        sample(curr_traj[i], time_data.uptime.toSec(), JointTrajectoryController::desired_joint_state_);
    if (curr_traj[i].end() == segment_it)
    {
      // Non-realtime safe, but should never happen under normal operation
      ROS_ERROR_NAMED(
          JointTrajectoryController::name_,
          "Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
      return;
    }

    // Add compliance adjustments (velocity and position, integrated from velocity)
    JointTrajectoryController::desired_joint_state_.velocity[0] =
        JointTrajectoryController::desired_joint_state_.velocity[0] + compliance_velocity_adjustment_.data[i];

    // Sometimes the control loop executes so fast, ROS time does not update
    if (time_data.uptime != last_trajectory_update_time_)
    {
      last_trajectory_update_time_ = time_data.uptime;
    }

    // Integrate compliance commands
    integrated_traj_position_adjustment_[i] += compliance_velocity_adjustment_.data[i] * time_data.period.toSec();
    JointTrajectoryController::desired_joint_state_.position[0] += integrated_traj_position_adjustment_[i];

    JointTrajectoryController::desired_state_.position[i] = JointTrajectoryController::desired_joint_state_.position[0];
    JointTrajectoryController::desired_state_.velocity[i] = JointTrajectoryController::desired_joint_state_.velocity[0];
    JointTrajectoryController::desired_state_.acceleration[i] =
        JointTrajectoryController::desired_joint_state_.acceleration[0];
    ;

    JointTrajectoryController::state_joint_error_.position[0] =
        angles::shortest_angular_distance(JointTrajectoryController::current_state_.position[i],
                                          JointTrajectoryController::desired_joint_state_.position[0]);
    JointTrajectoryController::state_joint_error_.velocity[0] =
        JointTrajectoryController::desired_joint_state_.velocity[0] -
        JointTrajectoryController::current_state_.velocity[i];
    JointTrajectoryController::state_joint_error_.acceleration[0] = 0.0;

    JointTrajectoryController::state_error_.position[i] =
        angles::shortest_angular_distance(JointTrajectoryController::current_state_.position[i],
                                          JointTrajectoryController::desired_joint_state_.position[0]);
    JointTrajectoryController::state_error_.velocity[i] = JointTrajectoryController::desired_joint_state_.velocity[0] -
                                                          JointTrajectoryController::current_state_.velocity[i];
    JointTrajectoryController::state_error_.acceleration[i] = 0.0;

    // Check tolerances
    const typename JointTrajectoryController::RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
    if (rt_segment_goal && rt_segment_goal == JointTrajectoryController::rt_active_goal_)
    {
      // Check tolerances
      if (time_data.uptime.toSec() < segment_it->endTime())
      {
        // Currently executing a segment: check path tolerances
        const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar>& joint_tolerances =
            segment_it->getTolerances();

        if (!checkStateTolerancePerJoint(JointTrajectoryController::state_joint_error_,
                                         joint_tolerances.state_tolerance))
        {
          if (JointTrajectoryController::verbose_)
          {
            ROS_ERROR_STREAM_NAMED(JointTrajectoryController::name_,
                                   "Path tolerances failed for joint: " << JointTrajectoryController::joint_names_[i]);
            checkStateTolerancePerJoint(JointTrajectoryController::state_joint_error_, joint_tolerances.state_tolerance,
                                        true);
          }
          rt_segment_goal->preallocated_result_->error_code =
              control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
          rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
          JointTrajectoryController::rt_active_goal_.reset();
          JointTrajectoryController::successful_joint_traj_.reset();
        }
      }
      else if (segment_it == --curr_traj[i].end())
      {
        if (JointTrajectoryController::verbose_)
          ROS_DEBUG_STREAM_THROTTLE_NAMED(1, JointTrajectoryController::name_,
                                          "Finished executing last segment, checking goal tolerances");

        // Controller uptime
        const ros::Time uptime = JointTrajectoryController::time_data_.readFromRT()->uptime;

        // Checks that we have ended inside the goal tolerances
        const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar>& tolerances = segment_it->getTolerances();
        const bool inside_goal_tolerances =
            checkStateTolerancePerJoint(JointTrajectoryController::state_joint_error_, tolerances.goal_state_tolerance);

        if (inside_goal_tolerances)
        {
          JointTrajectoryController::successful_joint_traj_[i] = 1;
        }
        else if (uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance)
        {
          // Still have some time left to meet the goal state tolerances
        }
        else
        {
          if (JointTrajectoryController::verbose_)
          {
            ROS_ERROR_STREAM_NAMED(JointTrajectoryController::name_,
                                   "Goal tolerances failed for joint: " << JointTrajectoryController::joint_names_[i]);
            // Check the tolerances one more time to output the errors that occurs
            checkStateTolerancePerJoint(JointTrajectoryController::state_joint_error_, tolerances.goal_state_tolerance,
                                        true);
          }
          rt_segment_goal->preallocated_result_->error_code =
              control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
          rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
          JointTrajectoryController::rt_active_goal_.reset();
          JointTrajectoryController::successful_joint_traj_.reset();
        }
      }
    }
  }

  // If there is an active goal and all segments finished successfully then set goal as succeeded
  typename JointTrajectoryController::RealtimeGoalHandlePtr current_active_goal(
      JointTrajectoryController::rt_active_goal_);
  if (current_active_goal and
      JointTrajectoryController::successful_joint_traj_.count() == JointTrajectoryController::joints_.size())
  {
    current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
    current_active_goal.reset();  // do not publish feedback
    JointTrajectoryController::rt_active_goal_.reset();
    JointTrajectoryController::successful_joint_traj_.reset();
  }

  // Hardware interface adapter: Generate and send commands
  JointTrajectoryController::hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period,
                                                             JointTrajectoryController::desired_state_,
                                                             JointTrajectoryController::state_error_);

  // Set action feedback
  if (current_active_goal)
  {
    current_active_goal->preallocated_feedback_->header.stamp =
        JointTrajectoryController::time_data_.readFromRT()->time;
    current_active_goal->preallocated_feedback_->desired.positions = JointTrajectoryController::desired_state_.position;
    current_active_goal->preallocated_feedback_->desired.velocities =
        JointTrajectoryController::desired_state_.velocity;
    current_active_goal->preallocated_feedback_->desired.accelerations =
        JointTrajectoryController::desired_state_.acceleration;
    current_active_goal->preallocated_feedback_->actual.positions = JointTrajectoryController::current_state_.position;
    current_active_goal->preallocated_feedback_->actual.velocities = JointTrajectoryController::current_state_.velocity;
    current_active_goal->preallocated_feedback_->error.positions = JointTrajectoryController::state_error_.position;
    current_active_goal->preallocated_feedback_->error.velocities = JointTrajectoryController::state_error_.velocity;
    current_active_goal->setFeedback(current_active_goal->preallocated_feedback_);
  }

  // Publish state
  JointTrajectoryController::publishState(time_data.uptime);
}

// Callback for compliant joint velocity adjustments.
template <class SegmentImpl, class HardwareInterface>
void ComplianceController<SegmentImpl, HardwareInterface>::complianceAdjustmentCallback(
    const compliance_control_msgs::CompliantVelocities::ConstPtr& msg)
{
  last_compliance_adjustment_stamp_ = ros::Time::now();
  compliance_velocity_adjustment_.data = msg->compliant_velocities.data;
  near_joint_limit_ = msg->near_joint_limit;

  if (near_joint_limit_)
  {
    // Stop executing trajectories if near a joint limit
    TrajOrJogController::stopTrajectoryExecution();
  }
}

// Callback to toggle compliance on/off
template <class SegmentImpl, class HardwareInterface>
bool ComplianceController<SegmentImpl, HardwareInterface>::toggleCompliance(std_srvs::Trigger::Request& req,
                                                                            std_srvs::Trigger::Response& res)
{
  compliance_enabled_ = !compliance_enabled_;

  res.success = true;
  res.message = (compliance_enabled_) ? "enabled" : "disabled";

  compliance_velocity_adjustment_.data.resize(TrajOrJogController::n_joints_, 0);
  integrated_traj_position_adjustment_.resize(TrajOrJogController::n_joints_, 0);

  JointTrajectoryController::rt_active_goal_.reset();
  JointTrajectoryController::successful_joint_traj_.reset();

  return true;
}

// Callback to turn compliance on
template <class SegmentImpl, class HardwareInterface>
bool ComplianceController<SegmentImpl, HardwareInterface>::enableCompliance(std_srvs::Trigger::Request& req,
                                                                            std_srvs::Trigger::Response& res)
{
  compliance_enabled_ = true;

  res.success = true;
  res.message = compliance_enabled_;

  compliance_velocity_adjustment_.data.resize(TrajOrJogController::n_joints_, 0);
  integrated_traj_position_adjustment_.resize(TrajOrJogController::n_joints_, 0);

  JointTrajectoryController::rt_active_goal_.reset();
  JointTrajectoryController::successful_joint_traj_.reset();

  return true;
}

// Callback to turn compliance off
template <class SegmentImpl, class HardwareInterface>
bool ComplianceController<SegmentImpl, HardwareInterface>::disableCompliance(std_srvs::Trigger::Request& req,
                                                                            std_srvs::Trigger::Response& res)
{
  compliance_enabled_ = false;

  res.success = true;
  res.message = compliance_enabled_;

  compliance_velocity_adjustment_.data.resize(TrajOrJogController::n_joints_, 0);
  integrated_traj_position_adjustment_.resize(TrajOrJogController::n_joints_, 0);

  JointTrajectoryController::rt_active_goal_.reset();
  JointTrajectoryController::successful_joint_traj_.reset();

  return true;
}

}  // namespace compliance_controller

// Set up namespacing of controllers and create their plugins.
namespace velocity_controllers
{
/**
 * \brief Add compliance to trajectories and/or velocity commands.
 */
typedef compliance_controller::ComplianceController<trajectory_interface::QuinticSplineSegment<double>,
                                                    hardware_interface::VelocityJointInterface>
    ComplianceController;
}

PLUGINLIB_EXPORT_CLASS(velocity_controllers::ComplianceController, controller_interface::ControllerBase)
