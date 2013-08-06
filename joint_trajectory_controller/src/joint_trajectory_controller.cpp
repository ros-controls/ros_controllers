///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
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

// C++ standard
#include <cassert>
#include <iterator>
#include <string>

// Pluginlib
#include <pluginlib/class_list_macros.h>

// URDF
#include <urdf/model.h>

// Project
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <trajectory_interface/trajectory_interface.h>
#include <joint_trajectory_controller/init_joint_trajectory.h>

namespace
{

// TODO: Move these convenience functions elsewhere?
std::vector<std::string> getStrings(const ros::NodeHandle& nh, const std::string& param_name)
{
  using namespace XmlRpc;
  XmlRpcValue xml_array;
  if (!nh.getParam(param_name, xml_array))
  {
    ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter in controller (namespace: " <<
                     nh.getNamespace() << ").");
    return std::vector<std::string>();
  }
  if (xml_array.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("The '" << param_name << "' parameter is not an array (namespace: " <<
                     nh.getNamespace() << ").");
    return std::vector<std::string>();
  }

  std::vector<std::string> out;
  for (int i = 0; i < xml_array.size(); ++i)
  {
    if (xml_array[i].getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR_STREAM("The '" << param_name << "' parameter contains a non-string element (namespace: " <<
                       nh.getNamespace() << ").");
      return std::vector<std::string>();
    }
    out.push_back(static_cast<std::string>(xml_array[i]));
  }
  return out;
}

boost::shared_ptr<urdf::Model> getUrdf(const ros::NodeHandle& nh, const std::string& param_name)
{
  boost::shared_ptr<urdf::Model> urdf(new urdf::Model);
  std::string urdf_str;
  if (!nh.getParam(param_name, urdf_str))
  {
    ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " <<
                     nh.getNamespace() << ").");
    return boost::shared_ptr<urdf::Model>();
  }
  if (!urdf->initString(urdf_str))
  {
    ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter (namespace: " <<
                     nh.getNamespace() << ").");
    return boost::shared_ptr<urdf::Model>();
  }
  return urdf;
}

typedef boost::shared_ptr<const urdf::Joint> UrdfJointConstPtr;
std::vector<UrdfJointConstPtr> getUrdfJoints(const urdf::Model& urdf, const std::vector<std::string>& joint_names)
{
  std::vector<UrdfJointConstPtr> out;
  for (unsigned int i = 0; i < joint_names.size(); ++i)
  {
    UrdfJointConstPtr urdf_joint = urdf.getJoint(joint_names[i]);
    if (urdf_joint)
    {
      out.push_back(urdf_joint);
    }
    else
    {
      ROS_ERROR_STREAM("Could not find joint '" << joint_names[i] << "' in URDF model.");
      return std::vector<UrdfJointConstPtr>();
    }
  }
  return out;
}

} // namespace

namespace joint_trajectory_controller
{

using std::string;
using std::vector;

JointTrajectoryController::JointTrajectoryController() {}

// TODO: joints vs. joint_names, command vs trajectory_command
bool JointTrajectoryController::init(hardware_interface::PositionJointInterface* hw,
                                     ros::NodeHandle& root_nh,
                                     ros::NodeHandle& controller_nh)
{
  // Cache controller node handle
  controller_nh_ = controller_nh;

  // State publish rate
  double state_publish_rate = 50.0;
  controller_nh_.getParam("state_publish_rate", state_publish_rate);
  ROS_DEBUG_STREAM("Controller state will be published at " << state_publish_rate << "Hz.");
  state_publish_period_ = ros::Duration(1.0 / state_publish_rate);

  // Action status checking update rate
  double action_monitor_rate = 20.0;
  controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
  action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
  ROS_DEBUG_STREAM("Action status changes will be monitored at " << action_monitor_rate << "Hz.");

  // List of controlled joints
  joint_names_ = getStrings(controller_nh_, "joints");
  if (joint_names_.empty()) {return false;}
  const unsigned int n_joints = joint_names_.size();

  // URDF joints
  boost::shared_ptr<urdf::Model> urdf = getUrdf(root_nh, "robot_description");
  if (!urdf) {return false;}

  std::vector<UrdfJointConstPtr> urdf_joints = getUrdfJoints(*urdf, joint_names_);
  if (urdf_joints.empty()) {return false;}
  assert(n_joints == urdf_joints.size());

  // Initialize members
  joints_.resize(n_joints);
  angle_wraparound_.resize(n_joints);
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    // Joint handle
    try {joints_[i] = hw->getHandle(joint_names_[i]);}
    catch (...)
    {
      ROS_ERROR_STREAM("Could not find joint '" << joint_names_[i] << "' in '" << getHardwareInterfaceType() << "'.");
      return false;
    }

    // Whether a joint is continuous (ie. has angle wraparound)
    angle_wraparound_[i] = urdf_joints[i]->type == urdf::Joint::CONTINUOUS;
    const string not_if = angle_wraparound_[i] ? "" : "non-";

    ROS_DEBUG_STREAM("Found " << not_if << "continuous joint '" << joint_names_[i] << "' in '" <<
                     getHardwareInterfaceType() << "'.");
  }

  assert(joints_.size() == angle_wraparound_.size());
  ROS_INFO_STREAM("Initialized controller of type '" << getHardwareInterfaceType() << "' with " <<
                  joints_.size() << " joints.");

  // Preeallocate resources
  state_            = typename Segment::State(n_joints);
  state_error_      = typename Segment::State(n_joints);
  hold_start_state_ = typename Segment::State(n_joints);
  hold_end_state_   = typename Segment::State(n_joints);


  Segment hold_segment(0.0, state_, 0.0, state_);
  hold_trajectory_.resize(1, hold_segment);

  // ROS API
  trajectory_command_sub_ = controller_nh_.subscribe("command", 1, &JointTrajectoryController::trajectoryCommandCB, this);

  action_server_.reset(new ActionServer(controller_nh_, "follow_joint_trajectory",
                                        boost::bind(&JointTrajectoryController::goalCB,   this, _1),
                                        boost::bind(&JointTrajectoryController::cancelCB, this, _1),
                                        false));
  action_server_->start();

//  controller_state_publisher_.reset(new ControllerStatePublisher(controller_nh_, "state", 1));
  // TODO: Preallocate message

  return true;
}

void JointTrajectoryController::update(const ros::Time& time, const ros::Duration& period)
{
  // Updated time data
  TimeData time_data;
  time_data.time   = time;                                     // Cache current time
  time_data.period = period;                                   // Cache current control period
  time_data.uptime = time_data_.readFromRT()->uptime + period; // Update controller uptime
  time_data_.writeFromNonRT(time_data); // TODO: Grrr, we need a lock-free data structure here!

  // Sample trajectory at current time
  Trajectory& curr_traj = **(curr_trajectory_ptr_.readFromRT());
  typename Trajectory::const_iterator segment_it = sample(curr_traj, time_data.uptime.toSec(), state_);
  if (curr_traj.end() == segment_it)
  {
    // Non-realtime safe, but should never happen under normal operation
    ROS_ERROR_STREAM("Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
    return;
  }

  // Check tolerances if segment corresponds to currently active action goal
  const RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
  if (rt_segment_goal && rt_segment_goal == rt_active_goal_)
  {
    // State error
    for (unsigned int i = 0; i < joints_.size(); ++i)
    {
      state_error_.position[i] = joints_[i].getPosition() - state_.position[i];
      state_error_.velocity[i] = joints_[i].getVelocity() - state_.velocity[i];
      state_error_.acceleration[i] = 0.0; // There's no acceleration data available in a joint handle
    }

    // Check tolerances
    if (time_data.uptime.toSec() < segment_it->endTime())
    {
      // Currently executing a segment: check path tolerances
      checkPathTolerances(state_error_,
                          *segment_it);
    }
    else if (segment_it == --curr_traj.end())
    {
      // Finished executing the LAST segment: check goal tolerances
      checkGoalTolerances(state_error_,
                           *segment_it);
    }
  }

  // Send commands
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    joints_[i].setCommand(state_.position[i]);
  }
}

void JointTrajectoryController::updateTrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh)
{
  typedef InitJointTrajectoryOptions<Trajectory> Options;

  // Preconditions
  if (!msg)
  {
    ROS_WARN("Received null-pointer trajectory message, skipping.");
    return;
  }

  // Time data
  TimeData* time_data = time_data_.readFromRT(); // TODO: Grrr, we need a lock-free data structure here!

  // Time of the next update
  const ros::Time next_update_time = time_data->time + time_data->period;

  // Uptime of the next update
  ros::Time next_update_uptime = time_data->uptime + time_data->period;

  // Hold current position if trajectory is empty
  if (msg->points.empty())
  {
    setHoldPosition(time_data->uptime);
    ROS_DEBUG("Empty trajectory command, stopping.");
    return;
  }

  // Trajectory initialization options
  Options options;
  options.other_time_base    = &next_update_uptime;
  options.current_trajectory = *(curr_trajectory_ptr_.readFromNonRT());
  options.joint_names        = &joint_names_;
  options.angle_wraparound   = &angle_wraparound_;
  options.rt_goal_handle     = gh;
//  options.tolerances         = TODO

  // Update currently executing trajectory
  try
  {
    msg_trajectory_ = initJointTrajectory<Trajectory>(*msg, next_update_time, options);
    if (!msg_trajectory_.empty()) {curr_trajectory_ptr_.writeFromNonRT(&msg_trajectory_);}
  }
  catch(...)
  {
    ROS_ERROR("Unexpected exception caught when initializing trajectory from ROS message data.");
  }
}

void JointTrajectoryController::goalCB(GoalHandle gh)
{
  // Precondition
  using internal::permutation;
  std::vector<unsigned int> permutation_vector = permutation(joint_names_, gh.getGoal()->trajectory.joint_names);
  if (permutation_vector.empty())
  {
    ROS_ERROR("Ignoring goal. It does not contain the expected joints.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    gh.setRejected(result);
    return;
  }

  // Accept new goal
  preemptActiveGoal();
  gh.setAccepted();

  // Update trajectory and setup goal status checking timer
  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
  goal_handle_timer_ = controller_nh_.createTimer(action_monitor_period_, &RealtimeGoalHandle::runNonRealtime, rt_goal);
  updateTrajectoryCommand(internal::share_member(gh.getGoal(), gh.getGoal()->trajectory), rt_goal);
  rt_active_goal_ = rt_goal;
  goal_handle_timer_.start();
}

void JointTrajectoryController::cancelCB(GoalHandle gh)
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Check that cancel request refers to currently active goal (if any)
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    // Reset current goal
    rt_active_goal_.reset();

    // Controller uptime
    const ros::Time uptime = time_data_.readFromRT()->uptime;

    // Enter hold current position mode
    setHoldPosition(uptime);
    ROS_DEBUG("Canceling active action goal.");

    // Mark the current goal as canceled
    current_active_goal->gh_.setCanceled();
  }
}

void JointTrajectoryController::setHoldPosition(const ros::Time& time)
{
  // NOTE: This is realtime-safe

  // Segment boundary conditions: Settle current position in a fixed time
  typename Segment::Time start_time = time.toSec();
  typename Segment::Time end_time   = time.toSec() + 2.0; // NOTE: Magic value

  const unsigned int n_joints = joints_.size();
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    hold_start_state_.position[i]     = joints_[i].getPosition();
    hold_start_state_.velocity[i]     = joints_[i].getVelocity();
    hold_start_state_.acceleration[i] = 0.0;

    hold_end_state_.position[i]       = joints_[i].getPosition();
    hold_end_state_.velocity[i]       = 0.0;
    hold_end_state_.acceleration[i]   = 0.0;
  }

  assert(1 == hold_trajectory_.size());
  hold_trajectory_.front().init(start_time, hold_start_state_,
                                end_time,   hold_end_state_);
  curr_trajectory_ptr_.initRT(&hold_trajectory_);
}

} // namespace

PLUGINLIB_DECLARE_CLASS(position_controllers,
                        JointTrajectoryController,
                        joint_trajectory_controller::JointTrajectoryController,
                        controller_interface::ControllerBase)
