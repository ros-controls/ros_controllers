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

JointTrajectoryController::JointTrajectoryController()
  : joints_(),
    angle_wraparound_(),
    rt_active_goal_(),
    trajectory_(),
    state_(),
    joint_names_(),
    time_(0.0),
    period_(0.0)
{}

// TODO: joints vs. joint_names, command vs trajectory_command
bool JointTrajectoryController::init(hardware_interface::PositionJointInterface* hw,
                                     ros::NodeHandle& root_nh,
                                     ros::NodeHandle& controller_nh)
{
  // List of controlled joints
  joint_names_ = getStrings(controller_nh, "joints");
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
  state_.resize(n_joints);

  // ROS API
  trajectory_command_sub_ = controller_nh.subscribe("command", 1, &JointTrajectoryController::trajectoryCommandCB, this);

//  controller_state_publisher_.reset(new ControllerStatePublisher(controller_nh, "state", 1));
  // TODO: Preallocate message

  return true;
}

void JointTrajectoryController::starting(const ros::Time& time)
{

}

void JointTrajectoryController::update(const ros::Time& time, const ros::Duration& period)
{
  // Cache current time and control period
  time_   = time;
  period_ = period;

  // Sample trajectory at current time
  Trajectory& curr_traj = *(trajectory_.readFromRT());
  typename Trajectory::const_iterator segment_it = sample(curr_traj, time.toSec(), state_);
  if (curr_traj.end() == segment_it)
  {
    ROS_ERROR_STREAM("Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
    return;
  }

  // Use this to get the joint tolerances to check, once we implement them:
  // tolerances_it = std::advance(tolerances.begin(), std::distance(curr_traj.begin(), segment_it));

  // Send commands
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    joints_[i].setCommand(state_[i].position);
  }
}


void JointTrajectoryController::updateTtrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh)
{
  typedef trajectory_interface::InitJointTrajectoryOptions<Trajectory> Options;
  using trajectory_interface::initJointTrajectory;

  // Preconditions
  if (!msg)
  {
    ROS_WARN("Received null-pointer trajectory message, skipping.");
    return;
  }

  // Time of the next update
  const ros::Time curr_time = time_ + period_;

  // Trajectory initialization options
  Options options;
  options.current_trajectory = trajectory_.readFromRT();
  options.joint_names        = &joint_names_;
  options.angle_wraparound   = &angle_wraparound_;

  // Update currently executing trajectory
  Trajectory new_traj = initJointTrajectory<Trajectory>(*msg, curr_time, options);
  if (!new_traj.empty()) {trajectory_.writeFromNonRT(new_traj);}
}

void JointTrajectoryController::goalCB(GoalHandle gh)
{

}

void JointTrajectoryController::cancelCB(GoalHandle gh)
{

}

} // namespace

