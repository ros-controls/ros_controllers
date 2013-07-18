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

/// \author Adolfo Rodriguez Tsouroukdissian, Stuart Glaser

#ifndef JOINT_TRAJECTORY_CONTROLLER_JOINT_TRAJECTORY_CONTROLLER_H
#define JOINT_TRAJECTORY_CONTROLLER_JOINT_TRAJECTORY_CONTROLLER_H

// Boost
#include <boost/shared_ptr.hpp>

// Actionlib
#include <actionlib/server/action_server.h>

// ros messages
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

// ros_control
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

// trajectory_interface
#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <joint_trajectory_controller/trajectory_interface_ros.h>

namespace joint_trajectory_controller
{

// TODO: Make interface-agnostic
class JointTrajectoryController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  JointTrajectoryController() {}

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& controller_nh);

  void starting(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);

private:
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
  typedef ActionServer::GoalHandle                                                            GoalHandle;
  typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;
  typedef trajectory_msgs::JointTrajectory::ConstPtr                                          JointTrajectoryConstPtr;

  typedef trajectory_interface::JointTrajectorySegment<double> Segment;
  typedef std::vector<Segment> Trajectory;

  std::vector<hardware_interface::JointHandle> joints_;
  RealtimeGoalHandlePtr                        rt_active_goal_;
  Trajectory trajectory_;

  void trajectoryCommandCB(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh = RealtimeGoalHandlePtr());
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);
  void preemptActiveGoal();
};

inline void JointTrajectoryController::preemptActiveGoal()
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Cancels the currently active goal
  if (current_active_goal)
  {
    // Marks the current goal as canceled
    rt_active_goal_.reset();
    current_active_goal->gh_.setCanceled();
  }
}

} // namespace

#endif
