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

// C++ standard
#include <cassert>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/node_handle.h>

// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

// actionlib
#include <actionlib/server/action_server.h>

// realtime_tools
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

// Project
#include <trajectory_interface/quintic_spline_segment.h> // TODO: Make segment type a template parameter
#include <joint_trajectory_controller/joint_trajectory_segment.h>

namespace joint_trajectory_controller
{

// TODO: Make interface-agnostic
class JointTrajectoryController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  JointTrajectoryController();

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  void starting(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);

private:
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
  typedef boost::shared_ptr<ActionServer>                                                     ActionServerPtr;
  typedef ActionServer::GoalHandle                                                            GoalHandle;
  typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;
  typedef trajectory_msgs::JointTrajectory::ConstPtr                                          JointTrajectoryConstPtr;
//  typedef realtime_tools::RealtimePublisher<controllers_msgs::JointControllerState>           ControllerStatePublisher;

  typedef JointTrajectorySegment<trajectory_interface::QuinticSplineSegment<double> > Segment;
  typedef std::vector<Segment> Trajectory;

  std::vector<hardware_interface::JointHandle> joints_;
  std::vector<bool>                            angle_wraparound_;
  RealtimeGoalHandlePtr                        rt_active_goal_;
  realtime_tools::RealtimeBuffer<Trajectory>   trajectory_;
  typename Segment::State                      state_;       ///< Workspace variable preallocated to the appropriate size
  typename Segment::State                      state_error_; ///< Workspace variable preallocated to the appropriate size
  std::vector<std::string>                     joint_names_;

  ros::Time     time_;   ///< Time of last update cycle
  ros::Duration period_; ///< Period of last update cycle

  ros::Duration state_publish_period_;
  ros::Duration action_monitor_period_;

  // ROS API
  ros::NodeHandle controller_nh_;
  ros::Timer      goal_handle_timer_;
  ros::Subscriber trajectory_command_sub_;
  ActionServerPtr action_server_;

//  boost::scoped_ptr<ControllerStatePublisher> controller_state_publisher_;

  void updateTrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh);
  void trajectoryCommandCB(const JointTrajectoryConstPtr& msg) {updateTrajectoryCommand(msg, RealtimeGoalHandlePtr());}
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);
  void preemptActiveGoal();

  void setHoldPosition(const ros::Time& time);

  /**
   * \brief Check path tolerances.
   *
   * If path tolerances are violated, the currently active action goal will be aborted.
   *
   * \param state_error Error between the current and desired trajectory states.
   * \param segment Currently active trajectory segment.
   *
   * \pre \p segment is associated to the active goal handle.
   **/
  void checkPathTolerances(const typename Segment::State& state_error,
                           const Segment&                 segment);

  /**
   * \brief Check goal tolerances.
   *
   * If goal tolerances are fulfilled, the currently active action goal will be considered successful.
   * If they are violated, the action goal will be aborted.
   *
   * \param state_error Error between the current and desired trajectory states.
   * \param segment Currently active trajectory segment.
   *
   * \pre \p segment is associated to the active goal handle.
   **/
  void checkGoalTolerances(const typename Segment::State& state_error,
                           const Segment&                 segment);
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

inline void JointTrajectoryController::checkPathTolerances(const typename Segment::State& state_error,
                                                           const Segment&                 segment)
{
  assert(segment.getGoalHandle() && segment.getGoalHandle() == rt_active_goal_);

  const SegmentTolerances& tolerances = segment.getTolerances();
  if (!checkStateTolerance(state_error, tolerances.state_tolerance))
  {
    rt_active_goal_->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
    rt_active_goal_->setAborted(rt_active_goal_->preallocated_result_);
  }
}

inline void JointTrajectoryController::checkGoalTolerances(const typename Segment::State& state_error,
                                                           const Segment&                 segment)
{
  assert(segment.getGoalHandle() && segment.getGoalHandle() == rt_active_goal_);

  // Checks that we have ended inside the goal tolerances
  const SegmentTolerances& tolerances = segment.getTolerances();
  const bool inside_goal_tolerances = checkStateTolerance(state_error, tolerances.goal_state_tolerance);

  if (inside_goal_tolerances)
  {
    rt_active_goal_->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    rt_active_goal_->setSucceeded(rt_active_goal_->preallocated_result_);
    rt_active_goal_.reset();
  }
  else if (time_.toSec() < segment.endTime() + tolerances.goal_time_tolerance)
  {
    // Still have some time left to meet the goal state tolerances
  }
  else
  {
    rt_active_goal_->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
    rt_active_goal_->setAborted(rt_active_goal_->preallocated_result_);
    rt_active_goal_.reset();
  }
}

namespace internal
{

template <class Enclosure, class Member>
inline boost::shared_ptr<Member> share_member(boost::shared_ptr<Enclosure> enclosure, Member &member)
{
  actionlib::EnclosureDeleter<Enclosure> d(enclosure);
  boost::shared_ptr<Member> p(&member, d);
  return p;
}

} // namespace

} // namespace

#endif
