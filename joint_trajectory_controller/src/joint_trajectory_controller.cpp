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

#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace joint_trajectory_controller
{

bool JointTrajectoryController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& controller_nh)
{
  return false;
}

void JointTrajectoryController::starting(const ros::Time& time)
{

}

void JointTrajectoryController::update(const ros::Time& time, const ros::Duration& period)
{

}


void JointTrajectoryController::trajectoryCommandCB(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh)
{
//  if (!msg)
//  {
//    ROS_WARN("Received null-pointer trajectory message, skipping.");
//    return;
//  }

//  const JointTrajectory& msg_ref = *msg;

//  const ros::Time time = time_ + period_; // Why + period?
//  ROS_DEBUG_STREAM("Figuring out new trajectory at " << time.toSec() <<
//                   ", with data from " << msg_ref.header.stamp.toSec());

//  // Hold current position if trajectory is empty
//  if (msg->points.empty())
//  {
//    ROS_DEBUG("Empty trajectory command, stopping.");
//    starting(); // TODO!
//    return;
//  }

//  Trajectory new_trajectory = init(msg_ref, time);
}

void JointTrajectoryController::goalCB(GoalHandle gh)
{

}

void JointTrajectoryController::cancelCB(GoalHandle gh)
{

}

} // namespace

