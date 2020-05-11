///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
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

/// \author Immanuel Martini

#include <functional>
#include <string>

#include <ros/ros.h>

#include <gtest/gtest.h>

#include "test_common.h"

namespace joint_trajectory_controller_tests
{
AssertionResult waitForEvent(const std::function<bool()>& check_event,
                             const std::string& event_description,
                             const ros::Duration& timeout,
                             unsigned int repeat)
{
  unsigned int count = 0;
  ros::Time start_time = ros::Time::now();
  while (ros::ok())
  {
    count = check_event() ? (count+1) : 0;
    if ((ros::Time::now() - start_time) > timeout)
    {
      return AssertionFailure() << "Timed out after " << timeout.toSec() << "s waiting for "
                                << event_description << ".";
    }
    ros::Duration(0.1).sleep();
    if (count == repeat)
    {
      return AssertionSuccess();
    }
  }
  return AssertionFailure() << "ROS shutdown.";
}

void initDefaultTrajectory(unsigned int number_of_joints, std::vector<TrajectoryPerJoint>& trajectory)
{
  Segment::State state{1};
  Segment segment(0.0, state, 1.0, state);
  TrajectoryPerJoint joint_traj{segment};
  trajectory.resize(number_of_joints, joint_traj);
}

}  // joint_trajectory_controller_tests
