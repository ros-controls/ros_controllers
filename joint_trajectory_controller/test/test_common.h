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

#pragma once


#include <ros/ros.h>

#include <gtest/gtest.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <trajectory_interface/quintic_spline_segment.h>


namespace joint_trajectory_controller_tests
{

static constexpr double JOINT_STATES_COMPARISON_EPS = 0.01;
static constexpr double EPS = 1e-9;

static constexpr double TIMEOUT_CONNECTIONS_S = 10.0;
static constexpr double TIMEOUT_ACTION_RESULT_S = 5.0;

//////////////////////////////
// general helper functions //
//////////////////////////////

inline bool vectorsAlmostEqual(const std::vector<double>& vec1,
                               const std::vector<double>& vec2,
                               const double& tolerance = JOINT_STATES_COMPARISON_EPS)
{
  return std::equal(vec1.begin(),
                    vec1.end(),
                    vec2.begin(),
                    [&tolerance](const double& a, const double& b){return (std::abs(b-a) < tolerance);});
}

testing::AssertionResult waitForEvent(const std::function<bool()>& check_event,
                                      const std::string& event_description,
                                      const ros::Duration& timeout,
                                      unsigned int repeat = 1);

//////////////////////////
// trajectory functions //
//////////////////////////

inline ros::Duration getTrajectoryDuration(const trajectory_msgs::JointTrajectory &traj)
{
  return traj.points.back().time_from_start;
}

inline bool trajectoryPointsAlmostEqual(const trajectory_msgs::JointTrajectoryPoint& p1,
                                        const trajectory_msgs::JointTrajectoryPoint& p2,
                                        const double& tolerance = JOINT_STATES_COMPARISON_EPS)
{
  return vectorsAlmostEqual(p1.positions, p2.positions, tolerance) &&
         vectorsAlmostEqual(p1.velocities, p2.velocities, tolerance) &&
         vectorsAlmostEqual(p1.accelerations, p2.accelerations, tolerance);
}

using QuinticSplineSegment = trajectory_interface::QuinticSplineSegment<double>;
using Segment = joint_trajectory_controller::JointTrajectorySegment<QuinticSplineSegment>;
using TrajectoryPerJoint = std::vector<Segment>;

void initDefaultTrajectory(unsigned int number_of_joints, std::vector<TrajectoryPerJoint>& trajectory);

/////////////////////////////
// action client functions //
/////////////////////////////

using actionlib::SimpleActionClient;
using actionlib::SimpleClientGoalState;
using testing::AssertionResult;
using testing::AssertionSuccess;
using testing::AssertionFailure;

template <typename T>
inline bool checkActionGoalState(const std::shared_ptr<SimpleActionClient<T>>& action_client,
                                 const SimpleClientGoalState& state)
{
  return action_client->getState() == state;
}

template <class T>
inline bool checkActionResultErrorCode(const std::shared_ptr<SimpleActionClient<T>>& action_client,
                                       typename T::_action_result_type::_result_type::_error_code_type error_code)
{
  return action_client->getResult()->error_code == error_code;
}

template <class T>
AssertionResult waitForActionServer(const std::shared_ptr<SimpleActionClient<T>>& action_client,
                                    const ros::Duration& timeout = ros::Duration(TIMEOUT_CONNECTIONS_S))
{
  if (!action_client->waitForServer(timeout))
  {
    return AssertionFailure() << "Timed out after " << timeout.toSec()
                              << "s waiting for connection to action server.";
  }
  return AssertionSuccess();
}

template <class T>
AssertionResult waitForActionResult(const std::shared_ptr<SimpleActionClient<T>>& action_client,
                                    const ros::Duration& timeout = ros::Duration(TIMEOUT_ACTION_RESULT_S))
{
  if (!action_client->waitForResult(timeout))
  {
    return AssertionFailure() << "Timed out after " << timeout.toSec() << "s waiting for action result.";
  }
  return AssertionSuccess();
}

template <class T>
AssertionResult waitForActionGoalState(const std::shared_ptr<SimpleActionClient<T>>& action_client,
                                       const SimpleClientGoalState& state,
                                       const ros::Duration& timeout = ros::Duration(TIMEOUT_ACTION_RESULT_S))
{
  return waitForEvent(std::bind(checkActionGoalState<T>, action_client, state),
                      "action goal state " + state.getText(),
                      timeout);
}

}  // namespace joint_trajectory_controller_tests
