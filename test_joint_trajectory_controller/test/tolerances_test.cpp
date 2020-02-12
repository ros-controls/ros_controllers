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

/// \author Adolfo Rodriguez Tsouroukdissian

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <trajectory_interface/pos_vel_acc_state.h>
#include <joint_trajectory_controller/tolerances.h>

using namespace joint_trajectory_controller;
typedef trajectory_interface::PosVelAccState<double> State;
typedef StateTolerances<double> StateTols;

TEST(TolerancesTest, CheckStateTolerance)
{
  const double tol = 1.0;

  State state_error_ok;
  state_error_ok.position.resize(2, -tol);
  state_error_ok.velocity.resize(2, -tol);
  state_error_ok.acceleration.resize(2, -tol);

  // Empty tolerances: No checks take place
  {
    std::vector<StateTols> state_tols(2);
    EXPECT_TRUE(checkStateTolerance(state_error_ok, state_tols));
  }

  // Check position-only tolerances
  {
    State state_error = state_error_ok;
    StateTols state_tol;
    state_tol.position = tol;
    std::vector<StateTols> state_tols(2, state_tol);

    // Tolerances OK
    EXPECT_TRUE(checkStateTolerance(state_error, state_tols));

    // Increase non-checked variable errors
    state_error.velocity.back() *= 2.0;
    EXPECT_TRUE(checkStateTolerance(state_error, state_tols));

    state_error.acceleration.back() *= 2.0;
    EXPECT_TRUE(checkStateTolerance(state_error, state_tols));

    // Increase position errors
    state_error.position.back() *= 2.0;
    EXPECT_FALSE(checkStateTolerance(state_error, state_tols));
  }

  // Check velocity-only tolerances
  {
    State state_error = state_error_ok;
    StateTols state_tol;
    state_tol.velocity = tol;
    std::vector<StateTols> state_tols(2, state_tol);

    // Tolerances OK
    EXPECT_TRUE(checkStateTolerance(state_error, state_tols));

    // Increase non-checked variable errors
    state_error.position.back() *= 2.0;
    EXPECT_TRUE(checkStateTolerance(state_error, state_tols));

    state_error.acceleration.back() *= 2.0;
    EXPECT_TRUE(checkStateTolerance(state_error, state_tols));

    // Increase velocity errors
    state_error.velocity.back() *= 2.0;
    EXPECT_FALSE(checkStateTolerance(state_error, state_tols));
  }

  // Check acceleration-only tolerances
  {
    State state_error = state_error_ok;
    StateTols state_tol;
    state_tol.acceleration = tol;
    std::vector<StateTols> state_tols(2, state_tol);

    // Tolerances OK
    EXPECT_TRUE(checkStateTolerance(state_error, state_tols));

    // Increase non-checked variable errors
    state_error.position.back() *= 2.0;
    EXPECT_TRUE(checkStateTolerance(state_error, state_tols));

    state_error.velocity.back() *= 2.0;
    EXPECT_TRUE(checkStateTolerance(state_error, state_tols));

    // Increase acceleration errors
    state_error.acceleration.back() *= 2.0;
    EXPECT_FALSE(checkStateTolerance(state_error, state_tols));
  }
}

TEST(TolerancesTest, UpdateStateTolerances)
{
  StateTols default_state_tols(1.0, 2.0, 3.0);

  control_msgs::JointTolerance default_tol_msg;
  default_tol_msg.name         = "foo_joint";
  default_tol_msg.position     = 0.0;
  default_tol_msg.velocity     = 0.0;
  default_tol_msg.acceleration = 0.0;

  // Zero tolerances: No-op
  {
    StateTols state_tols = default_state_tols;
    control_msgs::JointTolerance tol_msg = default_tol_msg;
    updateStateTolerances(tol_msg, state_tols);
    EXPECT_EQ(default_state_tols.position, state_tols.position);
    EXPECT_EQ(default_state_tols.velocity, state_tols.velocity);
    EXPECT_EQ(default_state_tols.acceleration, state_tols.acceleration);
  }

  // Negative tolerances: Reset existing values
  {
    // Position
    StateTols state_tols = default_state_tols;
    control_msgs::JointTolerance tol_msg = default_tol_msg;
    tol_msg.position = -1.0;
    updateStateTolerances(tol_msg, state_tols);
    EXPECT_EQ(0.0,                             state_tols.position);
    EXPECT_EQ(default_state_tols.velocity,     state_tols.velocity);
    EXPECT_EQ(default_state_tols.acceleration, state_tols.acceleration);
  }
  {
    // Velocity
    StateTols state_tols = default_state_tols;
    control_msgs::JointTolerance tol_msg = default_tol_msg;
    tol_msg.velocity = -1.0;
    updateStateTolerances(tol_msg, state_tols);
    EXPECT_EQ(default_state_tols.position,     state_tols.position);
    EXPECT_EQ(0.0,                             state_tols.velocity);
    EXPECT_EQ(default_state_tols.acceleration, state_tols.acceleration);
  }
  {
    // Acceleration
    StateTols state_tols = default_state_tols;
    control_msgs::JointTolerance tol_msg = default_tol_msg;
    tol_msg.acceleration = -1.0;
    updateStateTolerances(tol_msg, state_tols);
    EXPECT_EQ(default_state_tols.position, state_tols.position);
    EXPECT_EQ(default_state_tols.velocity, state_tols.velocity);
    EXPECT_EQ(0.0,                         state_tols.acceleration);
  }

  // Positive tolerances: Override existing values
  {
    // Position
    StateTols state_tols = default_state_tols;
    control_msgs::JointTolerance tol_msg = default_tol_msg;
    tol_msg.position = 0.5;
    updateStateTolerances(tol_msg, state_tols);
    EXPECT_EQ(tol_msg.position,                state_tols.position);
    EXPECT_EQ(default_state_tols.velocity,     state_tols.velocity);
    EXPECT_EQ(default_state_tols.acceleration, state_tols.acceleration);
  }
  {
    // Velocity
    StateTols state_tols = default_state_tols;
    control_msgs::JointTolerance tol_msg = default_tol_msg;
    tol_msg.velocity = 0.5;
    updateStateTolerances(tol_msg, state_tols);
    EXPECT_EQ(default_state_tols.position,     state_tols.position);
    EXPECT_EQ(tol_msg.velocity,                state_tols.velocity);
    EXPECT_EQ(default_state_tols.acceleration, state_tols.acceleration);
  }
  {
    // Acceleration
    StateTols state_tols = default_state_tols;
    control_msgs::JointTolerance tol_msg = default_tol_msg;
    tol_msg.acceleration = 0.5;
    updateStateTolerances(tol_msg, state_tols);
    EXPECT_EQ(default_state_tols.position, state_tols.position);
    EXPECT_EQ(default_state_tols.velocity, state_tols.velocity);
    EXPECT_EQ(tol_msg.acceleration,        state_tols.acceleration);
  }
}

TEST(TolerancesTest, UpdateSegmentTolerances)
{
  // Joint names
  std::vector<std::string> joint_names(2);
  joint_names[0] = "foo_joint";
  joint_names[1] = "bar_joint";

  // Tolerances to update from message data
  SegmentTolerances<double> ref_segment_tols(2);
  ref_segment_tols.state_tolerance[0]      = StateTolerances<double>(1.0, 1.0, 1.0);
  ref_segment_tols.state_tolerance[1]      = StateTolerances<double>(2.0, 2.0, 2.0);
  ref_segment_tols.goal_state_tolerance[0] = StateTolerances<double>(3.0, 3.0, 3.0);
  ref_segment_tols.goal_state_tolerance[1] = StateTolerances<double>(4.0, 4.0, 4.0);
  ref_segment_tols.goal_time_tolerance     = 1.0;

  // Message data
  control_msgs::JointTolerance invalid_tol_msg;
  invalid_tol_msg.name         = "invalid_joint";
  invalid_tol_msg.position     = -1.0;
  invalid_tol_msg.velocity     = -1.0;
  invalid_tol_msg.acceleration = -1.0;

  control_msgs::JointTolerance state_tol_msg;
  state_tol_msg.name         = joint_names[0];
  state_tol_msg.position     =  0.5;
  state_tol_msg.velocity     =  0.0;
  state_tol_msg.acceleration = -1.0;

  control_msgs::JointTolerance goal_state_tol_msg;
  goal_state_tol_msg.name         = joint_names[1];
  goal_state_tol_msg.position     =  0.25;
  goal_state_tol_msg.velocity     =  0.0;
  goal_state_tol_msg.acceleration = -1.0;

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.path_tolerance.push_back(invalid_tol_msg); // Useless data that should be ignored
  goal.path_tolerance.push_back(state_tol_msg);   // Only first joint has state tolerances
  goal.path_tolerance.push_back(invalid_tol_msg); // Useless data that should be ignored

  goal.goal_tolerance.push_back(invalid_tol_msg);    // Useless data that should be ignored
  goal.goal_tolerance.push_back(invalid_tol_msg);    // Useless data that should be ignored
  goal.goal_tolerance.push_back(goal_state_tol_msg); // Only second joint has goal state tolerances

  goal.goal_time_tolerance = ros::Duration(0.0); // No-op

  // Update tolerances from message
  SegmentTolerances<double> segment_tols = ref_segment_tols;
  updateSegmentTolerances(goal, joint_names, segment_tols);

  // First joint should get only state tolerances updated
  EXPECT_EQ(state_tol_msg.position,                       segment_tols.state_tolerance[0].position);     // Update
  EXPECT_EQ(ref_segment_tols.state_tolerance[0].velocity, segment_tols.state_tolerance[0].velocity);     // No-op
  EXPECT_EQ(0.0,                                          segment_tols.state_tolerance[0].acceleration); // Reset

  EXPECT_EQ(ref_segment_tols.state_tolerance[1].position,     segment_tols.state_tolerance[1].position);     // No-op
  EXPECT_EQ(ref_segment_tols.state_tolerance[1].velocity,     segment_tols.state_tolerance[1].velocity);     // No-op
  EXPECT_EQ(ref_segment_tols.state_tolerance[1].acceleration, segment_tols.state_tolerance[1].acceleration); // No-op

  // Second joint should get only goal state tolerances updated
  EXPECT_EQ(ref_segment_tols.goal_state_tolerance[0].position,     segment_tols.goal_state_tolerance[0].position);     // No-op
  EXPECT_EQ(ref_segment_tols.goal_state_tolerance[0].velocity,     segment_tols.goal_state_tolerance[0].velocity);     // No-op
  EXPECT_EQ(ref_segment_tols.goal_state_tolerance[0].acceleration, segment_tols.goal_state_tolerance[0].acceleration); // No-op

  EXPECT_EQ(goal_state_tol_msg.position,                       segment_tols.goal_state_tolerance[1].position);     // Update
  EXPECT_EQ(ref_segment_tols.goal_state_tolerance[1].velocity, segment_tols.goal_state_tolerance[1].velocity);     // No-op
  EXPECT_EQ(0.0,                                               segment_tols.goal_state_tolerance[1].acceleration); // Reset

  // Goal time constraint
  EXPECT_EQ(ref_segment_tols.goal_time_tolerance, segment_tols.goal_time_tolerance); // No-op

  goal.goal_time_tolerance = ros::Duration(1.0);
  updateSegmentTolerances(goal, joint_names, segment_tols);
  EXPECT_EQ(goal.goal_time_tolerance.toSec(), segment_tols.goal_time_tolerance);     // Update

  goal.goal_time_tolerance = ros::Duration(-1.0);
  updateSegmentTolerances(goal, joint_names, segment_tols);
  EXPECT_EQ(0.0, segment_tols.goal_time_tolerance);                                  // Reset
}

TEST(TolerancesTest, getSegmentTolerances)
{
  ros::NodeHandle nh("test/constraints");

  std::vector<std::string> joint_names(2);
  joint_names[0] = "foo_joint";
  joint_names[1] = "bar_joint";

  SegmentTolerances<double> segment_tols = getSegmentTolerances<double>(nh, joint_names);

  EXPECT_EQ(joint_names.size(), segment_tols.state_tolerance.size());
  EXPECT_EQ(joint_names.size(), segment_tols.goal_state_tolerance.size());

  EXPECT_EQ(0.05, segment_tols.state_tolerance[0].position);
  EXPECT_EQ(0.0,  segment_tols.state_tolerance[0].velocity);
  EXPECT_EQ(0.0,  segment_tols.state_tolerance[0].acceleration);

  EXPECT_EQ(0.0, segment_tols.state_tolerance[1].position);
  EXPECT_EQ(0.0, segment_tols.state_tolerance[1].velocity);
  EXPECT_EQ(0.0, segment_tols.state_tolerance[1].acceleration);

  EXPECT_EQ(0.03, segment_tols.goal_state_tolerance[0].position);
  EXPECT_EQ(0.02, segment_tols.goal_state_tolerance[0].velocity);
  EXPECT_EQ(0.0,  segment_tols.goal_state_tolerance[0].acceleration);

  EXPECT_EQ(0.01, segment_tols.goal_state_tolerance[1].position);
  EXPECT_EQ(0.02, segment_tols.goal_state_tolerance[1].velocity);
  EXPECT_EQ(0.0,  segment_tols.goal_state_tolerance[1].acceleration);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tolerances_test");
  return RUN_ALL_TESTS();
}
