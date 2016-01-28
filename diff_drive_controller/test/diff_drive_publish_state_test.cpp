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
//   * Neither the name of PAL Robotics, Inc. nor the names of its
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

/// \author Enrique Fernandez

#include "test_common.h"

#include <vector>

// TEST CASES
TEST_F(DiffDriveControllerTest, testError)
{
  // wait for ROS
  while (!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }
  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  // give some time to the controller to react to the command
  ros::Duration(0.1).sleep();
  // send a velocity command of 0.3 m/s
  cmd_vel.linear.x = 0.3;
  publish(cmd_vel);
  // wait a bit
  ros::Duration(1.0).sleep();

  diff_drive_controller::DiffDriveControllerState state = getLastState();

  const size_t num_joints = state.error.positions.size();
  for (size_t i = 0; i < num_joints; ++i)
  {
    EXPECT_EQ(state.error.positions[i], state.desired.positions[i] - state.actual.positions[i]);
    EXPECT_EQ(state.error.velocities[i], state.desired.velocities[i] - state.actual.velocities[i]);
    EXPECT_EQ(state.error.accelerations[i], state.desired.accelerations[i] - state.actual.accelerations[i]);
    EXPECT_EQ(state.error.effort[i], state.desired.effort[i] - state.actual.effort[i]);
  }
}

TEST_F(DiffDriveControllerTest, testPositionVelocityAcceleration)
{
  // wait for ROS
  while (!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }
  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  // give some time to the controller to react to the command
  ros::Duration(0.1).sleep();
  // send a velocity command of 0.3 m/s
  cmd_vel.linear.x = 0.3;
  publish(cmd_vel);
  // wait a bit
  ros::Duration(1.0).sleep();

  std::vector<diff_drive_controller::DiffDriveControllerState> states = getLastStates();

  EXPECT_GT(states.size(), 2);

  const size_t num_states = states.size();
  std::vector<double> times(num_states);
  for (size_t i = 0; i < num_states; ++i)
  {
    times[i] = states[i].header.stamp.toSec();
  }

  const size_t num_time_steps = num_states - 1;
  std::vector<double> time_steps(num_time_steps);
  for (size_t i = 0; i < num_time_steps; ++i)
  {
    time_steps[i] = times[i+1] - times[i];
  }

  const size_t num_joints = states[0].error.positions.size();
  for (size_t i = 0; i < num_joints; ++i)
  {
    EXPECT_EQ(states[1].error.velocities[i]  , (states[1].error.positions[i]   - states[0].error.positions[i]  ) / time_steps[0]);
    EXPECT_EQ(states[1].desired.velocities[i], (states[1].desired.positions[i] - states[0].desired.positions[i]) / time_steps[0]);
    EXPECT_EQ(states[1].actual.velocities[i] , (states[1].actual.positions[i]  - states[0].actual.positions[i] ) / time_steps[0]);

    EXPECT_EQ(states[2].error.velocities[i]  , (states[2].error.positions[i]   - states[1].error.positions[i]  ) / time_steps[1]);
    EXPECT_EQ(states[2].desired.velocities[i], (states[2].desired.positions[i] - states[1].desired.positions[i]) / time_steps[1]);
    EXPECT_EQ(states[2].actual.velocities[i] , (states[2].actual.positions[i]  - states[1].actual.positions[i] ) / time_steps[1]);

    EXPECT_EQ(states[1].error.accelerations[i]  , (states[1].error.velocities[i]   - states[0].error.velocities[i]  ) / time_steps[0]);
    EXPECT_EQ(states[1].desired.accelerations[i], (states[1].desired.velocities[i] - states[0].desired.velocities[i]) / time_steps[0]);
    EXPECT_EQ(states[1].actual.accelerations[i] , (states[1].actual.velocities[i]  - states[0].actual.velocities[i] ) / time_steps[0]);

    EXPECT_EQ(states[2].error.accelerations[i]  , (states[2].error.velocities[i]   - states[1].error.velocities[i]  ) / time_steps[1]);
    EXPECT_EQ(states[2].desired.accelerations[i], (states[2].desired.velocities[i] - states[1].desired.velocities[i]) / time_steps[1]);
    EXPECT_EQ(states[2].actual.accelerations[i] , (states[2].actual.velocities[i]  - states[1].actual.velocities[i] ) / time_steps[1]);
  }
}

TEST_F(DiffDriveControllerTest, testZeroErrorSteadyState)
{
  // wait for ROS
  while (!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }
  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  // give some time to the controller to react to the command
  ros::Duration(0.1).sleep();
  // send a velocity command of 0.3 m/s
  cmd_vel.linear.x = 0.3;
  publish(cmd_vel);
  // wait a bit so we reach the steady state
  ros::Duration(3.0).sleep();

  diff_drive_controller::DiffDriveControllerState state = getLastState();

  const size_t num_joints = state.error.positions.size();
  for (size_t i = 0; i < num_joints; ++i)
  {
    EXPECT_EQ(state.error.positions[i], 0.0);
    EXPECT_EQ(state.error.velocities[i], 0.0);
    EXPECT_EQ(state.error.accelerations[i], 0.0);
    EXPECT_EQ(state.error.effort[i], 0.0);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "diff_drive_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
