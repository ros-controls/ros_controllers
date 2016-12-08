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

// TEST CASES
TEST_F(DiffDriveControllerTest, testInsideLimits)
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
  // send a velocity command of 0.1 m/s and 0.1 rad/s (inside limits)
  cmd_vel.linear.x = 0.1;
  cmd_vel.angular.z = 0.1;
  publish(cmd_vel);
  // wait a bit (less than the cmd_vel timeout)
  ros::Duration(1.0).sleep();

  const geometry_msgs::TwistStamped cmd_vel_limited = getLastCmdVelLimited();

  // check cmd_vel limited is the same as the requested one
  EXPECT_EQ(cmd_vel.linear.x, cmd_vel_limited.twist.linear.x);
  EXPECT_EQ(cmd_vel.angular.z, cmd_vel_limited.twist.angular.z);
}

TEST_F(DiffDriveControllerTest, testOutsideLimits)
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
  // send a velocity command of 1.5 m/s and 2.5 rad/s (outside limits)
  cmd_vel.linear.x = 1.5;
  cmd_vel.angular.z = 2.5;
  publish(cmd_vel);
  // wait a bit (less than the cmd_vel timeout)
  ros::Duration(2.0).sleep();

  const geometry_msgs::TwistStamped cmd_vel_limited = getLastCmdVelLimited();

  // check cmd_vel limited is smaller than the requested one
  EXPECT_LT(cmd_vel_limited.twist.linear.x, cmd_vel.linear.x);
  EXPECT_LT(cmd_vel_limited.twist.angular.z, cmd_vel.angular.z);

  // check cmd_vel limited is equal to the maximum allowed one (limits)
  double max_linear_velocity = 1.0;
  double max_angular_velocity = 2.0;

  ros::param::param("/diffbot_controller/linear/x/max_velocity", max_linear_velocity, max_linear_velocity);
  ros::param::param("/diffbot_controller/angular/z/max_velocity", max_angular_velocity, max_angular_velocity);

  EXPECT_EQ(max_linear_velocity, cmd_vel_limited.twist.linear.x);
  EXPECT_EQ(max_angular_velocity, cmd_vel_limited.twist.angular.z);
}

TEST_F(DiffDriveControllerTest, testPreserveTurningRadius)
{
  // @todo
}

TEST_F(DiffDriveControllerTest, testOutsideJointLimits)
{
  // @todo
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "diff_drive_publish_cmd_vel_limited_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
