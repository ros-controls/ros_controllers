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

/// \author Paul Mathieu

#include "test_common.h"

// TEST CASES
TEST_F(DiffDriveControllerTest, testLinearJerkLimits)
{
  // wait for ROS
  waitForController();

  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  ros::Duration(2.0).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a big command
  cmd_vel.linear.x = 10.0;
  publish(cmd_vel);
  // wait for a while
  ros::Duration(0.5).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot speed is now 0.37m.s-1
  EXPECT_NEAR(new_odom.twist.twist.linear.x, 0.37, JERK_LINEAR_VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z - old_odom.twist.twist.angular.z), EPS);

  cmd_vel.linear.x = 0.0;
  publish(cmd_vel);
}

TEST_F(DiffDriveControllerTest, testLinearAccelerationLimits)
{
  // wait for ROS
  while(!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }
  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  ros::Duration(2.0).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a big command
  cmd_vel.linear.x = 10.0;
  publish(cmd_vel);
  // wait for a while
  ros::Duration(0.5).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
  EXPECT_LT(fabs(new_odom.twist.twist.linear.x - old_odom.twist.twist.linear.x), 0.5 + VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z - old_odom.twist.twist.angular.z), EPS);

  cmd_vel.linear.x = 0.0;
  publish(cmd_vel);
}

TEST_F(DiffDriveControllerTest, testLinearVelocityLimits)
{
  // wait for ROS
  while(!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }
  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  ros::Duration(2.0).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a big command
  cmd_vel.linear.x = 10.0;
  publish(cmd_vel);
  // wait for a while
  ros::Duration(5.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot speed is now 1.0 m.s-1, the limit
  EXPECT_LT(fabs(new_odom.twist.twist.linear.x - old_odom.twist.twist.linear.x), 1.0 + VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z - old_odom.twist.twist.angular.z), EPS);

  cmd_vel.linear.x = 0.0;
  publish(cmd_vel);
}

/* This test has been failing on Travis for a long time due to timing issues however it works well when ran manually
TEST_F(DiffDriveControllerTest, testAngularJerkLimits)
{
  // wait for ROS
  while(!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }
  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  ros::Duration(2.0).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a big command
  cmd_vel.angular.z = 10.0;
  publish(cmd_vel);
  // wait for a while
  ros::Duration(0.5).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot speed is now 0.7rad.s-1
  EXPECT_NEAR(new_odom.twist.twist.angular.z, 0.7, JERK_ANGULAR_VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.x - old_odom.twist.twist.linear.x), EPS);

  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
}
*/

TEST_F(DiffDriveControllerTest, testAngularAccelerationLimits)
{
  // wait for ROS
  while(!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }
  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  ros::Duration(2.0).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a big command
  cmd_vel.angular.z = 10.0;
  publish(cmd_vel);
  // wait for a while
  ros::Duration(0.5).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot speed is now 1.0rad.s-1, which is 2.0rad.s-2 * 0.5s
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z - old_odom.twist.twist.angular.z), 1.0 + VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.x - old_odom.twist.twist.linear.x), EPS);

  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
}

TEST_F(DiffDriveControllerTest, testAngularVelocityLimits)
{
  // wait for ROS
  while(!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }
  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  ros::Duration(2.0).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a big command
  cmd_vel.angular.z = 10.0;
  publish(cmd_vel);
  // wait for a while
  ros::Duration(5.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot speed is now 2.0rad.s-1, the limit
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z - old_odom.twist.twist.angular.z), 2.0 + VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.x - old_odom.twist.twist.linear.x), EPS);

  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "diff_drive_limits_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
