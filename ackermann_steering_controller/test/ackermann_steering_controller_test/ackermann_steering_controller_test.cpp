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

/// \author Bence Magyar
/// \author Masaru Morita

#include "../common/include/test_common.h"
#include <tf/transform_listener.h>

// TEST CASES
TEST_F(AckermannSteeringControllerTest, testForward)
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
  ros::Duration(0.1).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a velocity command of 0.1 m/s
  cmd_vel.linear.x = 0.1;
  publish(cmd_vel);
  // wait for 10s
  ros::Duration(10.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot traveled 1 meter in XY plane, changes in z should be ~~0
  const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
  const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
  const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
  EXPECT_NEAR(sqrt(dx*dx + dy*dy), 1.0, POSITION_TOLERANCE);
  EXPECT_LT(fabs(dz), EPS);

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old, pitch_old, yaw_old);
  tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_LT(fabs(roll_new - roll_old), EPS);
  EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
  EXPECT_LT(fabs(yaw_new - yaw_old), EPS);
  EXPECT_NEAR(fabs(new_odom.twist.twist.linear.x), cmd_vel.linear.x, EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.y), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

  EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z), EPS);
}

TEST_F(AckermannSteeringControllerTest, testTurn)
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
  ros::Duration(0.1).sleep();
  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a velocity command
  cmd_vel.angular.z = M_PI/10.0;
  // send linear command too 
  // because sending only angular command doesn't actuate wheels for steer drive mechanism
  cmd_vel.linear.x = 0.1;
  publish(cmd_vel);
  // wait for 10s
  ros::Duration(10.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot rotated PI around z, changes in x should be ~~0 and in y should be y_answer
  double x_answer = 0.0;
  double y_answer = 2.0 * cmd_vel.linear.x / cmd_vel.angular.z; // R = v/w, D = 2R
  EXPECT_NEAR(fabs(new_odom.pose.pose.position.x - old_odom.pose.pose.position.x), x_answer, POSITION_TOLERANCE);
  EXPECT_NEAR(fabs(new_odom.pose.pose.position.y - old_odom.pose.pose.position.y), y_answer, POSITION_TOLERANCE);
  EXPECT_LT(fabs(new_odom.pose.pose.position.z - old_odom.pose.pose.position.z), EPS);

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old, pitch_old, yaw_old);
  tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_LT(fabs(roll_new - roll_old), EPS);
  EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
  EXPECT_NEAR(fabs(yaw_new - yaw_old), M_PI, ORIENTATION_TOLERANCE);

  EXPECT_NEAR(new_odom.twist.twist.linear.x, 0.1, VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.y), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

  EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
  EXPECT_NEAR(fabs(new_odom.twist.twist.angular.z), M_PI/10.0, EPS);
}

TEST_F(AckermannSteeringControllerTest, testOdomFrame)
{
  // wait for ROS
  while(!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }
  // set up tf listener
  tf::TransformListener listener;
  ros::Duration(2.0).sleep();
  // check the odom frame exist
  EXPECT_TRUE(listener.frameExists("odom"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ackermann_steering_controller_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
