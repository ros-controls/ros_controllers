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

/// \author Bence Magyar

#include <algorithm>
#include <cmath>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// Floating-point value comparison threshold
const double EPS = 0.01;

class DiffDriveControllerTest : public ::testing::Test
{
public:

  DiffDriveControllerTest()
    : nh("~"),
      cmd_pub(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10)),
      odom_sub(nh.subscribe("/odom", 10, &DiffDriveControllerTest::odomCallback, this))
  {
  }

  nav_msgs::Odometry getLastOdom(){ return last_odom; }
  void publish(geometry_msgs::Twist cmd_vel){ cmd_pub.publish(cmd_vel); }

private:
  ros::NodeHandle nh;
  ros::Publisher cmd_pub;
  ros::Subscriber odom_sub;
  nav_msgs::Odometry last_odom;

  void odomCallback(const nav_msgs::Odometry& odom)
  {
    last_odom = odom;
  }

};

// TEST CASES

TEST_F(DiffDriveControllerTest, testForward)
{
  ros::Duration(0.1).sleep();

  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a velocity command of 0.1 m/s
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.1;
  publish(cmd_vel);
  // wait for 10s
  ros::Duration(10.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot travelled 1 meter
  ASSERT_TRUE(new_odom.pose.pose.position.x - old_odom.pose.pose.position.x < 1.0 + EPS);
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
