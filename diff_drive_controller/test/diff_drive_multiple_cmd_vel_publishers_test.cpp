///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, PAL Robotics S.L.
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

#include "test_common.h"

// TEST CASES
TEST_F(DiffDriveControllerTest, breakWithMultiplePublishers)
{
  // wait for ROS
  waitForController();
  waitForOdomMsgs();

  nav_msgs::Odometry old_odom = getLastOdom();
  //TODO: we should be programatically publish from 2 different nodes
  // not the current hacky solution with the launch files
  ros::Duration(1.0).sleep(); 
  nav_msgs::Odometry new_odom = getLastOdom();

  const double dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
  const double dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
  const double dz = new_odom.pose.pose.position.z - old_odom.pose.pose.position.z;
  EXPECT_NEAR(sqrt(dx*dx + dy*dy), 0.0, POSITION_TOLERANCE);
  EXPECT_LT(fabs(dz), EPS);

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old, pitch_old, yaw_old);
  tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_LT(fabs(roll_new - roll_old), EPS);
  EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
  EXPECT_LT(fabs(yaw_new - yaw_old), EPS);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "diff_drive_multiple_publishers_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
