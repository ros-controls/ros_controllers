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

/// \author Eric Tappan

#include "test_common.h"
#include <tf/transform_listener.h>

// TEST CASES
TEST_F(DiffDriveControllerTest, testNoOdomFrame)
{
  // wait for ROS
  while(!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }
  // set up tf listener
  tf::TransformListener listener;
  ros::Duration(2.0).sleep();
  // check the original odom frame doesn't exist
  EXPECT_FALSE(listener.frameExists("odom"));
}

TEST_F(DiffDriveControllerTest, testNewOdomFrame)
{
  // wait for ROS
  while(!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }
  // set up tf listener
  tf::TransformListener listener;
  ros::Duration(2.0).sleep();
  // check the new_odom frame does exist
  EXPECT_TRUE(listener.frameExists("new_odom"));
}

TEST_F(DiffDriveControllerTest, testOdomTopic)
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
  // get an odom message
  nav_msgs::Odometry odom_msg = getLastOdom();
  // check its frame_id
  ASSERT_STREQ(odom_msg.header.frame_id.c_str(), "new_odom");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "diff_drive_odom_frame_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
