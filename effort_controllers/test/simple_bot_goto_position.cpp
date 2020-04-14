/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Franco Fusco.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>


class SimpleBotFixture : public ::testing::Test {
protected:
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::NodeHandle nh;
  double position;

  void positionCB(const sensor_msgs::JointState& msg) {
    position = msg.position[0];
  }

  bool waitToReachTarget(double target, std::string& err, double precision=5e-2, double timeout=20.0) {
    position = 1e3*precision + target; // make position != target
    ros::Rate rate(20.0);
    std_msgs::Float64 cmd;
    cmd.data = target;
    ros::Time start = ros::Time::now();

    while(ros::ok()) {
      pub.publish(cmd);
      if(std::fabs(target-position) < precision)
        return true;
      if( (ros::Time::now()-start).toSec() > timeout ) {
        err = "Timed out, target = " + std::to_string(target) + ", current = " + std::to_string(position);
        return false;
      }
      rate.sleep();
    }

    err = "ROS shutdwon, target = " + std::to_string(target) + ", current = " + std::to_string(position);
    return false;
  }


  void SetUp() override {
    // setup
    position = 0.0;
    pub = nh.advertise<std_msgs::Float64>("position_controller/command", 1);
    sub = nh.subscribe("joint_states", 1, &SimpleBotFixture::positionCB, this);
  }
};


TEST_F(SimpleBotFixture, TestPosition) {
  ros::Duration(1.0).sleep();
  std::string err;
  EXPECT_TRUE(waitToReachTarget(1.0, err)) << err;
  EXPECT_TRUE(waitToReachTarget(-1.0, err)) << err;
  EXPECT_TRUE(waitToReachTarget(0.0, err)) << err;
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dummy");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
