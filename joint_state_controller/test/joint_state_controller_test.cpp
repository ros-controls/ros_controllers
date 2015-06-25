///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, PAL Robotics S.L.
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

#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <hardware_interface/joint_state_interface.h>
#include <joint_state_controller/joint_state_controller.h>

using namespace joint_state_controller;

class JointStateControllerTest : public ::testing::Test
{
public:
  JointStateControllerTest()
    : root_nh_(ros::NodeHandle()),
      controller_nh_("test_ok/joint_state_controller")
  {
    // Intialize raw joint state data
    names_.push_back("joint1"); names_.push_back("joint2");
    pos_[0] = 0.0; pos_[1] = 0.0;
    vel_[0] = 0.0; vel_[1] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0;

    // Setup the joint state interface
    hardware_interface::JointStateHandle state_handle_1(names_[0], &pos_[0], &vel_[0], &eff_[0]);
    js_iface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2(names_[1], &pos_[1], &vel_[1], &eff_[1]);
    js_iface_.registerHandle(state_handle_2);

    // Initialize ROS interfaces
    sub_ = root_nh_.subscribe<sensor_msgs::JointState>("joint_states",
                                                       1,
                                                       &JointStateControllerTest::jointStateCb,
                                                       this);
  }

protected:
  ros::NodeHandle root_nh_;
  ros::NodeHandle controller_nh_;
  ros::Subscriber sub_;
  hardware_interface::JointStateInterface js_iface_;

  // Raw joint state data
  std::vector<std::string> names_;
  double pos_[2];
  double vel_[2];
  double eff_[2];

  // Received joint state messages counter
  int rec_msgs_;

  // Last received joint state message
  sensor_msgs::JointState last_msg_;

  void jointStateCb(const sensor_msgs::JointStateConstPtr& msg)
  {
    last_msg_ = *msg;
    ++rec_msgs_;
  }
};

TEST_F(JointStateControllerTest, initOk)
{
  JointStateController jsc;
  EXPECT_TRUE(jsc.init(&js_iface_, root_nh_, controller_nh_));
}

TEST_F(JointStateControllerTest, initKo)
{
  JointStateController jsc;
  ros::NodeHandle bad_controller_nh("no_period_namespace");
  EXPECT_FALSE(jsc.init(&js_iface_, root_nh_, bad_controller_nh));
}

TEST_F(JointStateControllerTest, publishOk)
{
  JointStateController jsc;
  EXPECT_TRUE(jsc.init(&js_iface_, root_nh_, controller_nh_));

  int pub_rate;
  ASSERT_TRUE(controller_nh_.getParam("publish_rate", pub_rate));

  rec_msgs_ = 0;
  const int test_duration = 1; // seconds
  const int loop_freq = 2 * pub_rate; // faster than controller publish rate
  ros::Rate loop_rate(static_cast<double>(loop_freq));
  const ros::Time start_time = ros::Time::now();
  jsc.starting(start_time);

  while (true)
  {
    ros::Time now = ros::Time::now();
    jsc.update(now, ros::Duration()); // NOTE: Second parameter is unused by implementation
    ros::spinOnce();
    if (rec_msgs_ >= test_duration * pub_rate) break; // Objective reached
    if (now - start_time > ros::Duration(2.0 * test_duration)) break; // Prevents unexpected infinite loops
    loop_rate.sleep();
  }
  jsc.stopping(ros::Time::now());

  // NOTE: Below we subtract the loop rate because the the published message gets picked up in the
  // next iteration's spinOnce()
  const ros::Duration real_test_duration = ros::Time::now() - start_time - loop_rate.expectedCycleTime();
  const double real_pub_rate = static_cast<double>(rec_msgs_) / real_test_duration.toSec();

  // The publish rate should be close to the nominal value
  EXPECT_NEAR(real_pub_rate, pub_rate, 0.05 * pub_rate);
}

TEST_F(JointStateControllerTest, publishKo)
{
  JointStateController jsc;
  ros::NodeHandle negative_rate_nh("test_ko/joint_state_controller");
  EXPECT_TRUE(jsc.init(&js_iface_, root_nh_, negative_rate_nh));

  // Check non-positive publish rate
  int pub_rate;
  ASSERT_TRUE(negative_rate_nh.getParam("publish_rate", pub_rate));
  ASSERT_LE(pub_rate, 0);

  rec_msgs_ = 0;
  const ros::Duration test_duration(1.0);
  ros::Rate loop_rate(10.0);
  const ros::Time start_time = ros::Time::now();
  while(ros::Time::now() - start_time < test_duration)
  {
    ros::Time now = ros::Time::now();
    jsc.update(now, ros::Duration()); // NOTE: Second parameter is unused by implementation
    ros::spinOnce();
    loop_rate.sleep();
  }
  jsc.stopping(ros::Time::now());

  // No messages should have been published
  EXPECT_EQ(rec_msgs_, 0);
}

TEST_F(JointStateControllerTest, valuesOk)
{
  JointStateController jsc;
  EXPECT_TRUE(jsc.init(&js_iface_, root_nh_, controller_nh_));

  int pub_rate;
  ASSERT_TRUE(controller_nh_.getParam("publish_rate", pub_rate));
  ros::Duration period(1.0 / pub_rate);
  jsc.starting(ros::Time::now());

  pos_[0] = 1.0; pos_[1] = -1.0;
  vel_[0] = 2.0; vel_[1] = -2.0;
  eff_[0] = 3.0; eff_[1] = -3.0;

  period.sleep();
  jsc.update(ros::Time::now(), ros::Duration());
  period.sleep(); ros::spinOnce(); // To trigger callback

  // Check payload sizes
  ASSERT_EQ(names_.size(),         last_msg_.name.size());
  ASSERT_EQ(last_msg_.name.size(), last_msg_.position.size());
  ASSERT_EQ(last_msg_.name.size(), last_msg_.velocity.size());
  ASSERT_EQ(last_msg_.name.size(), last_msg_.effort.size());

  // Check payload values
  typedef std::vector<std::string>::size_type SizeType;
  typedef std::vector<std::string>::iterator  Iterator;
  for (SizeType raw_id = 0; raw_id < names_.size(); ++raw_id)
  {
    Iterator it = std::find(last_msg_.name.begin(), last_msg_.name.end(), names_[raw_id]);
    ASSERT_NE(last_msg_.name.end(), it);
    SizeType msg_id = std::distance(last_msg_.name.begin(), it);
    EXPECT_EQ(pos_[raw_id], last_msg_.position[msg_id]);
    EXPECT_EQ(vel_[raw_id], last_msg_.velocity[msg_id]);
    EXPECT_EQ(eff_[raw_id], last_msg_.effort[msg_id]);
  }
  jsc.stopping(ros::Time::now());
}

TEST_F(JointStateControllerTest, extraJointsOk)
{
  JointStateController jsc;
  ros::NodeHandle extra_joints_nh("test_extra_joints_ok/joint_state_controller");
  EXPECT_TRUE(jsc.init(&js_iface_, root_nh_, extra_joints_nh));

  XmlRpc::XmlRpcValue extra_joints;
  ASSERT_TRUE(extra_joints_nh.getParam("extra_joints", extra_joints));
  const int num_joints = names_.size() + extra_joints.size();

  int pub_rate;
  ASSERT_TRUE(controller_nh_.getParam("publish_rate", pub_rate));
  ros::Duration period(1.0 / pub_rate);
  jsc.starting(ros::Time::now());

  pos_[0] = 1.0; pos_[1] = -1.0;
  vel_[0] = 2.0; vel_[1] = -2.0;
  eff_[0] = 3.0; eff_[1] = -3.0;

  period.sleep();
  jsc.update(ros::Time::now(), ros::Duration());
  period.sleep(); ros::spinOnce(); // To trigger callback

  // Check payload sizes
  ASSERT_EQ(num_joints, last_msg_.name.size());
  ASSERT_EQ(num_joints, last_msg_.position.size());
  ASSERT_EQ(num_joints, last_msg_.velocity.size());
  ASSERT_EQ(num_joints, last_msg_.effort.size());

  // Check payload values: joint state interface resources
  typedef std::vector<std::string>::size_type SizeType;
  typedef std::vector<std::string>::iterator  Iterator;
  for (SizeType raw_id = 0; raw_id < names_.size(); ++raw_id)
  {
    Iterator it = std::find(last_msg_.name.begin(), last_msg_.name.end(), names_[raw_id]);
    ASSERT_NE(last_msg_.name.end(), it);
    SizeType msg_id = std::distance(last_msg_.name.begin(), it);
    EXPECT_EQ(pos_[raw_id], last_msg_.position[msg_id]);
    EXPECT_EQ(vel_[raw_id], last_msg_.velocity[msg_id]);
    EXPECT_EQ(eff_[raw_id], last_msg_.effort[msg_id]);
  }

  // Check payload values: Extra joints
  {
    Iterator it = std::find(last_msg_.name.begin(), last_msg_.name.end(), "extra1");
    ASSERT_NE(last_msg_.name.end(), it);
    SizeType msg_id = std::distance(last_msg_.name.begin(), it);
    EXPECT_EQ(10.0, last_msg_.position[msg_id]);
    EXPECT_EQ(20.0, last_msg_.velocity[msg_id]);
    EXPECT_EQ(30.0, last_msg_.effort[msg_id]);
  }
  {
    Iterator it = std::find(last_msg_.name.begin(), last_msg_.name.end(), "extra2");
    ASSERT_NE(last_msg_.name.end(), it);
    SizeType msg_id = std::distance(last_msg_.name.begin(), it);
    EXPECT_EQ(-10.0, last_msg_.position[msg_id]);
    EXPECT_EQ(  0.0, last_msg_.velocity[msg_id]);
    EXPECT_EQ(  0.0, last_msg_.effort[msg_id]);
  }
  {
    Iterator it = std::find(last_msg_.name.begin(), last_msg_.name.end(), "extra3");
    ASSERT_NE(last_msg_.name.end(), it);
    SizeType msg_id = std::distance(last_msg_.name.begin(), it);
    EXPECT_EQ(0.0, last_msg_.position[msg_id]);
    EXPECT_EQ(0.0, last_msg_.velocity[msg_id]);
    EXPECT_EQ(0.0, last_msg_.effort[msg_id]);
  }

  jsc.stopping(ros::Time::now());
}

TEST_F(JointStateControllerTest, extraJointsKo)
{
  JointStateController jsc;
  ros::NodeHandle extra_joints_nh("test_extra_joints_ko/joint_state_controller");
  EXPECT_TRUE(jsc.init(&js_iface_, root_nh_, extra_joints_nh));

  int pub_rate;
  ASSERT_TRUE(controller_nh_.getParam("publish_rate", pub_rate));
  ros::Duration period(1.0 / pub_rate);
  jsc.starting(ros::Time::now());

  pos_[0] = 1.0; pos_[1] = -1.0;
  vel_[0] = 2.0; vel_[1] = -2.0;
  eff_[0] = 3.0; eff_[1] = -3.0;

  period.sleep();
  jsc.update(ros::Time::now(), ros::Duration());
  period.sleep(); ros::spinOnce(); // To trigger callback

  // Check payload sizes
  ASSERT_EQ(names_.size(),         last_msg_.name.size());
  ASSERT_EQ(last_msg_.name.size(), last_msg_.position.size());
  ASSERT_EQ(last_msg_.name.size(), last_msg_.velocity.size());
  ASSERT_EQ(last_msg_.name.size(), last_msg_.effort.size());

  // Check payload values: No extra joints should be present
  typedef std::vector<std::string>::size_type SizeType;
  typedef std::vector<std::string>::iterator  Iterator;
  for (SizeType raw_id = 0; raw_id < names_.size(); ++raw_id)
  {
    Iterator it = std::find(last_msg_.name.begin(), last_msg_.name.end(), names_[raw_id]);
    ASSERT_NE(last_msg_.name.end(), it);
    SizeType msg_id = std::distance(last_msg_.name.begin(), it);
    EXPECT_EQ(pos_[raw_id], last_msg_.position[msg_id]);
    EXPECT_EQ(vel_[raw_id], last_msg_.velocity[msg_id]);
    EXPECT_EQ(eff_[raw_id], last_msg_.effort[msg_id]);
  }
  jsc.stopping(ros::Time::now());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "joint_state_controller_test");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
