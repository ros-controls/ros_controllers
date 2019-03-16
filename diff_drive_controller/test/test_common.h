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

#include <cmath>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <tf/tf.h>

#include <std_srvs/Empty.h>

// Floating-point value comparison threshold
const double EPS = 0.01;
const double POSITION_TOLERANCE = 0.01; // 1 cm-s precision
const double VELOCITY_TOLERANCE = 0.02; // 2 cm-s-1 precision
const double JERK_LINEAR_VELOCITY_TOLERANCE = 0.10; // 10 cm-s-1 precision
const double JERK_ANGULAR_VELOCITY_TOLERANCE = 0.05; // 3 deg-s-1 precision
const double ORIENTATION_TOLERANCE = 0.03; // 0.57 degree precision

class DiffDriveControllerTest : public ::testing::Test
{
public:

  DiffDriveControllerTest()
  : received_first_odom(false)
  , cmd_pub(nh.advertise<geometry_msgs::Twist>("cmd_vel", 100))
  , odom_sub(nh.subscribe("odom", 100, &DiffDriveControllerTest::odomCallback, this))
  , vel_out_sub(nh.subscribe("cmd_vel_out", 100, &DiffDriveControllerTest::cmdVelOutCallback, this))
  , joint_traj_controller_state_sub(nh.subscribe("wheel_joint_controller_state", 100, &DiffDriveControllerTest::jointTrajectoryControllerStateCallback, this))
  , start_srv(nh.serviceClient<std_srvs::Empty>("start"))
  , stop_srv(nh.serviceClient<std_srvs::Empty>("stop"))
  {
  }

  ~DiffDriveControllerTest()
  {
    odom_sub.shutdown();
    joint_traj_controller_state_sub.shutdown();
  }

  nav_msgs::Odometry getLastOdom(){ return last_odom; }
  geometry_msgs::TwistStamped getLastCmdVelOut(){ return last_cmd_vel_out; }
  control_msgs::JointTrajectoryControllerState getLastJointTrajectoryControllerState(){ return last_joint_traj_controller_state; }
  void publish(geometry_msgs::Twist cmd_vel){ cmd_pub.publish(cmd_vel); }
  bool isControllerAlive()const{ return (odom_sub.getNumPublishers() > 0) && (cmd_pub.getNumSubscribers() > 0); }
  bool isPublishingCmdVelOut(const ros::Duration &timeout=ros::Duration(1)) const
  {
    ros::Time start = ros::Time::now();
    int get_num_publishers = vel_out_sub.getNumPublishers();
    while ( (get_num_publishers == 0) && (ros::Time::now() < start + timeout) ) {
      ros::Duration(0.1).sleep();
      get_num_publishers = vel_out_sub.getNumPublishers();
    }
    return (get_num_publishers > 0);
  }
  bool isPublishingJointTrajectoryControllerState(){ return (joint_traj_controller_state_sub.getNumPublishers() > 0); }
  bool hasReceivedFirstOdom()const{ return received_first_odom; }

  void start(){ std_srvs::Empty srv; start_srv.call(srv); }
  void stop(){ std_srvs::Empty srv; stop_srv.call(srv); }

  void waitForController() const
  {
    while(!isControllerAlive() && ros::ok())
    {
      ROS_DEBUG_STREAM_THROTTLE(0.5, "Waiting for controller.");
      ros::Duration(0.1).sleep();
    }
    if (!ros::ok())
      FAIL() << "Something went wrong while executing test.";
  }

  void waitForOdomMsgs() const
  {
    while(!hasReceivedFirstOdom() && ros::ok())
    {
      ROS_DEBUG_STREAM_THROTTLE(0.5, "Waiting for odom messages to be published.");
      ros::Duration(0.01).sleep();
    }
    if (!ros::ok())
      FAIL() << "Something went wrong while executing test.";
  }

private:
  bool received_first_odom;
  ros::NodeHandle nh;
  ros::Publisher cmd_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber vel_out_sub;
  nav_msgs::Odometry last_odom;
  geometry_msgs::TwistStamped last_cmd_vel_out;
  ros::Subscriber joint_traj_controller_state_sub;
  control_msgs::JointTrajectoryControllerState last_joint_traj_controller_state;

  ros::ServiceClient start_srv;
  ros::ServiceClient stop_srv;

  void odomCallback(const nav_msgs::Odometry& odom)
  {
    ROS_INFO_STREAM("Callback received: pos.x: " << odom.pose.pose.position.x
                     << ", orient.z: " << odom.pose.pose.orientation.z
                     << ", lin_est: " << odom.twist.twist.linear.x
                     << ", ang_est: " << odom.twist.twist.angular.z);
    last_odom = odom;
    received_first_odom = true;
  }

  void jointTrajectoryControllerStateCallback(const control_msgs::JointTrajectoryControllerState& joint_traj_controller_state)
  {
    ROS_INFO_STREAM("Joint trajectory controller state callback.");
    ROS_DEBUG_STREAM("Joint trajectory controller state callback received:\n" <<
                     joint_traj_controller_state);

    last_joint_traj_controller_state = joint_traj_controller_state;
  }

  void cmdVelOutCallback(const geometry_msgs::TwistStamped& cmd_vel_out)
  {
    ROS_INFO_STREAM("Callback received: lin: " << cmd_vel_out.twist.linear.x
                     << ", ang: " << cmd_vel_out.twist.angular.z);
    last_cmd_vel_out = cmd_vel_out;
  }
};

inline tf::Quaternion tfQuatFromGeomQuat(const geometry_msgs::Quaternion& quat)
{
  return tf::Quaternion(quat.x, quat.y, quat.z, quat.w);
}

