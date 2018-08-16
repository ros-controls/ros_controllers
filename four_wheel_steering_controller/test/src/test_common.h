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

#include <geometry_msgs/Twist.h>
#include <four_wheel_steering_msgs/FourWheelSteering.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <std_srvs/Empty.h>

// Floating-point value comparison threshold
const double EPS = 0.01;
const double POSITION_TOLERANCE = 0.02; // 2 cm-s precision
const double VELOCITY_TOLERANCE = 0.02; // 2 cm-s-1 precision
const double JERK_LINEAR_VELOCITY_TOLERANCE = 0.10; // 10 cm-s-1 precision
const double JERK_ANGULAR_VELOCITY_TOLERANCE = 0.05; // 3 deg-s-1 precision
const double ORIENTATION_TOLERANCE = 0.08; // 4.58 degree precision

class FourWheelSteeringControllerTest : public ::testing::Test
{
public:

  FourWheelSteeringControllerTest()
  : received_first_odom(false)
  , cmd_twist_pub(nh.advertise<geometry_msgs::Twist>("cmd_vel", 100))
  , cmd_4ws_pub(nh.advertise<four_wheel_steering_msgs::FourWheelSteering>("cmd_four_wheel_steering", 100))
  , odom_sub(nh.subscribe("odom", 100, &FourWheelSteeringControllerTest::odomCallback, this))
  , start_srv(nh.serviceClient<std_srvs::Empty>("start"))
  , stop_srv(nh.serviceClient<std_srvs::Empty>("stop"))
  {
  }

  ~FourWheelSteeringControllerTest()
  {
    odom_sub.shutdown();
  }

  nav_msgs::Odometry getLastOdom(){ return last_odom; }
  void publish(geometry_msgs::Twist cmd_vel)
  {
    cmd_twist_pub.publish(cmd_vel);
  }
  void publish_4ws(four_wheel_steering_msgs::FourWheelSteering cmd_vel)
  {
    cmd_4ws_pub.publish(cmd_vel);
  }
  bool isControllerAlive()const{ return (odom_sub.getNumPublishers() > 0)
        && ((cmd_twist_pub.getNumSubscribers() > 0) || (cmd_4ws_pub.getNumSubscribers() > 0)); }

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
  ros::Publisher cmd_twist_pub, cmd_4ws_pub;
  ros::Subscriber odom_sub;
  nav_msgs::Odometry last_odom;

  ros::ServiceClient start_srv;
  ros::ServiceClient stop_srv;

  void odomCallback(const nav_msgs::Odometry& odom)
  {
    ROS_INFO_STREAM("Callback reveived: pos.x: " << odom.pose.pose.position.x
                     << ", orient.z: " << odom.pose.pose.orientation.z
                     << ", lin_est: " << odom.twist.twist.linear.x
                     << ", ang_est: " << odom.twist.twist.angular.z);
    last_odom = odom;
    received_first_odom = true;
  }
};

inline tf::Quaternion tfQuatFromGeomQuat(const geometry_msgs::Quaternion& quat)
{
  return tf::Quaternion(quat.x, quat.y, quat.z, quat.w);
}

