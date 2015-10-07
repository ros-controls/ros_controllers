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
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <control_toolbox/pid.h>
#include <angles/angles.h>

#include <diff_drive_controller/odometry.h>
#include <diff_drive_controller/DiffDriveControllerState.h>

#include <Eigen/Dense>

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
  : cmd_pub(nh.advertise<geometry_msgs::Twist>("cmd_vel", 100))
  , odom_sub(nh.subscribe("odom", 100, &DiffDriveControllerTest::odomCallback, this))
  , cmd_vel_limited_sub(nh.subscribe("cmd_vel_limited", 100, &DiffDriveControllerTest::cmdVelLimitedCallback, this))
  , state_sub(nh.subscribe("state", 100, &DiffDriveControllerTest::diffDriveControllerStateCallback, this))
  , last_states(3)
  , start_srv(nh.serviceClient<std_srvs::Empty>("start"))
  , stop_srv(nh.serviceClient<std_srvs::Empty>("stop"))
  {
  }

  ~DiffDriveControllerTest()
  {
    odom_sub.shutdown();
    cmd_vel_limited_sub.shutdown();
    state_sub.shutdown();
  }

  nav_msgs::Odometry getLastOdom(){ return last_odom; }
  geometry_msgs::TwistStamped getLastCmdVelLimited(){ return last_cmd_vel_limited; }
  diff_drive_controller::DiffDriveControllerState getLastState(){ return last_states[0]; }
  std::vector<diff_drive_controller::DiffDriveControllerState> getLastStates(){ return last_states; }
  void publish(geometry_msgs::Twist cmd_vel){ cmd_pub.publish(cmd_vel); }
  bool isControllerAlive(){ return (odom_sub.getNumPublishers() > 0) && (cmd_pub.getNumSubscribers() > 0); }

  void start(){ std_srvs::Empty srv; start_srv.call(srv); }
  void stop(){ std_srvs::Empty srv; stop_srv.call(srv); }

  void goToYaw(double target,
      double tolerance = ORIENTATION_TOLERANCE,
      double control_frequency = 10.0)
  {
    // Setup PID controller with gains: K_p, K_i, K_d, i_max, i_min
    control_toolbox::Pid pid(1.0, 0.1, 0.0, 1.0, -1.0);

    // Control loop:
    ros::Rate r(control_frequency);
    const ros::Duration dt = r.expectedCycleTime();
    while (true)
    {
      // Retrieve current state:
      const nav_msgs::Odometry odom = getLastOdom();
      const double yaw = tf::getYaw(odom.pose.pose.orientation);

      // Compute error wrt target:
      const double error = angles::shortest_angular_distance(yaw, target);

      // Exit when target reach up to some tolerance:
      if (error < tolerance)
      {
        break;
      }

      // Command new velocity control signal using the PID controller:
      geometry_msgs::Twist cmd_vel;
      cmd_vel.angular.z = pid.computeCommand(error, dt);
      publish(cmd_vel);

      r.sleep();
    }

    // Zero out (stop) velocity control signal:
    geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = 0.0;
    publish(cmd_vel);
  }

private:
  ros::NodeHandle nh;
  ros::Publisher cmd_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber cmd_vel_limited_sub;
  ros::Subscriber state_sub;
  nav_msgs::Odometry last_odom;
  geometry_msgs::TwistStamped last_cmd_vel_limited;
  std::vector<diff_drive_controller::DiffDriveControllerState> last_states;

  ros::ServiceClient start_srv;
  ros::ServiceClient stop_srv;

  void odomCallback(const nav_msgs::Odometry& odom)
  {
    ROS_INFO_STREAM("Odometry callback reveived: pos.x: " << odom.pose.pose.position.x
                     << ", orient.z: " << odom.pose.pose.orientation.z
                     << ", lin_est: " << odom.twist.twist.linear.x
                     << ", ang_est: " << odom.twist.twist.angular.z);
    last_odom = odom;
  }

  void cmdVelLimitedCallback(const geometry_msgs::TwistStamped& twist)
  {
    ROS_INFO_STREAM("Twist callback reveived: linear: " << twist.twist.linear.x
                     << ", angular: " << twist.twist.angular.z);
    last_cmd_vel_limited = twist;
  }

  void diffDriveControllerStateCallback(const diff_drive_controller::DiffDriveControllerState& msg)
  {
    ROS_INFO("Joint trajectory controller state callback reveived");

    last_states[2] = last_states[1];
    last_states[1] = last_states[0];
    last_states[0] = msg;
  }

};

inline tf::Quaternion tfQuatFromGeomQuat(const geometry_msgs::Quaternion& quat)
{
  return tf::Quaternion(quat.x, quat.y, quat.z, quat.w);
}

void propagate(double& x, double& y, double& yaw,
    double v_x, double v_y, double v_yaw, double dt)
{
  const double sin_yaw = std::sin(yaw);
  const double cos_yaw = std::cos(yaw);

  // (x, y) = (x, y) + R(yaw) * (v_x, v_y) * dt
  x += (v_x * cos_yaw - v_y * sin_yaw) * dt;
  y += (v_x * sin_yaw + v_y * cos_yaw) * dt;

  // yaw = yaw + v_yaw * dt (normalized)
  yaw += v_yaw * dt;
  yaw = angles::normalize_angle(yaw);
}
