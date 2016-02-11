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
///////////////////////////////////////////////////////////////////////////////

/// \author Bence Magyar
/// \author Enrique Fernandez

#include "test_common.h"

#include <diff_drive_controller/covariance.h>
#include <diff_drive_controller/rigid_body_motion.h>

#include <tf/transform_listener.h>

#include <limits>

const double POSE_COVARIANCE_MAX_CONDITION_NUMBER  = 1e8;
const double TWIST_COVARIANCE_MAX_CONDITION_NUMBER = POSE_COVARIANCE_MAX_CONDITION_NUMBER;

// TEST CASES
TEST_F(DiffDriveControllerTest, testNoMove)
{
  // wait for ROS
  while (!isControllerAlive())
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
  // don't send any velocity command, so it doesn't move
  // wait for 10s
  ros::Duration(10.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot didn't moved, changes on all fields should be ~~0
  EXPECT_NEAR(old_odom.pose.pose.position.x, new_odom.pose.pose.position.x, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(old_odom.pose.pose.position.y, new_odom.pose.pose.position.y, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(old_odom.pose.pose.position.z, new_odom.pose.pose.position.z, std::numeric_limits<double>::epsilon());

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old, pitch_old, yaw_old);
  tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_NEAR(roll_old , roll_new , std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(pitch_old, pitch_new, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(yaw_old  , yaw_new  , std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, new_odom.twist.twist.linear.x, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, new_odom.twist.twist.linear.y, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, new_odom.twist.twist.linear.z, std::numeric_limits<double>::epsilon());

  EXPECT_NEAR(0.0, new_odom.twist.twist.angular.x, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, new_odom.twist.twist.angular.y, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, new_odom.twist.twist.angular.z, std::numeric_limits<double>::epsilon());

  // propagation
  double x = old_odom.pose.pose.position.x;
  double y = old_odom.pose.pose.position.y;
  double yaw = tf::getYaw(old_odom.pose.pose.orientation);

  const double v_x = new_odom.twist.twist.linear.x;
  const double v_y = new_odom.twist.twist.linear.y;
  const double v_yaw = new_odom.twist.twist.angular.z;

  const double dt = (new_odom.header.stamp - old_odom.header.stamp).toSec();
  propagate(x, y, yaw, v_x, v_y, v_yaw, dt);

  EXPECT_NEAR(new_odom.pose.pose.position.x, x, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(new_odom.pose.pose.position.y, y, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, angles::shortest_angular_distance(yaw,
        tf::getYaw(new_odom.pose.pose.orientation)), std::numeric_limits<double>::epsilon());

  // covariance
  using namespace diff_drive_controller;

  typedef Odometry::PoseCovariance PoseCovariance;
  typedef Odometry::TwistCovariance TwistCovariance;

  PoseCovariance pose_covariance;
  TwistCovariance twist_covariance;

  msgToCovariance(new_odom.pose.covariance, pose_covariance, no_tag());
  msgToCovariance(new_odom.twist.covariance, twist_covariance, no_tag());

  // check we have a positive definite covariance matrix
  // isCovariance = isSymmetric + isPositiveDefinite
  testCovariance(pose_covariance, POSE_COVARIANCE_MAX_CONDITION_NUMBER);
  testCovariance(twist_covariance, TWIST_COVARIANCE_MAX_CONDITION_NUMBER);

  // when the robot doesn't move the twist covariance should be exactly the
  // minimum twist covariance, which is defined as a diagonal covariance matrix
  // like this:
  //   Odometry::DEFAULT_MINIMUM_TWIST_COVARIANCE * TwistCovariance::Identity()
  // where Odometry::DEFAULT_MINIMUM_TWIST_COVARIANCE == 1e-9
  const Eigen::IOFormat HeavyFmt(
      Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

  TwistCovariance minimum_twist_covariance = 1e-9 * TwistCovariance::Identity();
  EXPECT_TRUE(((twist_covariance - minimum_twist_covariance).array() == 0).all())
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt) << "\n"
    << "Minimum twist covariance =\n" << minimum_twist_covariance.format(HeavyFmt);

  // Take the last two odometry messages:
  std::vector<nav_msgs::Odometry> odoms = getLastOdoms();

  EXPECT_GT(odoms.size(), 1);

  // Take pose and twist state and covariance for the current odometry:
  const ros::Time t_0 = odoms[0].header.stamp;

  const double x_0   = odoms[0].pose.pose.position.x;
  const double y_0   = odoms[0].pose.pose.position.y;
  const double yaw_0 = tf::getYaw(odoms[0].pose.pose.orientation);

  const double v_x_0   = odoms[0].twist.twist.linear.x;
  const double v_y_0   = odoms[0].twist.twist.linear.y;
  const double v_yaw_0 = odoms[0].twist.twist.angular.z;

  PoseCovariance  pose_covariance_0;
  msgToCovariance(odoms[0].pose.covariance, pose_covariance_0, no_tag());

  TwistCovariance twist_covariance_0;
  msgToCovariance(odoms[0].twist.covariance, twist_covariance_0, no_tag());

  // Take pose and twist state and covariance for the previous odometry:
  const ros::Time t_1 = odoms[1].header.stamp;

  const double x_1   = odoms[1].pose.pose.position.x;
  const double y_1   = odoms[1].pose.pose.position.y;
  const double yaw_1 = tf::getYaw(odoms[1].pose.pose.orientation);

  PoseCovariance  pose_covariance_1;
  msgToCovariance(odoms[1].pose.covariance, pose_covariance_1, no_tag());

  TwistCovariance twist_covariance_1;
  msgToCovariance(odoms[1].twist.covariance, twist_covariance_1, no_tag());

  // Integrate motion:
  const double dt_odoms = (t_0 - t_1).toSec();
  x   = x_1;
  y   = y_1;
  yaw = yaw_1;
  Eigen::Matrix3d J_pose, J_twist;
  diff_drive_controller::integrate_motion(x, y, yaw,
      v_x_0, v_y_0, v_yaw_0,
      dt_odoms,
      J_pose, J_twist);

  // Correct the twist covariance removing the minimum twist covariance, which
  // is always added to it to avoid ill-conditioned covariance matrices.
  // We need to do this to obtain the correct expected pose covariance,
  // because this one doesn't include the minimum twist covariance.
  // We could also set the minimum twist covariance to zero, but that would
  // generate a zero covariance matrix when the robot doesn't move (as in this
  // test):
  const TwistCovariance twist_covariance_0_corrected =
      twist_covariance_0 - 1e-9 * TwistCovariance::Identity();

  // Propagate covariance:
  PoseCovariance pose_covariance_0_expected =
    J_pose * pose_covariance_1 * J_pose.transpose() +
    J_twist * twist_covariance_0_corrected * J_twist.transpose();

  // Check new pose is equal to the expected one:
  EXPECT_EQ(x_0, x);
  EXPECT_EQ(y_0, y);
  EXPECT_EQ(yaw_0, yaw);

  // Check new pose covariance is equal to the expected one:
  EXPECT_TRUE(((pose_covariance_0_expected - pose_covariance_0).array().abs() < std::numeric_limits<double>::epsilon()).all())
    << "Pose covariance actual =\n" << pose_covariance_0.format(HeavyFmt)
    << "\nPose covariance expected =\n" << pose_covariance_0_expected.format(HeavyFmt);
}

TEST_F(DiffDriveControllerTest, testForward)
{
  // wait for ROS
  while (!isControllerAlive())
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
  EXPECT_NEAR(0.0, dz, std::numeric_limits<double>::epsilon());

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old, pitch_old, yaw_old);
  tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_NEAR(roll_old , roll_new , std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(pitch_old, pitch_new, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(yaw_old  , yaw_new  , std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(cmd_vel.linear.x, new_odom.twist.twist.linear.x, EPS);
  EXPECT_NEAR(0.0, new_odom.twist.twist.linear.y, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, new_odom.twist.twist.linear.z, std::numeric_limits<double>::epsilon());

  EXPECT_NEAR(0.0, new_odom.twist.twist.angular.x, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, new_odom.twist.twist.angular.y, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, new_odom.twist.twist.angular.z, std::numeric_limits<double>::epsilon());

  // propagation
  double x = old_odom.pose.pose.position.x;
  double y = old_odom.pose.pose.position.y;
  double yaw = tf::getYaw(old_odom.pose.pose.orientation);

  const double v_x = new_odom.twist.twist.linear.x;
  const double v_y = new_odom.twist.twist.linear.y;
  const double v_yaw = new_odom.twist.twist.angular.z;

  const double dt = (new_odom.header.stamp - old_odom.header.stamp).toSec();
  propagate(x, y, yaw, v_x, v_y, v_yaw, dt);

  EXPECT_NEAR(new_odom.pose.pose.position.x, x, EPS);
  EXPECT_NEAR(new_odom.pose.pose.position.y, y, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, angles::shortest_angular_distance(yaw,
        tf::getYaw(new_odom.pose.pose.orientation)), std::numeric_limits<double>::epsilon());

  // covariance
  using namespace diff_drive_controller;

  typedef Odometry::PoseCovariance PoseCovariance;
  typedef Odometry::TwistCovariance TwistCovariance;

  PoseCovariance pose_covariance;
  TwistCovariance twist_covariance;

  msgToCovariance(new_odom.pose.covariance, pose_covariance, no_tag());
  msgToCovariance(new_odom.twist.covariance, twist_covariance, no_tag());

  // check we have a positive definite covariance matrix
  // isCovariance = isSymmetric + isPositiveDefinite
  testCovariance(pose_covariance, POSE_COVARIANCE_MAX_CONDITION_NUMBER);
  testCovariance(twist_covariance, TWIST_COVARIANCE_MAX_CONDITION_NUMBER);

  // Take the last two odometry messages:
  std::vector<nav_msgs::Odometry> odoms = getLastOdoms();

  EXPECT_GT(odoms.size(), 1);

  // Take pose and twist state and covariance for the current odometry:
  const ros::Time t_0 = odoms[0].header.stamp;

  const double x_0   = odoms[0].pose.pose.position.x;
  const double y_0   = odoms[0].pose.pose.position.y;
  const double yaw_0 = tf::getYaw(odoms[0].pose.pose.orientation);

  const double v_x_0   = odoms[0].twist.twist.linear.x;
  const double v_y_0   = odoms[0].twist.twist.linear.y;
  const double v_yaw_0 = odoms[0].twist.twist.angular.z;

  PoseCovariance  pose_covariance_0;
  msgToCovariance(odoms[0].pose.covariance, pose_covariance_0, no_tag());

  TwistCovariance twist_covariance_0;
  msgToCovariance(odoms[0].twist.covariance, twist_covariance_0, no_tag());

  // Take pose and twist state and covariance for the previous odometry:
  const ros::Time t_1 = odoms[1].header.stamp;

  const double x_1   = odoms[1].pose.pose.position.x;
  const double y_1   = odoms[1].pose.pose.position.y;
  const double yaw_1 = tf::getYaw(odoms[1].pose.pose.orientation);

  PoseCovariance  pose_covariance_1;
  msgToCovariance(odoms[1].pose.covariance, pose_covariance_1, no_tag());

  TwistCovariance twist_covariance_1;
  msgToCovariance(odoms[1].twist.covariance, twist_covariance_1, no_tag());

  // Integrate motion:
  const double dt_odoms = (t_0 - t_1).toSec();
  x   = x_1;
  y   = y_1;
  yaw = yaw_1;
  Eigen::Matrix3d J_pose, J_twist;
  diff_drive_controller::integrate_motion(x, y, yaw,
      v_x_0, v_y_0, v_yaw_0,
      dt_odoms,
      J_pose, J_twist);

  // Correct the twist covariance removing the minimum twist covariance, which
  // is always added to it to avoid ill-conditioned covariance matrices.
  // We need to do this to obtain the correct expected pose covariance,
  // because this one doesn't include the minimum twist covariance.
  // We could also set the minimum twist covariance to zero, but that would
  // generate a zero covariance matrix when the robot doesn't move (as in this
  // test):
  const TwistCovariance twist_covariance_0_corrected =
      twist_covariance_0 - 1e-9 * TwistCovariance::Identity();

  // Propagate covariance:
  PoseCovariance pose_covariance_0_expected =
    J_pose * pose_covariance_1 * J_pose.transpose() +
    J_twist * twist_covariance_0_corrected * J_twist.transpose();

  // Check new pose is equal to the expected one:
  EXPECT_NEAR(x_0, x, POSITION_TOLERANCE);
  EXPECT_NEAR(y_0, y, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(yaw_0, yaw, std::numeric_limits<double>::epsilon());

  // Check new pose covariance is equal to the expected one:
  const Eigen::IOFormat HeavyFmt(
      Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

  EXPECT_TRUE(((pose_covariance_0_expected - pose_covariance_0).array().abs() < 1e-5).all())
    << "Pose covariance actual =\n" << pose_covariance_0.format(HeavyFmt)
    << "\nPose covariance expected =\n" << pose_covariance_0_expected.format(HeavyFmt);
}

TEST_F(DiffDriveControllerTest, testTurn)
{
  // wait for ROS
  while (!isControllerAlive())
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
  publish(cmd_vel);
  // wait for 10s
  ros::Duration(10.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // check if the robot rotated PI around z, changes in the other fields should be ~~0
  EXPECT_NEAR(old_odom.pose.pose.position.x, new_odom.pose.pose.position.x, std::numeric_limits<double>::epsilon());
  // Cannot use std::numeric_limits<double>::epsilon() in the next check because
  // it's to strict for the test_diff_drive_multipliers rostest:
  EXPECT_NEAR(old_odom.pose.pose.position.y, new_odom.pose.pose.position.y, 1e-13);
  EXPECT_NEAR(old_odom.pose.pose.position.z, new_odom.pose.pose.position.z, std::numeric_limits<double>::epsilon());

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old, pitch_old, yaw_old);
  tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_NEAR(roll_old , roll_new , std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(pitch_old, pitch_new, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(M_PI, fabs(yaw_new - yaw_old), ORIENTATION_TOLERANCE);

  // Cannot use std::numeric_limits<double>::epsilon() in the next check because
  // it's to strict for the test_diff_drive_multipliers rostest:
  EXPECT_NEAR(0.0, new_odom.twist.twist.linear.x, 1e-12);
  EXPECT_NEAR(0.0, new_odom.twist.twist.linear.y, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, new_odom.twist.twist.linear.z, std::numeric_limits<double>::epsilon());

  EXPECT_NEAR(0.0, new_odom.twist.twist.angular.x, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(0.0, new_odom.twist.twist.angular.y, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(M_PI/10.0, new_odom.twist.twist.angular.z, EPS);

  // propagation
  double x = old_odom.pose.pose.position.x;
  double y = old_odom.pose.pose.position.y;
  double yaw = tf::getYaw(old_odom.pose.pose.orientation);

  const double v_x = new_odom.twist.twist.linear.x;
  const double v_y = new_odom.twist.twist.linear.y;
  const double v_yaw = new_odom.twist.twist.angular.z;

  const double dt = (new_odom.header.stamp - old_odom.header.stamp).toSec();
  propagate(x, y, yaw, v_x, v_y, v_yaw, dt);

  EXPECT_NEAR(new_odom.pose.pose.position.x, x, EPS);
  // Cannot use std::numeric_limits<double>::epsilon() in the next check because
  // it's to strict for the test_diff_drive_multipliers rostest:
  EXPECT_NEAR(new_odom.pose.pose.position.y, y, 1e-13);
  EXPECT_NEAR(0.0, angles::shortest_angular_distance(yaw,
        tf::getYaw(new_odom.pose.pose.orientation)), ORIENTATION_TOLERANCE);

  // covariance
  using namespace diff_drive_controller;

  typedef Odometry::PoseCovariance PoseCovariance;
  typedef Odometry::TwistCovariance TwistCovariance;

  PoseCovariance pose_covariance;
  TwistCovariance twist_covariance;

  msgToCovariance(new_odom.pose.covariance, pose_covariance, no_tag());
  msgToCovariance(new_odom.twist.covariance, twist_covariance, no_tag());

  // check we have a positive definite covariance matrix
  // isCovariance = isSymmetric + isPositiveDefinite
  testCovariance(pose_covariance, POSE_COVARIANCE_MAX_CONDITION_NUMBER);
  testCovariance(twist_covariance, TWIST_COVARIANCE_MAX_CONDITION_NUMBER);

  // Take the last two odometry messages:
  std::vector<nav_msgs::Odometry> odoms = getLastOdoms();

  EXPECT_GT(odoms.size(), 1);

  // Take pose and twist state and covariance for the current odometry:
  const ros::Time t_0 = odoms[0].header.stamp;

  const double x_0   = odoms[0].pose.pose.position.x;
  const double y_0   = odoms[0].pose.pose.position.y;
  const double yaw_0 = tf::getYaw(odoms[0].pose.pose.orientation);

  const double v_x_0   = odoms[0].twist.twist.linear.x;
  const double v_y_0   = odoms[0].twist.twist.linear.y;
  const double v_yaw_0 = odoms[0].twist.twist.angular.z;

  PoseCovariance  pose_covariance_0;
  msgToCovariance(odoms[0].pose.covariance, pose_covariance_0, no_tag());

  TwistCovariance twist_covariance_0;
  msgToCovariance(odoms[0].twist.covariance, twist_covariance_0, no_tag());

  // Take pose and twist state and covariance for the previous odometry:
  const ros::Time t_1 = odoms[1].header.stamp;

  const double x_1   = odoms[1].pose.pose.position.x;
  const double y_1   = odoms[1].pose.pose.position.y;
  const double yaw_1 = tf::getYaw(odoms[1].pose.pose.orientation);

  PoseCovariance  pose_covariance_1;
  msgToCovariance(odoms[1].pose.covariance, pose_covariance_1, no_tag());

  TwistCovariance twist_covariance_1;
  msgToCovariance(odoms[1].twist.covariance, twist_covariance_1, no_tag());

  // Integrate motion:
  const double dt_odoms = (t_0 - t_1).toSec();
  x   = x_1;
  y   = y_1;
  yaw = yaw_1;
  Eigen::Matrix3d J_pose, J_twist;
  diff_drive_controller::integrate_motion(x, y, yaw,
      v_x_0, v_y_0, v_yaw_0,
      dt_odoms,
      J_pose, J_twist);

  // Correct the twist covariance removing the minimum twist covariance, which
  // is always added to it to avoid ill-conditioned covariance matrices.
  // We need to do this to obtain the correct expected pose covariance,
  // because this one doesn't include the minimum twist covariance.
  // We could also set the minimum twist covariance to zero, but that would
  // generate a zero covariance matrix when the robot doesn't move (as in this
  // test):
  const TwistCovariance twist_covariance_0_corrected =
      twist_covariance_0 - 1e-9 * TwistCovariance::Identity();

  // Propagate covariance:
  PoseCovariance pose_covariance_0_expected =
    J_pose * pose_covariance_1 * J_pose.transpose() +
    J_twist * twist_covariance_0_corrected * J_twist.transpose();

  // Check new pose is equal to the expected one:
  EXPECT_NEAR(x_0, x, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(y_0, y, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(yaw_0, yaw, ORIENTATION_TOLERANCE);

  // Check new pose covariance is equal to the expected one:
  const Eigen::IOFormat HeavyFmt(
      Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

  EXPECT_TRUE(((pose_covariance_0_expected - pose_covariance_0).array().abs() < 1e-5).all())
    << "Pose covariance actual =\n" << pose_covariance_0.format(HeavyFmt)
    << "\nPose covariance expected =\n" << pose_covariance_0_expected.format(HeavyFmt);
}

TEST_F(DiffDriveControllerTest, testMoveX)
{
  // wait for ROS
  while (!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }
  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  ros::Duration(0.1).sleep();

  // make yaw 0, so then we can move forward along x-axis
  goToYaw(0.0);

  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a velocity command
  cmd_vel.linear.x = 1.0;
  publish(cmd_vel);
  // wait for 10s
  ros::Duration(10.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // covariance
  using namespace diff_drive_controller;

  typedef Odometry::PoseCovariance PoseCovariance;

  PoseCovariance old_pose_covariance, new_pose_covariance;

  msgToCovariance(old_odom.pose.covariance, old_pose_covariance, no_tag());
  msgToCovariance(new_odom.pose.covariance, new_pose_covariance, no_tag());

  const double old_cov_xx = old_pose_covariance(0, 0);
  const double old_cov_yy = old_pose_covariance(1, 1);

  const double new_cov_xx = new_pose_covariance(0, 0);
  const double new_cov_yy = new_pose_covariance(1, 1);

  const double diff_cov_xx = std::abs(new_cov_xx - old_cov_xx);
  const double diff_cov_yy = std::abs(new_cov_yy - old_cov_yy);

  EXPECT_GT(diff_cov_yy, diff_cov_xx);
}

TEST_F(DiffDriveControllerTest, testMoveY)
{
  // wait for ROS
  while (!isControllerAlive())
  {
    ros::Duration(0.1).sleep();
  }
  // zero everything before test
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  publish(cmd_vel);
  ros::Duration(0.1).sleep();

  // make yaw 90 degrees, so then we can move forward along y-axis
  goToYaw(angles::from_degrees(90.0));

  // get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
  // send a velocity command
  cmd_vel.linear.x = 1.0;
  publish(cmd_vel);
  // wait for 10s
  ros::Duration(10.0).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

  // covariance
  using namespace diff_drive_controller;

  typedef Odometry::PoseCovariance PoseCovariance;

  PoseCovariance old_pose_covariance, new_pose_covariance;

  msgToCovariance(old_odom.pose.covariance, old_pose_covariance, no_tag());
  msgToCovariance(new_odom.pose.covariance, new_pose_covariance, no_tag());

  const double old_cov_xx = old_pose_covariance(0, 0);
  const double old_cov_yy = old_pose_covariance(1, 1);

  const double new_cov_xx = new_pose_covariance(0, 0);
  const double new_cov_yy = new_pose_covariance(1, 1);

  const double diff_cov_xx = std::abs(new_cov_xx - old_cov_xx);
  const double diff_cov_yy = std::abs(new_cov_yy - old_cov_yy);

  EXPECT_GT(diff_cov_xx, diff_cov_yy);
}

TEST_F(DiffDriveControllerTest, testOdomFrame)
{
  // wait for ROS
  while (!isControllerAlive())
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
  ros::init(argc, argv, "diff_drive_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
