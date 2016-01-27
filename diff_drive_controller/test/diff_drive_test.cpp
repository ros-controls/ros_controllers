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
/// \author Enrique Fernandez

#include "test_common.h"

#include <diff_drive_controller/covariance.h>

#include <tf/transform_listener.h>

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
  EXPECT_LT(fabs(new_odom.pose.pose.position.x - old_odom.pose.pose.position.x), EPS);
  EXPECT_LT(fabs(new_odom.pose.pose.position.y - old_odom.pose.pose.position.y), EPS);
  EXPECT_LT(fabs(new_odom.pose.pose.position.z - old_odom.pose.pose.position.z), EPS);

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old, pitch_old, yaw_old);
  tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_LT(fabs(roll_new - roll_old), EPS);
  EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
  EXPECT_LT(fabs(yaw_new - yaw_old), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.x), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.y), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

  EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z), EPS);

  // propagation
  double x = old_odom.pose.pose.position.x;
  double y = old_odom.pose.pose.position.y;
  double yaw = tf::getYaw(old_odom.pose.pose.orientation);

  const double v_x = new_odom.twist.twist.linear.x;
  const double v_y = new_odom.twist.twist.linear.y;
  const double v_yaw = new_odom.twist.twist.angular.z;

  const double dt = (new_odom.header.stamp - old_odom.header.stamp).toSec();
  propagate(x, y, yaw, v_x, v_y, v_yaw, dt);

  EXPECT_LT(std::abs(x - new_odom.pose.pose.position.x), EPS);
  EXPECT_LT(std::abs(y - new_odom.pose.pose.position.y), EPS);
  EXPECT_LT(angles::shortest_angular_distance(yaw,
        tf::getYaw(new_odom.pose.pose.orientation)), EPS);

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
  const Eigen::IOFormat HeavyFmt(
      Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

  typedef Eigen::SelfAdjointEigenSolver<PoseCovariance> PoseEigenSolver;
  typedef Eigen::SelfAdjointEigenSolver<TwistCovariance> TwistEigenSolver;

  PoseEigenSolver pose_eigensolver(pose_covariance);
  TwistEigenSolver twist_eigensolver(twist_covariance);

  EXPECT_TRUE(isSymmetric(pose_covariance))
    << "Pose covariance =\n" << pose_covariance.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(pose_covariance, no_tag()))
    << "Pose covariance =\n" << pose_covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << pose_eigensolver.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(pose_covariance),
      POSE_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Pose covariance =\n" << pose_covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << pose_eigensolver.eigenvalues().transpose().format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(twist_covariance))
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(twist_covariance, no_tag()))
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << twist_eigensolver.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(pose_covariance),
      TWIST_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << twist_eigensolver.eigenvalues().transpose().format(HeavyFmt);

  // when the robot doesn't move the twist covariance should be exactly the
  // minimum twist covariance, which is defined as a diagonal covariance matrix
  // like this:
  //   Odometry::DEFAULT_MINIMUM_TWIST_COVARIANCE * TwistCovariance::Identity()
  // where Odometry::DEFAULT_MINIMUM_TWIST_COVARIANCE == 1e-9
  TwistCovariance minimum_twist_covariance = 1e-9 * TwistCovariance::Identity();
  EXPECT_TRUE(((twist_covariance - minimum_twist_covariance).array() == 0).all())
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt);
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

  // propagation
  double x = old_odom.pose.pose.position.x;
  double y = old_odom.pose.pose.position.y;
  double yaw = tf::getYaw(old_odom.pose.pose.orientation);

  const double v_x = new_odom.twist.twist.linear.x;
  const double v_y = new_odom.twist.twist.linear.y;
  const double v_yaw = new_odom.twist.twist.angular.z;

  const double dt = (new_odom.header.stamp - old_odom.header.stamp).toSec();
  propagate(x, y, yaw, v_x, v_y, v_yaw, dt);

  EXPECT_LT(std::abs(x - new_odom.pose.pose.position.x), EPS);
  EXPECT_LT(std::abs(y - new_odom.pose.pose.position.y), EPS);
  EXPECT_LT(angles::shortest_angular_distance(yaw,
        tf::getYaw(new_odom.pose.pose.orientation)), EPS);

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
  const Eigen::IOFormat HeavyFmt(
      Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

  typedef Eigen::SelfAdjointEigenSolver<PoseCovariance> PoseEigenSolver;
  typedef Eigen::SelfAdjointEigenSolver<TwistCovariance> TwistEigenSolver;

  PoseEigenSolver pose_eigensolver(pose_covariance);
  TwistEigenSolver twist_eigensolver(twist_covariance);

  EXPECT_TRUE(isSymmetric(pose_covariance))
    << "Pose covariance =\n" << pose_covariance.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(pose_covariance, no_tag()))
    << "Pose covariance =\n" << pose_covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << pose_eigensolver.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(pose_covariance),
      POSE_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Pose covariance =\n" << pose_covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << pose_eigensolver.eigenvalues().transpose().format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(twist_covariance))
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(twist_covariance, no_tag()))
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << twist_eigensolver.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(pose_covariance),
      TWIST_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << twist_eigensolver.eigenvalues().transpose().format(HeavyFmt);
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
  EXPECT_LT(fabs(new_odom.pose.pose.position.x - old_odom.pose.pose.position.x), EPS);
  EXPECT_LT(fabs(new_odom.pose.pose.position.y - old_odom.pose.pose.position.y), EPS);
  EXPECT_LT(fabs(new_odom.pose.pose.position.z - old_odom.pose.pose.position.z), EPS);

  // convert to rpy and test that way
  double roll_old, pitch_old, yaw_old;
  double roll_new, pitch_new, yaw_new;
  tf::Matrix3x3(tfQuatFromGeomQuat(old_odom.pose.pose.orientation)).getRPY(roll_old, pitch_old, yaw_old);
  tf::Matrix3x3(tfQuatFromGeomQuat(new_odom.pose.pose.orientation)).getRPY(roll_new, pitch_new, yaw_new);
  EXPECT_LT(fabs(roll_new - roll_old), EPS);
  EXPECT_LT(fabs(pitch_new - pitch_old), EPS);
  EXPECT_NEAR(fabs(yaw_new - yaw_old), M_PI, ORIENTATION_TOLERANCE);

  EXPECT_LT(fabs(new_odom.twist.twist.linear.x), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.y), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.linear.z), EPS);

  EXPECT_LT(fabs(new_odom.twist.twist.angular.x), EPS);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.y), EPS);
  EXPECT_NEAR(fabs(new_odom.twist.twist.angular.z), M_PI/10.0, EPS);

  // propagation
  double x = old_odom.pose.pose.position.x;
  double y = old_odom.pose.pose.position.y;
  double yaw = tf::getYaw(old_odom.pose.pose.orientation);

  const double v_x = new_odom.twist.twist.linear.x;
  const double v_y = new_odom.twist.twist.linear.y;
  const double v_yaw = new_odom.twist.twist.angular.z;

  const double dt = (new_odom.header.stamp - old_odom.header.stamp).toSec();
  propagate(x, y, yaw, v_x, v_y, v_yaw, dt);

  EXPECT_LT(std::abs(x - new_odom.pose.pose.position.x), EPS);
  EXPECT_LT(std::abs(y - new_odom.pose.pose.position.y), EPS);
  EXPECT_LT(angles::shortest_angular_distance(yaw,
        tf::getYaw(new_odom.pose.pose.orientation)), EPS);

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
  const Eigen::IOFormat HeavyFmt(
      Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

  typedef Eigen::SelfAdjointEigenSolver<PoseCovariance> PoseEigenSolver;
  typedef Eigen::SelfAdjointEigenSolver<TwistCovariance> TwistEigenSolver;

  PoseEigenSolver pose_eigensolver(pose_covariance);
  TwistEigenSolver twist_eigensolver(twist_covariance);

  EXPECT_TRUE(pose_eigensolver.info() == Eigen::Success);
  EXPECT_TRUE(twist_eigensolver.info() == Eigen::Success);

  EXPECT_TRUE(isSymmetric(pose_covariance))
    << "Pose covariance =\n" << pose_covariance.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(pose_covariance, no_tag()))
    << "Pose covariance =\n" << pose_covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << pose_eigensolver.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(pose_covariance),
      POSE_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Pose covariance =\n" << pose_covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << pose_eigensolver.eigenvalues().transpose().format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(twist_covariance))
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(twist_covariance, no_tag()))
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << twist_eigensolver.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(pose_covariance),
      TWIST_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << twist_eigensolver.eigenvalues().transpose().format(HeavyFmt);
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
