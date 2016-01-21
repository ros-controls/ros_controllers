///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2016, Clearpath Robotics Inc.
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

/// \author Enrique Fernandez

#include <diff_drive_controller/odometry.h>
#include <diff_drive_controller/covariance.h>
#include <diff_drive_controller/rigid_body_motion.h>

#include <gtest/gtest.h>

#include <limits>

#include <cmath>

const double POSE_COVARIANCE_MAX_CONDITION_NUMBER  = 1e8;
const double TWIST_COVARIANCE_MAX_CONDITION_NUMBER = POSE_COVARIANCE_MAX_CONDITION_NUMBER;

const double WHEEL_SEPARATION   = 0.5;
const double LEFT_WHEEL_RADIUS  = 0.1;
const double RIGHT_WHEEL_RADIUS = LEFT_WHEEL_RADIUS;

const double K_L = 0.01;
const double K_R = K_L;
// Instead of a more realistic wheel resolution like the one below, we can also
// use 0.0 to avoid minor errors (see EPS_ constants below), but even with 0.0
// we have them:
const double WHEEL_RESOLUTION = 2.0 * M_PI / (25.0 * 1024);

const size_t CONTROL_STEPS = 100;
const double CONTROL_PERIOD = 0.02;  // [s]
const double POSITION_INCREMENT = 0.02;  // [rad]

const double EPS_INTEGRATE_MOTION_COVARIANCE = std::numeric_limits<double>::epsilon();

/**
 * \brief Setup the odometry params
 * \param[in, out] odometry Odometry
 */
static void setupOdometry(diff_drive_controller::Odometry& odometry)
{
  odometry.setWheelParams(WHEEL_SEPARATION,
      LEFT_WHEEL_RADIUS, RIGHT_WHEEL_RADIUS);

  odometry.setMeasCovarianceParams(K_L, K_R, WHEEL_RESOLUTION);
}

static void moveOdometry(diff_drive_controller::Odometry& odometry,
    double& left_position, double& right_position,
    ros::Time& t,
    const size_t control_steps, const double control_period,
    const double left_position_increment,
    const double right_position_increment)
{
  const double left_velocity  = left_position_increment  / control_period;
  const double right_velocity = right_position_increment / control_period;

  for (size_t i = 0; i < control_steps; ++i)
  {
    left_position  += left_position_increment;
    right_position += right_position_increment;

    odometry.updateCloseLoop(left_position, right_position,
        left_velocity, right_velocity, t);

    t += ros::Duration(control_period);
  }
}

TEST(OdometryTest, testInitial)
{
  // Setup odometry:
  diff_drive_controller::Odometry odometry(1);
  setupOdometry(odometry);

  // Check initial odometry pose and twist is zero:
  EXPECT_EQ(0.0, odometry.getX());
  EXPECT_EQ(0.0, odometry.getY());
  EXPECT_EQ(0.0, odometry.getHeading());

  EXPECT_EQ(0.0, odometry.getVx());
  EXPECT_EQ(0.0, odometry.getVy());
  EXPECT_EQ(0.0, odometry.getVyaw());

  // Check initial odometry pose and twist covariance are valid:
  using namespace diff_drive_controller;

  typedef Odometry::PoseCovariance PoseCovariance;
  typedef Odometry::TwistCovariance TwistCovariance;

  const PoseCovariance  pose_covariance  = odometry.getPoseCovariance();
  const TwistCovariance twist_covariance = odometry.getTwistCovariance();

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
    << "Condition number = " << conditionNumber(pose_covariance) << "\n"
    << "Eigenvalues = " << pose_eigensolver.eigenvalues().transpose().format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(twist_covariance))
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(twist_covariance, no_tag()))
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << twist_eigensolver.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(twist_covariance),
      TWIST_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Twist covariance =\n" << twist_covariance.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(twist_covariance) << "\n"
    << "Eigenvalues = " << twist_eigensolver.eigenvalues().transpose().format(HeavyFmt);

  // Check initial odometry pose and twist covariance are the small expected
  // ones:
  PoseCovariance pose_covariance_expected = PoseCovariance::Identity();
  pose_covariance_expected *= diff_drive_controller::Odometry::DEFAULT_POSE_COVARIANCE;

  const TwistCovariance twist_covariance_expected = odometry.getMinimumTwistCovariance();

  EXPECT_TRUE(((pose_covariance_expected - pose_covariance).array() == 0.0).all())
    << "Pose covariance actual =\n" << pose_covariance.format(HeavyFmt)
    << "\nPose covariance expected =\n" << pose_covariance_expected.format(HeavyFmt);
  EXPECT_TRUE(((twist_covariance_expected - twist_covariance).array() == 0.0).all())
    << "Twist covariance actual =\n" << twist_covariance.format(HeavyFmt)
    << "\nTwist covariance expected =\n" << twist_covariance_expected.format(HeavyFmt);
}

TEST(OdometryTest, testIntegrateMotionNoMoveFromInitial)
{
  // Setup odometry:
  using namespace diff_drive_controller;

  typedef Odometry::PoseCovariance PoseCovariance;
  typedef Odometry::TwistCovariance TwistCovariance;

  diff_drive_controller::Odometry odometry(1);
  setupOdometry(odometry);

  // Set wheel resolution to 0.0, because when it's != 0.0 the covariance
  // always grows a little on every single step, even if the robot doesn't move:
  odometry.setMeasCovarianceParams(K_L, K_R, 0.0);

  // Save initial/current pose and twist state and covariance:
  const double x_0   = odometry.getX();
  const double y_0   = odometry.getY();
  const double yaw_0 = odometry.getHeading();

  const double v_x_0   = odometry.getVx();
  const double v_y_0   = odometry.getVy();
  const double v_yaw_0 = odometry.getVyaw();

  const PoseCovariance  pose_covariance_0  = odometry.getPoseCovariance();
  const TwistCovariance twist_covariance_0 = odometry.getTwistCovariance();

  // Update the odometry moving the wheels forward:
  double left_position  = 0.0;
  double right_position = 0.0;

  const ros::Time t0(0.0);
  ros::Time t(t0);
  moveOdometry(odometry, left_position, right_position, t,
      CONTROL_STEPS, CONTROL_PERIOD, 0.0, 0.0);

  // Update the twist (from the internal incremental odometry pose):
  EXPECT_TRUE(odometry.updateTwist(t));

  // Retrieve new/current pose and twist state and covariance:
  const double x_1   = odometry.getX();
  const double y_1   = odometry.getY();
  const double yaw_1 = odometry.getHeading();

  const double v_x_1   = odometry.getVx();
  const double v_y_1   = odometry.getVy();
  const double v_yaw_1 = odometry.getVyaw();

  const PoseCovariance  pose_covariance_1  = odometry.getPoseCovariance();
  const TwistCovariance twist_covariance_1 = odometry.getTwistCovariance();

  // Correct the twist covariance removing the minimum twist covariance, which
  // is always added to it to avoid ill-conditioned covariance matrices.
  // We need to do this to obtain the correct expected pose covariance,
  // because this one doesn't include the minimum twist covariance.
  // We could also set the minimum twist covariance to zero, but that would
  // generate a zero covariance matrix when the robot doesn't move (as in this
  // test):
  const TwistCovariance twist_covariance_1_corrected =
      twist_covariance_1 - odometry.getMinimumTwistCovariance();

  // Integrate motion:
  const double dt = (t - t0).toSec();
  double x   = x_0;
  double y   = y_0;
  double yaw = yaw_0;
  Eigen::Matrix3d J_pose, J_twist;
  diff_drive_controller::integrate_motion(x, y, yaw,
      v_x_1, v_y_1, v_yaw_1,
      dt,
      J_pose, J_twist);

  // Propagate covariance:
  PoseCovariance pose_covariance_1_expected =
    J_pose * pose_covariance_0 * J_pose.transpose() +
    J_twist * twist_covariance_1_corrected * J_twist.transpose();

  // Check odometry pose and twist is zero (no move):
  EXPECT_EQ(0.0, x_1);
  EXPECT_EQ(0.0, y_1);
  EXPECT_EQ(0.0, yaw_1);

  EXPECT_EQ(0.0, v_x_1);
  EXPECT_EQ(0.0, v_y_1);
  EXPECT_EQ(0.0, v_yaw_1);

  // Check new pose is equal to the expected one:
  EXPECT_EQ(x_1, x);
  EXPECT_EQ(y_1, y);
  EXPECT_EQ(yaw_1, yaw);

  // Check all pose and twist covariances are valid:
  const Eigen::IOFormat HeavyFmt(
      Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

  typedef Eigen::SelfAdjointEigenSolver<PoseCovariance> PoseEigenSolver;
  typedef Eigen::SelfAdjointEigenSolver<TwistCovariance> TwistEigenSolver;

  PoseEigenSolver pose_eigensolver_0(pose_covariance_0);
  TwistEigenSolver twist_eigensolver_0(twist_covariance_0);

  PoseEigenSolver pose_eigensolver_1(pose_covariance_1);
  TwistEigenSolver twist_eigensolver_1(twist_covariance_1);

  EXPECT_TRUE(isSymmetric(pose_covariance_0))
    << "Pose covariance =\n" << pose_covariance_0.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(pose_covariance_0, no_tag()))
    << "Pose covariance =\n" << pose_covariance_0.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << pose_eigensolver_0.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(pose_covariance_0),
      POSE_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Pose covariance =\n" << pose_covariance_0.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(pose_covariance_0) << "\n"
    << "Eigenvalues = " << pose_eigensolver_0.eigenvalues().transpose().format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(twist_covariance_0))
    << "Twist covariance =\n" << twist_covariance_0.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(twist_covariance_0, no_tag()))
    << "Twist covariance =\n" << twist_covariance_0.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << twist_eigensolver_0.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(twist_covariance_0),
      TWIST_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Twist covariance =\n" << twist_covariance_0.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(twist_covariance_0) << "\n"
    << "Eigenvalues = " << twist_eigensolver_0.eigenvalues().transpose().format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(pose_covariance_1))
    << "Pose covariance =\n" << pose_covariance_1.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(pose_covariance_1, no_tag()))
    << "Pose covariance =\n" << pose_covariance_1.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << pose_eigensolver_1.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(pose_covariance_1),
      POSE_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Pose covariance =\n" << pose_covariance_1.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(pose_covariance_1) << "\n"
    << "Eigenvalues = " << pose_eigensolver_1.eigenvalues().transpose().format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(twist_covariance_1))
    << "Twist covariance =\n" << twist_covariance_1.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(twist_covariance_1, no_tag()))
    << "Twist covariance =\n" << twist_covariance_1.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << twist_eigensolver_1.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(twist_covariance_1),
      TWIST_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Twist covariance =\n" << twist_covariance_1.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(twist_covariance_1) << "\n"
    << "Eigenvalues = " << twist_eigensolver_1.eigenvalues().transpose().format(HeavyFmt);

  // Check new pose covariance is equal to the expected one:
  EXPECT_TRUE(((pose_covariance_1_expected - pose_covariance_1).array().abs() < EPS_INTEGRATE_MOTION_COVARIANCE).all())
    << "Pose covariance actual =\n" << pose_covariance_1.format(HeavyFmt)
    << "\nPose covariance expected =\n" << pose_covariance_1_expected.format(HeavyFmt);

  // Check initial odometry pose and twist covariance are the small expected
  // ones:
  PoseCovariance pose_covariance_expected = PoseCovariance::Identity();
  pose_covariance_expected *= diff_drive_controller::Odometry::DEFAULT_POSE_COVARIANCE;

  const TwistCovariance twist_covariance_expected = odometry.getMinimumTwistCovariance();

  EXPECT_TRUE(((pose_covariance_expected - pose_covariance_1).array() == 0.0).all())
    << "Pose covariance actual =\n" << pose_covariance_1.format(HeavyFmt)
    << "\nPose covariance expected =\n" << pose_covariance_expected.format(HeavyFmt);
  EXPECT_TRUE(((twist_covariance_expected - twist_covariance_1).array() == 0.0).all())
    << "Twist covariance actual =\n" << twist_covariance_1.format(HeavyFmt)
    << "\nTwist covariance expected =\n" << twist_covariance_expected.format(HeavyFmt);
}

TEST(OdometryTest, testIntegrateMotionForwardFromInitial)
{
  // Setup odometry:
  using namespace diff_drive_controller;

  typedef Odometry::PoseCovariance PoseCovariance;
  typedef Odometry::TwistCovariance TwistCovariance;

  diff_drive_controller::Odometry odometry(1);
  setupOdometry(odometry);

  // Save initial/current pose and twist state and covariance:
  const double x_0   = odometry.getX();
  const double y_0   = odometry.getY();
  const double yaw_0 = odometry.getHeading();

  const double v_x_0   = odometry.getVx();
  const double v_y_0   = odometry.getVy();
  const double v_yaw_0 = odometry.getVyaw();

  const PoseCovariance  pose_covariance_0  = odometry.getPoseCovariance();
  const TwistCovariance twist_covariance_0 = odometry.getTwistCovariance();

  // Update the odometry moving the wheels forward:
  double left_position  = 0.0;
  double right_position = 0.0;

  const ros::Time t0(0.0);
  ros::Time t(t0);
  moveOdometry(odometry, left_position, right_position, t,
      CONTROL_STEPS, CONTROL_PERIOD, POSITION_INCREMENT, POSITION_INCREMENT);

  // Update the twist (from the internal incremental odometry pose):
  EXPECT_TRUE(odometry.updateTwist(t));

  // Retrieve new/current pose and twist state and covariance:
  const double x_1   = odometry.getX();
  const double y_1   = odometry.getY();
  const double yaw_1 = odometry.getHeading();

  const double v_x_1   = odometry.getVx();
  const double v_y_1   = odometry.getVy();
  const double v_yaw_1 = odometry.getVyaw();

  const PoseCovariance  pose_covariance_1  = odometry.getPoseCovariance();
  const TwistCovariance twist_covariance_1 = odometry.getTwistCovariance();

  // Integrate motion:
  const double dt = (t - t0).toSec();
  double x   = x_0;
  double y   = y_0;
  double yaw = yaw_0;
  Eigen::Matrix3d J_pose, J_twist;
  diff_drive_controller::integrate_motion(x, y, yaw,
      v_x_1, v_y_1, v_yaw_1,
      dt,
      J_pose, J_twist);

  // Correct the twist covariance removing the minimum twist covariance, which
  // is always added to it to avoid ill-conditioned covariance matrices.
  // We need to do this to obtain the correct expected pose covariance,
  // because this one doesn't include the minimum twist covariance.
  // We could also set the minimum twist covariance to zero, but that would
  // generate a zero covariance matrix when the robot doesn't move (as in this
  // test):
  const TwistCovariance twist_covariance_1_corrected =
      twist_covariance_1 - odometry.getMinimumTwistCovariance();

  // Propagate covariance:
  PoseCovariance pose_covariance_1_expected =
    J_pose * pose_covariance_0 * J_pose.transpose() +
    J_twist * twist_covariance_1_corrected * J_twist.transpose();

  // Check new pose is equal to the expected one:
  EXPECT_EQ(x_1, x);
  EXPECT_EQ(y_1, y);
  EXPECT_EQ(yaw_1, yaw);

  // Check all pose and twist covariances are valid:
  const Eigen::IOFormat HeavyFmt(
      Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

  typedef Eigen::SelfAdjointEigenSolver<PoseCovariance> PoseEigenSolver;
  typedef Eigen::SelfAdjointEigenSolver<TwistCovariance> TwistEigenSolver;

  PoseEigenSolver pose_eigensolver_0(pose_covariance_0);
  TwistEigenSolver twist_eigensolver_0(twist_covariance_0);

  PoseEigenSolver pose_eigensolver_1(pose_covariance_1);
  TwistEigenSolver twist_eigensolver_1(twist_covariance_1);

  EXPECT_TRUE(isSymmetric(pose_covariance_0))
    << "Pose covariance =\n" << pose_covariance_0.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(pose_covariance_0, no_tag()))
    << "Pose covariance =\n" << pose_covariance_0.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << pose_eigensolver_0.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(pose_covariance_0),
      POSE_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Pose covariance =\n" << pose_covariance_0.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(pose_covariance_0) << "\n"
    << "Eigenvalues = " << pose_eigensolver_0.eigenvalues().transpose().format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(twist_covariance_0))
    << "Twist covariance =\n" << twist_covariance_0.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(twist_covariance_0, no_tag()))
    << "Twist covariance =\n" << twist_covariance_0.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << twist_eigensolver_0.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(twist_covariance_0),
      TWIST_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Twist covariance =\n" << twist_covariance_0.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(twist_covariance_0) << "\n"
    << "Eigenvalues = " << twist_eigensolver_0.eigenvalues().transpose().format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(pose_covariance_1))
    << "Pose covariance =\n" << pose_covariance_1.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(pose_covariance_1, no_tag()))
    << "Pose covariance =\n" << pose_covariance_1.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << pose_eigensolver_1.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(pose_covariance_1),
      POSE_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Pose covariance =\n" << pose_covariance_1.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(pose_covariance_1) << "\n"
    << "Eigenvalues = " << pose_eigensolver_1.eigenvalues().transpose().format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(twist_covariance_1))
    << "Twist covariance =\n" << twist_covariance_1.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(twist_covariance_1, no_tag()))
    << "Twist covariance =\n" << twist_covariance_1.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << twist_eigensolver_1.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(twist_covariance_1),
      TWIST_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Twist covariance =\n" << twist_covariance_1.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(twist_covariance_1) << "\n"
    << "Eigenvalues = " << twist_eigensolver_1.eigenvalues().transpose().format(HeavyFmt);

  // Check new pose covariance is equal to the expected one:
  EXPECT_TRUE(((pose_covariance_1_expected - pose_covariance_1).array().abs() < EPS_INTEGRATE_MOTION_COVARIANCE).all())
    << "Pose covariance actual =\n" << pose_covariance_1.format(HeavyFmt)
    << "\nPose covariance expected =\n" << pose_covariance_1_expected.format(HeavyFmt);
}

TEST(OdometryTest, testIntegrateMotionForwardFromNotInitial)
{
  // Setup odometry:
  using namespace diff_drive_controller;

  typedef Odometry::PoseCovariance PoseCovariance;
  typedef Odometry::TwistCovariance TwistCovariance;

  diff_drive_controller::Odometry odometry(1);
  setupOdometry(odometry);

  // Update the odometry moving the wheels forward and turning:
  double left_position  = 0.0;
  double right_position = 0.0;

  const ros::Time t0(0.0);
  ros::Time t(t0);
  moveOdometry(odometry, left_position, right_position, t,
      CONTROL_STEPS, CONTROL_PERIOD, POSITION_INCREMENT, 2 * POSITION_INCREMENT);

  // Update the twist (from the internal incremental odometry pose):
  EXPECT_TRUE(odometry.updateTwist(t));

  // Save initial/current pose and twist state and covariance:
  const double x_0   = odometry.getX();
  const double y_0   = odometry.getY();
  const double yaw_0 = odometry.getHeading();

  const double v_x_0   = odometry.getVx();
  const double v_y_0   = odometry.getVy();
  const double v_yaw_0 = odometry.getVyaw();

  const PoseCovariance  pose_covariance_0  = odometry.getPoseCovariance();
  const TwistCovariance twist_covariance_0 = odometry.getTwistCovariance();

  // Update the odometry moving the wheels forward:
  const ros::Time t1(t);
  moveOdometry(odometry, left_position, right_position, t,
      CONTROL_STEPS, CONTROL_PERIOD, POSITION_INCREMENT, POSITION_INCREMENT);

  // Update the twist (from the internal incremental odometry pose):
  EXPECT_TRUE(odometry.updateTwist(t));

  // Retrieve new/current pose and twist state and covariance:
  const double x_1   = odometry.getX();
  const double y_1   = odometry.getY();
  const double yaw_1 = odometry.getHeading();

  const double v_x_1   = odometry.getVx();
  const double v_y_1   = odometry.getVy();
  const double v_yaw_1 = odometry.getVyaw();

  const PoseCovariance  pose_covariance_1  = odometry.getPoseCovariance();
  const TwistCovariance twist_covariance_1 = odometry.getTwistCovariance();

  // Integrate motion:
  const double dt = (t - t1).toSec();
  double x   = x_0;
  double y   = y_0;
  double yaw = yaw_0;
  Eigen::Matrix3d J_pose, J_twist;
  diff_drive_controller::integrate_motion(x, y, yaw,
      v_x_1, v_y_1, v_yaw_1,
      dt,
      J_pose, J_twist);

  // Correct the twist covariance removing the minimum twist covariance, which
  // is always added to it to avoid ill-conditioned covariance matrices.
  // We need to do this to obtain the correct expected pose covariance,
  // because this one doesn't include the minimum twist covariance.
  // We could also set the minimum twist covariance to zero, but that would
  // generate a zero covariance matrix when the robot doesn't move (as in this
  // test):
  const TwistCovariance twist_covariance_1_corrected =
      twist_covariance_1 - odometry.getMinimumTwistCovariance();

  // Propagate covariance:
  PoseCovariance pose_covariance_1_expected =
    J_pose * pose_covariance_0 * J_pose.transpose() +
    J_twist * twist_covariance_1_corrected * J_twist.transpose();

  // Check new pose is equal to the expected one:
  // Note that at this point the pose is not computed using the (internal)
  // incremental pose, so we have an error greater than the double eps!
  EXPECT_NEAR(x_1, x, 1e-14);
  EXPECT_NEAR(y_1, y, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(yaw_1, yaw, 1e-14);

  // Check all pose and twist covariances are valid:
  const Eigen::IOFormat HeavyFmt(
      Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

  typedef Eigen::SelfAdjointEigenSolver<PoseCovariance> PoseEigenSolver;
  typedef Eigen::SelfAdjointEigenSolver<TwistCovariance> TwistEigenSolver;

  PoseEigenSolver pose_eigensolver_0(pose_covariance_0);
  TwistEigenSolver twist_eigensolver_0(twist_covariance_0);

  PoseEigenSolver pose_eigensolver_1(pose_covariance_1);
  TwistEigenSolver twist_eigensolver_1(twist_covariance_1);

  EXPECT_TRUE(isSymmetric(pose_covariance_0))
    << "Pose covariance =\n" << pose_covariance_0.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(pose_covariance_0, no_tag()))
    << "Pose covariance =\n" << pose_covariance_0.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << pose_eigensolver_0.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(pose_covariance_0),
      POSE_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Pose covariance =\n" << pose_covariance_0.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(pose_covariance_0) << "\n"
    << "Eigenvalues = " << pose_eigensolver_0.eigenvalues().transpose().format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(twist_covariance_0))
    << "Twist covariance =\n" << twist_covariance_0.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(twist_covariance_0, no_tag()))
    << "Twist covariance =\n" << twist_covariance_0.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << twist_eigensolver_0.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(twist_covariance_0),
      TWIST_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Twist covariance =\n" << twist_covariance_0.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(twist_covariance_0) << "\n"
    << "Eigenvalues = " << twist_eigensolver_0.eigenvalues().transpose().format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(pose_covariance_1))
    << "Pose covariance =\n" << pose_covariance_1.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(pose_covariance_1, no_tag()))
    << "Pose covariance =\n" << pose_covariance_1.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << pose_eigensolver_1.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(pose_covariance_1),
      POSE_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Pose covariance =\n" << pose_covariance_1.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(pose_covariance_1) << "\n"
    << "Eigenvalues = " << pose_eigensolver_1.eigenvalues().transpose().format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(twist_covariance_1))
    << "Twist covariance =\n" << twist_covariance_1.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(twist_covariance_1, no_tag()))
    << "Twist covariance =\n" << twist_covariance_1.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << twist_eigensolver_1.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(twist_covariance_1),
      TWIST_COVARIANCE_MAX_CONDITION_NUMBER)
    << "Twist covariance =\n" << twist_covariance_1.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(twist_covariance_1) << "\n"
    << "Eigenvalues = " << twist_eigensolver_1.eigenvalues().transpose().format(HeavyFmt);

  // Check the initial pose and twist state and covariance aren't zero or the
  // initial ones (since it's been moved before retrieving it):
  EXPECT_NE(0.0, x_0);
  EXPECT_NE(0.0, y_0);
  EXPECT_NE(0.0, yaw_0);

  EXPECT_NE(0.0, v_x_0);
  EXPECT_NE(0.0, v_y_0);
  EXPECT_NE(0.0, v_yaw_0);

  PoseCovariance pose_covariance_expected = PoseCovariance::Identity();
  pose_covariance_expected *= diff_drive_controller::Odometry::DEFAULT_POSE_COVARIANCE;

  const TwistCovariance twist_covariance_expected = odometry.getMinimumTwistCovariance();

  EXPECT_FALSE(((pose_covariance_expected - pose_covariance_0).array() == 0.0).all())
    << "Pose covariance actual =\n" << pose_covariance_0.format(HeavyFmt)
    << "\nPose covariance expected =\n" << pose_covariance_expected.format(HeavyFmt);
  EXPECT_FALSE(((twist_covariance_expected - twist_covariance_0).array() == 0.0).all())
    << "Twist covariance actual =\n" << twist_covariance_0.format(HeavyFmt)
    << "\nTwist covariance expected =\n" << twist_covariance_expected.format(HeavyFmt);

  // Check new pose covariance is equal to the expected one:
  EXPECT_TRUE(((pose_covariance_1_expected - pose_covariance_1).array().abs() < EPS_INTEGRATE_MOTION_COVARIANCE).all())
    << "Pose covariance actual =\n" << pose_covariance_1.format(HeavyFmt)
    << "\nPose covariance expected =\n" << pose_covariance_1_expected.format(HeavyFmt);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
