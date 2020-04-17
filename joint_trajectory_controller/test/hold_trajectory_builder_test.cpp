///////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2020 Pilz GmbH & Co. KG
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#include <vector>

#include <gtest/gtest.h>

#include <actionlib/server/action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>

#include <joint_trajectory_controller/hold_trajectory_builder.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>

#include <realtime_tools/realtime_server_goal_handle.h>

#include <trajectory_interface/quintic_spline_segment.h>

#include "test_common.h"

namespace hold_trajectory_builder_test
{
static constexpr double EPS{1e-9};

template<class SegmentType>
bool statesAlmostEqual(const typename SegmentType::State& state1,
                       const typename SegmentType::State& state2,
                       const double& tolerance=EPS)
{
  using namespace joint_trajectory_controller_tests;
  return vectorsAlmostEqual(state1.position, state2.position, tolerance) &&
         vectorsAlmostEqual(state1.velocity, state2.velocity, tolerance) &&
         vectorsAlmostEqual(state1.acceleration, state2.acceleration, tolerance);
}

using JointHandle = hardware_interface::JointHandle;
using PosVelJointHandle = hardware_interface::PosVelJointHandle;
using PosVelAccJointHandle = hardware_interface::PosVelAccJointHandle;
using JointStateHandle = hardware_interface::JointStateHandle;

using QuinticSplineSegment = trajectory_interface::QuinticSplineSegment<double>;
using Segment = joint_trajectory_controller::JointTrajectorySegment<QuinticSplineSegment>;
using TrajectoryPerJoint = std::vector<Segment>;
using Trajectory = std::vector<TrajectoryPerJoint>;

using GoalHandle = actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle;
using RealTimeServerGoalHandle = realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction>;

/**
 * @brief Provides all known hardware interfaces.
 */
class FakeRobot
{
public:
  FakeRobot()
    : jsh1_("joint1", &pos1_, &vel2_, &eff1_),
      jsh2_("joint2", &pos2_, &vel2_, &eff2_)
  {
  }

  template<typename HardwareInterface>
  std::vector<typename HardwareInterface::ResourceHandleType> getJointHandles();

  void setPositions(const double& pos1, const double& pos2)
  {
    pos1_ = pos1;
    pos2_ = pos2;
  }

private:
  std::vector<JointHandle> getJointCommandHandles()
  {
	  JointHandle joint1(jsh1_, &cmd1_);
	  JointHandle joint2(jsh2_, &cmd2_);
    return {joint1, joint2};
  }

private:
	double pos1_{0.0}, vel1_{0.0}, eff1_{0.0}, cmd1_{0.0}, vel_cmd1_{0.0}, acc_cmd1_{0.0};
	double pos2_{0.0}, vel2_{0.0}, eff2_{0.0}, cmd2_{0.0}, vel_cmd2_{0.0}, acc_cmd2_{0.0};
  JointStateHandle jsh1_, jsh2_;
};

template<>
std::vector<JointHandle> FakeRobot::getJointHandles<hardware_interface::PositionJointInterface>()
{
  return getJointCommandHandles();
}

template<>
std::vector<JointHandle> FakeRobot::getJointHandles<hardware_interface::VelocityJointInterface>()
{
  return getJointCommandHandles();
}

template<>
std::vector<JointHandle> FakeRobot::getJointHandles<hardware_interface::EffortJointInterface>()
{
  return getJointCommandHandles();
}

template<>
std::vector<PosVelJointHandle> FakeRobot::getJointHandles<hardware_interface::PosVelJointInterface>()
{
	PosVelJointHandle joint1(jsh1_, &cmd1_, &vel_cmd1_);
	PosVelJointHandle joint2(jsh2_, &cmd2_, &vel_cmd2_);
  return {joint1, joint2};
}

template<>
std::vector<PosVelAccJointHandle> FakeRobot::getJointHandles<hardware_interface::PosVelAccJointInterface>()
{
	PosVelAccJointHandle joint1(jsh1_, &cmd1_, &vel_cmd1_, &acc_cmd1_);
	PosVelAccJointHandle joint2(jsh2_, &cmd2_, &vel_cmd2_, &acc_cmd2_);
  return {joint1, joint2};
}

template<typename HardwareInterface>
class HoldTrajectoryBuilderTest : public testing::Test
{
protected:
  std::vector<typename HardwareInterface::ResourceHandleType> getJointHandles()
  {
    return robot_->getJointHandles<HardwareInterface>();
  }

  std::shared_ptr<FakeRobot> getRobot()
  {
    return robot_;
  }

private:
  std::shared_ptr<FakeRobot> robot_ {std::make_shared<FakeRobot>()};
};

using HardwareInterfaceTypes = testing::Types<hardware_interface::PositionJointInterface,
                                              hardware_interface::VelocityJointInterface,
                                              hardware_interface::EffortJointInterface,
                                              hardware_interface::PosVelJointInterface,
                                              hardware_interface::PosVelAccJointInterface>;

TYPED_TEST_CASE(HoldTrajectoryBuilderTest, HardwareInterfaceTypes);

TYPED_TEST(HoldTrajectoryBuilderTest, testBuildNoStartTime)
{
  using Builder = joint_trajectory_controller::HoldTrajectoryBuilder<QuinticSplineSegment, TypeParam>;

  const std::vector<typename TypeParam::ResourceHandleType> joints = this->getJointHandles();
  const auto number_of_joints = joints.size();

  Builder builder(joints);

  Trajectory trajectory;
  joint_trajectory_controller_tests::initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_FALSE(builder.buildTrajectory(&trajectory))
		<< "buildTrajectory() success despite start time is not set.";
}

TYPED_TEST(HoldTrajectoryBuilderTest, testBuildIncompleteTrajectory)
{
  using Builder = joint_trajectory_controller::HoldTrajectoryBuilder<QuinticSplineSegment, TypeParam>;

  const std::vector<typename TypeParam::ResourceHandleType> joints = this->getJointHandles();
  const auto number_of_joints = joints.size();
  const double start_time{0.11};

  Builder builder(joints);
	builder.setStartTime(start_time);

  Trajectory trajectory;
  EXPECT_FALSE(builder.buildTrajectory(&trajectory))
		<< "buildTrajectory() success despite trajectory does not have enough TrajectoriesPerJoint.";

  trajectory.resize(joints.size());
  EXPECT_FALSE(builder.buildTrajectory(&trajectory))
		<< "buildTrajectory() success despite trajectory does not have enough segments.";
}

TYPED_TEST(HoldTrajectoryBuilderTest, testBuildSuccess)
{
  using Builder = joint_trajectory_controller::HoldTrajectoryBuilder<QuinticSplineSegment, TypeParam>;

  const std::vector<typename TypeParam::ResourceHandleType> joints = this->getJointHandles();
  const auto number_of_joints = joints.size();
  const double start_time{0.11};

  Builder builder(joints);
	builder.setStartTime(start_time);

  Trajectory trajectory;
  joint_trajectory_controller_tests::initDefaultTrajectory(number_of_joints, trajectory);

  std::vector<double> positions{-48.9, 2.1};  // set some arbitrary positions
  std::shared_ptr<FakeRobot> robot = this->getRobot();
  robot->setPositions(positions.at(0), positions[1]);

  EXPECT_TRUE(builder.buildTrajectory(&trajectory)) << "buildTrajectory() should have been successful.";

  // check built trajectory
  EXPECT_EQ(trajectory.size(), number_of_joints) << "Built trajectory has wrong number of trajectories per joint.";
  for (const auto& jt : trajectory)
  {
    EXPECT_EQ(jt.size(), 1U) << "Unexpected number of trajectory points.";
  }
  EXPECT_NEAR(trajectory.at(0).at(0).startTime(), start_time, EPS) << "Unexpected deviation in start time.";

  // check start and end state
  Segment::State sampled_state{1};
  Segment::State expected_state{1};
  expected_state.velocity.at(0) = 0.0;
  expected_state.acceleration.at(0) = 0.0;
  for (unsigned int i = 0; i < number_of_joints; ++i)
  {
    expected_state.position.at(0) = positions.at(i);

    trajectory.at(i).at(0).sample(trajectory.at(i).at(0).startTime(), sampled_state);
    EXPECT_TRUE(statesAlmostEqual<Segment>(sampled_state, expected_state, EPS))
      << "Start states not equal for joint " << joints.at(i).getName();

    trajectory.at(i).at(0).sample(trajectory.at(i).at(0).endTime(), sampled_state);
    EXPECT_TRUE(statesAlmostEqual<Segment>(sampled_state, expected_state, EPS))
      << "End states not equal for joint " << joints.at(i).getName();
  }
}

TYPED_TEST(HoldTrajectoryBuilderTest, testResetStartTime)
{
  using Builder = joint_trajectory_controller::HoldTrajectoryBuilder<QuinticSplineSegment, TypeParam>;

  const std::vector<typename TypeParam::ResourceHandleType> joints = this->getJointHandles();
  const auto number_of_joints = joints.size();
  const double start_time{0.0};

  Builder builder(joints);
	builder.setStartTime(start_time);

  builder.reset();

  Trajectory trajectory;
  joint_trajectory_controller_tests::initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_FALSE(builder.buildTrajectory(&trajectory))
		<< "buildTrajectory() success despite start time war reset.";
}

/**
 * @note A non-empty goal handle cannot be created without an action server,
 * therefore we check only how the use_count of the shared pointer changes.
 */
TYPED_TEST(HoldTrajectoryBuilderTest, testSetGoalHandle)
{
  using Builder = joint_trajectory_controller::HoldTrajectoryBuilder<QuinticSplineSegment, TypeParam>;

  const std::vector<typename TypeParam::ResourceHandleType> joints = this->getJointHandles();
  const auto number_of_joints = joints.size();
  const double start_time{0.0};
  GoalHandle gh;
  boost::shared_ptr<RealTimeServerGoalHandle> rt_goal_handle = boost::make_shared<RealTimeServerGoalHandle>(gh);

  Builder builder(joints);
	builder.setStartTime(start_time);
  builder.setGoalHandle(rt_goal_handle);

  EXPECT_EQ(rt_goal_handle.use_count(), 1) << "Builder should only store a reference on GoalHandlePtr.";

  Trajectory trajectory;
  joint_trajectory_controller_tests::initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_TRUE(builder.buildTrajectory(&trajectory)) << "buildTrajectory() should have been successful.";

  EXPECT_EQ(rt_goal_handle.use_count(), 1 + joints.size())
    << "Unexpected owner count of the goal handle after building the trajectory.";
}

/**
 * @note A non-empty goal handle cannot be created without an action server,
 * therefore we check only how the use_count of the shared pointer changes.
 */
TYPED_TEST(HoldTrajectoryBuilderTest, testResetGoalHandle)
{
  using Builder = joint_trajectory_controller::HoldTrajectoryBuilder<QuinticSplineSegment, TypeParam>;

  const std::vector<typename TypeParam::ResourceHandleType> joints = this->getJointHandles();
  const auto number_of_joints = joints.size();
  const double start_time{0.0};
  GoalHandle gh;
  boost::shared_ptr<RealTimeServerGoalHandle> rt_goal_handle = boost::make_shared<RealTimeServerGoalHandle>(gh);

  Builder builder(joints);
  builder.setGoalHandle(rt_goal_handle);

  EXPECT_EQ(rt_goal_handle.use_count(), 1) << "Builder should only store a reference on GoalHandlePtr.";

  builder.reset();
	builder.setStartTime(start_time);

  Trajectory trajectory;
  joint_trajectory_controller_tests::initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_TRUE(builder.buildTrajectory(&trajectory)) << "buildTrajectory() should have been successful.";

  EXPECT_EQ(rt_goal_handle.use_count(), 1) << "Modified owner count of the goal handle despite reset.";
}

}  // namespace hold_trajectory_builder_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
