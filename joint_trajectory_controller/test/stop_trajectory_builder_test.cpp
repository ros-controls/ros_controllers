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

#include <gtest/gtest.h>

#include <joint_trajectory_controller/stop_trajectory_builder.h>

#include <trajectory_interface/quintic_spline_segment.h>

#include "test_common.h"

namespace stop_trajectory_builder_test
{
static constexpr double EPS{1e-9};

using QuinticSplineSegment = trajectory_interface::QuinticSplineSegment<double>;
using Segment = joint_trajectory_controller::JointTrajectorySegment<QuinticSplineSegment>;
using TrajectoryPerJoint = std::vector<Segment>;
using Trajectory = std::vector<TrajectoryPerJoint>;

using GoalHandle = actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle;
using RealTimeServerGoalHandle = realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction>;

TEST(StopTrajectoryBuilderTest, testBuildNoStartTime)
{
  using Builder = joint_trajectory_controller::StopTrajectoryBuilder<QuinticSplineSegment>;

	const unsigned int number_of_joints{2};
	const double stop_trajectory_duration{0.2};
  Segment::State hold_state{number_of_joints};
  Builder builder(stop_trajectory_duration, hold_state);

  Trajectory trajectory;
  joint_trajectory_controller_tests::initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_FALSE(builder.buildTrajectory(&trajectory))
		<< "buildTrajectory() success despite start time is not set.";
}

TEST(StopTrajectoryBuilderTest, testBuildIncompleteTrajectory)
{
  using Builder = joint_trajectory_controller::StopTrajectoryBuilder<QuinticSplineSegment>;

	const unsigned int number_of_joints{2};
	const double stop_trajectory_duration{0.2};
  Segment::State hold_state{number_of_joints};
  Builder builder(stop_trajectory_duration, hold_state);

  Trajectory trajectory;
  EXPECT_FALSE(builder.buildTrajectory(&trajectory))
		<< "buildTrajectory() success despite trajectory does not have enough TrajectoriesPerJoint.";

  trajectory.resize(number_of_joints);
  EXPECT_FALSE(builder.buildTrajectory(&trajectory))
		<< "buildTrajectory() success despite trajectory does not have enough segments.";
}

TEST(StopTrajectoryBuilderTest, testBuildSuccess)
{
  using Builder = joint_trajectory_controller::StopTrajectoryBuilder<QuinticSplineSegment>;

	const unsigned int number_of_joints{2};
	const double stop_duration{0.2};
  const double start_time{0.11};
  Segment::State hold_state{number_of_joints};
	hold_state.position.at(0) = 0.9;  // set arbitrary state
	hold_state.velocity.at(0) = -0.03;
	hold_state.position.at(1) = 1.68;
	hold_state.velocity.at(1) = 3.7;
  Builder builder(stop_duration, hold_state);

	builder.setStartTime(start_time);

  Trajectory trajectory;
  joint_trajectory_controller_tests::initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_TRUE(builder.buildTrajectory(&trajectory)) << "buildTrajectory() should have been successful.";

	// check built trajectory
  EXPECT_EQ(trajectory.size(), number_of_joints) << "Built trajectory has wrong number of trajectories per joint.";
  for (const auto& jt : trajectory)
  {
    EXPECT_EQ(jt.size(), 1U) << "Unexpected number of trajectory points.";
  }
  EXPECT_NEAR(trajectory.at(0).at(0).startTime(), start_time, EPS) << "Unexpected deviation in start time.";
	EXPECT_NEAR(trajectory.at(0).at(0).endTime(), start_time + stop_duration, EPS) << "Unexpected deviation in end time.";

  // check start and end state
  Segment::State sampled_state{1};
  for (unsigned int i = 0; i < number_of_joints; ++i)
  {
    trajectory.at(i).at(0).sample(trajectory.at(i).at(0).startTime(), sampled_state);
		EXPECT_NEAR(sampled_state.position.at(0), hold_state.position.at(i), EPS)
      << "Start state positions not equal for joint " << i;
		EXPECT_NEAR(sampled_state.velocity.at(0), hold_state.velocity.at(i), EPS)
      << "Start state velocities not equal for joint " << i;

    trajectory.at(i).at(0).sample(trajectory.at(i).at(0).endTime(), sampled_state);
		EXPECT_NEAR(sampled_state.velocity.at(0), 0.0, EPS) << "End state velocity should be zero.";
	}
}

TEST(StopTrajectoryBuilderTest, testResetStartTime)
{
  using Builder = joint_trajectory_controller::StopTrajectoryBuilder<QuinticSplineSegment>;

	const unsigned int number_of_joints{2};
	const double stop_duration{0.2};
  const double start_time{0.11};
  Segment::State hold_state{number_of_joints};

  Builder builder(stop_duration, hold_state);
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
TEST(StopTrajectoryBuilderTest, testSetGoalHandle)
{
  using Builder = joint_trajectory_controller::StopTrajectoryBuilder<QuinticSplineSegment>;

	const unsigned int number_of_joints{2};
	const double stop_duration{0.2};
  const double start_time{0.11};
  Segment::State hold_state{number_of_joints};
  GoalHandle gh;
  boost::shared_ptr<RealTimeServerGoalHandle> rt_goal_handle = boost::make_shared<RealTimeServerGoalHandle>(gh);

  Builder builder(stop_duration, hold_state);
	builder.setStartTime(start_time);
  builder.setGoalHandle(rt_goal_handle);

  EXPECT_EQ(rt_goal_handle.use_count(), 1) << "Builder should only store a reference on GoalHandlePtr.";

  Trajectory trajectory;
  joint_trajectory_controller_tests::initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_TRUE(builder.buildTrajectory(&trajectory)) << "buildTrajectory() should have been successful.";

  EXPECT_EQ(rt_goal_handle.use_count(), 1 + number_of_joints)
    << "Unexpected owner count of the goal handle after building the trajectory.";
}

/**
 * @note A non-empty goal handle cannot be created without an action server,
 * therefore we check only how the use_count of the shared pointer changes.
 */
TEST(StopTrajectoryBuilderTest, testResetGoalHandle)
{
  using Builder = joint_trajectory_controller::StopTrajectoryBuilder<QuinticSplineSegment>;

	const unsigned int number_of_joints{2};
	const double stop_duration{0.2};
  const double start_time{0.11};
  Segment::State hold_state{number_of_joints};
  GoalHandle gh;
  boost::shared_ptr<RealTimeServerGoalHandle> rt_goal_handle = boost::make_shared<RealTimeServerGoalHandle>(gh);

  Builder builder(stop_duration, hold_state);
  builder.setGoalHandle(rt_goal_handle);

  EXPECT_EQ(rt_goal_handle.use_count(), 1) << "Builder should only store a reference on GoalHandlePtr.";

	builder.reset();
	builder.setStartTime(start_time);

  Trajectory trajectory;
  joint_trajectory_controller_tests::initDefaultTrajectory(number_of_joints, trajectory);

  EXPECT_TRUE(builder.buildTrajectory(&trajectory)) << "buildTrajectory() should have been successful.";

  EXPECT_EQ(rt_goal_handle.use_count(), 1) << "Modified owner count of the goal handle despite reset.";
}

}  // namespace stop_trajectory_builder_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
