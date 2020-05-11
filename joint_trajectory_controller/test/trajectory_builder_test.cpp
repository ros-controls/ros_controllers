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

#include <actionlib/server/action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <joint_trajectory_controller/trajectory_builder.h>

#include <realtime_tools/realtime_server_goal_handle.h>

#include <trajectory_interface/quintic_spline_segment.h>

namespace trajectory_builder_tests
{
using QuinticSplineSegment = trajectory_interface::QuinticSplineSegment<double>;
using TrajectoryBuilder = joint_trajectory_controller::TrajectoryBuilder<QuinticSplineSegment>;

using GoalHandle = actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle;
using RealTimeServerGoalHandle = realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction>;

class FakeTrajectoryBuilder : public TrajectoryBuilder
{
private:
  using Segment = joint_trajectory_controller::JointTrajectorySegment<QuinticSplineSegment>;
  using TrajectoryPerJoint = std::vector<Segment>;
  using Trajectory = std::vector<TrajectoryPerJoint>;

  FRIEND_TEST(TrajectoryBuilderTest, testSetStartTime);
  FRIEND_TEST(TrajectoryBuilderTest, testResetStartTime);
  FRIEND_TEST(TrajectoryBuilderTest, testSetGoalHandle);
  FRIEND_TEST(TrajectoryBuilderTest, testResetGoalHandle);
  FRIEND_TEST(TrajectoryBuilderTest, testDefaultGoalHandle);
protected:
  /**
   * @brief Override pure virtual method. Not included in the tests.
   */
  bool buildTrajectory(Trajectory* hold_traj) override
  {
    return true;
  }
};

TEST(TrajectoryBuilderTest, testSetStartTime)
{
  FakeTrajectoryBuilder builder;
  EXPECT_FALSE(builder.getStartTime().is_initialized()) << "Obtained start time despite not set.";

  const double start_time {0.1};
  builder.setStartTime(start_time);

  EXPECT_EQ(builder.getStartTime().get(), start_time) << "Start time was not set/returned correctly.";
}

TEST(TrajectoryBuilderTest, testResetStartTime)
{
  FakeTrajectoryBuilder builder;

  const double start_time {0.1};
  builder.setStartTime(start_time);

  builder.reset();
  EXPECT_FALSE(builder.getStartTime().is_initialized()) << "Obtained start time despite reset.";
}

TEST(TrajectoryBuilderTest, testSetGoalHandle)
{
  FakeTrajectoryBuilder builder;

  EXPECT_FALSE(builder.createGoalHandlePtr()) << "Obtained goal handle despite not set.";

  GoalHandle gh;
  boost::shared_ptr<RealTimeServerGoalHandle> rt_goal_handle = boost::make_shared<RealTimeServerGoalHandle>(gh);
  builder.setGoalHandle(rt_goal_handle);

  EXPECT_EQ(rt_goal_handle.use_count(), 1) << "Builder should only store a reference on GoalHandlePtr.";

  auto gh_return = builder.createGoalHandlePtr();
  EXPECT_EQ(rt_goal_handle.get(), gh_return.get()) << "Goal handle pointer was not set/returned correctly.";
}

TEST(TrajectoryBuilderTest, testResetGoalHandle)
{
  FakeTrajectoryBuilder builder;

  GoalHandle gh;
  boost::shared_ptr<RealTimeServerGoalHandle> rt_goal_handle = boost::make_shared<RealTimeServerGoalHandle>(gh);
  builder.setGoalHandle(rt_goal_handle);

  builder.reset();
  EXPECT_FALSE(builder.createGoalHandlePtr()) << "Obtained goal handle despite reset.";
}

TEST(TrajectoryBuilderTest, testDefaultGoalHandle)
{
  FakeTrajectoryBuilder builder;
  EXPECT_FALSE(builder.createDefaultGoalHandle()) << "Default goal handle pointer is expected to be empty";
}

}  // namespace trajectory_builder_tests

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
