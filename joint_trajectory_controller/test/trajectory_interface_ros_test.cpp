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
//   * Neither the name of hiDOF, Inc. nor the names of its
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

/// \author Adolfo Rodriguez Tsouroukdissian

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <joint_trajectory_controller/trajectory_interface_ros.h>

using namespace trajectory_interface;
using namespace trajectory_msgs;
using std::vector;
using std::string;

// Floating-point value comparison threshold
const double EPS = 1e-9;


typedef JointTrajectorySegment<double> Segment;
typedef vector<Segment>           Trajectory;


TEST(StartTimeTest, StartTime)
{
  const ros::Time now(1.0);
  // Zero start time
  {
    JointTrajectory msg;
    msg.header.stamp = ros::Time(0.0);
    EXPECT_NE(msg.header.stamp, internal::startTime(msg, now));
    EXPECT_EQ(now, internal::startTime(msg, now));
  }

  // Nonzero start time
  {
    JointTrajectory msg;
    msg.header.stamp = ros::Time(2.0);
    EXPECT_EQ(msg.header.stamp, internal::startTime(msg, now));
    EXPECT_NE(now, internal::startTime(msg, now));
  }
}

TEST(PermutationTest, Permutation)
{
  vector<string> t1(3);
  t1[0] = "A";
  t1[1] = "B";
  t1[2] = "C";

  vector<string> t2(3);
  t2[0] = "B";
  t2[1] = "A";
  t2[2] = "C";

  typedef vector<vector<string>::size_type> PermutationType;
  PermutationType ground_truth(3);
  ground_truth[0] = 1;
  ground_truth[1] = 0;
  ground_truth[2] = 2;

  // Mismatching sizes: Return empty permutation vector
  {
    vector<string> t2_bad(1,"A");
    PermutationType perm = internal::permutation(t1, t2_bad);
    EXPECT_TRUE(perm.empty());
  }

  // Mismatching contents: Return empty permutation vector
  {
    vector<string> t2_bad(3,"A");
    PermutationType perm = internal::permutation(t1, t2_bad);
    EXPECT_TRUE(perm.empty());
  }

  // Valid parameters
  {
    PermutationType perm = internal::permutation(t1, t2);
    EXPECT_EQ(ground_truth.size(), perm.size());
    EXPECT_EQ(ground_truth[0], perm[0]);
    EXPECT_EQ(ground_truth[1], perm[1]);
    EXPECT_EQ(ground_truth[2], perm[2]);
  }
}

class TrajectoryInterfaceRosTest : public ::testing::Test
{
public:
  TrajectoryInterfaceRosTest()
    : points(3)
  {
    points[0].positions.resize(1, 2.0);
    points[0].velocities.resize(1, 0.0);
    points[0].accelerations.resize(1, 0.0);
    points[0].time_from_start = ros::Duration(1.0);

    points[1].positions.resize(1, 4.0);
    points[1].velocities.resize(1, 0.0);
    points[1].accelerations.resize(1, 0.0);
    points[1].time_from_start = ros::Duration(2.0);

    points[2].positions.resize(1, 3.0);
    points[2].velocities.resize(1, 0.0);
    points[2].accelerations.resize(1, 0.0);
    points[2].time_from_start = ros::Duration(4.0);

    trajectory_msg.header.stamp = ros::Time(0.5);
    trajectory_msg.joint_names.resize(1, "foo_joint");
    trajectory_msg.points = points;
  }

protected:
  vector<JointTrajectoryPoint> points;
  trajectory_msgs::JointTrajectory trajectory_msg;
};

TEST_F(TrajectoryInterfaceRosTest, IsValid)
{
  // Empty trajectory
  {
    JointTrajectory msg;
    EXPECT_TRUE(isValid(msg));
  }

  // Joint names size mismatch
  {
    JointTrajectory msg = trajectory_msg;
    msg.joint_names.push_back("bar_joint");
    EXPECT_FALSE(isValid(msg));
  }

  // Position size mismatch
  {
    JointTrajectory msg = trajectory_msg;
    msg.points.front().positions.push_back(0.0);
    EXPECT_FALSE(isValid(msg));

    msg.points.front().positions.clear(); // Empty container is not a size mismatch
    EXPECT_TRUE(isValid(msg));
  }

  // Velocity size mismatch
  {
    JointTrajectory msg = trajectory_msg;
    msg.points.front().velocities.push_back(0.0);
    EXPECT_FALSE(isValid(msg));

    msg.points.front().velocities.clear(); // Empty container is not a size mismatch
    EXPECT_TRUE(isValid(msg));
  }

  // Acceleration size mismatch
  {
    JointTrajectory msg = trajectory_msg;
    msg.points.front().accelerations.push_back(0.0);
    EXPECT_FALSE(isValid(msg));

    msg.points.front().accelerations.clear(); // Empty container is not a size mismatch
    EXPECT_TRUE(isValid(msg));
  }

  // Valid trajectory
  {
    EXPECT_TRUE(isValid(trajectory_msg));
  }
}

TEST_F(TrajectoryInterfaceRosTest, FindPoint)
{
  const ros::Time msg_start_time = trajectory_msg.header.stamp;
  const vector<JointTrajectoryPoint>& msg_points = trajectory_msg.points;

  // Before first point: No points found
  {
    const ros::Time time = msg_start_time;
    EXPECT_EQ(msg_points.end(), findPoint(trajectory_msg, time));
  }

  // First point
  {
    const ros::Time time = msg_start_time + msg_points.begin()->time_from_start;
    EXPECT_EQ(msg_points.begin(), findPoint(trajectory_msg, time));
  }

  // Between the first and second points
  {
    const ros::Time time = msg_start_time +
    ros::Duration((msg_points.begin()->time_from_start + (++msg_points.begin())->time_from_start).toSec() / 2.0);
    EXPECT_EQ(msg_points.begin(), findPoint(trajectory_msg, time));
  }

  // Second point
  {
    const ros::Time time = msg_start_time + (++msg_points.begin())->time_from_start;
    EXPECT_EQ(++msg_points.begin(), findPoint(trajectory_msg, time));
  }

  // Between the second and third points
  {
    const ros::Time time = msg_start_time +
    ros::Duration(((++msg_points.begin())->time_from_start + (--msg_points.end())->time_from_start).toSec() / 2.0);
    EXPECT_EQ(++msg_points.begin(), findPoint(trajectory_msg, time));
  }

  // Last point
  {
    const ros::Time time = msg_start_time + (--msg_points.end())->time_from_start;
    EXPECT_EQ(--msg_points.end(), findPoint(trajectory_msg, time));
  }

  // After the last point
  {
    const ros::Time time = msg_start_time + (--msg_points.end())->time_from_start + ros::Duration(1.0);
    EXPECT_EQ(--msg_points.end(), findPoint(trajectory_msg, time));
  }
}

TEST_F(TrajectoryInterfaceRosTest, InitException)
{
  // Invalid input should throw an exception
  trajectory_msg.points.back().positions.push_back(0.0); // Inconsistent size in last element
  EXPECT_THROW(init<Trajectory>(trajectory_msg, trajectory_msg.header.stamp), std::invalid_argument);
}

TEST_F(TrajectoryInterfaceRosTest, InitLogic)
{
  const ros::Time msg_start_time = trajectory_msg.header.stamp;
  const vector<JointTrajectoryPoint>& msg_points = trajectory_msg.points;

  // Empty trajectory message: Return empty trajectory
  {
    const ros::Time time = msg_start_time;
    Trajectory trajectory = init<Trajectory>(trajectory_msgs::JointTrajectory(), msg_start_time);
    EXPECT_TRUE(trajectory.empty());
  }

  // Input time is...

  // Before first point: Return full trajectory (2 segments)
  {
    const ros::Time time = msg_start_time;
    Trajectory trajectory = init<Trajectory>(trajectory_msg, time);
    EXPECT_EQ(points.size() - 1, trajectory.size());
  }

  // First point: Return partial trajectory (last segment)
  {
    const ros::Time time = msg_start_time + msg_points.begin()->time_from_start;
    Trajectory trajectory = init<Trajectory>(trajectory_msg, time);
    EXPECT_EQ(points.size() - 2, trajectory.size());
  }

  // Between the first and second points: Return partial trajectory (last segment)
  {
    const ros::Time time = msg_start_time +
    ros::Duration((msg_points.begin()->time_from_start + (++msg_points.begin())->time_from_start).toSec() / 2.0);
    Trajectory trajectory = init<Trajectory>(trajectory_msg, time);
    EXPECT_EQ(msg_points.size() - 2, trajectory.size());
  }

  // Second point: Return empty trajectory
  {
    const ros::Time time = msg_start_time + (++msg_points.begin())->time_from_start;
    Trajectory trajectory = init<Trajectory>(trajectory_msg, time);
    EXPECT_TRUE(trajectory.empty());
  }

  // Between the second and third points: Return empty trajectory
  {
    const ros::Time time = msg_start_time +
    ros::Duration(((++msg_points.begin())->time_from_start + (--msg_points.end())->time_from_start).toSec() / 2.0);
    Trajectory trajectory = init<Trajectory>(trajectory_msg, time);
    EXPECT_TRUE(trajectory.empty());
  }

  // Last point: Return empty trajectory
  {
    const ros::Time time = msg_start_time + (--msg_points.end())->time_from_start;
    Trajectory trajectory = init<Trajectory>(trajectory_msg, time);
    EXPECT_TRUE(trajectory.empty());
  }

  // After the last point: Return empty trajectory
  {
    const ros::Time time(msg_start_time + msg_points.back().time_from_start + ros::Duration(1.0));
    Trajectory trajectory = init<Trajectory>(trajectory_msg, time);
    EXPECT_TRUE(trajectory.empty());
  }
}

TEST_F(TrajectoryInterfaceRosTest, InitValues)
{
  const ros::Time msg_start_time = trajectory_msg.header.stamp;
  const vector<JointTrajectoryPoint>& msg_points = trajectory_msg.points;

  // Input time is before first point: Return full trajectory (2 segments)
  const ros::Time time = msg_start_time;
  Trajectory trajectory = init<Trajectory>(trajectory_msg, time);
  ASSERT_EQ(points.size() - 1, trajectory.size());

  // Check all segment start/end times and states
  for (unsigned int i = 0; i < trajectory.size(); ++i)
  {
    const Segment& segment = trajectory[i];
    const JointTrajectoryPoint& p_start = msg_points[i];
    const JointTrajectoryPoint& p_end   = msg_points[i + 1];

    // Check start/end times
    {
      const typename Segment::Time start_time = (msg_start_time + p_start.time_from_start).toSec();
      const typename Segment::Time end_time   = (msg_start_time + p_end.time_from_start).toSec();
      EXPECT_EQ(start_time, segment.startTime());
      EXPECT_EQ(end_time,   segment.endTime());
    }

    // Check start state
    {
      typename Segment::State state;
      segment.sample(segment.startTime(), state);
      EXPECT_EQ(p_start.positions.front(),  state.front().position);
      EXPECT_EQ(p_start.velocities.front(), state.front().velocity);
      EXPECT_EQ(p_start.accelerations.front(), state.front().acceleration);
    }

    // Check end state
    {
      typename Segment::State state;
      segment.sample(segment.endTime(), state);
      EXPECT_EQ(p_end.positions.front(),  state.front().position);
      EXPECT_EQ(p_end.velocities.front(), state.front().velocity);
      EXPECT_EQ(p_end.accelerations.front(), state.front().acceleration);
    }
  }
}

TEST_F(TrajectoryInterfaceRosTest, PermutationTest)
{
  // Add an extra joint to the trajectory message created in the fixture
  trajectory_msg.points[0].positions.push_back(-2.0);
  trajectory_msg.points[0].velocities.push_back(0.0);
  trajectory_msg.points[0].accelerations.push_back(0.0);

  trajectory_msg.points[1].positions.push_back(-4.0);
  trajectory_msg.points[1].velocities.push_back(0.0);
  trajectory_msg.points[1].accelerations.push_back(0.0);

  trajectory_msg.points[2].positions.push_back(-3.0);
  trajectory_msg.points[2].velocities.push_back(0.0);
  trajectory_msg.points[2].accelerations.push_back(0.0);

  trajectory_msg.joint_names.push_back("bar_joint");

  // No reference joint names: Original message order is preserved
  {
    Trajectory trajectory = init<Trajectory>(trajectory_msg, trajectory_msg.header.stamp);

    // Check position values only
    const Segment& segment = trajectory.front();
    typename Segment::State state;
    segment.sample(segment.startTime(), state);
    EXPECT_EQ(trajectory_msg.points[0].positions[0],  state[0].position);
    EXPECT_EQ(trajectory_msg.points[0].positions[1],  state[1].position);
  }

  // Reference joint names vector that reverses joint order
  {
    vector<string> joint_names; // Joints are reversed
    joint_names.push_back(trajectory_msg.joint_names[1]);
    joint_names.push_back(trajectory_msg.joint_names[0]);

    Trajectory trajectory = init<Trajectory>(trajectory_msg, trajectory_msg.header.stamp, joint_names);

    // Check position values only
    const Segment& segment = trajectory.front();
    typename Segment::State state;
    segment.sample(segment.startTime(), state);
    EXPECT_NE(trajectory_msg.points[0].positions[0],  state[0].position);
    EXPECT_NE(trajectory_msg.points[0].positions[1],  state[1].position);
    EXPECT_EQ(trajectory_msg.points[0].positions[0],  state[1].position);
    EXPECT_EQ(trajectory_msg.points[0].positions[1],  state[0].position);
  }

  // Reference joint names size mismatch
  {
    vector<string> joint_names;
    joint_names.push_back(trajectory_msg.joint_names[0]);

    Trajectory trajectory = init<Trajectory>(trajectory_msg, trajectory_msg.header.stamp, joint_names);
    EXPECT_TRUE(trajectory.empty());
  }

  // Reference joint names value mismatch
  {
    vector<string> joint_names;
    joint_names.push_back(trajectory_msg.joint_names[0]);
    joint_names.push_back("baz_joint");

    Trajectory trajectory = init<Trajectory>(trajectory_msg, trajectory_msg.header.stamp, joint_names);
    EXPECT_TRUE(trajectory.empty());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

