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
//   * Neither the name of PAL Robotics S.L. nor the names of its
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
#include <actionlib/server/action_server.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/init_joint_trajectory.h>

using namespace joint_trajectory_controller;
using namespace trajectory_msgs;
using std::vector;
using std::string;

// Floating-point value comparison threshold
const double EPS = 1e-9;

typedef JointTrajectorySegment<trajectory_interface::QuinticSplineSegment<double> > Segment;
typedef vector<Segment>                Trajectory;

TEST(PermutationTest, Permutation)
{
  vector<string> t1(4);
  t1[0] = "A";
  t1[1] = "B";
  t1[2] = "C";
  t1[3] = "D";

  vector<string> t2(4);
  t2[0] = "B";
  t2[1] = "D";
  t2[2] = "A";
  t2[3] = "C";

  typedef vector<unsigned int> PermutationType;
  PermutationType ground_truth(4);
  ground_truth[0] = 2;
  ground_truth[1] = 0;
  ground_truth[2] = 3;
  ground_truth[3] = 1;

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
    EXPECT_EQ(ground_truth[3], perm[3]);
  }

  // Valid parameters, inverse parameter order yields inverse permutation vector
  {
    PermutationType perm = internal::permutation(t2, t1);
    EXPECT_EQ(ground_truth.size(), perm.size());
    EXPECT_EQ(ground_truth[3], perm[0]);
    EXPECT_EQ(ground_truth[2], perm[1]);
    EXPECT_EQ(ground_truth[1], perm[2]);
    EXPECT_EQ(ground_truth[0], perm[3]);
  }
}

class InitTrajectoryTest : public ::testing::Test
{
public:
  InitTrajectoryTest()
    : points(3)
  {
    // Trajectory message
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

    // Current trajectory
    typename Segment::State state[2];

    state[0].position.push_back(0.0);
    state[0].velocity.push_back(0.0);
    state[0].acceleration.push_back(0.0);

    state[1].position.push_back(1.0);
    state[1].velocity.push_back(1.0);
    state[1].acceleration.push_back(1.0);

    curr_traj.push_back(Segment(0.0, state[0],
                                1.0, state[1]));

    curr_traj.push_back(Segment(3.0,  state[1],
                                10.0, state[0])); // This is just junk that should be discarded in the tests
  }

protected:
  // Trajectory message to parse
  vector<JointTrajectoryPoint> points;
  trajectory_msgs::JointTrajectory trajectory_msg;

  // Currently executed trajectory
  Trajectory curr_traj;
};

TEST_F(InitTrajectoryTest, InitException)
{
  // Invalid input should throw an exception
  trajectory_msg.points.back().positions.push_back(0.0); // Inconsistent size in last element
  EXPECT_THROW(initJointTrajectory<Trajectory>(trajectory_msg, trajectory_msg.header.stamp), std::invalid_argument);
}

// Test logic of parsing a trajectory message. No current trajectory is specified
TEST_F(InitTrajectoryTest, InitLogic)
{
  const ros::Time msg_start_time = trajectory_msg.header.stamp;
  const vector<JointTrajectoryPoint>& msg_points = trajectory_msg.points;

  // Empty trajectory message: Return empty trajectory
  {
    const ros::Time time = msg_start_time;
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msgs::JointTrajectory(), msg_start_time);
    EXPECT_TRUE(trajectory.empty());
  }

  // Input time is...

  // Before first point: Return full trajectory (2 segments)
  {
    const ros::Time time = msg_start_time;
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time);
    EXPECT_EQ(points.size() - 1, trajectory.size());
  }

  // First point: Return partial trajectory (last segment)
  {
    const ros::Time time = msg_start_time + msg_points.begin()->time_from_start;
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time);
    EXPECT_EQ(points.size() - 2, trajectory.size());
  }

  // Between the first and second points: Return partial trajectory (last segment)
  {
    const ros::Time time = msg_start_time +
    ros::Duration((msg_points.begin()->time_from_start + (++msg_points.begin())->time_from_start).toSec() / 2.0);
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time);
    EXPECT_EQ(msg_points.size() - 2, trajectory.size());
  }

  // Second point: Return empty trajectory
  {
    const ros::Time time = msg_start_time + (++msg_points.begin())->time_from_start;
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time);
    EXPECT_TRUE(trajectory.empty());
  }

  // Between the second and third points: Return empty trajectory
  {
    const ros::Time time = msg_start_time +
    ros::Duration(((++msg_points.begin())->time_from_start + (--msg_points.end())->time_from_start).toSec() / 2.0);
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time);
    EXPECT_TRUE(trajectory.empty());
  }

  // Last point: Return empty trajectory
  {
    const ros::Time time = msg_start_time + (--msg_points.end())->time_from_start;
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time);
    EXPECT_TRUE(trajectory.empty());
  }

  // After the last point: Return empty trajectory
  {
    const ros::Time time(msg_start_time + msg_points.back().time_from_start + ros::Duration(1.0));
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time);
    EXPECT_TRUE(trajectory.empty());
  }
}

// Test logic of parsing a trajectory message. Current trajectory is specified, hence trajectory combination takes place
TEST_F(InitTrajectoryTest, InitLogicCombine)
{
  const ros::Time msg_start_time = trajectory_msg.header.stamp;
  const vector<JointTrajectoryPoint>& msg_points = trajectory_msg.points;
  InitJointTrajectoryOptions<Trajectory> options;
  options.current_trajectory = &curr_traj;

  // Empty trajectory message: Return empty trajectory
  {
    const ros::Time time = msg_start_time;
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msgs::JointTrajectory(), msg_start_time, options);
    EXPECT_TRUE(trajectory.empty());
  }

  // Input time is...

  // Before first point, starting the current trajectory: Return 4 segments: Last of current + bridge + full message
  {
    const ros::Time time(curr_traj[0].startTime());
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time, options);
    EXPECT_EQ(points.size() + 1, trajectory.size());
  }

  // Before first point: Return 4 segments: Last of current + bridge + full message
  {
    const ros::Time time = msg_start_time;
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time, options);
    EXPECT_EQ(points.size() + 1, trajectory.size());
  }

  // First point: Return partial trajectory, 3 segments: Last of current + bridge + last of message
  {
    const ros::Time time = msg_start_time + msg_points.begin()->time_from_start;
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time, options);
    EXPECT_EQ(points.size(), trajectory.size());
  }

  // Between the first and second points: Same as before
  {
    const ros::Time time = msg_start_time +
    ros::Duration((msg_points.begin()->time_from_start + (++msg_points.begin())->time_from_start).toSec() / 2.0);
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time, options);
    EXPECT_EQ(msg_points.size(), trajectory.size());
  }

  // Second point: Return partial trajectory, 2 segments: Last of current + bridge (only one point of message made it)
  {
    const ros::Time time = msg_start_time + (++msg_points.begin())->time_from_start;
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time, options);
    EXPECT_EQ(msg_points.size() - 1, trajectory.size());
  }

  // Between the second and third points: Same as before
  {
    const ros::Time time = msg_start_time +
    ros::Duration(((++msg_points.begin())->time_from_start + (--msg_points.end())->time_from_start).toSec() / 2.0);
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time, options);
    EXPECT_EQ(msg_points.size() - 1, trajectory.size());
  }

  // Last point: Return empty trajectory
  {
    const ros::Time time = msg_start_time + (--msg_points.end())->time_from_start;
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time, options);
    EXPECT_TRUE(trajectory.empty());
  }

  // After the last point: Return empty trajectory
  {
    const ros::Time time(msg_start_time + msg_points.back().time_from_start + ros::Duration(1.0));
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time, options);
    EXPECT_TRUE(trajectory.empty());
  }
}

TEST_F(InitTrajectoryTest, InitValues)
{
  const ros::Time msg_start_time = trajectory_msg.header.stamp;
  const vector<JointTrajectoryPoint>& msg_points = trajectory_msg.points;

  // Input time is before first point: Return full trajectory (2 segments)
  const ros::Time time = msg_start_time;
  Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time);
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
      EXPECT_EQ(p_start.positions[0],  state.position[0]);
      EXPECT_EQ(p_start.velocities[0], state.velocity[0]);
      EXPECT_EQ(p_start.accelerations[0], state.acceleration[0]);
    }

    // Check end state
    {
      typename Segment::State state;
      segment.sample(segment.endTime(), state);
      EXPECT_EQ(p_end.positions[0],  state.position[0]);
      EXPECT_EQ(p_end.velocities[0], state.velocity[0]);
      EXPECT_EQ(p_end.accelerations[0], state.acceleration[0]);
    }
  }
}

TEST_F(InitTrajectoryTest, InitValuesCombine)
{
  // Before first point: Return 4 segments: Last of current + bridge + full message
  const ros::Time time(curr_traj[0].startTime());
  InitJointTrajectoryOptions<Trajectory> options;
  options.current_trajectory = &curr_traj;

  Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time, options);
  ASSERT_EQ(points.size() + 1, trajectory.size());

  // Check current trajectory segment start/end times and states (only one)
  {
    const Segment& ref_segment = curr_traj[0];
    const Segment& segment     = trajectory[0];

    // Check start/end times
    {
      EXPECT_EQ(ref_segment.startTime(), segment.startTime());
      EXPECT_EQ(ref_segment.endTime(),   segment.endTime());
    }

    // Check start state
    {
      typename Segment::State ref_state, state;
      ref_segment.sample(ref_segment.startTime(), ref_state);
      segment.sample(segment.startTime(), state);
      EXPECT_EQ(ref_state.position[0],     state.position[0]);
      EXPECT_EQ(ref_state.velocity[0],     state.velocity[0]);
      EXPECT_EQ(ref_state.acceleration[0], state.acceleration[0]);
    }

    // Check end state
    {
      typename Segment::State ref_state, state;
      ref_segment.sample(ref_segment.endTime(), ref_state);
      segment.sample(segment.endTime(), state);
      EXPECT_EQ(ref_state.position[0],     state.position[0]);
      EXPECT_EQ(ref_state.velocity[0],     state.velocity[0]);
      EXPECT_EQ(ref_state.acceleration[0], state.acceleration[0]);
    }
  }

  // Check bridge trajectory segment start/end times and states (only one)
  {
    const Segment& ref_segment = curr_traj[0];
    const Segment& segment     = trajectory[1];

    const vector<JointTrajectoryPoint>& msg_points = trajectory_msg.points;

    // Segment start time should correspond to message start time
    // Segment end time should correspond to first trajectory message point
    const ros::Time msg_start_time = trajectory_msg.header.stamp;
    const typename Segment::Time start_time = msg_start_time.toSec();
    const typename Segment::Time end_time   = (msg_start_time +  msg_points[0].time_from_start).toSec();

    // Check start/end times
    {
      EXPECT_EQ(start_time, segment.startTime());
      EXPECT_EQ(end_time,   segment.endTime());
    }

    // Check start state. Corresponds to current trajectory sampled at start_time
    {
      typename Segment::State ref_state, state;
      ref_segment.sample(start_time, ref_state);
      segment.sample(segment.startTime(), state);
      EXPECT_EQ(ref_state.position[0],     state.position[0]);
      EXPECT_EQ(ref_state.velocity[0],     state.velocity[0]);
      EXPECT_EQ(ref_state.acceleration[0], state.acceleration[0]);
    }

    // Check end state. Corresponds to first trajectory message point
    {
      typename Segment::State state;
      segment.sample(segment.endTime(), state);
      EXPECT_EQ(msg_points[0].positions[0],     state.position[0]);
      EXPECT_EQ(msg_points[0].velocities[0],    state.velocity[0]);
      EXPECT_EQ(msg_points[0].accelerations[0], state.acceleration[0]);
    }
  }

  // Check all segment start/end times and states (2 segments)
  for (unsigned int traj_it = 2, msg_it = 0; traj_it < trajectory.size(); ++traj_it, ++msg_it)
  {
    const ros::Time msg_start_time = trajectory_msg.header.stamp;
    const vector<JointTrajectoryPoint>& msg_points = trajectory_msg.points;

    const Segment& segment = trajectory[traj_it];
    const JointTrajectoryPoint& p_start = msg_points[msg_it];
    const JointTrajectoryPoint& p_end   = msg_points[msg_it + 1];

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
      EXPECT_EQ(p_start.positions[0],  state.position[0]);
      EXPECT_EQ(p_start.velocities[0], state.velocity[0]);
      EXPECT_EQ(p_start.accelerations[0], state.acceleration[0]);
    }

    // Check end state
    {
      typename Segment::State state;
      segment.sample(segment.endTime(), state);
      EXPECT_EQ(p_end.positions[0],  state.position[0]);
      EXPECT_EQ(p_end.velocities[0], state.velocity[0]);
      EXPECT_EQ(p_end.accelerations[0], state.acceleration[0]);
    }
  }
}

TEST_F(InitTrajectoryTest, JointPermutation)
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
    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, trajectory_msg.header.stamp);

    // Check position values only
    const Segment& segment = trajectory[0];
    typename Segment::State state;
    segment.sample(segment.startTime(), state);
    EXPECT_EQ(trajectory_msg.points[0].positions[0],  state.position[0]);
    EXPECT_EQ(trajectory_msg.points[0].positions[1],  state.position[1]);
  }

  // Reference joint names vector that reverses joint order
  {
    vector<string> joint_names; // Joints are reversed
    joint_names.push_back(trajectory_msg.joint_names[1]);
    joint_names.push_back(trajectory_msg.joint_names[0]);
    InitJointTrajectoryOptions<Trajectory> options;
    options.joint_names = &joint_names;

    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, trajectory_msg.header.stamp, options);

    // Check position values only
    const Segment& segment = trajectory[0];
    typename Segment::State state;
    segment.sample(segment.startTime(), state);
    EXPECT_NE(trajectory_msg.points[0].positions[0],  state.position[0]);
    EXPECT_NE(trajectory_msg.points[0].positions[1],  state.position[1]);
    EXPECT_EQ(trajectory_msg.points[0].positions[0],  state.position[1]);
    EXPECT_EQ(trajectory_msg.points[0].positions[1],  state.position[0]);
  }

  // Reference joint names size mismatch
  {
    vector<string> joint_names;
    joint_names.push_back(trajectory_msg.joint_names[0]);
    InitJointTrajectoryOptions<Trajectory> options;
    options.joint_names = &joint_names;

    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, trajectory_msg.header.stamp, options);
    EXPECT_TRUE(trajectory.empty());
  }

  // Reference joint names value mismatch
  {
    vector<string> joint_names;
    joint_names.push_back(trajectory_msg.joint_names[0]);
    joint_names.push_back("baz_joint");
    InitJointTrajectoryOptions<Trajectory> options;
    options.joint_names = &joint_names;

    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, trajectory_msg.header.stamp, options);
    EXPECT_TRUE(trajectory.empty());
  }
}

TEST_F(InitTrajectoryTest, WrappingSpec)
{
  // Modify trajectory message created in the fixture to wrap around
  const double wrap = 4 * M_PI;
  for (unsigned int i = 0; i < trajectory_msg.points.size(); ++i)
  {
    trajectory_msg.points[i].positions[0] += wrap; // Add wrapping value to all points
  }

  // Before first point: Return 4 segments: Last of current + bridge + full message
  const ros::Time time(curr_traj[0].startTime());
  std::vector<bool> angle_wraparound(1, true);
  InitJointTrajectoryOptions<Trajectory> options;
  options.current_trajectory = &curr_traj;
  options.angle_wraparound      = &angle_wraparound;

  Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time, options);
  ASSERT_EQ(points.size() + 1, trajectory.size());

  // Check current trajectory segment start/end times and states (only one)
  {
    const Segment& ref_segment = curr_traj[0];
    const Segment& segment     = trajectory[0];

    // Check start/end times
    {
      EXPECT_EQ(ref_segment.startTime(), segment.startTime());
      EXPECT_EQ(ref_segment.endTime(),   segment.endTime());
    }

    // Check start state
    {
      typename Segment::State ref_state, state;
      ref_segment.sample(ref_segment.startTime(), ref_state);
      segment.sample(segment.startTime(), state);
      EXPECT_EQ(ref_state.position[0],     state.position[0]);
      EXPECT_EQ(ref_state.velocity[0],     state.velocity[0]);
      EXPECT_EQ(ref_state.acceleration[0], state.acceleration[0]);
    }

    // Check end state
    {
      typename Segment::State ref_state, state;
      ref_segment.sample(ref_segment.endTime(), ref_state);
      segment.sample(segment.endTime(), state);
      EXPECT_EQ(ref_state.position[0],     state.position[0]);
      EXPECT_EQ(ref_state.velocity[0],     state.velocity[0]);
      EXPECT_EQ(ref_state.acceleration[0], state.acceleration[0]);
    }
  }

  // Check bridge trajectory segment start/end times and states (only one)
  {
    const Segment& ref_segment = curr_traj[0];
    const Segment& segment     = trajectory[1];

    const vector<JointTrajectoryPoint>& msg_points = trajectory_msg.points;

    // Segment start time should correspond to message start time
    // Segment end time should correspond to first trajectory message point
    const ros::Time msg_start_time = trajectory_msg.header.stamp;
    const typename Segment::Time start_time = msg_start_time.toSec();
    const typename Segment::Time end_time   = (msg_start_time +  msg_points[0].time_from_start).toSec();

    // Check start/end times
    {
      EXPECT_EQ(start_time, segment.startTime());
      EXPECT_EQ(end_time,   segment.endTime());
    }

    // Check start state. Corresponds to current trajectory sampled at start_time
    {
      typename Segment::State ref_state, state;
      ref_segment.sample(start_time, ref_state);
      segment.sample(segment.startTime(), state);
      EXPECT_EQ(ref_state.position[0],     state.position[0]);
      EXPECT_EQ(ref_state.velocity[0],     state.velocity[0]);
      EXPECT_EQ(ref_state.acceleration[0], state.acceleration[0]);
    }

    // Check end state. Corresponds to first trajectory message point
    {
      typename Segment::State state;
      segment.sample(segment.endTime(), state);
      EXPECT_EQ(msg_points[0].positions[0] - wrap, state.position[0]); // NOTE: Wrap used
      EXPECT_EQ(msg_points[0].velocities[0],       state.velocity[0]);
      EXPECT_EQ(msg_points[0].accelerations[0],    state.acceleration[0]);
    }
  }

  // Check all segment start/end times and states (2 segments)
  for (unsigned int traj_it = 2, msg_it = 0; traj_it < trajectory.size(); ++traj_it, ++msg_it)
  {
    const ros::Time msg_start_time = trajectory_msg.header.stamp;
    const vector<JointTrajectoryPoint>& msg_points = trajectory_msg.points;

    const Segment& segment = trajectory[traj_it];
    const JointTrajectoryPoint& p_start = msg_points[msg_it];
    const JointTrajectoryPoint& p_end   = msg_points[msg_it + 1];

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
      EXPECT_EQ(p_start.positions[0] - wrap, state.position[0]); // NOTE: Wrap used
      EXPECT_EQ(p_start.velocities[0],       state.velocity[0]);
      EXPECT_EQ(p_start.accelerations[0],    state.acceleration[0]);
    }

    // Check end state
    {
      typename Segment::State state;
      segment.sample(segment.endTime(), state);
      EXPECT_EQ(p_end.positions[0] - wrap, state.position[0]); // NOTE: Wrap used
      EXPECT_EQ(p_end.velocities[0],       state.velocity[0]);
      EXPECT_EQ(p_end.accelerations[0],    state.acceleration[0]);
    }
  }

  // Reference joint names size mismatch
  {
    std::vector<bool> angle_wraparound(2, true);
    InitJointTrajectoryOptions<Trajectory> options;
    options.current_trajectory = &curr_traj;
    options.angle_wraparound      = &angle_wraparound;

    Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, trajectory_msg.header.stamp, options);
    EXPECT_TRUE(trajectory.empty());
  }
}

// If current trajectory is not specified, but wrapping joints are, then the wrapping spec is ignored
TEST_F(InitTrajectoryTest, IgnoreWrappingSpec)
{
  const ros::Time msg_start_time = trajectory_msg.header.stamp;
  const vector<JointTrajectoryPoint>& msg_points = trajectory_msg.points;

  // Modify trajectory message created in the fixture to wrap around
  const double wrap = 4 * M_PI;
  for (unsigned int i = 0; i < trajectory_msg.points.size(); ++i)
  {
    trajectory_msg.points[i].positions[0] += wrap; // Add wrapping value to all points
  }

  // Before first point: Return 2 segments: Full message
  const ros::Time time(curr_traj[0].startTime());
  std::vector<bool> angle_wraparound(1, true);
  InitJointTrajectoryOptions<Trajectory> options;
  options.angle_wraparound = &angle_wraparound;

  Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time, options);
  ASSERT_EQ(points.size() - 1, trajectory.size());

  // Check all segment start/end times and states (2 segments)
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
      EXPECT_EQ(p_start.positions[0],  state.position[0]);
      EXPECT_EQ(p_start.velocities[0], state.velocity[0]);
      EXPECT_EQ(p_start.accelerations[0], state.acceleration[0]);
    }

    // Check end state
    {
      typename Segment::State state;
      segment.sample(segment.endTime(), state);
      EXPECT_EQ(p_end.positions[0],  state.position[0]);
      EXPECT_EQ(p_end.velocities[0], state.velocity[0]);
      EXPECT_EQ(p_end.accelerations[0], state.acceleration[0]);
    }
  }
}

// If current trajectory is not specified, but wrapping joints are, then the wrapping spec is ignored
TEST_F(InitTrajectoryTest, GoalHandleTest)
{
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
  typedef ActionServer::GoalHandle                                                            GoalHandle;
  typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;

  GoalHandle gh;
  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));

  // Before first point: Return 4 segments: Last of current + bridge + full message
  const ros::Time time(curr_traj[0].startTime());
  InitJointTrajectoryOptions<Trajectory> options;
  options.current_trajectory = &curr_traj;
  options.rt_goal_handle     = rt_goal;

  Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time, options);
  ASSERT_EQ(points.size() + 1, trajectory.size());

  // Segment of current trajectory preserves existing null goal handle
  ASSERT_FALSE(trajectory[0].getGoalHandle());

  // All further segments should be associated to the new goal handle
  for (unsigned int i = 1; i < trajectory.size(); ++i) // NOTE: Start index != 0
  {
    ASSERT_EQ(rt_goal, trajectory[i].getGoalHandle());
  }
}

TEST_F(InitTrajectoryTest, OtherTimeBase)
{
  const ros::Time msg_start_time = trajectory_msg.header.stamp;

  // Options with a different time base
  InitJointTrajectoryOptions<Trajectory> options;
  ros::Duration time_offset(1.0);
  ros::Time other_time_base = msg_start_time + time_offset;
  options.current_trajectory = &curr_traj;
  options.other_time_base = &other_time_base;

  // Before first point: Return 4 segments: Last of current + bridge + full message
  const ros::Time time = msg_start_time;
  Trajectory trajectory = initJointTrajectory<Trajectory>(trajectory_msg, time, options);
  ASSERT_EQ(points.size() + 1, trajectory.size());

  // Check current trajectory segment start/end times (only one). Time offset does NOT apply
  {
    const Segment& ref_segment = curr_traj[0];
    const Segment& segment     = trajectory[0];

    // Check start/end times
    {
      EXPECT_EQ(ref_segment.startTime(), segment.startTime());
      EXPECT_EQ(ref_segment.endTime(),   segment.endTime());
    }
  }

  // Check bridge trajectory segment start/end times and states (only one). Time offset applies to end state only
  {
    const Segment& segment = trajectory[1];

    const vector<JointTrajectoryPoint>& msg_points = trajectory_msg.points;

    // Segment start time should correspond to message start time
    // Segment end time should correspond to first trajectory message point
    const ros::Time msg_start_time = trajectory_msg.header.stamp;
    const typename Segment::Time start_time = msg_start_time.toSec();
    const typename Segment::Time end_time   = (msg_start_time +  msg_points[0].time_from_start).toSec();

    // Check start/end times
    {
      EXPECT_EQ(start_time + time_offset.toSec(), segment.startTime()); // NOTE: offsets applied
      EXPECT_EQ(end_time   + time_offset.toSec(), segment.endTime());
    }
  }

  // Check all segment start/end times and states (2 segments). Time offset does apply
  for (unsigned int traj_it = 2, msg_it = 0; traj_it < trajectory.size(); ++traj_it, ++msg_it)
  {
    const ros::Time msg_start_time = trajectory_msg.header.stamp;
    const vector<JointTrajectoryPoint>& msg_points = trajectory_msg.points;

    const Segment& segment = trajectory[traj_it];
    const JointTrajectoryPoint& p_start = msg_points[msg_it];
    const JointTrajectoryPoint& p_end   = msg_points[msg_it + 1];

    // Check start/end times
    {
      const typename Segment::Time start_time = (msg_start_time + p_start.time_from_start).toSec();
      const typename Segment::Time end_time   = (msg_start_time + p_end.time_from_start).toSec();
      EXPECT_EQ(start_time + time_offset.toSec(), segment.startTime()); // NOTE: offsets applied
      EXPECT_EQ(end_time   + time_offset.toSec(), segment.endTime());
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
