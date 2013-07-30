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
#include <stdexcept>
#include <gtest/gtest.h>
#include <ros/console.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>

using namespace trajectory_interface;
using namespace trajectory_msgs;

// Floating-point value comparison threshold
const double EPS = 1e-9;

typedef JointTrajectorySegment<double> Segment;

TEST(WraparoundOffsetTest, WrappingPositions)
{
  typedef  typename JointTrajectorySegment<double>::State State;

  // Setup with state increments that cause multi-loop wrapping
  const double half_pi = M_PI / 2.0;
  const double two_pi  = 2.0 * M_PI;

  State state1(2);
  state1[0].position =  half_pi / 2.0;
  state1[1].position =  half_pi / 2.0;

  State state2(2);
  state2[0].position =  two_pi + half_pi;
  state2[1].position =  2.0 * two_pi + half_pi;

  // No wrapping joints
  {
    std::vector<bool> is_wrapping(2, false);
    std::vector<double> wrap_offset = wraparoundOffset<double>(state1, state2, is_wrapping);
    EXPECT_NEAR(state1.size(), wrap_offset.size(), EPS);
    EXPECT_NEAR(0.0, wrap_offset[0], EPS);
    EXPECT_NEAR(0.0, wrap_offset[1], EPS);
  }

  // Single wrapping joint
  {
    std::vector<bool> is_wrapping(2);
    is_wrapping[0] = true;
    is_wrapping[1] = false;

    // From state1 to state2
    {
      std::vector<double> wrap_offset = wraparoundOffset<double>(state1, state2, is_wrapping);
      EXPECT_NEAR(state1.size(), wrap_offset.size(), EPS);
      EXPECT_NEAR(-two_pi, wrap_offset[0], EPS);
      EXPECT_NEAR(0.0, wrap_offset[1], EPS);
    }

    // From state2 to state1
    {
      std::vector<double> wrap_offset = wraparoundOffset<double>(state2, state1, is_wrapping);
      EXPECT_NEAR(state1.size(), wrap_offset.size(), EPS);
      EXPECT_NEAR(two_pi, wrap_offset[0], EPS);
      EXPECT_NEAR(0.0, wrap_offset[1], EPS);
    }
  }

  // Both wrapping joints
  {
    std::vector<bool> is_wrapping(2, true);

    // From state1 to state2
    {
      std::vector<double> wrap_offset = wraparoundOffset<double>(state1, state2, is_wrapping);
      EXPECT_NEAR(state1.size(), wrap_offset.size(), EPS);
      EXPECT_NEAR(-two_pi, wrap_offset[0], EPS);
      EXPECT_NEAR(-2.0 * two_pi, wrap_offset[1], EPS);
    }

    // From state2 to state1
    {
      std::vector<double> wrap_offset = wraparoundOffset<double>(state2, state1, is_wrapping);
      EXPECT_NEAR(state1.size(), wrap_offset.size(), EPS);
      EXPECT_NEAR(two_pi, wrap_offset[0], EPS);
      EXPECT_NEAR(2.0 * two_pi, wrap_offset[1], EPS);
    }
  }
}

TEST(WraparoundOffsetTest, NonWrappingPositions)
{
  typedef  typename JointTrajectorySegment<double>::State State;

  // Setup with state increments that don't cause multi-loop wrapping
  const double half_pi = M_PI / 2.0;

  State state1(2);
  state1[0].position =  half_pi / 2.0;
  state1[1].position =  half_pi / 2.0;

  State state2(2);
  state2[0].position =  0.0;
  state2[1].position =  2.0 * half_pi;

  // Both wrapping joints
  {
    std::vector<bool> is_wrapping(2, true);

    // From state1 to state2
    {
      std::vector<double> wrap_offset = wraparoundOffset<double>(state1, state2, is_wrapping);
      EXPECT_NEAR(state1.size(), wrap_offset.size(), EPS);
      EXPECT_NEAR(0.0, wrap_offset[0], EPS);
      EXPECT_NEAR(0.0, wrap_offset[1], EPS);
    }

    // From state2 to state1
    {
      std::vector<double> wrap_offset = wraparoundOffset<double>(state2, state1, is_wrapping);
      EXPECT_NEAR(state1.size(), wrap_offset.size(), EPS);
      EXPECT_NEAR(0.0, wrap_offset[0], EPS);
      EXPECT_NEAR(0.0, wrap_offset[1], EPS);
    }
  }
}

class JointTrajectorySegmentTest : public ::testing::Test
{
public:
  JointTrajectorySegmentTest()
    : traj_start_time(0.5)
  {
    p_start.positions.resize(1, 2.0);
    p_start.velocities.resize(1, -1.0);
    p_start.time_from_start = ros::Duration(1.0);

    p_end.positions.resize(1, 4.0);
    p_end.velocities.resize(1, -2.0);
    p_end.time_from_start = ros::Duration(2.0);

    start_time = (traj_start_time + p_start.time_from_start).toSec();
    start_state.resize(1);
    start_state.front().position = p_start.positions.front();
    start_state.front().velocity = p_start.velocities.front();

    end_time = (traj_start_time + p_end.time_from_start).toSec();
    end_state.resize(1);
    end_state.front().position = p_end.positions.front();
    end_state.front().velocity = p_end.velocities.front();
  }

protected:
  ros::Time traj_start_time;
  JointTrajectoryPoint p_start;
  JointTrajectoryPoint p_end;

  typename Segment::Time start_time, end_time;
  typename Segment::State start_state, end_state;
};

TEST_F(JointTrajectorySegmentTest, InvalidSegmentConstruction)
{
  // Empty start point
  {
    JointTrajectoryPoint p_start_bad;
    EXPECT_THROW(Segment(traj_start_time, p_start_bad, p_end), std::invalid_argument);
    try {Segment(traj_start_time, JointTrajectoryPoint(), p_end);}
    catch (const std::invalid_argument& ex) {ROS_ERROR_STREAM(ex.what());}
  }

  // Start/end data size mismatch
  {
    JointTrajectoryPoint p_start_bad = p_start;
    p_start_bad.positions.push_back(0.0);
    EXPECT_THROW(Segment(traj_start_time, p_start_bad, p_end), std::invalid_argument);
    try {Segment(traj_start_time, p_start_bad, p_end);}
    catch (const std::invalid_argument& ex) {ROS_ERROR_STREAM(ex.what());}
  }

  // Invalid start state
  {
    JointTrajectoryPoint p_start_bad = p_start;
    p_start_bad.velocities.push_back(0.0);
    EXPECT_THROW(Segment(traj_start_time, p_start_bad, p_end), std::invalid_argument);
    try {Segment(traj_start_time, p_start_bad, p_end);}
    catch (const std::invalid_argument& ex) {ROS_ERROR_STREAM(ex.what());}
  }

  // Invalid end state
  {
    JointTrajectoryPoint p_end_bad = p_end;
    p_end_bad.velocities.push_back(0.0);
    EXPECT_THROW(Segment(traj_start_time, p_start, p_end_bad), std::invalid_argument);
    try {Segment(traj_start_time, p_start, p_end_bad);}
    catch (const std::invalid_argument& ex) {ROS_ERROR_STREAM(ex.what());}
  }

  // Invalid permutation vector size
  {
    std::vector<unsigned int> permutation(2, 1);
    EXPECT_THROW(Segment(traj_start_time, p_start, p_end, permutation), std::invalid_argument);
    try {Segment(traj_start_time, p_start, p_end, permutation);}
    catch (const std::invalid_argument& ex) {ROS_ERROR_STREAM(ex.what());}
  }

  // Invalid permutation vector indices
  {
    std::vector<unsigned int> permutation(1, 1);
    EXPECT_THROW(Segment(traj_start_time, p_start, p_end, permutation), std::invalid_argument);
    try {Segment(traj_start_time, p_start, p_end, permutation);}
    catch (const std::invalid_argument& ex) {ROS_ERROR_STREAM(ex.what());}
  }

  // Invalid joint wraparound specification
  {
    std::vector<unsigned int> permutation;
    std::vector<double> pos_offset(2);
    EXPECT_THROW(Segment(traj_start_time, p_start, p_end, permutation, pos_offset), std::invalid_argument);
    try {Segment(traj_start_time, p_start, p_end, permutation, pos_offset);}
    catch (const std::invalid_argument& ex) {ROS_ERROR_STREAM(ex.what());}
  }
}

TEST_F(JointTrajectorySegmentTest, ValidSegmentConstructionRos)
{
  // Construct segment from ROS message
  EXPECT_NO_THROW(Segment(traj_start_time, p_start, p_end));
  Segment segment(traj_start_time, p_start, p_end);

  // Check start/end times
  {
    const typename Segment::Time start_time = (traj_start_time + p_start.time_from_start).toSec();
    const typename Segment::Time end_time   = (traj_start_time + p_end.time_from_start).toSec();
    EXPECT_EQ(start_time, segment.startTime());
    EXPECT_EQ(end_time,   segment.endTime());
  }

  // Check start state
  {
    typename Segment::State state;
    segment.sample(segment.startTime(), state);
    EXPECT_EQ(p_start.positions.front(),  state.front().position);
    EXPECT_EQ(p_start.velocities.front(), state.front().velocity);
  }

  // Check end state
  {
    typename Segment::State state;
    segment.sample(segment.endTime(), state);
    EXPECT_EQ(p_end.positions.front(),  state.front().position);
    EXPECT_EQ(p_end.velocities.front(), state.front().velocity);
  }
}

TEST_F(JointTrajectorySegmentTest, ValidSegmentConstruction)
{
  // Construct segment from ROS message
  EXPECT_NO_THROW(Segment(start_time,start_state, end_time, end_state));
  Segment segment(start_time,start_state, end_time, end_state);

  // Check start/end times
  {
    const typename Segment::Time start_time = (traj_start_time + p_start.time_from_start).toSec();
    const typename Segment::Time end_time   = (traj_start_time + p_end.time_from_start).toSec();
    EXPECT_EQ(start_time, segment.startTime());
    EXPECT_EQ(end_time,   segment.endTime());
  }

  // Check start state
  {
    typename Segment::State state;
    segment.sample(segment.startTime(), state);
    EXPECT_EQ(p_start.positions.front(),  state.front().position);
    EXPECT_EQ(p_start.velocities.front(), state.front().velocity);
  }

  // Check end state
  {
    typename Segment::State state;
    segment.sample(segment.endTime(), state);
    EXPECT_EQ(p_end.positions.front(),  state.front().position);
    EXPECT_EQ(p_end.velocities.front(), state.front().velocity);
  }
}

// TODO: Test with dimension four data
TEST_F(JointTrajectorySegmentTest, PermutationTest)
{
  // Add an extra joint to the trajectory point messages created in the fixture
  p_start.positions.push_back(-p_start.positions.front());
  p_start.velocities.push_back(-p_start.velocities.front());

  p_end.positions.push_back(-p_end.positions.front());
  p_end.velocities.push_back(-p_end.velocities.front());

  // No permutation vector
  {
    // Construct segment from ROS message
    EXPECT_NO_THROW(Segment(traj_start_time, p_start, p_end));
    Segment segment(traj_start_time, p_start, p_end);

    // Check position values of start state only
    typename Segment::State state;
    segment.sample(segment.startTime(), state);
    EXPECT_EQ(p_start.positions[0], state[0].position);
    EXPECT_EQ(p_start.positions[1], state[1].position);
  }

  // Permutation vector preserving trajectory message joint order
  {
    std::vector<unsigned int> permutation(2);
    permutation[0] = 0;
    permutation[1] = 1;

    // Construct segment from ROS message
    EXPECT_NO_THROW(Segment(traj_start_time, p_start, p_end, permutation));
    Segment segment(traj_start_time, p_start, p_end, permutation);

    // Check position values of start state only
    typename Segment::State state;
    segment.sample(segment.startTime(), state);
    EXPECT_EQ(p_start.positions[0], state[0].position);
    EXPECT_EQ(p_start.positions[1], state[1].position);
  }

  // Permutation vector reversing trajectory message joint order
  {
    std::vector<unsigned int> permutation(2);
    permutation[0] = 1;
    permutation[1] = 0;

    // Construct segment from ROS message
    EXPECT_NO_THROW(Segment(traj_start_time, p_start, p_end, permutation));
    Segment segment(traj_start_time, p_start, p_end, permutation);

    // Check position values of start state only
    typename Segment::State state;
    segment.sample(segment.startTime(), state);
    EXPECT_NE(p_start.positions[0], state[0].position);
    EXPECT_NE(p_start.positions[1], state[1].position);
    EXPECT_EQ(p_start.positions[0], state[1].position);
    EXPECT_EQ(p_start.positions[1], state[0].position);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
