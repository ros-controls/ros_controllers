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

#include <stdexcept>
#include <gtest/gtest.h>
#include <ros/console.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>

using namespace trajectory_interface;
using namespace trajectory_msgs;

typedef JointTrajectorySegment<double> Segment;

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
  }

protected:
  ros::Time traj_start_time;
  JointTrajectoryPoint p_start;
  JointTrajectoryPoint p_end;
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
}

TEST_F(JointTrajectorySegmentTest, ValidSegmentConstruction)
{
  // Construct segment
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

