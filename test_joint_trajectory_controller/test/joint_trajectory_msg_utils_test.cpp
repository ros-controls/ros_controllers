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
#include <joint_trajectory_controller/joint_trajectory_msg_utils.h>

using namespace joint_trajectory_controller;
using namespace trajectory_msgs;
using std::vector;
using std::string;

// Floating-point value comparison threshold
const double EPS = 1e-9;

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

TEST_F(TrajectoryInterfaceRosTest, IsTimeStrictlyIncreasing)
{
  // Empty trajectory
  {
    JointTrajectory msg;
    EXPECT_TRUE(isTimeStrictlyIncreasing(msg));
  }

  // Single-waypoint trajectory
  {
    JointTrajectory msg;
    msg.points.push_back(points[0]);
    EXPECT_TRUE(isTimeStrictlyIncreasing(msg));
  }

  // Multi-waypoint tajectory with strictly increasing times
  {
    EXPECT_TRUE(isTimeStrictlyIncreasing(trajectory_msg));
  }

  // Multi-waypoint tajectory with monotonically increasing (non-decreasing) times
  {
    JointTrajectory msg;
    msg = trajectory_msg;
    msg.points[2].time_from_start = msg.points[1].time_from_start;
    EXPECT_FALSE(isTimeStrictlyIncreasing(msg));
  }

  // Multi-waypoint tajectory with non-monotonic times
  {
    JointTrajectory msg;
    msg = trajectory_msg;
    msg.points[2].time_from_start = msg.points[0].time_from_start;
    EXPECT_FALSE(isTimeStrictlyIncreasing(msg));
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

