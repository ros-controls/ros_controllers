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

#include <gtest/gtest.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <trajectory_interface/trajectory_interface.h>

using namespace trajectory_interface;

// Floating-point value comparison threshold
const double EPS = 1e-9;

typedef QuinticSplineSegment<double> Segment;
typedef typename Segment::State State;
typedef typename Segment::Time  Time;
typedef std::vector<Segment>    Trajectory;

class TrajectoryInterfaceTest : public ::testing::Test
{
public:
  TrajectoryInterfaceTest()
  {
    times[0] = 1.0;
    states[0].position.push_back(2.0);
    states[0].velocity.push_back(0.0);

    times[1] = 2.0;
    states[1].position.push_back(4.0);
    states[1].velocity.push_back(0.0);

    times[2] = 4.0;
    states[2].position.push_back(3.0);
    states[2].velocity.push_back(0.0);

    times[3] = 5.0;
    states[3].position.push_back(4.0);
    states[3].velocity.push_back(0.0);

    times[4] = 6.0;
    states[4].position.push_back(2.0);
    states[4].velocity.push_back(0.0);

    trajectory.push_back(Segment(times[0], states[0], times[1], states[1]));
    trajectory.push_back(Segment(times[1], states[1], times[2], states[2]));
    // There is a period without spline interpolation here, between times[2] and times[3]
    // Position hold should be used when sampling in this interval
    trajectory.push_back(Segment(times[3], states[3], times[4], states[4]));
    // After this segment position hold should be used
  }

protected:
  State states[5];
  Time  times[5];

  Trajectory trajectory;
};

TEST(EmptyTrajectoryInterfaceTest, FindSegment)
{
  Trajectory trajectory;
  EXPECT_EQ(trajectory.end(), findSegment(trajectory, 0.0));
}

TEST(EmptyTrajectoryInterfaceTest, SampleTrajectory)
{
  Trajectory trajectory;
  State state;

  EXPECT_EQ(trajectory.end(), sample(trajectory, 0.0, state));
  EXPECT_TRUE(state.position.empty());
  EXPECT_TRUE(state.velocity.empty());
  EXPECT_TRUE(state.acceleration.empty());
}

TEST_F(TrajectoryInterfaceTest, FindSegmentOverloads)
{
  // The important thing here is that the test builds
  // We want to make sure that we can get both const and non-const iterators
  const Time time = times[0];
  Trajectory::const_iterator const_it = findSegment(trajectory, time);
  Trajectory::iterator       it       = findSegment(trajectory, time);
  EXPECT_EQ(const_it, it); // This is here only to silence the compiler for unused variables
}

TEST_F(TrajectoryInterfaceTest, FindSegment)
{
  // Before trajectory start: No segments found
  {
    const Time time = times[0] - 1.0;
    EXPECT_EQ(trajectory.end(), findSegment(trajectory, time));
  }

  // First segment start
  {
    const Time time = times[0];
    EXPECT_EQ(trajectory.begin(), findSegment(trajectory, time));
  }

  // During the first segment
  {
    const Time time = (times[0] + times[1]) / 2.0;
    EXPECT_EQ(trajectory.begin(), findSegment(trajectory, time));
  }

  // Second segment start
  {
    const Time time = times[1];
    EXPECT_EQ(++trajectory.begin(), findSegment(trajectory, time));
  }

  // During the second segment
  {
    const Time time = (times[1] + times[2]) / 2.0;
    EXPECT_EQ(++trajectory.begin(), findSegment(trajectory, time));
  }

  // After the second segment end time, but before the third segment start time (there is a gap)
  // The second segment should be still returned
  {
    const Time time = (times[2] + times[3]) / 2.0;
    EXPECT_EQ(++trajectory.begin(), findSegment(trajectory, time));
  }

  // Last segment start
  {
    const Time time = times[3];
    EXPECT_EQ(--trajectory.end(), findSegment(trajectory, time));
  }

  // During the last segment
  {
    const Time time = (times[3] + times[4]) / 2.0;
    EXPECT_EQ(--trajectory.end(), findSegment(trajectory, time));
  }

  // After the last segment end time
  // The last segment should be still returned
  {
    const Time time = times[4] + 1.0;
    EXPECT_EQ(--trajectory.end(), findSegment(trajectory, time));
  }
}

TEST_F(TrajectoryInterfaceTest, SampleTrajectory)
{
  // Before trajectory start: No segments found
  {
    const Time time = times[0] - 1.0;
    State state;
    sample(trajectory, time, state);
    EXPECT_NEAR(states[0].position[0], state.position[0], EPS);
    EXPECT_NEAR(states[0].velocity[0], state.velocity[0], EPS);
  }

  // First segment start
  {
    const Time time = times[0];
    State state;
    sample(trajectory, time, state);
    EXPECT_NEAR(states[0].position[0], state.position[0], EPS);
    EXPECT_NEAR(states[0].velocity[0], state.velocity[0], EPS);
  }

  // During the first segment
  {
    const Time time = (times[0] + times[1]) / 2.0;
    const double position = (states[0].position[0] + states[1].position[0]) / 2.0;
    const double velocity = 3.0;
    State state;
    sample(trajectory, time, state);
    EXPECT_NEAR(position, state.position[0], EPS);
    EXPECT_NEAR(velocity, state.velocity[0], EPS);
  }

  // Second segment start
  {
    const Time time = times[1];
    State state;
    sample(trajectory, time, state);
    EXPECT_NEAR(states[1].position[0], state.position[0], EPS);
    EXPECT_NEAR(states[1].velocity[0], state.velocity[0], EPS);
  }

  // During the second segment
  {
    const Time time = (times[1] + times[2]) / 2.0;
    const double position = (states[1].position[0] + states[2].position[0]) / 2.0;
    const double velocity = -0.75;
    State state;
    sample(trajectory, time, state);
    EXPECT_NEAR(position, state.position[0], EPS);
    EXPECT_NEAR(velocity, state.velocity[0], EPS);
  }

  // After the second segment end time, but before the third segment start time (there is a gap)
  // The second segment should be still returned
  {
    const Time time = (times[2] + times[3]) / 2.0;
    State state;
    sample(trajectory, time, state);
    EXPECT_NEAR(states[2].position[0], state.position[0], EPS);
    EXPECT_NEAR(states[2].velocity[0], state.velocity[0], EPS);
  }

  // Just before last segment start
  // There should be a discontinuity between this point and the last segment start. We make it explicit here
  // The discontinuity arises by the way in which the trajectory segments were specified
  {
    const Time time = times[3] - EPS;
    State state;
    sample(trajectory, time, state);
    EXPECT_NEAR(states[2].position[0], state.position[0], EPS);
    EXPECT_NEAR(states[2].velocity[0], state.velocity[0], EPS);
  }

  // Last segment start
  {
    const Time time = times[3];
    State state;
    sample(trajectory, time, state);
    EXPECT_NEAR(states[3].position[0], state.position[0], EPS);
    EXPECT_NEAR(states[3].velocity[0], state.velocity[0], EPS);
  }

  // During the last segment
  {
    const Time time = (times[3] + times[4]) / 2.0;
    const double position = (states[3].position[0] + states[4].position[0]) / 2.0;
    const double velocity = -3.0;
    State state;
    sample(trajectory, time, state);
    EXPECT_NEAR(position, state.position[0], EPS);
    EXPECT_NEAR(velocity, state.velocity[0], EPS);
  }

  // After the last segment end time
  // The last segment should be still returned
  {
    const Time time = times[4] + 1.0;
    State state;
    sample(trajectory, time, state);
    EXPECT_NEAR(states[4].position[0], state.position[0], EPS);
    EXPECT_NEAR(states[4].velocity[0], state.velocity[0], EPS);
  }
}

TEST(OverlappingTrajectoryInterfaceTest, SampleTrajectory)
{
  State states[4];
  Time  times[4];

  Trajectory trajectory;

  times[0] = 0.0;
  states[0].position.push_back(0.0);

  times[1] = 2.0;
  states[1].position.push_back(2.0);

  times[2] = 1.0;
  states[2].position.push_back(1.0);

  times[3] = 3.0;
  states[3].position.push_back(5.0);

  // Both segments are defined in the [1.0, 2.0) interval. The second one should take precedence
  trajectory.push_back(Segment(times[0], states[0], times[1], states[1]));
  trajectory.push_back(Segment(times[2], states[2], times[3], states[3]));

  const double velocity1 = (states[1].position[0] - states[0].position[0]) / (times[1] - times[0]);
  const double velocity2 = (states[3].position[0] - states[2].position[0]) / (times[3] - times[2]);

  // First segment start
  {
    const Time time = times[0];
    State state;
    sample(trajectory, time, state);
    EXPECT_NEAR(states[0].position[0], state.position[0], EPS);
    EXPECT_NEAR(velocity1, state.velocity[0], EPS);
  }

  // During the first segment
  {
    const Time time = (times[0] + times[2]) / 2.0; // Midway between first and second segment start
    const double position = (states[0].position[0] + states[2].position[0]) / 2.0;
    State state;
    EXPECT_EQ(trajectory.begin(), sample(trajectory, time, state));
    EXPECT_NEAR(position, state.position[0], EPS);
    EXPECT_NEAR(velocity1, state.velocity[0], EPS);
  }

  // Second segment start
  {
    const Time time = times[2];
    State state;
    EXPECT_EQ(++trajectory.begin(), sample(trajectory, time, state));
    EXPECT_NEAR(states[2].position[0], state.position[0], EPS);
    EXPECT_NEAR(velocity2, state.velocity[0], EPS);
  }

  // During the second segment
  {
    const Time time = (times[2] + times[3]) / 2.0; // Midway between second segment start and end
    const double position = (states[2].position[0] + states[3].position[0]) / 2.0;
    State state;
    EXPECT_EQ(++trajectory.begin(), sample(trajectory, time, state));
    EXPECT_NEAR(position, state.position[0], EPS);
    EXPECT_NEAR(velocity2, state.velocity[0], EPS);
  }

  // Second segment end
  {
    const Time time = times[3];
    State state;
    EXPECT_EQ(++trajectory.begin(), sample(trajectory, time, state));
    EXPECT_NEAR(states[3].position[0], state.position[0], EPS);
    EXPECT_NEAR(velocity2, state.velocity[0], EPS);
  }

  // After the second segment end time (and specified first segment end time)
  // The second segment should be still returned
  {
    const Time time = times[3] + 1.0;
    State state;
    EXPECT_EQ(++trajectory.begin(), sample(trajectory, time, state));
    EXPECT_NEAR(states[3].position[0], state.position[0], EPS);
    EXPECT_NEAR(0.0, state.velocity[0], EPS);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

