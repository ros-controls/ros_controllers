
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
#include <trajectory_interface/quintic_spline_segment.h>
#include <trajectory_interface/multi_dof_segment.h>

using namespace trajectory_interface;

// Floating-point value comparison threshold
const double EPS = 1e-9;

typedef QuinticSplineSegment<double> Segment;
typedef typename Segment::State State;
typedef typename Segment::Time  Time;

class MultiDofSegmentTest : public ::testing::Test
{
public:
  MultiDofSegmentTest()
    : dim(2),
      start_time(1.0), end_time(2.0),
      start_states(dim), end_states(dim), states(dim)
  {
    start_states[0].position.push_back(0.0);
    start_states[1].position.push_back(-1.0);

    end_states[0].position.push_back(1.0);
    end_states[1].position.push_back(-2.0);
  }

protected:
  unsigned int dim;
  Time start_time, end_time;
  std::vector<State> start_states, end_states, states;
};

TEST_F(MultiDofSegmentTest, DefaultConstructor)
{
  MultiDofSegment<Segment> segments;

  EXPECT_FALSE(states.empty());
  segments.sample(1.0, states);
  EXPECT_TRUE(states.empty());
}

TEST_F(MultiDofSegmentTest, InvalidSegmentConstruction)
{
  // Mismatching start/end state dimensions
  end_states.pop_back();
  EXPECT_THROW(MultiDofSegment<Segment>(start_time, start_states, end_time, end_states), std::invalid_argument);
  try{MultiDofSegment<Segment>(start_time, start_states, end_time, end_states);}
  catch(const std::invalid_argument& ex) {ROS_ERROR_STREAM(ex.what());}
}

TEST_F(MultiDofSegmentTest, Accessors)
{
  // Empty container
  {
    MultiDofSegment<Segment> segments;
    EXPECT_EQ(0,   segments.size());
    EXPECT_EQ(0.0, segments.startTime());
    EXPECT_EQ(0.0, segments.endTime());
  }

  // Valid container
  {
    MultiDofSegment<Segment> segments(start_time, start_states, end_time, end_states);
    EXPECT_EQ(dim, segments.size());
    EXPECT_EQ(start_time, segments.startTime());
    EXPECT_EQ(end_time, segments.endTime());
  }
}

TEST_F(MultiDofSegmentTest, SegmentSampler)
{
  MultiDofSegment<Segment> segments(start_time, start_states, end_time, end_states);
  const Time duration = segments.endTime() - segments.startTime();

  // Sample before segments start
  {
    segments.sample(start_time - duration, states);
    EXPECT_EQ(dim, segments.size());

    for (unsigned int i = 0; i < segments.size(); ++i)
    {
      EXPECT_NEAR(start_states[i].position[0], states[i].position[0], EPS);
      EXPECT_NEAR(0.0, states[i].velocity[0], EPS);
      EXPECT_NEAR(0.0, states[i].acceleration[0], EPS);
    }
  }

  // Sample at mid-segment
  {
    segments.sample(start_time + duration / 2.0, states);
    EXPECT_EQ(dim, segments.size());

    for (unsigned int i = 0; i < segments.size(); ++i)
    {
      const double position = (end_states[i].position[0] + start_states[i].position[0]) / 2.0;
      const double velocity = (end_states[i].position[0] - start_states[i].position[0]) / duration;
      EXPECT_NEAR(position, states[i].position[0], EPS);
      EXPECT_NEAR(velocity, states[i].velocity[0], EPS);
      EXPECT_NEAR(0.0, states[i].acceleration[0], EPS);
    }
  }

  // Sample past segment end
  {
    segments.sample(end_time + duration, states);
    EXPECT_EQ(dim, segments.size());

    for (unsigned int i = 0; i < segments.size(); ++i)
    {
      EXPECT_NEAR(end_states[i].position[0], states[i].position[0], EPS);
      EXPECT_NEAR(0.0, states[i].velocity[0], EPS);
      EXPECT_NEAR(0.0, states[i].acceleration[0], EPS);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

