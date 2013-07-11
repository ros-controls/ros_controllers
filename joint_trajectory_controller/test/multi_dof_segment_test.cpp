
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

#include <gtest/gtest.h>
#include <ros/console.h>
#include <joint_trajectory_controller/quintic_spline_segment.h>
#include <joint_trajectory_controller/multi_dof_segment.h>

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
    : start_time(1.0), end_time(2.0),
      states(2)
  {
    start_states[0].position =  0.0;
    start_states[1].position = -1.0;

    end_states[0].position =  1.0;
    end_states[1].position = -2.0;

    segments_data.push_back(Segment(start_time, start_states[0], end_time, end_states[0]));
    segments_data.push_back(Segment(start_time, start_states[1], end_time, end_states[1]));
  }

protected:
  State start_states[2], end_states[2];
  Time start_time, end_time;

  std::vector<Segment> segments_data;
  std::vector<State>   states;
};

TEST_F(MultiDofSegmentTest, DefaultConstructor)
{
  MultiDofSegment<Segment> segments;

  EXPECT_FALSE(states.empty());
  segments.sample(1.0, states);
  EXPECT_TRUE(states.empty());

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
    MultiDofSegment<Segment> segments(segments_data);
    EXPECT_EQ(segments_data.size(), segments.size());
    EXPECT_EQ(start_time, segments.startTime());
    EXPECT_EQ(end_time, segments.endTime());
  }
}

TEST_F(MultiDofSegmentTest, SegmentSampler)
{
  MultiDofSegment<Segment> segments(segments_data);
  const Time duration = segments.endTime() - segments.startTime();

  // Sample before segments start
  {
    segments.sample(start_time - duration, states);
    EXPECT_EQ(segments_data.size(), segments.size());

    for (unsigned int i = 0; i < segments.size(); ++i)
    {
      EXPECT_NEAR(start_states[i].position, states[i].position, EPS);
      EXPECT_NEAR(0.0, states[i].velocity, EPS);
      EXPECT_NEAR(0.0, states[i].acceleration, EPS);
    }
  }

  // Sample at mid-segment
  {
    segments.sample(start_time + duration / 2.0, states);
    EXPECT_EQ(segments_data.size(), segments.size());

    for (unsigned int i = 0; i < segments.size(); ++i)
    {
      const double position = (end_states[i].position + start_states[i].position) / 2.0;
      const double velocity = (end_states[i].position - start_states[i].position) / duration;
      EXPECT_NEAR(position, states[i].position, EPS);
      EXPECT_NEAR(velocity, states[i].velocity, EPS);
      EXPECT_NEAR(0.0, states[i].acceleration, EPS);
    }
  }

  // Sample past segment end
  {
    segments.sample(end_time + duration, states);
    EXPECT_EQ(segments_data.size(), segments.size());

    for (unsigned int i = 0; i < segments.size(); ++i)
    {
      EXPECT_NEAR(end_states[i].position, states[i].position, EPS);
      EXPECT_NEAR(0.0, states[i].velocity, EPS);
      EXPECT_NEAR(0.0, states[i].acceleration, EPS);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

