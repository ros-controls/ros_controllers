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
#include <trajectory_interface/quintic_spline_segment.h>

using namespace trajectory_interface;

// Floating-point value comparison threshold
const double EPS = 1e-9;

typedef QuinticSplineSegment<double> Segment;
typedef typename Segment::State State;
typedef typename Segment::Time  Time;

TEST(QuinticSplineSegmentTest, Accessors)
{
  const Time start_time = 1.0;
  State start_state;
  start_state.position = 0.0;

  const Time end_time = 2.0;
  State end_state;
  end_state.position = 2.0;

  Segment segment(start_time, start_state, end_time, end_state);
  EXPECT_EQ(start_time, segment.startTime());
  EXPECT_EQ(end_time, segment.endTime());
}

TEST(QuinticSplineSegmentTest, InvalidSegmentConstruction)
{
  State empty_state;
  State valid_state;
  valid_state.position     = 0.0;
  valid_state.velocity     = 0.0;
  valid_state.acceleration = 0.0;

  const Time start_time       =  1.0;
  const Time valid_end_time   =  2.0;
  const Time invalid_end_time = -1.0;

  // Invalid states trigger an exception
  EXPECT_THROW(Segment(start_time, empty_state, valid_end_time, valid_state), std::invalid_argument);
  EXPECT_THROW(Segment(start_time, valid_state, valid_end_time, empty_state), std::invalid_argument);
  EXPECT_THROW(Segment(start_time, empty_state, valid_end_time, empty_state), std::invalid_argument);
  try{Segment(start_time, empty_state, valid_end_time, empty_state);}
  catch(const std::invalid_argument& ex) {ROS_ERROR_STREAM(ex.what());}

  // Invalid duration triggers an exception
  EXPECT_THROW(Segment(start_time, valid_state, invalid_end_time, valid_state), std::invalid_argument);
  try{Segment(start_time, valid_state, invalid_end_time, valid_state);}
  catch(const std::invalid_argument& ex) {ROS_ERROR_STREAM(ex.what());}
}

TEST(QuinticSplineSegmentTest, DefaultSegmentSampler)
{
  EXPECT_NO_THROW(Segment());
  Segment segment;

  // Negative time sample
  {
    State state;
    segment.sample(-1.0, state);
    EXPECT_EQ(0.0, state.position);
    EXPECT_EQ(0.0, state.velocity);
    EXPECT_EQ(0.0, state.acceleration);
  }

  // Zero time sample
  {
    State state;
    segment.sample(0.0, state);
    EXPECT_EQ(0.0, state.position);
    EXPECT_EQ(0.0, state.velocity);
    EXPECT_EQ(0.0, state.acceleration);
  }

  // Positive time sample
  {
    State state;
    segment.sample(1.0, state);
    EXPECT_EQ(0.0, state.position);
    EXPECT_EQ(0.0, state.velocity);
    EXPECT_EQ(0.0, state.acceleration);
  }
}

TEST(QuinticSplineSegmentTest, ZeroDurationPosEnpointsSampler)
{
  const Time start_time = 1.0;
  State start_state;
  start_state.position = 1.0;

  State end_state;
  end_state.position = 2.0;

  // Construct segment
  EXPECT_NO_THROW(Segment(start_time, start_state, start_time, end_state));
  Segment segment(start_time, start_state, start_time, end_state);

  // Sample before segment start
  {
    State state;
    segment.sample(-start_time, state);
    EXPECT_EQ(start_state.position, state.position);
    EXPECT_EQ(0.0, state.velocity);
    EXPECT_EQ(0.0, state.acceleration);
  }

  // Sample at segment start
  {
    State state;
    segment.sample(start_time, state);
    EXPECT_EQ(start_state.position, state.position);
    EXPECT_EQ(0.0, state.velocity);
    EXPECT_EQ(0.0, state.acceleration);
  }

  // Sample after segment end
  {
    State state;
    segment.sample(2.0 * start_time, state);
    EXPECT_EQ(start_state.position, state.position);
    EXPECT_EQ(0.0, state.velocity);
    EXPECT_EQ(0.0, state.acceleration);
  }
}

TEST(QuinticSplineSegmentTest, ZeroDurationPosVelEnpointsSampler)
{
  const Time start_time = 1.0;
  State start_state;
  start_state.position = 1.0;
  start_state.velocity = 2.0;

  State end_state;
  end_state.position = 2.0;
  end_state.velocity = 4.0;

  // Construct segment
  EXPECT_NO_THROW(Segment(start_time, start_state, start_time, end_state));
  Segment segment(start_time, start_state, start_time, end_state);

  // Sample before segment start
  {
    State state;
    segment.sample(-start_time, state);
    EXPECT_EQ(start_state.position, state.position);
    EXPECT_EQ(0.0, state.velocity);
    EXPECT_EQ(0.0, state.acceleration);
  }

  // Sample at segment start
  {
    State state;
    segment.sample(start_time, state);
    EXPECT_EQ(start_state.position, state.position);
    EXPECT_EQ(start_state.velocity, state.velocity);
    EXPECT_EQ(0.0, state.acceleration);
  }

  // Sample after segment end
  {
    State state;
    segment.sample(2.0 * start_time, state);
    EXPECT_EQ(start_state.position, state.position);
    EXPECT_EQ(0.0, state.velocity);
    EXPECT_EQ(0.0, state.acceleration);
  }
}

TEST(QuinticSplineSegmentTest, ZeroDurationPosVelAccEnpointsSampler)
{
  const Time start_time = 1.0;
  State start_state;
  start_state.position     = 1.0;
  start_state.velocity     = 2.0;
  start_state.acceleration = 3.0;

  State end_state;
  end_state.position     = 2.0;
  end_state.velocity     = 4.0;
  end_state.acceleration = 6.0;

  // Construct segment
  EXPECT_NO_THROW(Segment(start_time, start_state, start_time, end_state));
  Segment segment(start_time, start_state, start_time, end_state);

  // Sample before segment start
  {
    State state;
    segment.sample(-start_time, state);
    EXPECT_EQ(start_state.position, state.position);
    EXPECT_EQ(0.0, state.velocity);
    EXPECT_EQ(0.0, state.acceleration);
  }

  // Sample at segment start
  {
    State state;
    segment.sample(start_time, state);
    EXPECT_EQ(start_state.position, state.position);
    EXPECT_EQ(start_state.velocity, state.velocity);
    EXPECT_EQ(start_state.acceleration, state.acceleration);
  }

  // Sample after segment end
  {
    State state;
    segment.sample(2.0 * start_time, state);
    EXPECT_EQ(start_state.position, state.position);
    EXPECT_EQ(0.0, state.velocity);
    EXPECT_EQ(0.0, state.acceleration);
  }
}

TEST(QuinticSplineSegmentTest, PosEnpointsSampler)
{
  const Time start_time = 1.0;
  State start_state;
  start_state.position = 0.0;

  const Time end_time = 3.0;
  State end_state;
  end_state.position     = 2.0;
  end_state.velocity     = 0.0; // Should be ignored, as start state does not specify it
  end_state.acceleration = 0.0; // Should be ignored, as start state does not specify it

  const Time duration = end_time - start_time;

  const double velocity = (end_state.position - start_state.position) / duration;

  // Construct segment
  EXPECT_NO_THROW(Segment(start_time, start_state, end_time, end_state));
  Segment segment(start_time, start_state, end_time, end_state);

  // Sample before segment start
  {
    State state;
    segment.sample(start_time - duration, state);
    EXPECT_NEAR(start_state.position, state.position, EPS);
    EXPECT_NEAR(0.0, state.velocity, EPS);
    EXPECT_NEAR(0.0, state.acceleration, EPS);
  }

  // Sample at segment start
  {
    State state;
    segment.sample(start_time, state);
    EXPECT_NEAR(start_state.position, state.position, EPS);
    EXPECT_NEAR(velocity, state.velocity, EPS);
    EXPECT_NEAR(0.0, state.acceleration, EPS);
  }

  // Sample at mid-segment
  {
    State state;
    segment.sample(start_time + duration / 2.0, state);
    EXPECT_NEAR(end_state.position / 2.0, state.position, EPS);
    EXPECT_NEAR(velocity, state.velocity, EPS);
    EXPECT_NEAR(0.0, state.acceleration, EPS);
  }

  // Sample at segment end
  {
    State state;
    segment.sample(end_time, state);
    EXPECT_NEAR(end_state.position, state.position, EPS);
    EXPECT_NEAR(velocity, state.velocity, EPS);
    EXPECT_NEAR(0.0, state.acceleration, EPS);
  }

  // Sample past segment end
  {
    State state;
    segment.sample(end_time + duration, state);
    EXPECT_NEAR(end_state.position, state.position, EPS);
    EXPECT_NEAR(0.0, state.velocity, EPS);
    EXPECT_NEAR(0.0, state.acceleration, EPS);
  }
}

TEST(QuinticSplineSegmentTest, PosVelEnpointsSampler)
{
  // Start and end state taken from x^3 - 2x
  const Time start_time = 1.0;
  State start_state;
  start_state.position =  0.0;
  start_state.velocity = -2.0;

  const Time end_time = 3.0;
  State end_state;
  end_state.position     =  4.0;
  end_state.velocity     = 10.0;
  end_state.acceleration =  0.0; // Should be ignored, as start state does not specify it

  const Time duration = end_time - start_time;

  // Construct segment
  EXPECT_NO_THROW(Segment(start_time, start_state, end_time, end_state));
  Segment segment(start_time, start_state, end_time, end_state);

  // Sample before segment start
  {
    State state;
    segment.sample(start_time - duration, state);
    EXPECT_NEAR(start_state.position, state.position, EPS);
    EXPECT_NEAR(0.0, state.velocity, EPS);
    EXPECT_NEAR(0.0, state.acceleration, EPS);
  }

  // Sample at segment start
  {
    State state;
    segment.sample(start_time, state);
    EXPECT_NEAR(start_state.position, state.position, EPS);
    EXPECT_NEAR(start_state.velocity, state.velocity, EPS);
    EXPECT_NEAR(0.0, state.acceleration, EPS);
  }

  // Sample at mid-segment: Zero-crossing
  {
    const double time = start_time + std::sqrt(2.0);

    State state;
    segment.sample(time, state);
    EXPECT_NEAR(0.0, state.position, EPS);
    EXPECT_NEAR(4.0, state.velocity, EPS);
    EXPECT_NEAR(6.0 * std::sqrt(2.0), state.acceleration, EPS);
  }

  // Sample at segment end
  {
    State state;
    segment.sample(end_time, state);
    EXPECT_NEAR(end_state.position, state.position, EPS);
    EXPECT_NEAR(end_state.velocity, state.velocity, EPS);
    EXPECT_NEAR(12.0, state.acceleration, EPS);
  }

  // Sample past segment end
  {
    State state;
    segment.sample(end_time + duration, state);
    EXPECT_NEAR(end_state.position, state.position, EPS);
    EXPECT_NEAR(0.0, state.velocity, EPS);
    EXPECT_NEAR(0.0, state.acceleration, EPS);
  }
}

TEST(QuinticSplineSegmentTest, PosVeAcclEnpointsSampler)
{
  // Start and end state taken from x(x-1)(x-2)(x-3)(x-4) = x^5 -10x^4 + 35x^3 -50x^2 + 24x
  const Time start_time = 1.0;
  State start_state;
  start_state.position     =    0.0;
  start_state.velocity     =   24.0;
  start_state.acceleration = -100.0;

  const Time end_time = 3.0;
  State end_state;
  end_state.position     = 0.0;
  end_state.velocity     = 4.0;
  end_state.acceleration = 0.0;

  const Time duration = end_time - start_time;

  // Construct segment
  EXPECT_NO_THROW(Segment(start_time, start_state, end_time, end_state));
  Segment segment(start_time, start_state, end_time, end_state);

  // Sample before segment start
  {
    State state;
    segment.sample(start_time - duration, state);
    EXPECT_NEAR(start_state.position, state.position, EPS);
    EXPECT_NEAR(0.0, state.velocity, EPS);
    EXPECT_NEAR(0.0, state.acceleration, EPS);
  }

  // Sample at segment start
  {
    State state;
    segment.sample(start_time, state);
    EXPECT_NEAR(start_state.position, state.position, EPS);
    EXPECT_NEAR(start_state.velocity, state.velocity, EPS);
    EXPECT_NEAR(start_state.acceleration, state.acceleration, EPS);
  }

  // Sample at mid-segment: Zero-crossing
  {
    State state;
    segment.sample(start_time + 1.0, state);
    EXPECT_NEAR( 0.0, state.position, EPS);
    EXPECT_NEAR(-6.0, state.velocity, EPS);
    EXPECT_NEAR(10.0, state.acceleration, EPS);
  }

  // Sample at segment end
  {
    State state;
    segment.sample(end_time, state);
    EXPECT_NEAR(end_state.position, state.position, EPS);
    EXPECT_NEAR(end_state.velocity, state.velocity, EPS);
    EXPECT_NEAR(end_state.acceleration, state.acceleration, EPS);
  }

  // Sample past segment end
  {
    State state;
    segment.sample(end_time + duration, state);
    EXPECT_NEAR(end_state.position, state.position, EPS);
    EXPECT_NEAR(0.0, state.velocity, EPS);
    EXPECT_NEAR(0.0, state.acceleration, EPS);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

