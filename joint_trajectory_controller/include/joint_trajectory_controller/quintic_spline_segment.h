///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
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

/// \author Adolfo Rodriguez Tsouroukdissian, Stuart Glaser, Mrinal Kalakrishnan

#ifndef TRAJECTORY_INTERFACE_QUINTIC_SPLINE_SEGMENT_H
#define TRAJECTORY_INTERFACE_QUINTIC_SPLINE_SEGMENT_H

#include <cmath>
#include <limits>
#include <stdexcept>

#include <boost/array.hpp>
#include <boost/foreach.hpp>

namespace trajectory_interface
{

/**
 * \brief Class representing a one-dimensional quintic spline segment.
 *
 * The segment has a finite duration, which starts at time zero and ends at a specified duration.
 *
 * \tparam Scalar Scalar type
 */
template<class Scalar>
class QuinticSplineSegment
{
public:
  typedef Scalar Time;

  /**
   * \brief Quintic spline sample state.
   * \note When default-constructed, its members are set to NaN to indicate their uninitialized state.
   */
  struct State
  {
    State()
      : position(    std::numeric_limits<Scalar>::quiet_NaN()),
        velocity(    std::numeric_limits<Scalar>::quiet_NaN()),
        acceleration(std::numeric_limits<Scalar>::quiet_NaN())
    {}

    Scalar position;
    Scalar velocity;
    Scalar acceleration;
  };

  /**
   * \brief Creates a segment with zero duration, position, velocity and acceleration.
   */
  QuinticSplineSegment()
    : duration_(static_cast<Scalar>(0))
  {
    BOOST_FOREACH(Scalar& coef, coefs_) {coef = static_cast<Scalar>(0);}
  }

  /**
   * \brief Construct segment from start and end states (boundary conditions).
   *
   * The start and end states need not necessarily be specified all the way to the acceleration level:
   * - If only \b positions are specified, linear interpolation will be used.
   * - If \b positions and \b velocities are specified, a cubic spline will be used.
   * - If \b positions, \b velocities and \b accelerations are specified, a quintic spline will be used.
   *
   * \note If start and end states have different specifications
   * (eg. start is positon-only, end is position-velocity), the lowest common specification will be used
   * (position-only in the example).
   *
   * \param start_state Spline state at time zero.
   * \param end_state Spline state at time \p duration
   * \param duration Segment duration, ie. time at which the segment state equals \p end_state
   *
   * \throw std::invalid_argument If the segment duration is negative or if one of the states is completely
   * uninitialized.
   */
  QuinticSplineSegment(const State& start_state,
                       const State& end_state,
                       const Time&  duration);

  /**
   * \brief Sample the segment at \p state
   *
   * \note Within the <tt>[0,duration]</tt> interval, spline interpolation takes place, outside it this method will output the
   * start/end states with zero velocity and acceleration.
   *
   * \param[in] time Where the segment is to be sampled.
   * \param[out] state Segment state at \p state.
   */
  void sample(const Time& time, State& state)
  {
    sampleWithTimeBounds(coefs_, duration_, time, state.position, state.velocity, state.acceleration);
  }

  /** \return Segment duration. */
  Time duration() const {return duration_;}

private:
  typedef boost::array<Scalar, 6> SplineCoefficients;

  /** Coefficients represent a quintic polynomial like so:
   *
   * <tt> coefs_[0] + coefs_[1]*x + coefs_[2]*x^2 + coefs_[3]*x^3 + coefs_[4]*x^4 + coefs_[5]*x^5 </tt>
   */
  SplineCoefficients coefs_;
  Scalar duration_;

  // These methods are borrowed from the previous controller's implementation
  static void generatePowers(int n, const Scalar& x, Scalar* powers);

  static void computeCoefficients(const Scalar& start_pos,
                                  const Scalar& end_pos,
                                  const Scalar& time,
                                  SplineCoefficients& coefficients);

  static void computeCoefficients(const Scalar& start_pos, const Scalar& start_vel,
                                  const Scalar& end_pos,   const Scalar& end_vel,
                                  const Scalar& time,
                                  SplineCoefficients& coefficients);

  static void computeCoefficients(const Scalar& start_pos, const Scalar& start_vel, const Scalar& start_acc,
                                  const Scalar& end_pos,   const Scalar& end_vel,   const Scalar& end_acc,
                                  const Scalar& time,
                                  SplineCoefficients& coefficients);

  static void sample(const SplineCoefficients& coefficients, const Scalar& time,
                     Scalar& position, Scalar& velocity, Scalar& acceleration);

  static void sampleWithTimeBounds(const SplineCoefficients& coefficients, const Scalar& duration, const Scalar& time,
                                  Scalar& position, Scalar& velocity, Scalar& acceleration);
};

template<class Scalar>
QuinticSplineSegment<Scalar>::QuinticSplineSegment(const State& start_state,
                                                   const State& end_state,
                                                   const Time&  duration)
  : duration_(duration)
{
  if (duration_ < 0.0)
  {
    throw(std::invalid_argument("Quintic spline segment can't be constructed: Negative duration provided."));
  }

  if (std::isnan(start_state.position) || std::isnan(end_state.position))
  {
    throw(std::invalid_argument("Quintic spline segment can't be constructed: Endpoint positions contain NaNs."));
  }
  else if (std::isnan(start_state.velocity) || std::isnan(end_state.velocity))
  {
    computeCoefficients(start_state.position,
                        end_state.position,
                        duration_,
                        coefs_);
  }
  else if (std::isnan(start_state.acceleration) || std::isnan(end_state.acceleration))
  {
    computeCoefficients(start_state.position, start_state.velocity,
                        end_state.position,   end_state.velocity,
                        duration_,
                        coefs_);
  }
  else
  {
    computeCoefficients(start_state.position, start_state.velocity, start_state.acceleration,
                        end_state.position,   end_state.velocity,   end_state.acceleration,
                        duration_,
                        coefs_);
  }
}

template<class Scalar>
inline void QuinticSplineSegment<Scalar>::generatePowers(int n, const Scalar& x, Scalar* powers)
{
  powers[0] = 1.0;
  for (int i=1; i<=n; ++i)
  {
    powers[i] = powers[i-1]*x;
  }
}

template<class Scalar>
void QuinticSplineSegment<Scalar>::
computeCoefficients(const Scalar& start_pos,
                    const Scalar& end_pos,
                    const Scalar& time,
                    SplineCoefficients& coefficients)
{
  coefficients[0] = start_pos;
  coefficients[1] = (time == 0.0) ? 0.0 : (end_pos - start_pos) / time;
  coefficients[2] = 0.0;
  coefficients[3] = 0.0;
  coefficients[4] = 0.0;
  coefficients[5] = 0.0;
}

template<class Scalar>
void QuinticSplineSegment<Scalar>::
computeCoefficients(const Scalar& start_pos, const Scalar& start_vel,
                    const Scalar& end_pos,   const Scalar& end_vel,
                    const Scalar& time,
                    SplineCoefficients& coefficients)
{
  if (time == 0.0)
  {
    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = 0.0;
    coefficients[3] = 0.0;
  }
  else
  {
    double T[4];
    generatePowers(3, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = (-3.0*start_pos + 3.0*end_pos - 2.0*start_vel*T[1] - end_vel*T[1]) / T[2];
    coefficients[3] = (2.0*start_pos - 2.0*end_pos + start_vel*T[1] + end_vel*T[1]) / T[3];
  }
  coefficients[4] = 0.0;
  coefficients[5] = 0.0;
}

template<class Scalar>
void QuinticSplineSegment<Scalar>::
computeCoefficients(const Scalar& start_pos, const Scalar& start_vel, const Scalar& start_acc,
                    const Scalar& end_pos,   const Scalar& end_vel,   const Scalar& end_acc,
                    const Scalar& time,
                    SplineCoefficients& coefficients)
{
  if (time == 0.0)
  {
    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = 0.5*start_acc;
    coefficients[3] = 0.0;
    coefficients[4] = 0.0;
    coefficients[5] = 0.0;
  }
  else
  {
    double T[6];
    generatePowers(5, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = 0.5*start_acc;
    coefficients[3] = (-20.0*start_pos + 20.0*end_pos - 3.0*start_acc*T[2] + end_acc*T[2] -
                       12.0*start_vel*T[1] - 8.0*end_vel*T[1]) / (2.0*T[3]);
    coefficients[4] = (30.0*start_pos - 30.0*end_pos + 3.0*start_acc*T[2] - 2.0*end_acc*T[2] +
                       16.0*start_vel*T[1] + 14.0*end_vel*T[1]) / (2.0*T[4]);
    coefficients[5] = (-12.0*start_pos + 12.0*end_pos - start_acc*T[2] + end_acc*T[2] -
                       6.0*start_vel*T[1] - 6.0*end_vel*T[1]) / (2.0*T[5]);
  }
}

template<class Scalar>
void QuinticSplineSegment<Scalar>::
sample(const SplineCoefficients& coefficients, const Scalar& time,
       Scalar& position, Scalar& velocity, Scalar& acceleration)
{
  // create powers of time:
  double t[6];
  generatePowers(5, time, t);

  position = t[0]*coefficients[0] +
             t[1]*coefficients[1] +
             t[2]*coefficients[2] +
             t[3]*coefficients[3] +
             t[4]*coefficients[4] +
             t[5]*coefficients[5];

  velocity = t[0]*coefficients[1] +
         2.0*t[1]*coefficients[2] +
         3.0*t[2]*coefficients[3] +
         4.0*t[3]*coefficients[4] +
         5.0*t[4]*coefficients[5];

  acceleration = 2.0*t[0]*coefficients[2] +
                 6.0*t[1]*coefficients[3] +
                12.0*t[2]*coefficients[4] +
                20.0*t[3]*coefficients[5];
}

template<class Scalar>
void QuinticSplineSegment<Scalar>::
sampleWithTimeBounds(const SplineCoefficients& coefficients, const Scalar& duration, const Scalar& time,
                     Scalar& position, Scalar& velocity, Scalar& acceleration)
{
  if (time < 0)
  {
    Scalar _;
    sample(coefficients, 0.0, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else if (time > duration)
  {
    Scalar _;
    sample(coefficients, duration, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else
  {
    sample(coefficients, time,
           position, velocity, acceleration);
  }
}

} // namespace

#endif // header guard
