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

/// \author Adolfo Rodriguez Tsouroukdissian, Stuart Glaser, Mrinal Kalakrishnan

#ifndef TRAJECTORY_INTERFACE_QUINTIC_SPLINE_SEGMENT_H
#define TRAJECTORY_INTERFACE_QUINTIC_SPLINE_SEGMENT_H

#include <iterator>
#include <stdexcept>

#include <boost/array.hpp>
#include <boost/foreach.hpp>

#include <trajectory_interface/pos_vel_acc_state.h>

namespace trajectory_interface
{

/**
 * \brief Class representing a multi-dimensional quintic spline segment with a start and end time.
 *
 * \tparam ScalarType Scalar type
 */
template<class ScalarType>
class QuinticSplineSegment
{
public:
  typedef ScalarType             Scalar;
  typedef Scalar                 Time;
  typedef PosVelAccState<Scalar> State;

  /**
   * \brief Creates an empty segment.
   *
   * \note Calling <tt> size() </tt> on an empty segment will yield zero, and sampling it will yield a state with empty
   * data.
   */
  QuinticSplineSegment()
    : coefs_(),
      duration_(static_cast<Scalar>(0)),
      start_time_(static_cast<Scalar>(0))
  {}

  /**
   * \brief Construct segment from start and end states (boundary conditions).
   *
   * Please refer to the \ref init method documentation for the description of each parameter and the exceptions that
   * can be thrown.
   */
  QuinticSplineSegment(const Time&  start_time,
                       const State& start_state,
                       const Time&  end_time,
                       const State& end_state)
  {
    init(start_time, start_state, end_time, end_state);
  }

  /**
   * \brief Initialize segment from start and end states (boundary conditions).
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
   * \param start_time Time at which the segment state equals \p start_state.
   * \param start_state State at \p start_time.
   * \param end_time Time at which the segment state equals \p end_state.
   * \param end_state State at time \p end_time.
   *
   * \throw std::invalid_argument If the \p end_time is earlier than \p start_time or if one of the states is
   * uninitialized.
   */
  void init(const Time&  start_time,
            const State& start_state,
            const Time&  end_time,
            const State& end_state);

  /**
   * \brief Sample the segment at a specified time.
   *
   * \note Within the <tt>[start_time, end_time]</tt> interval, spline interpolation takes place, outside it this method
   * will output the start/end states with zero velocity and acceleration.
   *
   * \param[in] time Where the segment is to be sampled.
   * \param[out] state Segment state at \p time.
   */
  void sample(const Time& time, State& state) const
  {
    // Resize state data. Should be a no-op if appropriately sized
    state.position.resize(coefs_.size());
    state.velocity.resize(coefs_.size());
    state.acceleration.resize(coefs_.size());

    // Sample each dimension
    typedef typename std::vector<SplineCoefficients>::const_iterator ConstIterator;
    for(ConstIterator coefs_it = coefs_.begin(); coefs_it != coefs_.end(); ++coefs_it)
    {
      const typename std::vector<Scalar>::size_type id = std::distance(coefs_.begin(), coefs_it);
      sampleWithTimeBounds(*coefs_it,
                           duration_, (time - start_time_),
                           state.position[id], state.velocity[id], state.acceleration[id]);
    }
  }

  /** \return Segment start time. */
  Time startTime() const {return start_time_;}

  /** \return Segment end time. */
  Time endTime() const {return start_time_ + duration_;}

  /** \return Segment size (dimension). */
  unsigned int size() const {return coefs_.size();}

private:
  typedef boost::array<Scalar, 6> SplineCoefficients;

  /** Coefficients represent a quintic polynomial like so:
   *
   * <tt> coefs_[0] + coefs_[1]*x + coefs_[2]*x^2 + coefs_[3]*x^3 + coefs_[4]*x^4 + coefs_[5]*x^5 </tt>
   */
  std::vector<SplineCoefficients> coefs_;
  Time duration_;
  Time start_time_;

  // These methods are borrowed from the previous controller's implementation
  // TODO: Clean their implementation, use the Horner algorithm for more numerically stable polynomial evaluation
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

template<class ScalarType>
void QuinticSplineSegment<ScalarType>::init(const Time&  start_time,
                                            const State& start_state,
                                            const Time&  end_time,
                                            const State& end_state)
{
  // Preconditions
  if (end_time < start_time)
  {
    throw(std::invalid_argument("Quintic spline segment can't be constructed: end_time < start_time."));
  }
  if (start_state.position.empty() || end_state.position.empty())
  {
    throw(std::invalid_argument("Quintic spline segment can't be constructed: Endpoint positions can't be empty."));
  }
  if (start_state.position.size() != end_state.position.size())
  {
    throw(std::invalid_argument("Quintic spline segment can't be constructed: Endpoint positions size mismatch."));
  }

  const unsigned int dim = start_state.position.size();
  const bool has_velocity     = !start_state.velocity.empty()     && !end_state.velocity.empty();
  const bool has_acceleration = !start_state.acceleration.empty() && !end_state.acceleration.empty();

  if (has_velocity && dim != start_state.velocity.size())
  {
    throw(std::invalid_argument("Quintic spline segment can't be constructed: Start state velocity size mismatch."));
  }
  if (has_velocity && dim != end_state.velocity.size())
  {
    throw(std::invalid_argument("Quintic spline segment can't be constructed: End state velocity size mismatch."));
  }
  if (has_acceleration && dim!= start_state.acceleration.size())
  {
    throw(std::invalid_argument("Quintic spline segment can't be constructed: Start state acceleration size mismatch."));
  }
  if (has_acceleration && dim != end_state.acceleration.size())
  {
    throw(std::invalid_argument("Quintic spline segment can't be constructed: End state acceleratios size mismatch."));
  }

  // Time data
  start_time_ = start_time;
  duration_   = end_time - start_time;

  // Spline coefficients
  coefs_.resize(dim);

  typedef typename std::vector<SplineCoefficients>::iterator Iterator;
  if (!has_velocity)
  {
    // Linear interpolation
    for(Iterator coefs_it = coefs_.begin(); coefs_it != coefs_.end(); ++coefs_it)
    {
      const typename std::vector<Scalar>::size_type id = std::distance(coefs_.begin(), coefs_it);

      computeCoefficients(start_state.position[id],
                          end_state.position[id],
                          duration_,
                          *coefs_it);
    }
  }
  else if (!has_acceleration)
  {
    // Cubic interpolation
    for(Iterator coefs_it = coefs_.begin(); coefs_it != coefs_.end(); ++coefs_it)
    {
      const typename std::vector<Scalar>::size_type id = std::distance(coefs_.begin(), coefs_it);

      computeCoefficients(start_state.position[id], start_state.velocity[id],
                          end_state.position[id],   end_state.velocity[id],
                          duration_,
                          *coefs_it);
    }
  }
  else
  {
    // Quintic interpolation
    for(Iterator coefs_it = coefs_.begin(); coefs_it != coefs_.end(); ++coefs_it)
    {
      const typename std::vector<Scalar>::size_type id = std::distance(coefs_.begin(), coefs_it);

      computeCoefficients(start_state.position[id], start_state.velocity[id], start_state.acceleration[id],
                          end_state.position[id],   end_state.velocity[id],   end_state.acceleration[id],
                          duration_,
                          *coefs_it);
    }
  }
}

template<class ScalarType>
inline void QuinticSplineSegment<ScalarType>::generatePowers(int n, const Scalar& x, Scalar* powers)
{
  powers[0] = 1.0;
  for (int i=1; i<=n; ++i)
  {
    powers[i] = powers[i-1]*x;
  }
}

template<class ScalarType>
void QuinticSplineSegment<ScalarType>::
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

template<class ScalarType>
void QuinticSplineSegment<ScalarType>::
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
    Scalar T[4];
    generatePowers(3, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = (-3.0*start_pos + 3.0*end_pos - 2.0*start_vel*T[1] - end_vel*T[1]) / T[2];
    coefficients[3] = (2.0*start_pos - 2.0*end_pos + start_vel*T[1] + end_vel*T[1]) / T[3];
  }
  coefficients[4] = 0.0;
  coefficients[5] = 0.0;
}

template<class ScalarType>
void QuinticSplineSegment<ScalarType>::
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
    Scalar T[6];
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

template<class ScalarType>
void QuinticSplineSegment<ScalarType>::
sample(const SplineCoefficients& coefficients, const Scalar& time,
       Scalar& position, Scalar& velocity, Scalar& acceleration)
{
  // create powers of time:
  Scalar t[6];
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

template<class ScalarType>
void QuinticSplineSegment<ScalarType>::
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
