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

#pragma once


#include <algorithm>
#include <iterator>
#include <vector>

// Things to generalize:
// - State specification: What values (pos, vel, acc, ...), and their dimension (4 for quats, 1 for scalars)
// - State might be different for derivatives: Quat == 4, ang vel == 3: Solve doing w=[w0, w1, w2, 0]
// - Sample state vs. sample pos and derivatives separately
// - Manage and find active segments of heretogeneous types

// NOTES:
// - Segments in original implementation are an open interval on the start time. We are making them closed
// - If an action goal superceeds another and the new trajectory starts in the future, until that time no constraint
//   enforcement will take place. Odd.

namespace trajectory_interface
{

namespace internal
{

// NOTE: Implement as a lambda expression when C++11 becomes mainstream
template<class Time, class Segment>
inline bool isBeforeSegment(const Time& time, const Segment& segment)
{
  return time < segment.startTime();
}

} // namespace

/**
 * \brief Find an iterator to the segment containing a specified \p time.
 *
 * \param first Input iterator to the initial position in the segment sequence.
 * \param last Input iterator to the final position in the segment sequence.
 * The range searched is <tt>[first,last)</tt>.
 * \param time Time to search for in the range.
 *
 * \return Iterator to the trajectory segment containing \p time. If no segment contains \p time (ie. it's earlier
 * than the start time of \p first), then \p last is returned.
 *
 * \pre The range <tt>[first,last)</tt> should be sorted by segment start time.
 *
 * \note On average, this method has logarithmic time complexity when used on \b random-access iterators.
 * On \b non-random-access iterators, iterator advances incur an additional linear time cost.
 */
template<class TrajectoryIterator, class Time>
inline TrajectoryIterator findSegment(TrajectoryIterator first, TrajectoryIterator last, const Time& time)
{
  typedef typename std::iterator_traits<TrajectoryIterator>::value_type Segment;
  return (first == last || internal::isBeforeSegment(time, *first))
         ? last // Optimization when time preceeds all segments, or when an empty range is passed
         : --std::upper_bound(first, last, time, internal::isBeforeSegment<Time, Segment>); // Notice decrement operator
}

/**
 * \brief Find an iterator to the segment containing a specified \p time.
 *
 * This is a convenience method wrapping the iterator-based
 * \ref findSegment(TrajectoryIterator first, TrajectoryIterator last, const Time& time) "findSegment" overload.
 *
 * \tparam Trajectory Trajectory type. Should be a \e sequence container \e sorted by segment start time.
 *
 * \sa findSegment(TrajectoryIterator first, TrajectoryIterator last, const Time& time)
 */
template<class Trajectory, class Time>
inline typename Trajectory::const_iterator findSegment(const Trajectory& trajectory, const Time& time)
{
  return findSegment(trajectory.begin(), trajectory.end(), time);
}

/**
 * \brief Equivalent to \ref findSegment(TrajectoryIterator first, TrajectoryIterator last, const Time& time) "findSegment"
 * but returning a non-const iterator.
 *
 * \note Although \p is passed by non-const reference, it is not modified by this function. It's a workaround to allow
 * overloads of this function to coexist (they need different signatures to prevent ambiguities).
 */
template<class Trajectory, class Time>
inline typename Trajectory::iterator findSegment(Trajectory& trajectory, const Time& time)
{
  return findSegment(trajectory.begin(), trajectory.end(), time);
}

/**
 * \brief Sample a trajectory at a specified time.
 *
 * This is a convenience function that combines finding the segment associated to a specified time (see \ref findSegment)
 * and sampling it.
 *
 * \tparam Trajectory Trajectory type. Should be a \e sequence container \e sorted by segment start time.
 * \param[in] trajectory Holds a sequence of segments.
 * \param[in] time Where the trajectory is to be sampled.
 * \param[out] state Segment state at \p time.
 *
 * \return Iterator to the trajectory segment containing \p time. If no segment contains \p time, then
 * <tt>trajectory.end()</tt> is returned.
 *
 * \note The segment implementation should allow sampling outside the trajectory time interval, implementing a policy
 * such as holding the first/last position. In such a case, it is possible to get a valid sample for all time inputs,
 * even for the cases when this function returns <tt>trajectory.end()</tt>.
 *
 * \sa findSegment
 */
template<class Trajectory>
inline typename Trajectory::const_iterator sample(const Trajectory&                             trajectory,
                                                  const typename Trajectory::value_type::Time&  time,
                                                        typename Trajectory::value_type::State& state)
{
  typename Trajectory::const_iterator it = findSegment(trajectory, time);
  if (it != trajectory.end())
  {
    it->sample(time, state); // Segment found at specified time
  }
  else if (!trajectory.empty())
  {
    trajectory.front().sample(time, state); // Specified time preceeds trajectory start time
  }
  return it;
}

} // namespace
