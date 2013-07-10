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

#ifndef TRAJECTORY_INTERFACE_TRAJECTORY_INTERFACE_H
#define TRAJECTORY_INTERFACE_TRAJECTORY_INTERFACE_H

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
// TODO: Add a traits class to ease wrapping of 3rd party code?

namespace trajectory_interface
{

/**
 * TODO: Doc!
 */
template<class Segment>
class Trajectory
{
public:
  typedef typename Segment::State State;
  typedef typename Segment::Time  Time;

  typedef int SegmentId;
  typedef unsigned int TrajectoryDim;
  typedef unsigned int TrajectorySize;

  // TODO: Add constructor

  SegmentId sample(const Time& time, State& state)
  {
    const int seg = getSegmentId(time);
    const Time seg_time = time - start_times_[seg]; // Time from segment start
    if (seg != -1) {multi_dof_segments_[seg].sample(seg_time, state);}
    return seg;
  }

  TrajectoryDim dimension() const {return dim_;}
  TrajectoryDim size() const {return multi_dof_segments_.size();}
  TrajectoryDim empty() const {return multi_dof_segments_.empty();}

  /**
   * \brief Get an identifier to the segment containing \p time, that is, a segment that starts before or at \p time.
   * \param time Time the desired segment should contain.
   * \return Segment identifier >= 0 if found, -1 if the trajectory is empty or \p time preceeds the trajectory start
   * time.
   */
  SegmentId getSegmentId(const Time& time)
  {
    typename std::vector<Segment>::const_iterator it = std::upper_bound(start_times_.begin(), start_times_.end(), time);
    return std::distance(start_times_.begin(), --it);
  }

private:
  TrajectoryDim dim_;
  std::vector<Segment> multi_dof_segments_;
  std::vector<Time>  start_times_;
};

} // namespace

#endif // header guard
