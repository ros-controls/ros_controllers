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

#ifndef TRAJECTORY_INTERFACE_MULTI_DOF_SEGMENT_H
#define TRAJECTORY_INTERFACE_MULTI_DOF_SEGMENT_H

#include <vector>

namespace trajectory_interface
{

/**
 * \brief Class representing a multi-dimensional trajectory segment.
 *
 * \tparam Segment Must implement the following types:
 * \code
 * Segment::State;
 * Segment::Time;
 * \endcode
 * and the following methods:
 * \code
 * void sample(const Time& time, const State& state);
 * \endcode
 */
template<class Segment>
class MultiDofSegment
{
public:
  typedef typename Segment::Time               Time;
  typedef std::vector<typename Segment::State> State;

  /**
   * \brief Creates an empty container.
   */
  MultiDofSegment() {}

  /**
   * \brief Creates a multi-dof segment from \p multidof_segment.
   */
  MultiDofSegment(const std::vector<Segment>& multidof_segment) : multidof_segment_(multidof_segment) {}

  /**
   * \brief sample
   * \param[in] time Where the segment is to be sampled.
   * \param[out] state Segment state at \p state.
   */
  void sample(const Time& time, State& state)
  {
    state.resize(multidof_segment_.size()); // Should be a no-op if appropriately sized
    for(unsigned int i = 0; i < multidof_segment_.size(); ++i) {multidof_segment_[i].sample(time, state[i]);}
  }

  /** \return Number of degrees of freedom handled by this instance. */
  unsigned int size() const {return multidof_segment_.size();}

private:
  std::vector<Segment> multidof_segment_;
};

} // namespace

#endif // header guard
