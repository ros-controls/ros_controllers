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

#ifndef TRAJECTORY_INTERFACE_JOINT_TRAJECTORY_SEGMENT_H
#define TRAJECTORY_INTERFACE_JOINT_TRAJECTORY_SEGMENT_H

#include <stdexcept>

#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <trajectory_interface/quintic_spline_segment.h>
#include <trajectory_interface/multi_dof_segment.h>
#include <joint_trajectory_controller/trajectory_interface_ros.h>

namespace trajectory_interface
{
/**
 * \brief Class representing a multi-dimensional quintic spline segment with a start and end time.
 *
 * It additionally allows to construct the segment from ROS message data.
 */
template <class Scalar>
class JointTrajectorySegment : public MultiDofSegment<QuinticSplineSegment<Scalar> >
{
public:
  typedef typename MultiDofSegment<QuinticSplineSegment<Scalar> >::Time  Time;
  typedef typename MultiDofSegment<QuinticSplineSegment<Scalar> >::State State;

  /**
   * \brief Construct segment from start and end states (boundary conditions).
   *
   * \param start_time Time at which the segment state equals \p start_state.
   * \param start_state State at \p start_time.
   * \param end_time Time at which the segment state equals \p end_state.
   * \param end_state State at time \p end_time.
   */
  JointTrajectorySegment(const Time&  start_time,
                         const State& start_state,
                         const Time&  end_time,
                         const State& end_state)
  {
    MultiDofSegment<QuinticSplineSegment<Scalar> >::init(start_time, start_state, end_time, end_state);
  }

  /**
   * \brief Construct a segment from start and end points (boundary conditions) specified in ROS message format.
   *
   *
   * \param traj_start_time Time at which the trajectory containing the segment starts. Note that this is \e not the
   * segment start time.
   * \param start_point Start state in ROS message format.
   * \param end_point End state in ROS message format.
   *
   * \throw std::invalid_argument If input parameters are inconsistent and a valid segment can't be constructed.
   */
  JointTrajectorySegment(const ros::Time&                             traj_start_time,
                         const trajectory_msgs::JointTrajectoryPoint& start_point,
                         const trajectory_msgs::JointTrajectoryPoint& end_point)
  {
    using std::invalid_argument;

    // Preconditions
    if (start_point.positions.empty())
    {
      throw(invalid_argument("Can't construct segment from ROS message: Position of start point is unset."));
    }
    if (start_point.positions.size() != end_point.positions.size())
    {
      throw(invalid_argument("Can't construct segment from ROS message: Start/end points data size mismatch."));
    }
    const unsigned int joint_dim = start_point.positions.size();
    if (!isValid(start_point, joint_dim))
    {
      throw(invalid_argument("Can't construct segment from ROS message: "
                             "Start point has a size mismatch in position, velocity or acceleration data."));
    }
    if (!isValid(end_point, joint_dim))
    {
      throw(invalid_argument("Can't construct segment from ROS message: "
                             "End point has a size mismatch in position, velocity or acceleration data."));
    }

    // Construct start and end time/state from message data
    const bool has_velocity     = !start_point.velocities.empty() && !end_point.velocities.empty();
    const bool has_acceleration = !start_point.accelerations.empty() && !end_point.accelerations.empty();

    typedef QuinticSplineSegment<Scalar>      SingleDofSegment;
    typedef MultiDofSegment<SingleDofSegment> MultiDofSegment;

    const typename MultiDofSegment::Time start_time = (traj_start_time + start_point.time_from_start).toSec();
    const typename MultiDofSegment::Time end_time   = (traj_start_time + end_point.time_from_start).toSec();

    std::vector<typename SingleDofSegment::State> multi_dof_start_state;
    std::vector<typename SingleDofSegment::State> multi_dof_end_state;
    for (unsigned int i = 0; i < start_point.positions.size(); ++i)
    {
      typename SingleDofSegment::State start_state;
      typename SingleDofSegment::State end_state;

      start_state.position                            = start_point.positions[i];
      if (has_velocity)     {start_state.velocity     = start_point.velocities[i];}
      if (has_acceleration) {start_state.acceleration = start_point.accelerations[i];}

      end_state.position                            = end_point.positions[i];
      if (has_velocity)     {end_state.velocity     = end_point.velocities[i];}
      if (has_acceleration) {end_state.acceleration = end_point.accelerations[i];}

      multi_dof_start_state.push_back(start_state);
      multi_dof_end_state.push_back(end_state);
    }

    this->init(start_time, multi_dof_start_state,
               end_time,   multi_dof_end_state);
  }
};

} // namespace

#endif // header guard
