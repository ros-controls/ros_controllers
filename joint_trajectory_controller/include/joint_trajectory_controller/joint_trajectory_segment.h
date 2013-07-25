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
#include <vector>

#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <trajectory_interface/quintic_spline_segment.h> // TODO: Remove?
#include <trajectory_interface/multi_dof_segment.h>
#include <joint_trajectory_controller/trajectory_interface_ros.h>

namespace trajectory_interface
{

/**
 * \brief Class representing a multi-dimensional quintic spline segment with a start and end time.
 *
 * It additionally allows to construct the segment and its state type from ROS message data.
 */
template <class Scalar>
class JointTrajectorySegment : public MultiDofSegment<QuinticSplineSegment<Scalar> >
{
public:
  typedef typename MultiDofSegment<QuinticSplineSegment<Scalar> >::Time  Time;

  struct State : public MultiDofSegment<QuinticSplineSegment<Scalar> >::State // NOTE: Inheriting from std::vector, not particularly nice!
  {
    State() : MultiDofSegment<QuinticSplineSegment<Scalar> >::State() {}

    /**
     * \param point Trajectory point.
     *
     * \param permutation Permutation vector for mapping the joint order of a \p point to a desired order.
     * For instance, if \p point contains data associated to joints <tt>"{B, D, A, C}"</tt>, and we are interested in
     * constructing a segment with joints ordered as <tt>"{A, B, C, D}"</tt>, the permutation vector should
     * be set to <tt>"{2, 0, 3, 1}"</tt>.
     * If unspecified (empty), the joint order of \p point is preserved; if specified, its size must coincide with that
     * of \p point.
     * This vector can be computed using the \ref trajectory_interface::permutation()
     * "trajectory_interface::permutation" utility function.
     *
     * \param position_offset Position offset to applpy to the data in \p point. This parameter is useful for handling
     * joints that wrap around (ie. continuous), to compensate for multi-turn offsets.
     * If unspecified (empty), zero offsets are applied; if specified, its size must coincide with that of \p point.
     *
     * \note The offsets in \p position_offsets correspond to joints not ordered according to \p point, but to joints
     * in the expected order, that is \p point with \p permutation applied to it.
     */
    State(const trajectory_msgs::JointTrajectoryPoint& point,
          const std::vector<unsigned int>&             permutation     = std::vector<unsigned int>(),
          const std::vector<double>&                   position_offset = std::vector<double>())
    {
      using std::invalid_argument;

      const unsigned int joint_dim = point.positions.size();
      if (!isValid(point, joint_dim))
      {
        throw(invalid_argument("Size mismatch in trajectory point position, velocity or acceleration data."));
      }
      if (!permutation.empty() && joint_dim != permutation.size())
      {
        throw(invalid_argument("Size mismatch between trajectory point and permutation vector."));
      }
      for (unsigned int i = 0; i < permutation.size(); ++i)
      {
        if (permutation[i] >= joint_dim)
        {
          throw(invalid_argument("Permutation vector contains out-of-range indices."));
        }
      }
      if (!position_offset.empty() && joint_dim != position_offset.size())
      {
        throw(invalid_argument("Size mismatch between trajectory point "
                               "and vector specifying whether joints wrap around."));
      }

      this->resize(joint_dim);
      for (unsigned int i = 0; i < joint_dim; ++i)
      {
        // Apply permutation only if it was specified, otherwise preserve original message order
        const unsigned int id = permutation.empty() ? i : permutation[i];

        // Apply position offset only if it was specified
        const double offset = position_offset.empty() ? 0.0 : position_offset[i];

        typename QuinticSplineSegment<Scalar>::State single_joint_state;
        if (!point.positions.empty())     {single_joint_state.position     = point.positions[id] + offset;}
        if (!point.velocities.empty())    {single_joint_state.velocity     = point.velocities[id];}
        if (!point.accelerations.empty()) {single_joint_state.acceleration = point.accelerations[id];}

        (*this)[i] = single_joint_state;
      }
    }
  };

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
   * \param traj_start_time Time at which the trajectory containing the segment starts. Note that this is \e not the
   * segment start time.
   * \param start_point Start state in ROS message format.
   * \param end_point End state in ROS message format.
   * \param permutation See \ref JointTrajectorySegment::State.
   * \param position_offset See \ref JointTrajectorySegment::State.
   *
   * \throw std::invalid_argument If input parameters are inconsistent and a valid segment can't be constructed.
   */
  JointTrajectorySegment(const ros::Time&                             traj_start_time,
                         const trajectory_msgs::JointTrajectoryPoint& start_point,
                         const trajectory_msgs::JointTrajectoryPoint& end_point,
                         const std::vector<unsigned int>&             permutation     = std::vector<unsigned int>(),
                         const std::vector<double>&                   position_offset = std::vector<double>())
  {
    if (start_point.positions.size() != end_point.positions.size())
    {
      throw(std::invalid_argument("Can't construct segment from ROS message: "
                                  "Start/end points data size mismatch."));
    }

    const Time start_time = (traj_start_time + start_point.time_from_start).toSec();
    const Time end_time   = (traj_start_time + end_point.time_from_start).toSec();

    try
    {
      const State start_state(start_point, permutation, position_offset);
      const State end_state(end_point,     permutation, position_offset);

      this->init(start_time, start_state,
                 end_time,   end_state);
    }
    catch(const std::invalid_argument& ex)
    {
      std::string msg = "Can't construct segment from ROS message: " + std::string(ex.what());
      throw(std::invalid_argument(msg));
    }
  }
};

} // namespace

#endif // header guard
