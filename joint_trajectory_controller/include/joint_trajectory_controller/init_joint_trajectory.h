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

#ifndef TRAJECTORY_INTERFACE_INIT_JOINT_TRAJECTORY_H
#define TRAJECTORY_INTERFACE_INIT_JOINT_TRAJECTORY_H

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <angles/angles.h>

#include <joint_trajectory_controller/trajectory_interface_ros.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>

namespace trajectory_interface
{

namespace internal
{
/**
 * \return The permutation vector between two containers.
 * If \p t1 is <tt>"{A, B, C, D}"</tt> and \p t2 is <tt>"{B, D, A, C}"</tt>, the associated permutation vector is
 * <tt>"{2, 0, 3, 1}"</tt>.
 */
template <class T>
inline std::vector<typename T::size_type> permutation(const T& t1, const T& t2)
{
  typedef typename T::size_type SizeType;

  // Arguments must have the same size
  if (t1.size() != t2.size()) {return std::vector<SizeType>();}

  std::vector<SizeType> permutation_vector(t1.size()); // Return value
  for (typename T::const_iterator t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    typename T::const_iterator t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it) {return std::vector<SizeType>();}
    else
    {
      const typename T::size_type t1_dist = std::distance(t1.begin(), t1_it);
      const typename T::size_type t2_dist = std::distance(t2.begin(), t2_it);
      permutation_vector[t1_dist] = t2_dist;
    }
  }
  return permutation_vector;
}

} // namespace


/**
 * \brief Initialize a joint trajectory from ROS message data.
 *
 * \param msg Trajectory message.
 *
 * \param time Time from which data is to be extracted. All trajectory points in \p msg occurring \b after
 * \p time <b>will be extracted</b>; or put otherwise, all points occurring at a time <b>less or equal</b> than \p time
 * <b>will be discarded</b>. Set this value to zero to process all points in \p msg.
 *
 * \param joint_names Joints expected to be found in \p msg. If specified, this function will return an empty trajectory
 * when \p msg contains joints that differ in number or names to \p joint_names. If \p msg contains the same joints as
 * \p  joint_names, but in a different order, the resulting trajectory will be ordered according to \p joint_names
 * (and not \p msg). Finally, if this parameter is unspecified (empty), no checks will be performed against expected
 * joints, and the resulting trajectory will preserve the joint ordering of \p msg.
 *
 * \param is_continuous_joint TODO
 *
 * \return Trajectory container.
 *
 * \tparam Trajectory Trajectory type. Should be a \e sequence container \e sorted by segment start time.
 * Additionally, the contained segment type must implement a constructor with the following signature:
 * \code
 * Segment(const ros::Time&                             traj_start_time,
 *         const trajectory_msgs::JointTrajectoryPoint& start_point,
 *         const trajectory_msgs::JointTrajectoryPoint& end_point,
 *         const std::vector<unsigned int>&             permutation,
           const std::vector<double>&                   position_offset)
 * \endcode
 *
 * \note This function does not throw any exceptions by itself, but the segment constructor might.
 * In such a case, this method should be wrapped inside a \p try block.
 */
// TODO: Return empty if input msg is invalid?
template <class Trajectory>
Trajectory initJointTrajectory(const trajectory_msgs::JointTrajectory& msg,
                               const ros::Time&                        time,
                               const Trajectory&                       curr_traj           = Trajectory(),
                               const std::vector<std::string>&         joint_names         = std::vector<std::string>(),
                               const std::vector<bool>&                is_continuous_joint = std::vector<bool>())
{
  // Input:
  // - Currently followed trajectory
  // - Trajectory message
  // - Joint names in the same order as the currently followed trajectory
  // - Vector specifying whether which joints are continuous.
  //
  // Output:
  // - New trajectory to follow containing useful parts of currently followed one + message. Empty if something's amiss.
  //
  // Acronyms:
  // - SI: Segment implementation independent. Only leverages common segment API.
  // - SD: Segment implementation dependent. Requires knowledge of the segment implementation.
  //
  // Algorithm:
  // -* SI: Compute permutation vector mapping message joint order to current trajectory order.
  //
  // -* SI: Get useful segments of current trajectory.
  // -* SI: Get last useful (time, state) of current trajectory.
  //
  // -* SI: Find first trajectory point occurring after current time.
  // -* SD: Get first useful state of new trajectory taking into account the permutation vector.
  // - SI: Compute wrapping offset vector (affects only continuous joints).
  // - SI: Compute segment bridging current and new trajectories
  // -* SD: Convert useful segments of trajectory message taking into account the permutation vector and wrapping offsets.
  // -* SI: Append useful segments of new trajectory to useful segments of current trajectory.
  //
  // SD functionality to implement:
  // - Construct state from trajectory point message, and optionally permutation vector and wrapping offsets (3 params).
  // - Construct segment from trajectory start time, start/end trajectory points, and optionally permutation vector and
  //   wrapping offsets (5 params).

  // TODO: Use single return instance to make sure that RVO kicks in?

  typedef typename Trajectory::value_type Segment;
  const ros::Time msg_start_time = internal::startTime(msg, time); // Message start time

  ROS_DEBUG_STREAM("Figuring out new trajectory starting at time " << msg_start_time.toSec());

  //Empty trajectory
  if (msg.points.empty())
  {
    ROS_DEBUG("Trajectory message contains empty trajectory. Nothing to convert.");
    return Trajectory();
  }

  // Permutation vector mapping the expected joint order to the message joint order
  // If unspecified, a trivial map (no permutation) is computed
  typedef std::vector<std::string>::size_type SizeType;
  std::vector<SizeType> permutation_vector = joint_names.empty() ?
                                             internal::permutation(msg.joint_names, msg.joint_names) : // Trivial map
                                             internal::permutation(joint_names,     msg.joint_names);

  if (permutation_vector.empty())
  {
    ROS_ERROR("Cannot create trajectory from message. It does not contain the expected joints.");
    return Trajectory();
  }

  // Find first point of new trajectory occurring after current time
  // This point is used later on in this function, but is computed here, in advance because if the trajectory message
  // contains a trajectory in the past, we can quickly return without spending additional computational resources
  std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator
  it = findPoint(msg, time); // Points to last point occurring before current time
  if (it == msg.points.end())
  {
    it = msg.points.begin();  // Entire trajectory is after current time
  }
  else
  {
    ++it;                     // Points to first point after current time OR sequence end
    if (it == msg.points.end())
    {
      ROS_WARN_STREAM("Dropping all " << msg.points.size() <<
                      " trajectory points, as they occur before the specified time.");
      return Trajectory();
    }
    else
    {
      ROS_WARN_STREAM("Dropping first " << std::distance(msg.points.begin(), it) <<
                      " trajectory points out of " << msg.points.size() << ", as they occur before the specified time.");
    }
  }

  // Initialize result trajectory: combination of:
  // - Useful segments of currently followed trajectory
  // - Useful segments of new trajectory (contained in ROS message)
  Trajectory result_traj; // Currently empty

  // Initialize offsets due to wrapping joints to zero
  std::vector<double> position_offset(msg.joint_names.size(), 0.0);

  // Bridge current trajectory to new one
  if (!curr_traj.empty())
  {
    // Get the last time and state that will be executed from the current trajectory
    const typename Segment::Time last_curr_time = std::max(msg_start_time.toSec(), time.toSec()); // Important!
    typename Segment::State last_curr_state;
    sample(curr_traj, last_curr_time, last_curr_state);

    // Get the first time and state that will be executed from the new trajectory
    const typename Segment::Time first_new_time = msg_start_time.toSec() + (it->time_from_start).toSec();
    typename Segment::State first_new_state(*it, permutation_vector); // Here offsets are not yet applied

    // Compute offsets due to wrapping joints
    if (!is_continuous_joint.empty())
    {
      position_offset = wraparoundOffset<double>(last_curr_state,
                                                 first_new_state,
                                                 is_continuous_joint);
      if (position_offset.empty())
      {
        ROS_ERROR("Cannot create trajectory from message. "
                  "Vector specifying whether joints are continuous has an invalid size.");
        return Trajectory();
      }
    }

    // Apply offset to first state that will be executed from the new trajectory
    first_new_state = typename Segment::State(*it, permutation_vector, position_offset); // Now offsets are applied

    // Add useful segments of current trajectory to result
    {
      typedef typename Trajectory::const_iterator TrajIter;
      TrajIter first = findSegment(curr_traj, time.toSec());   // Currently active segment
      TrajIter last  = findSegment(curr_traj, last_curr_time); // Segment active when new trajectory starts
      if (first == curr_traj.end() || last == curr_traj.end())
      {
        ROS_ERROR("Unexpected error: Could not find segments in current trajectory. Please contact the package maintainer.");
        return Trajectory();
      }
      result_traj.insert(result_traj.begin(), first, ++last); // Range [first,last) will still be executed
    }

    // Add segment bridging current and new trajectories to result
    Segment bridge_seg(last_curr_time, last_curr_state,
                       first_new_time, first_new_state);

    result_traj.push_back(bridge_seg);
  }

  // Constants used in log statement at the end
  const unsigned int num_old_segments = result_traj.size() -1;
  const unsigned int num_new_segments = std::distance(it, msg.points.end()) -1;

  // Add useful segments of new trajectory to result
  // - Construct all trajectory segments occurring after current time
  // - As long as there remain two trajectory points we can construct the next trajectory segment
  while (std::distance(it, msg.points.end()) >= 2)
  {
    std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator next_it = it; ++next_it;
    Segment segment(msg_start_time, *it, *next_it, permutation_vector, position_offset);
    result_traj.push_back(segment);
    ++it;
  }

  // Useful debug info
  std::stringstream log_str;
  log_str << "Trajectory has " << result_traj.size() << " segments";
  if (!curr_traj.empty())
  {
    log_str << ":";
    log_str << "\n- " << num_old_segments << " segments will still be executed from current trajectory.";
    log_str << "\n- 1 segment for transitioning between the current trajectory and first point of the input message.";
    if (num_new_segments > 0) {log_str << "\n- " << num_new_segments << " new segments taken from the input message.";}
  }
  else {log_str << ".";}
  ROS_DEBUG_STREAM(log_str.str());

  return result_traj;
}

} // namespace

#endif // header guard
