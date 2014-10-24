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

#ifndef JOINT_TRAJECTORY_CONTROLLER_INIT_JOINT_TRAJECTORY_H
#define JOINT_TRAJECTORY_CONTROLLER_INIT_JOINT_TRAJECTORY_H

// C++ standard
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

// Boost
#include <boost/shared_ptr.hpp>

// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>

// Project
#include <joint_trajectory_controller/joint_trajectory_msg_utils.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>

namespace joint_trajectory_controller
{

namespace internal
{
/**
 * \return The permutation vector between two containers.
 * If \p t1 is <tt>"{A, B, C, D}"</tt> and \p t2 is <tt>"{B, D, A, C}"</tt>, the associated permutation vector is
 * <tt>"{2, 0, 3, 1}"</tt>.
 */
template <class T>
inline std::vector<unsigned int> permutation(const T& t1, const T& t2)
{
  typedef unsigned int SizeType;

  // Arguments must have the same size
  if (t1.size() != t2.size()) {return std::vector<SizeType>();}

  std::vector<SizeType> permutation_vector(t1.size()); // Return value
  for (typename T::const_iterator t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    typename T::const_iterator t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it) {return std::vector<SizeType>();}
    else
    {
      const SizeType t1_dist = std::distance(t1.begin(), t1_it);
      const SizeType t2_dist = std::distance(t2.begin(), t2_it);
      permutation_vector[t1_dist] = t2_dist;
    }
  }
  return permutation_vector;
}

} // namespace

/**
 * \brief Options used when initializing a joint trajectory from ROS message data.
 * \sa initJointTrajectory
 */
template <class Trajectory>
struct InitJointTrajectoryOptions
{
  typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;
  typedef typename Trajectory::value_type::Scalar                                             Scalar;

  InitJointTrajectoryOptions()
    : current_trajectory(0),
      joint_names(0),
      angle_wraparound(0),
      rt_goal_handle(),
      default_tolerances(0),
      other_time_base(0)
  {}

  Trajectory*                current_trajectory;
  std::vector<std::string>*  joint_names;
  std::vector<bool>*         angle_wraparound;
  RealtimeGoalHandlePtr      rt_goal_handle;
  SegmentTolerances<Scalar>* default_tolerances;
  ros::Time*                 other_time_base;
};

/**
 * \brief Initialize a joint trajectory from ROS message data.
 *
 * \param msg Trajectory message.
 *
 * \param time Time from which data is to be extracted. All trajectory points in \p msg occurring \b after
 * \p time will be extracted; or put otherwise, all points occurring at a time <b>less or equal</b> than \p time
 * will be discarded. Set this value to zero to process all points in \p msg.
 *
 * \param options Options that change how the trajectory gets initialized.
 *
 * The \ref InitJointTrajectoryOptions "options" parameter is optional. The meaning of its different members follows:
 * - \b current_trajectory Currently executed trajectory. Use this parameter if you want to update an existing
 * trajectory with the data in \p msg; that is, keep the useful parts of \p current_trajectory and \p msg.
 * If specified, the output trajectory will not only contain data in \p msg occurring \b after \p time, but will also
 * contain data from \p current_trajectory \b between \p time and the start time of \p msg
 * (which might not be equal to \p time).
 *
 * - \b joint_names Joints expected to be found in \p msg. If specified, this function will return an empty trajectory
 * when \p msg contains joints that differ in number or names to \p joint_names. If \p msg contains the same joints as
 * \p  joint_names, but in a different order, the resulting trajectory will be ordered according to \p joint_names
 * (and not \p msg). If unspecified (empty), no checks will be performed against expected joints, and the resulting
 * trajectory will preserve the joint ordering of \p msg.
 *
 * - \b angle_wraparound Vector of booleans where true values correspond to joints that wrap around (ie. are continuous).
 * If specified, combining \p current_trajectory with \p msg will not result in joints performing multiple turns at the
 * transition. This parameter \b requires \p current_trajectory to also be specified, otherwise it is ignored.
 *
 * - \b rt_goal_handle Action goal handle associated to the new trajectory. If specified, newly added segments will have
 * a pointer to it, and to the trajectory tolerances it contains (if any).
 *
 * - \b default_tolerances Default trajectory tolerances. This option is only used when \p rt_goal_handle is also
 * specified. It contains the default tolernaces to check when executing an action goal. If the action goal specifies
 * tolerances (totally or partially), these values will take precedence over the defaults.
 *
 * - \b other_time_base When initializing a new trajectory, it might be the case that we desire the result expressed in
 * a \b different time base than that contained in \p msg. If specified, the value of this variable should be the
 * equivalent of the \p time parameter, but expressed in the desired time base.
 * If the \p current_trajectory option is also specified, it must be expressed in \p other_time_base.
 * The typical usecase for this variable is when the \p current_trajectory option is specified, and contains data in
 * a different time base (eg. monotonically increasing) than \p msg (eg. system-clock synchronized).
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
 *         const std::vector<Scalar>&                   position_offset)
 * \endcode
 * The following function must also be defined to properly handle continuous joints:
 * \code
 * std::vector<Scalar> wraparoundOffset(const typename Segment::State&  prev_state,
 *                                      const typename Segment::State&  next_state,
 *                                      const std::vector<bool>&        angle_wraparound)
 * \endcode
 *
 * \note This function does not throw any exceptions by itself, but the segment constructor might.
 * In such a case, this method should be wrapped inside a \p try block.
 */
// TODO: Return useful bits of current trajectory if input msg is useless?
template <class Trajectory>
Trajectory initJointTrajectory(const trajectory_msgs::JointTrajectory&       msg,
                               const ros::Time&                              time,
                               const InitJointTrajectoryOptions<Trajectory>& options =
                               InitJointTrajectoryOptions<Trajectory>())
{
  typedef typename Trajectory::value_type Segment;
  typedef typename Segment::Scalar Scalar;

  const unsigned int n_joints = msg.joint_names.size();

  const ros::Time msg_start_time = internal::startTime(msg, time); // Message start time

  ROS_DEBUG_STREAM("Figuring out new trajectory starting at time "
                   << std::fixed << std::setprecision(3) << msg_start_time.toSec());

  // Empty trajectory
  if (msg.points.empty())
  {
    ROS_DEBUG("Trajectory message contains empty trajectory. Nothing to convert.");
    return Trajectory();
  }

  // Non strictly-monotonic waypoints
  if (!isTimeStrictlyIncreasing(msg))
  {
    ROS_ERROR("Trajectory message contains waypoints that are not strictly increasing in time.");
    return Trajectory();
  }

  // Validate options
  const bool has_current_trajectory = options.current_trajectory && !options.current_trajectory->empty();
  const bool has_joint_names        = options.joint_names        && !options.joint_names->empty();
  const bool has_angle_wraparound   = options.angle_wraparound   && !options.angle_wraparound->empty();
  const bool has_rt_goal_handle     = options.rt_goal_handle;
  const bool has_other_time_base    = options.other_time_base;
  const bool has_default_tolerances = options.default_tolerances;

  if (!has_current_trajectory && has_angle_wraparound)
  {
    ROS_WARN("Vector specifying whether joints wrap around will not be used because no current trajectory was given.");
  }

  // Compute trajectory start time and data extraction time associated to the 'other' time base, if it applies
  // The o_ prefix indicates that time values are represented in this 'other' time base.
  ros::Time o_time;
  ros::Time o_msg_start_time;
  if (has_other_time_base)
  {
    ros::Duration msg_start_duration = msg_start_time - time;
    o_time = *options.other_time_base;
    o_msg_start_time = o_time + msg_start_duration;
    ROS_DEBUG_STREAM("Using alternate time base. In it, the new trajectory starts at time "
                     << std::fixed << std::setprecision(3) << o_msg_start_time.toSec());
  }
  else
  {
    o_time = time;
    o_msg_start_time = msg_start_time;
  }

  // Permutation vector mapping the expected joint order to the message joint order
  // If unspecified, a trivial map (no permutation) is computed
  const std::vector<std::string> joint_names = has_joint_names ? *(options.joint_names) : msg.joint_names;

  std::vector<unsigned int> permutation_vector = internal::permutation(joint_names, msg.joint_names);

  if (permutation_vector.empty())
  {
    ROS_ERROR("Cannot create trajectory from message. It does not contain the expected joints.");
    return Trajectory();
  }

  // Tolerances to be used in all new segments
  SegmentTolerances<Scalar> tolerances = has_default_tolerances ?
                                         *(options.default_tolerances) : SegmentTolerances<Scalar>(n_joints);

  if (has_rt_goal_handle && options.rt_goal_handle->gh_.getGoal())
  {
    updateSegmentTolerances<Scalar>(*(options.rt_goal_handle->gh_.getGoal()), joint_names, tolerances);
  }

  // Find first point of new trajectory occurring after current time
  // This point is used later on in this function, but is computed here, in advance because if the trajectory message
  // contains a trajectory in the past, we can quickly return without spending additional computational resources
  std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator
  it = findPoint(msg, time); // Points to last point occurring before current time (NOTE: Using time, not o_time)
  if (it == msg.points.end())
  {
    it = msg.points.begin();  // Entire trajectory is after current time
  }
  else
  {
    ++it;                     // Points to first point after current time OR sequence end
    if (it == msg.points.end())
    {
      ros::Duration last_point_dur = time - (msg_start_time + (--it)->time_from_start);
      ROS_WARN_STREAM("Dropping all " << msg.points.size() <<
                      " trajectory point(s), as they occur before the current time.\n" <<
                      "Last point is " << std::fixed << std::setprecision(3) << last_point_dur.toSec() <<
                      "s in the past.");
      return Trajectory();
    }
    else
    {
      ros::Duration next_point_dur = msg_start_time + it->time_from_start - time;
      ROS_WARN_STREAM("Dropping first " << std::distance(msg.points.begin(), it) <<
                      " trajectory point(s) out of " << msg.points.size() <<
                      ", as they occur before the current time.\n" <<
                      "First valid point will be reached in " << std::fixed << std::setprecision(3) <<
                      next_point_dur.toSec() << "s.");
    }
  }

  // Initialize result trajectory: combination of:
  // - Useful segments of currently followed trajectory
  // - Useful segments of new trajectory (contained in ROS message)
  Trajectory result_traj; // Currently empty

  // Initialize offsets due to wrapping joints to zero
  std::vector<Scalar> position_offset(n_joints, 0.0);

  // Bridge current trajectory to new one
  if (has_current_trajectory)
  {
    const Trajectory& curr_traj = *(options.current_trajectory);

    // Get the last time and state that will be executed from the current trajectory
    const typename Segment::Time last_curr_time = std::max(o_msg_start_time.toSec(), o_time.toSec()); // Important!
    typename Segment::State last_curr_state;
    sample(curr_traj, last_curr_time, last_curr_state);

    // Get the first time and state that will be executed from the new trajectory
    const typename Segment::Time first_new_time = o_msg_start_time.toSec() + (it->time_from_start).toSec();
    typename Segment::State first_new_state(*it, permutation_vector); // Here offsets are not yet applied

    // Compute offsets due to wrapping joints
    if (has_angle_wraparound)
    {
      position_offset = wraparoundOffset(last_curr_state.position,
                                         first_new_state.position,
                                         *(options.angle_wraparound));
      if (position_offset.empty())
      {
        ROS_ERROR("Cannot create trajectory from message. "
                  "Vector specifying whether joints wrap around has an invalid size.");
        return Trajectory();
      }
    }

    // Apply offset to first state that will be executed from the new trajectory
    first_new_state = typename Segment::State(*it, permutation_vector, position_offset); // Now offsets are applied

    // Add useful segments of current trajectory to result
    {
      typedef typename Trajectory::const_iterator TrajIter;
      TrajIter first = findSegment(curr_traj, o_time.toSec());   // Currently active segment
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
    bridge_seg.setGoalHandle(options.rt_goal_handle);
    if (has_rt_goal_handle) {bridge_seg.setTolerances(tolerances);}
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
    Segment segment(o_msg_start_time, *it, *next_it, permutation_vector, position_offset);
    segment.setGoalHandle(options.rt_goal_handle);
    if (has_rt_goal_handle) {segment.setTolerances(tolerances);}
    result_traj.push_back(segment);
    ++it;
  }

  // Useful debug info
  std::stringstream log_str;
  log_str << "Trajectory has " << result_traj.size() << " segments";
  if (has_current_trajectory)
  {
    log_str << ":";
    log_str << "\n- " << num_old_segments << " segment(s) will still be executed from previous trajectory.";
    log_str << "\n- 1 segment added for transitioning between the current trajectory and first point of the input message.";
    if (num_new_segments > 0) {log_str << "\n- " << num_new_segments << " new segments (" << (num_new_segments + 1) <<
                               " points) taken from the input trajectory.";}
  }
  else {log_str << ".";}
  ROS_DEBUG_STREAM(log_str.str());

  return result_traj;
}

} // namespace

#endif // header guard
