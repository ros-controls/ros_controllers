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

#ifndef TRAJECTORY_INTERFACE_TRAJECTORY_INTERFACE_ROS_H
#define TRAJECTORY_INTERFACE_TRAJECTORY_INTERFACE_ROS_H

#include <algorithm>
#include <iterator>

#include <ros/console.h>
#include <ros/time.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <trajectory_interface/trajectory_interface.h>

namespace trajectory_interface
{
namespace internal
{

class IsBeforePoint
{
public:
  IsBeforePoint(const ros::Time& msg_start_time) : msg_start_time_(msg_start_time) {}

  bool operator()(const ros::Time& time, const trajectory_msgs::JointTrajectoryPoint& point)
  {
    const ros::Time point_start_time = msg_start_time_ + point.time_from_start;
    return time < point_start_time;
  }

private:
  ros::Time msg_start_time_;
};

/**
 * \param msg Trajectory message.
 * \param time Trajectory start time, if unspecified in message.
 * \return Start time specified in message. If unspecified (set to zero) return \p time.
 */
inline ros::Time startTime(const trajectory_msgs::JointTrajectory& msg,
                           const ros::Time&                        time)
{
  return msg.header.stamp.isZero() ? time : msg.header.stamp;
}

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
  const SizeType size = t1.size();

  std::vector<SizeType> permutation_vector(size);
  for (SizeType t1_it = 0; t1_it < size; ++t1_it)
  {
    bool found = false;
    for (SizeType t2_it = 0; t2_it < size; ++t2_it)
    {
      if (t1[t1_it] == t2[t2_it])
      {
        permutation_vector[t1_it] = t2_it;
        found = true;
        break;
      }
    }
    if (!found) {return std::vector<SizeType>();}
  }

  return permutation_vector;
}

} // namespace

/**
 * \param point Trajectory point message.
 * \param joint_dim Expected dimension of the position, velocity and acceleration fields.
 * \return True if sizes of the position, velocity and acceleration fields are consistent. An empty field means that
 * it's unspecified, so in this particular case its dimension must not coincide with \p joint_dim.
 */
inline bool isValid(const trajectory_msgs::JointTrajectoryPoint& point, const unsigned int joint_dim)
{
  if (!point.positions.empty()     && point.positions.size()     != joint_dim) {return false;}
  if (!point.velocities.empty()    && point.velocities.size()    != joint_dim) {return false;}
  if (!point.accelerations.empty() && point.accelerations.size() != joint_dim) {return false;}
  return true;
}

/**
 * \param msg Trajectory message.
 * \return True if sizes of input message are consistent (joint names, position, velocity, acceleration).
 */
inline bool isValid(const trajectory_msgs::JointTrajectory& msg)
{
  const std::vector<std::string>::size_type joint_dim = msg.joint_names.size();

  typedef std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator PointConstIterator;

  for (PointConstIterator it = msg.points.begin(); it != msg.points.end(); ++it)
  {
    const std::iterator_traits<PointConstIterator>::difference_type index = std::distance(msg.points.begin(), it);

    if(!isValid(*it, joint_dim))
    {
      ROS_ERROR_STREAM("Invalid trajectory point at index: " << index <<
                       ". Size mismatch in joint names, position, velocity or acceleration data.");
      return false;
    }
  }
  return true;
}

// TODO: Add similar method for action goal message


/**
 * \brief Find an iterator to the trajectory point with the greatest start time < \p time.
 *
 * \param msg Trajectory message.
 * \param time Time to search for in the range.
 *
 * \return Iterator to the trajectory point with the greatest start time < \p time.
 * If all points occur at a time greater than \p time, then \p the points sequence end is returned.
 *
 * \pre The points in \p msg have monotonically increasing times.
 *
 * \note On average, this method has logarithmic time complexity when used on containers with \b random-access iterators.
 * On \b non-random-access iterators, iterator advances incur an additional linear time cost.
 */
inline std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator
findPoint(const trajectory_msgs::JointTrajectory& msg,
          const ros::Time&                        time)
{
  // Message trajectory start time
  // If message time is == 0.0, the trajectory should start at the current time
  const ros::Time msg_start_time = internal::startTime(msg, time);
  internal::IsBeforePoint isBeforePoint(msg_start_time);

  typedef std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator ConstIterator;
  const ConstIterator first = msg.points.begin();
  const ConstIterator last  = msg.points.end();

  return (first == last || isBeforePoint(time, *first))
         ? last // Optimization when time preceeds all segments, or when an empty range is passed
         : --std::upper_bound(first, last, time, isBeforePoint); // Notice decrement operator
}


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
template <class Trajectory>
Trajectory init(/*const Trajectory&                       curr_traj,*/
                const trajectory_msgs::JointTrajectory& msg,
                const ros::Time&                        time,
                const std::vector<std::string>&         joint_names         = std::vector<std::string>(),
                const std::vector<bool>&                is_continuous_joint = std::vector<bool>())
{
  typedef typename Trajectory::value_type Segment;
  typedef std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator MsgPointConstIterator;

  // Empty trajectory
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
    ROS_ERROR_STREAM("Cannot create trajectory from message. It does not contain the expected joints.");
    return Trajectory();
  }

  // Find first trajectory point occurring after current time
  MsgPointConstIterator it = findPoint(msg, time);  // Points to last point occurring before current time
  if (it == msg.points.end())
  {
    it = msg.points.begin(); // Entire trajectory is after current time
  }
  else
  {
    ++it;                    // Points to first point after current time OR sequence end
    if (it == msg.points.end())
    {
      ROS_WARN_STREAM("Dropping all " << msg.points.size() <<
                      " trajectory points, as they occur before the specified time.");
    }
    else
    {
      ROS_WARN_STREAM("Dropping first " << std::distance(msg.points.begin(), it) <<
                      " trajectory points out of " << msg.points.size() << ", as they occur before the specified time.");
    }
  }

  std::vector<double> position_offset; // TODO!!!

  // Construct all trajectory segments occurring after current time
  // As long as there remain two trajectory points we can construct the next trajectory segment
  const ros::Time msg_start_time = internal::startTime(msg, time); // Trajectory start time
  Trajectory trajectory;                                           // Return value
  while (std::distance(it, msg.points.end()) >= 2)
  {
    MsgPointConstIterator next_it = it; ++next_it;
    Segment segment(msg_start_time, *it, *next_it, permutation_vector, position_offset);
    trajectory.push_back(segment);
    ++it;
  }
  if (trajectory.empty())
  {
    ROS_DEBUG_STREAM("Created empty trajectory, as there were less than two valid trajectory points.");
  }
  else
  {
    ROS_DEBUG_STREAM("Created trajectory from " << (trajectory.size() + 1) << " points (has " << trajectory.size() <<
                    " segments).");
  }

  return trajectory;
}

} // namespace

#endif // header guard
