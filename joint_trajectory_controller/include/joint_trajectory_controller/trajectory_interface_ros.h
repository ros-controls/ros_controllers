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
 * \param time Current time.
 * \return Start time specified in message. If unspecified (set to zero) return current time.
 */
inline ros::Time startTime(const trajectory_msgs::JointTrajectory& msg,
                           const ros::Time&                        current_time)
{
  return msg.header.stamp.isZero() ? current_time : msg.header.stamp;
}

} // namespace

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

    if (!it->positions.empty() && it->positions.size() != joint_dim)
    {
      ROS_ERROR_STREAM("Trajectory point " << index << " positions size mismatch. Expected " << joint_dim <<
                       " elements, got " << it->positions.size() << ".");
      return false;
    }
    if (!it->velocities.empty() && it->velocities.size() != joint_dim)
    {
      ROS_ERROR_STREAM("Trajectory point " << index << " velocities size mismatch. Expected " << joint_dim <<
                       " elements, got " << it->velocities.size() << ".");
      return false;
    }
    if (!it->accelerations.empty() && it->accelerations.size() != joint_dim)
    {
      ROS_ERROR_STREAM("Trajectory point " << index << " acceleration size mismatch. Expected " << joint_dim <<
                       " elements, got " << it->accelerations.size() << ".");
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
 * \brief init
 * \param msg
 * \param time
 * \tparam Trajectory TODO: Must be a multi-dof segment implementing pos, vel, acc
 * TODO: pre on init function
 */
template <class Trajectory>
Trajectory init(const trajectory_msgs::JointTrajectory&msg, const ros::Time& time)
{
  typedef typename Trajectory::value_type Segment;
  typedef std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator MsgPointConstIterator;

  Trajectory trajectory; // Return value

  if (msg.points.empty()) {return trajectory;} // No-op

  // Find first trajectory point occurring after current time
  MsgPointConstIterator it = findPoint(msg, time);  // Points to last point occurring before current time
  if (it == msg.points.end())
  {
    it = msg.points.begin(); // Entire trajectory is after current time
  }
  else
  {
    ++it;                    // Points to first point after current time OR sequence end
    ROS_DEBUG_STREAM("Dropping first " << std::distance(msg.points.begin(), it) <<
                     " trajectory points, as they occur in the past.");
  }

  // Construct all trajectory segments occurring after current time
  // As long as there remain two trajectory points we can construct the next trajectory segment
  const ros::Time msg_start_time = internal::startTime(msg, time); // Trajectory start time
  while (std::distance(it, msg.points.end()) >= 2)
  {
    MsgPointConstIterator next_it = ++it;
    Segment segment(msg_start_time, *it, *next_it);
    trajectory.push_back(segment);
    ++it;
  }

  return trajectory;
}

} // namespace

#endif // header guard
