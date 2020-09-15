///////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2020 Pilz GmbH & Co. KG
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#ifndef TRAJECTORY_BUILDER_H
#define TRAJECTORY_BUILDER_H

#include <vector>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>

// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>

// realtime_tools
#include <realtime_tools/realtime_server_goal_handle.h>

#include <joint_trajectory_controller/joint_trajectory_segment.h>

namespace joint_trajectory_controller
{

/**
 * @brief Base class for classes used to construct diffent trajectory types.
 *
 * A derived class has to implement buildTrajectory().
 *
 * The following code snippet shows the intended usage of a builder class:
 * @code
 * builder.setStartTime(start_time);
 * builder.setGoalHandle(goal_handle);
 * builder.buildTrajectory(trajectory);
 * @endcode
 *
 * Setting a start time is mandatory whereas setting a goal handle can be omitted.
 * Also note that \p trajectory has to reference a valid trajectory (see isTrajectoryValid()).
 */
template<class SegmentImpl>
class TrajectoryBuilder
{
public:
  /**
   * @brief Virtual destructor because this class is a base class.
   */
  virtual ~TrajectoryBuilder<SegmentImpl>() = default;

private:
  using Segment               = JointTrajectorySegment<SegmentImpl>;
  using TrajectoryPerJoint    = std::vector<Segment>;
  using Trajectory            = std::vector<TrajectoryPerJoint>;

  using RealtimeGoalHandle    = realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction>;
  using RealtimeGoalHandlePtr = boost::shared_ptr<RealtimeGoalHandle>;

public:
  /**
   * @brief Set the start time [seconds] of the trajectory to be built.
   *
   * The start time is reset by reset().
   */
  TrajectoryBuilder<SegmentImpl>* setStartTime(const typename Segment::Time& start_time);

  /**
   * @brief Set the goal handle that will be attached to the trajectory segments.
   *
   * The goal handle is reset by reset().
   */
  TrajectoryBuilder<SegmentImpl>* setGoalHandle(RealtimeGoalHandlePtr& goal_handle);

public:
  /**
   * @brief Ensures re-usability by allowing the user to reset members
   * of the builder which should be reset between calls to buildTrajectory().
   */
  virtual void reset();

public:
  /**
   * @brief Creates the type of trajectory described by the builder.
   *
   * @param trajectory [Out] Trajectory which has to be build.
   *
   */
  virtual bool buildTrajectory(Trajectory* trajectory) = 0;

protected:
  //! @brief Return the set goal handle or a default one, if no goal handle was set.
  RealtimeGoalHandlePtr createGoalHandlePtr() const;
  //! @brief Return the set start time.
  const boost::optional<typename Segment::Time>& getStartTime() const;

protected:
  static RealtimeGoalHandlePtr createDefaultGoalHandle();

  /**
   * @brief Check if the elements of a given trajectory are sized properly.
   * @param trajectory
   * @param expected_number_of_joints The count of TrajectoryPerJoint in \p trajectory has to match the number of joints.
   * @param expected_number_of_segments Each TrajectoryPerJoint in \p trajectory has to contain this number of segments.
   */
  static bool isTrajectoryValid(const Trajectory* trajectory,
                                const unsigned int expected_number_of_joints,
                                const unsigned int expected_number_of_segments);

private:
  boost::optional<typename Segment::Time> start_time_   {boost::none};
  // We do not want to participate in the life time management of the
  // goal handle, therefore, only a reference is stored.
  boost::optional<RealtimeGoalHandlePtr&> goal_handle_  {boost::none};

};

template<class SegmentImpl>
inline TrajectoryBuilder<SegmentImpl>* TrajectoryBuilder<SegmentImpl>::setStartTime(const typename TrajectoryBuilder<SegmentImpl>::Segment::Time& start_time)
{
  start_time_ = start_time;
  return this;
}

template<class SegmentImpl>
inline const boost::optional<typename TrajectoryBuilder<SegmentImpl>::Segment::Time>& TrajectoryBuilder<SegmentImpl>::getStartTime() const
{
  return start_time_;
}

template<class SegmentImpl>
inline TrajectoryBuilder<SegmentImpl>* TrajectoryBuilder<SegmentImpl>::setGoalHandle(TrajectoryBuilder<SegmentImpl>::RealtimeGoalHandlePtr& goal_handle)
{
  goal_handle_ = goal_handle;
  return this;
}

template<class SegmentImpl>
inline typename TrajectoryBuilder<SegmentImpl>::RealtimeGoalHandlePtr TrajectoryBuilder<SegmentImpl>::createGoalHandlePtr() const
{
  return goal_handle_ ? goal_handle_.value() : createDefaultGoalHandle();
}

template<class SegmentImpl>
inline typename TrajectoryBuilder<SegmentImpl>::RealtimeGoalHandlePtr TrajectoryBuilder<SegmentImpl>::createDefaultGoalHandle()
{
  return RealtimeGoalHandlePtr();
}

template<class SegmentImpl>
inline void TrajectoryBuilder<SegmentImpl>::reset()
{
  start_time_ = boost::none;
  goal_handle_ = boost::none;
}

template<class SegmentImpl>
bool TrajectoryBuilder<SegmentImpl>::isTrajectoryValid(const Trajectory* trajectory,
                                                       const unsigned int expected_number_of_joints,
                                                       const unsigned int expected_number_of_segments)
{
  if (trajectory->size() != expected_number_of_joints)
  {
    return false;
  }
  for (const auto& traj_per_joint : *trajectory)
  {
    if (traj_per_joint.size() != expected_number_of_segments)
    {
      return false;
    }
  }
  return true;
}

}

#endif // TRAJECTORY_BUILDER_H
