///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
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
#ifndef STOP_TRAJECTORY_BUILDER_H
#define STOP_TRAJECTORY_BUILDER_H

#include <vector>

#include <joint_trajectory_controller/trajectory_builder.h>

namespace joint_trajectory_controller
{

/**
 * @brief Builder creating a trajectory stopping the robot.
 */
template<class SegmentImpl>
class StopTrajectoryBuilder : public TrajectoryBuilder<SegmentImpl>
{
private:
  using Segment               = JointTrajectorySegment<SegmentImpl>;
  using TrajectoryPerJoint    = std::vector<Segment>;
  using Trajectory            = std::vector<TrajectoryPerJoint>;

  using RealtimeGoalHandle    = realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction>;
  using RealtimeGoalHandlePtr = boost::shared_ptr<RealtimeGoalHandle>;

public:
  /**
   * @param stop_traj_duration Desired time span [seconds] in which the settle position has to be reached.
   * @param hold_state Reference to the state where the stop motion shall start.
   *
   * @note If there is a time delay in the system it is better to calculate the stop trajectory
   * starting from the desired position. Otherwise there would be a jerk in the motion.
   */
  StopTrajectoryBuilder(const typename Segment::Time& stop_traj_duration,
                        const typename Segment::State& hold_state);

public:
  /**
   * @brief Creates a trajectory which reaches the settle position in a fixed time.
   *
   * The calculation is done as follows:
   * - Create segment that goes from current (pos,vel) to (pos,-vel) in 2x the desired stop time.

   * - Assuming segment symmetry, sample segment at its midpoint (desired stop time). It should have zero velocity.
   * - Create segment that goes from current state to above zero velocity state, in the desired time.
   *
   * @note The symmetry assumption from the second point above might not hold for all possible segment types.
   *
   * @returns true if the building of the stop trajectory succeeds, otherwise false.
   */
  bool buildTrajectory(Trajectory* hold_traj) override;

private:
  const typename Segment::Time stop_traj_duration_;
  //! @brief Stores a reference to the state where the stop motion shall start.
  const typename Segment::State& hold_state_;

private: //Pre-allocated memory for real time usage of build function
  typename Segment::State hold_start_state_ {typename Segment::State(1)};
  typename Segment::State hold_end_state_   {typename Segment::State(1)};
};

template<class SegmentImpl>
StopTrajectoryBuilder<SegmentImpl>::StopTrajectoryBuilder(const typename Segment::Time& stop_traj_duration,
                                                          const typename Segment::State& hold_state)
  : stop_traj_duration_(stop_traj_duration)
  , hold_state_(hold_state)
{
}

template<class SegmentImpl>
bool StopTrajectoryBuilder<SegmentImpl>::buildTrajectory(Trajectory* hold_traj)
{
  const unsigned int number_of_joints = hold_state_.position.size();

  if(!TrajectoryBuilder<SegmentImpl>::getStartTime())
  {
    return false;
  }

  if(!TrajectoryBuilder<SegmentImpl>::isTrajectoryValid(hold_traj, number_of_joints, 1))
  {
    return false;
  }

  const typename Segment::Time start_time {TrajectoryBuilder<SegmentImpl>::getStartTime().value()};
  RealtimeGoalHandlePtr goal_handle {TrajectoryBuilder<SegmentImpl>::createGoalHandlePtr()};
  const typename Segment::Time end_time    {start_time + stop_traj_duration_};
  const typename Segment::Time end_time_2x {start_time + 2.0 * stop_traj_duration_};
  for (unsigned int joint_index = 0; joint_index < number_of_joints; ++joint_index)
  {
    hold_start_state_.position[0]     =  hold_state_.position[joint_index];
    hold_start_state_.velocity[0]     =  hold_state_.velocity[joint_index];
    hold_start_state_.acceleration[0] =  0.0;

    hold_end_state_.position[0]       =  hold_state_.position[joint_index];
    hold_end_state_.velocity[0]       = -hold_state_.velocity[joint_index];
    hold_end_state_.acceleration[0]   =  0.0;

    Segment& segment {(*hold_traj)[joint_index].front()};
    segment.init(start_time,  hold_start_state_,
                 end_time_2x, hold_end_state_);
    // Sample segment at its midpoint, that should have zero velocity
    segment.sample(end_time, hold_end_state_);
    // Now create segment that goes from current state to one with zero end velocity
    segment.init(start_time,  hold_start_state_,
                 end_time,    hold_end_state_);

    segment.setGoalHandle(goal_handle);
  }

  return true;
}


} // namespace joint_trajectory_controller

#endif // STOP_TRAJECTORY_BUILDER_H
