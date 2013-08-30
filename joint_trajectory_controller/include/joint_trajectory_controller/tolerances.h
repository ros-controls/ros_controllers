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

#ifndef JOINT_TRAJECTORY_CONTROLLER_TOLERANCES_H
#define JOINT_TRAJECTORY_CONTROLLER_TOLERANCES_H

// C++ standard
#include <cassert>
#include <cmath>
#include <string>
#include <vector>

// ROS
#include <ros/node_handle.h>

// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace joint_trajectory_controller
{

/**
 * \brief Trajectory state tolerances for position, velocity and acceleration variables.
 *
 * A tolerance value of zero means that no tolerance will be applied for that variable.
 */
template<class Scalar>
struct StateTolerances
{
  StateTolerances(Scalar position_tolerance     = static_cast<Scalar>(0.0),
                  Scalar velocity_tolerance     = static_cast<Scalar>(0.0),
                  Scalar acceleration_tolerance = static_cast<Scalar>(0.0))
    : position(position_tolerance),
      velocity(velocity_tolerance),
      acceleration(acceleration_tolerance)
  {}

  Scalar position;
  Scalar velocity;
  Scalar acceleration;
};

/**
 * \brief Trajectory segment tolerances.
 */
template<class Scalar>
struct SegmentTolerances
{
  SegmentTolerances(const typename std::vector<StateTolerances<Scalar> >::size_type& size = 0)
    : state_tolerance(size, static_cast<Scalar>(0.0)),
      goal_state_tolerance(size, static_cast<Scalar>(0.0)),
      goal_time_tolerance(static_cast<Scalar>(0.0))
  {}

  /** State tolerances that appply during segment execution. */
  std::vector<StateTolerances<Scalar> > state_tolerance;

  /** State tolerances that apply for the goal state only.*/
  std::vector<StateTolerances<Scalar> > goal_state_tolerance;

  /** Extra time after the segment end time allowed to reach the goal state tolerances. */
  Scalar goal_time_tolerance;
};

/**
 * \param state_error State error to check.
 * \param state_tolerance State tolerances to check \p state_error against.
 * \return True if \p state_error fulfills \p state_tolerance.
 */
template <class State>
inline bool checkStateTolerance(const State&                                                 state_error,
                                const std::vector<StateTolerances<typename State::Scalar> >& state_tolerance)
{
  const unsigned int n_joints = state_tolerance.size();

  // Preconditions
  assert(n_joints == state_error.position.size());
  assert(n_joints == state_error.velocity.size());
  assert(n_joints == state_error.acceleration.size());

  for (unsigned int i = 0; i < n_joints; ++i)
  {
    using std::abs;
    const StateTolerances<typename State::Scalar>& tol = state_tolerance[i]; // Alias for brevity
    const bool is_valid = !(tol.position     > 0.0 && abs(state_error.position[i])     > tol.position) &&
                          !(tol.velocity     > 0.0 && abs(state_error.velocity[i])     > tol.velocity) &&
                          !(tol.acceleration > 0.0 && abs(state_error.acceleration[i]) > tol.acceleration);

    if (!is_valid) {return false;}
  }
  return true;
}

/**
 * \brief Update data in \p tols from data in \p msg_tol.
 *
 * - If a value in \p tol_msg is positive, the corresponding values in \p tols will be overritten.
 * - If a value in \p tol_msg is negative, the corresponding values in \p tols will be reset.
 * - If a value in \p tol_msg is zero, the corresponding values in \p tols is unchanged.
 *
 * \param[in] tol_msg Message containing tolerance values \p tols will be updated with.
 * \param[out] tols Tolerances values to update.
 *
 **/
template<class Scalar>
void updateStateTolerances(const control_msgs::JointTolerance& tol_msg, StateTolerances<Scalar>& tols)
{
  if      (tol_msg.position     > 0.0) {tols.position     = static_cast<Scalar>(tol_msg.position);}
  else if (tol_msg.position     < 0.0) {tols.position     = static_cast<Scalar>(0.0);}

  if      (tol_msg.velocity     > 0.0) {tols.velocity     = static_cast<Scalar>(tol_msg.velocity);}
  else if (tol_msg.velocity     < 0.0) {tols.velocity     = static_cast<Scalar>(0.0);}

  if      (tol_msg.acceleration > 0.0) {tols.acceleration = static_cast<Scalar>(tol_msg.acceleration);}
  else if (tol_msg.acceleration < 0.0) {tols.acceleration = static_cast<Scalar>(0.0);}
}

/**
 * \brief Update data in \p tols from data in \p goal.
 *
 * \param[in] goal Action goal data containing tolerance values \p tols will be updated with.
 * \param[in] joint_names Names of joints in \p tols, with the same ordering.
 * \param[out] tols Tolerances values to update.
 */
template<class Scalar>
void updateSegmentTolerances(const control_msgs::FollowJointTrajectoryGoal& goal,
                             const std::vector<std::string>&                joint_names,
                             SegmentTolerances<Scalar>&                     tols
)
{
  // Preconditions
  assert(joint_names.size() == tols.state_tolerance.size());
  assert(joint_names.size() == tols.goal_state_tolerance.size());

  typedef typename std::vector<std::string>::const_iterator                  StringConstIterator;
  typedef typename std::vector<control_msgs::JointTolerance>::const_iterator TolMsgConstIterator;

  for (StringConstIterator names_it = joint_names.begin(); names_it != joint_names.end(); ++names_it)
  {
    const typename std::vector<std::string>::size_type id = std::distance(joint_names.begin(), names_it);

    // Update path tolerances
    const std::vector<control_msgs::JointTolerance>& state_tol = goal.path_tolerance;
    for(TolMsgConstIterator state_tol_it = state_tol.begin(); state_tol_it != state_tol.end(); ++state_tol_it)
    {
      if (*names_it == state_tol_it->name) {updateStateTolerances(*state_tol_it, tols.state_tolerance[id]);}
    }

    // Update goal state tolerances
    const std::vector<control_msgs::JointTolerance>& g_state_tol = goal.goal_tolerance;
    for(TolMsgConstIterator g_state_tol_it = g_state_tol.begin(); g_state_tol_it != g_state_tol.end(); ++g_state_tol_it)
    {
      if (*names_it == g_state_tol_it->name) {updateStateTolerances(*g_state_tol_it, tols.goal_state_tolerance[id]);}
    }
  }

  // Update goal time tolerance
  const ros::Duration& goal_time_tolerance = goal.goal_time_tolerance;
  if      (goal_time_tolerance < ros::Duration(0.0)) {tols.goal_time_tolerance = 0.0;}
  else if (goal_time_tolerance > ros::Duration(0.0)) {tols.goal_time_tolerance = goal_time_tolerance.toSec();}
}

/**
 * \brief Populate trajectory segment tolerances from data in the ROS parameter server.
 *
 * It is assumed that the following parameter structure is followed on the provided NodeHandle. Unspecified parameters
 * will take the defaults shown in the comments:
 *
 * \code
 * constraints:
 *  goal_time: 1.0                   # Defaults to zero
 *  stopped_velocity_tolerance: 0.02 # Defaults to 0.01
 *  foo_joint:
 *    trajectory: 0.05               # Defaults to zero (ie. the tolerance is not enforced)
 *    goal:       0.03               # Defaults to zero (ie. the tolerance is not enforced)
 *  bar_joint:
 *    goal: 0.01
 * \endcode
 *
 * \param nh NodeHandle where the tolerances are specified.
 * \param joint_names Names of joints to look for in the parameter server for a tolerance specification.
 * \return Trajectory segment tolerances.
 */
template<class Scalar>
SegmentTolerances<Scalar> getSegmentTolerances(const ros::NodeHandle& nh,
                                               const std::vector<std::string>& joint_names)
{
  const unsigned int n_joints = joint_names.size();
  joint_trajectory_controller::SegmentTolerances<Scalar> tolerances;

  // State and goal state tolerances
  double stopped_velocity_tolerance;
  nh.param("stopped_velocity_tolerance", stopped_velocity_tolerance, 0.01);

  tolerances.state_tolerance.resize(n_joints);
  tolerances.goal_state_tolerance.resize(n_joints);
  for (unsigned int i = 0; i < n_joints; ++i)
  {
    nh.param(joint_names[i] + "/trajectory", tolerances.state_tolerance[i].position,      0.0);
    nh.param(joint_names[i] + "/goal",       tolerances.goal_state_tolerance[i].position, 0.0);
    tolerances.goal_state_tolerance[i].velocity = stopped_velocity_tolerance;
  }

  // Goal time tolerance
  nh.param("goal_time", tolerances.goal_time_tolerance, 0.0);

  return tolerances;
}

} // namespace

#endif // header guard
