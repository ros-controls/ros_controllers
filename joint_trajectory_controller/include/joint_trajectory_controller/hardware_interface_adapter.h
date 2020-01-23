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

#pragma once


#include <cassert>
#include <string>
#include <vector>
#include <memory>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>

/**
 * \brief Helper class to simplify integrating the JointTrajectoryController with different hardware interfaces.
 *
 * The JointTrajectoryController outputs position, velocity and acceleration command triplets, while the more common hardware
 * interfaces accept position, velocity or effort commands.
 *
 * Use one of the available template specializations of this class (or create your own) to adapt the
 * JointTrajectoryController to a specific hardware interface.
 */
template <class HardwareInterface, class State>
class HardwareInterfaceAdapter
{
public:
  bool init(std::vector<typename HardwareInterface::ResourceHandleType>& /*joint_handles*/, ros::NodeHandle& /*controller_nh*/)
  {
    return false;
  }

  void starting(const ros::Time& /*time*/) {}
  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         /*desired_state*/,
                     const State&         /*state_error*/) {}
};

/**
 * \brief Adapter for a position-controlled hardware interface. Forwards desired positions as commands.
 *
 * The following is an example configuration of a controller that uses this adapter.
 * \code
 * head_controller:
 *   type: "position_controllers/JointTrajectoryController"
 *   joints:
 *     - head_1_joint
 *     - head_2_joint
 *
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     head_1_joint: {trajectory: 0.05, goal: 0.02}
 *     head_2_joint: {trajectory: 0.05, goal: 0.02}
 *   stop_trajectory_duration: 0.5
 *   state_publish_rate:  25
 * \endcode
 */
template <class State>
class HardwareInterfaceAdapter<hardware_interface::PositionJointInterface, State>
{
public:
  HardwareInterfaceAdapter() : joint_handles_ptr_(nullptr) {}

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& /*controller_nh*/)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    return true;
  }

  void starting(const ros::Time& /*time*/)
  {
    if (!joint_handles_ptr_) {return;}

    // Semantic zero for commands
    for (auto& jh : *joint_handles_ptr_)
    {
      jh.setCommand(jh.getPosition());
    }
  }

  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         desired_state,
                     const State&         /*state_error*/)
  {
    // Forward desired position to command
    const unsigned int n_joints = joint_handles_ptr_->size();
    for (unsigned int i = 0; i < n_joints; ++i) {(*joint_handles_ptr_)[i].setCommand(desired_state.position[i]);}
  }

private:
  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
};

/**
 * \brief Helper base class template for closed loop HardwareInterfaceAdapter implementations.
 *
 * Adapters leveraging (specializing) this class will generate a command given the desired state and state error using a
 * velocity feedforward term plus a corrective PID term.
 *
 * Use one of the available template specializations of this class (or create your own) to adapt the
 * JointTrajectoryController to a specific hardware interface.
 */
template <class State>
class ClosedLoopHardwareInterfaceAdapter
{
public:
  ClosedLoopHardwareInterfaceAdapter() : joint_handles_ptr_(nullptr) {}

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    // Initialize PIDs
    pids_.resize(joint_handles.size());
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      // Node handle to PID gains
      ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + joint_handles[i].getName());

      // Init PID gains from ROS parameter server
      pids_[i].reset(new control_toolbox::Pid());
      if (!pids_[i]->init(joint_nh))
      {
        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
        return false;
      }
    }

    // Load velocity feedforward gains from parameter server
    velocity_ff_.resize(joint_handles.size());
    for (unsigned int i = 0; i < velocity_ff_.size(); ++i)
    {
      controller_nh.param(std::string("velocity_ff/") + joint_handles[i].getName(), velocity_ff_[i], 0.0);
    }

    return true;
  }

  void starting(const ros::Time& /*time*/)
  {
    if (!joint_handles_ptr_) {return;}

    // Reset PIDs, zero commands
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      pids_[i]->reset();
      (*joint_handles_ptr_)[i].setCommand(0.0);
    }
  }

  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& period,
                     const State&         desired_state,
                     const State&         state_error)
  {
    const unsigned int n_joints = joint_handles_ptr_->size();

    // Preconditions
    if (!joint_handles_ptr_)
      return;
    assert(n_joints == state_error.position.size());
    assert(n_joints == state_error.velocity.size());

    // Update PIDs
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      const double command = (desired_state.velocity[i] * velocity_ff_[i]) + pids_[i]->computeCommand(state_error.position[i], state_error.velocity[i], period);
      (*joint_handles_ptr_)[i].setCommand(command);
    }
  }

private:
  typedef std::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;

  std::vector<double> velocity_ff_;

  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
};

/**
 * \brief Adapter for an velocity-controlled hardware interface. Maps position and velocity errors to velocity commands
 * through a velocity PID loop.
 *
 * The following is an example configuration of a controller that uses this adapter. Notice the \p gains and \p velocity_ff
 * entries:
 * \code
 * head_controller:
 *   type: "velocity_controllers/JointTrajectoryController"
 *   joints:
 *     - head_1_joint
 *     - head_2_joint
 *   gains:
 *     head_1_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *     head_2_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *   velocity_ff:
 *     head_1_joint: 1.0
 *     head_2_joint: 1.0
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     head_1_joint: {trajectory: 0.05, goal: 0.02}
 *     head_2_joint: {trajectory: 0.05, goal: 0.02}
 *   stop_trajectory_duration: 0.5
 *   state_publish_rate:  25
 * \endcode
 */
template <class State>
class HardwareInterfaceAdapter<hardware_interface::VelocityJointInterface, State> : public ClosedLoopHardwareInterfaceAdapter<State>
{};

/**
 * \brief Adapter for an effort-controlled hardware interface. Maps position and velocity errors to effort commands
 * through a position PID loop.
 *
 * The following is an example configuration of a controller that uses this adapter. Notice the \p gains and \p velocity_ff
 * entries:
 * \code
 * head_controller:
 *   type: "effort_controllers/JointTrajectoryController"
 *   joints:
 *     - head_1_joint
 *     - head_2_joint
 *   gains:
 *     head_1_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *     head_2_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *   velocity_ff:
 *     head_1_joint: 1.0
 *     head_2_joint: 1.0
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     head_1_joint: {trajectory: 0.05, goal: 0.02}
 *     head_2_joint: {trajectory: 0.05, goal: 0.02}
 *   stop_trajectory_duration: 0.5
 *   state_publish_rate:  25
 * \endcode
 */
template <class State>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, State> : public ClosedLoopHardwareInterfaceAdapter<State>
{};

/**
 * \brief Adapter for a pos-vel hardware interface. Forwards desired positions with velcities as commands.
 */
template <class State>
class HardwareInterfaceAdapter<hardware_interface::PosVelJointInterface, State>
{
public:
  HardwareInterfaceAdapter() : joint_handles_ptr_(nullptr) {}

  bool init(std::vector<hardware_interface::PosVelJointHandle>& joint_handles, ros::NodeHandle& /*controller_nh*/)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    return true;
  }

  void starting(const ros::Time& /*time*/) {}
  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         desired_state,
                     const State&         /*state_error*/)
  {
    // Forward desired position to command
    const unsigned int n_joints = joint_handles_ptr_->size();
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      (*joint_handles_ptr_)[i].setCommand(desired_state.position[i], desired_state.velocity[i]);
    }
  }

private:
  std::vector<hardware_interface::PosVelJointHandle>* joint_handles_ptr_;
};

/**
 * \brief Adapter for a spline-controlled hardware interface. Forwards desired positions as commands.
 */
template <class State>
class HardwareInterfaceAdapter<hardware_interface::PosVelAccJointInterface, State>
{
public:
  HardwareInterfaceAdapter() : joint_handles_ptr_(nullptr) {}

  bool init(std::vector<hardware_interface::PosVelAccJointHandle>& joint_handles, ros::NodeHandle& /*controller_nh*/)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    return true;
  }

  void starting(const ros::Time& /*time*/) {}
  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         desired_state,
                     const State&         /*state_error*/)
  {
    // Forward desired position to command
    const unsigned int n_joints = joint_handles_ptr_->size();
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      (*joint_handles_ptr_)[i].setCommand(desired_state.position[i], desired_state.velocity[i], desired_state.acceleration[i]);
    }
  }

private:
  std::vector<hardware_interface::PosVelAccJointHandle>* joint_handles_ptr_;
};
