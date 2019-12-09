///////////////////////////////////////////////////////////////////////////////
//      Title     : compliance_controller.h
//      Project   : compliance_controller
//      Created   : 12/4/2018
//      Author    : Andy Zelenak
//
// BSD 3-Clause License
//
// Copyright (c) 2018, Los Alamos National Security, LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef COMPLIANCE_CONTROLLER_COMPLIANCE_CONTROLLER_H
#define COMPLIANCE_CONTROLLER_COMPLIANCE_CONTROLLER_H

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// Project
#include <actionlib/server/action_server.h>
#include <compliance_control_msgs/CompliantVelocities.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_srvs/Trigger.h>
#include <traj_or_jog_controller/traj_or_jog_controller.h>
#include <trajectory_interface/quintic_spline_segment.h>

namespace compliance_controller
{
// ComplianceController inherits from TrajOrJogController, which inherits from JointTrajectoryController
// and has been modified to accept real-time velocity commands, as well.
// This gives flexibility to handle trajectory commands and real-time commands.
template <class SegmentImpl, class HardwareInterface>
class ComplianceController : public traj_or_jog_controller::TrajOrJogController<SegmentImpl, HardwareInterface>
{
public:
  /**
   * \brief Initialize the plugin.
   */
  bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

protected:
  typedef traj_or_jog_controller::TrajOrJogController<SegmentImpl, HardwareInterface> TrajOrJogController;
  typedef joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>
      JointTrajectoryController;

private:
  /** \brief Override updates of the base class. */
  virtual void update(const ros::Time& time, const ros::Duration& period);
  /*\}*/

  /**
   * \brief Flag that the real-time velocity controller should be active.
   */
  virtual void activateVelocityStreaming();

  /**
   * \brief Update the JointTrajectoryController, adding small compliant adjustments.
   */
  virtual void updateJointTrajControllerWithCompliace(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Callback for messages of compliant joint velocity adjustments.
   */
  virtual void complianceAdjustmentCallback(const compliance_control_msgs::CompliantVelocities::ConstPtr& msg);

  /**
   * \brief Provides a service to toggle compliance
   */
  bool toggleCompliance(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * \brief Provides a service to enable compliance
   */
  bool enableCompliance(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * \brief Provides a service to disabel compliance
   */
  bool disableCompliance(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  ros::NodeHandle controller_nh_;
  ros::Subscriber compliance_adjustment_sub_;
  ros::ServiceServer toggle_compliance_service_;
  ros::ServiceServer enable_compliance_service_;
  ros::ServiceServer disable_compliance_service_;

  // This stamp helps in tracking whether compliance commands are stale.
  // If stale, they will be ignored.
  ros::Time last_compliance_adjustment_stamp_;

  // Threshold to determine whether compliance commands are stale.
  ros::Duration compliance_command_timeout_;

  // Timestamp of previous updateJointTrajControllerWithCompliace()
  ros::Time last_trajectory_update_time_;

  // Cache the latest compliance command
  std_msgs::Float64MultiArray compliance_velocity_adjustment_;

  // The total compliant position adjustment for a trajectory
  std::vector<double> integrated_traj_position_adjustment_;

  // If true, we're near a joint limit
  bool near_joint_limit_ = false;

  // Toggles compliance
  bool compliance_enabled_ = false;
};
}

#endif  // header guard
