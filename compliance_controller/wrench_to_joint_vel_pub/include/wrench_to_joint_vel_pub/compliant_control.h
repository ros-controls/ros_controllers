///////////////////////////////////////////////////////////////////////////////
//      Title     : compliant_control.h
//      Project   : wrench_to_twist_pub
//      Created   : 9/27/2017
//      Author    : Nitish Sharma, Andy Zelenak
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

#pragma once

/**
 * compliant control class. Allows you to control each dimension with a
 * compliant constant.
 * The key equation for each dimension is compliance_velocity[i] =
 * wrench[i]/stiffness[i] + wrench_dot[i]/damping[i]
 */

#include <geometry_msgs/WrenchStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <string>
#include <vector>
#include <wrench_to_joint_vel_pub/low_pass_filter.h>

namespace wrench_to_joint_vel_pub
{
static const std::string LOGNAME = "compliant_control";

const size_t NUM_DIMS = 6; // 3 translational, 3 rotational dimensions

/**
 * ExitCondition enum.
 * The ExitCondition enum is used to know the condition of the controller in
 * the end.
 */
enum ExitCondition
{
  NOT_CONTROLLED = 0,     // None of the dimensions is set to be controlled.
  FT_VIOLATION = 1,       // Force or torque was read as maximum allowable.
  CONDITION_MET = 2,      // One of the compliant conditions is met.
  CONDITION_NOT_MET = 3,  // No violation or condition.
  POSE_ACHIEVED = 4       // The target pose was reached within tolerances.
};

class CompliantControl;
class LowPassFilter;

class CompliantControl
{
public:
  /**
   * \brief Construct an object to take wrench data and calculate compliance velocity
   * adjustments.
   */
  CompliantControl(const std::vector<double>& stiffness, const std::vector<double>& damping,
                   const std::vector<double>& deadband, const std::vector<double>& endConditionWrench,
                   double filter_param, geometry_msgs::WrenchStamped bias, double highest_allowable_force,
                   double highest_allowable_torque);

  /**
   * Set the "springiness" of compliance in each direction.
   * First three element are Newtons/(m/s), last 3 are N*m/(rad/s).
   * Should be positive.
   */
  bool setStiffness(const std::vector<double>& stiffness);

  /**
   * Set the "springiness" of compliance in each direction.
   * First three element are Newtons/m, last 3 are N*m/rad.
   * Should be positive.
   */
  bool setDamping(const std::vector<double>& damping);

  /**
   * \brief Exit when the given force/torque wrench is achieved in any direction
   */
  void setEndCondition(const std::vector<double>& endConditionWrench);

  /**
   * \brief Update member variables with current, filtered forces/torques
   */
  void updateWrench(geometry_msgs::WrenchStamped wrench_data);

  /**
   * \brief Update Force/Torque values
   */
  void dataCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  /**
   * \brief Bias the FT values, i.e. reset the baseline wrench measurement.
   */
  void biasSensor(const geometry_msgs::WrenchStamped& bias);

  /**
   * \brief Remove the weight of the EE from the measured wrench.
   */
  void subtractEEWeight(const geometry_msgs::WrenchStamped& bias);

  /**
   * \brief Calculate a velocity adjustment due to compliance
   */
  wrench_to_joint_vel_pub::ExitCondition getVelocity(std::vector<double> v_in, geometry_msgs::WrenchStamped wrench_data,
                                                     std::vector<double>& v_out, ros::Time time);

  std::vector<double> stiffness_;
  std::vector<double> damping_;
  std::vector<double> deadband_;
  std::vector<double> end_condition_wrench_;
  std::vector<double> wrench_;
  std::vector<double> wrench_dot_;
  // Initial biased force
  std::vector<double> bias_;
  // EE Weight bias
  std::vector<double> ee_weight_bias_;
  // Quit if these forces/torques are exceeded
  double safe_force_limit_, safe_torque_limit_;
  std::vector<wrench_to_joint_vel_pub::LowPassFilter> wrench_filters_;
  std::vector<wrench_to_joint_vel_pub::LowPassFilter> wrench_derivative_filters_;

  // Used in derivative calculations
  ros::Time prev_time_ = ros::Time(0);
  ros::Duration delta_t_ = ros::Duration(0);
  std::vector<double> prev_wrench_;
};
}  // namespace wrench_to_joint_vel_pub
