///////////////////////////////////////////////////////////////////////////////
//      Title     : compliant_control.cpp
//      Project   : wrench_to_joint_vel_pub
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

// Provides a library for compliance.

#include <wrench_to_joint_vel_pub/compliant_control.h>

namespace wrench_to_joint_vel_pub
{
CompliantControl::CompliantControl(const std::vector<double>& stiffness, const std::vector<double>& damping,
                                   const std::vector<double>& deadband, const std::vector<double>& end_condition_wrench,
                                   double filter_param, geometry_msgs::WrenchStamped bias,
                                   double highest_allowable_force, double highest_allowable_torque)
  : deadband_(deadband)
  , end_condition_wrench_(end_condition_wrench)
  , safe_force_limit_(highest_allowable_force)
  , safe_torque_limit_(highest_allowable_torque)
{
  setStiffness(stiffness);
  setDamping(damping);

  bias_.resize(wrench_to_joint_vel_pub::NUM_DIMS, 0);
  ee_weight_bias_.resize(wrench_to_joint_vel_pub::NUM_DIMS, 0);
  wrench_.resize(wrench_to_joint_vel_pub::NUM_DIMS, 0);
  wrench_dot_.resize(wrench_to_joint_vel_pub::NUM_DIMS, 0);

  for (int i = 0; i < wrench_to_joint_vel_pub::NUM_DIMS; ++i)
  {
    wrench_filters_.push_back(LowPassFilter(filter_param));
    wrench_derivative_filters_.push_back(LowPassFilter(filter_param));
  }

  biasSensor(bias);
  wrench_ = bias_;
}

void CompliantControl::biasSensor(const geometry_msgs::WrenchStamped& bias)
{
  bias_[0] = bias.wrench.force.x;
  bias_[1] = bias.wrench.force.y;
  bias_[2] = bias.wrench.force.z;
  bias_[3] = bias.wrench.torque.x;
  bias_[4] = bias.wrench.torque.y;
  bias_[5] = bias.wrench.torque.z;

  // Reset
  wrench_dot_.resize(wrench_to_joint_vel_pub::NUM_DIMS, 0);
  for (int i = 0; i < wrench_to_joint_vel_pub::NUM_DIMS; ++i)
  {
    wrench_filters_[i].reset(0.);
    wrench_derivative_filters_[i].reset(0.);
  }
}

void CompliantControl::subtractEEWeight(const geometry_msgs::WrenchStamped& bias)
{
  ee_weight_bias_[0] = bias.wrench.force.x;
  ee_weight_bias_[1] = bias.wrench.force.y;
  ee_weight_bias_[2] = bias.wrench.force.z;
  ee_weight_bias_[3] = bias.wrench.torque.x;
  ee_weight_bias_[4] = bias.wrench.torque.y;
  ee_weight_bias_[5] = bias.wrench.torque.z;
}

bool CompliantControl::setStiffness(const std::vector<double>& stiffness)
{
  if (stiffness.size() != wrench_to_joint_vel_pub::NUM_DIMS)
  {
    ROS_ERROR_NAMED(LOGNAME, "Invalid stiffness vector");
    return false;
  }
  else
  {
    stiffness_.resize(wrench_to_joint_vel_pub::NUM_DIMS, 0);
    for (int i = 0; i < wrench_to_joint_vel_pub::NUM_DIMS; ++i)
    {
      if (stiffness[i] <= 1e-3)
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Stiffness must be > zero.Ignoring "
                                        "stiffness in direction: "
                                            << i);
        stiffness_[i] = DBL_MAX;  // Very stiff, practically no effect
        return false;
      }
      else
      {
        stiffness_[i] = stiffness[i];
      }
    }
  }

  return true;
}

bool CompliantControl::setDamping(const std::vector<double>& damping)
{
  if (damping.size() != wrench_to_joint_vel_pub::NUM_DIMS)
  {
    ROS_ERROR_NAMED(LOGNAME, "Invalid damping vector");
    return false;
  }
  else
  {
    damping_.resize(wrench_to_joint_vel_pub::NUM_DIMS, 0);
    for (int i = 0; i < wrench_to_joint_vel_pub::NUM_DIMS; ++i)
    {
      // The sign on damping should be +. See for example,
      // Hogan, 1985, Impedance Control of Industrial Robots, Appendix 3
      if (damping[i] <= 1e-3)
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Damping must be > zero. Ignoring "
                                        "damping in direction: "
                                            << i);
        damping_[i] = DBL_MAX;  // Very large, practically no effect
      }
      else
      {
        damping_[i] = damping[i];
      }
    }
  }
  return true;
}

void CompliantControl::setEndCondition(const std::vector<double>& end_condition_wrench)
{
  if (end_condition_wrench.size() != wrench_to_joint_vel_pub::NUM_DIMS)
  {
    ROS_ERROR_NAMED(LOGNAME, "Invalid vector end_condition_wrench: ");
  }
  else
  {
    for (int i = 0; i < wrench_to_joint_vel_pub::NUM_DIMS; ++i)
    {
      end_condition_wrench_[i] = end_condition_wrench[i];
    }
  }
}

void CompliantControl::updateWrench(geometry_msgs::WrenchStamped wrench_data)
{
  std::vector<double> biasedFT(6, 0.);

  // Apply the deadband
  if (fabs(wrench_data.wrench.force.x - bias_[0] - ee_weight_bias_[0]) < fabs(deadband_[0]))
    biasedFT[0] = 0.;
  else
    biasedFT[0] = wrench_data.wrench.force.x - bias_[0] - ee_weight_bias_[0];
  if (fabs(wrench_data.wrench.force.y - bias_[1] - ee_weight_bias_[1]) < fabs(deadband_[1]))
    biasedFT[1] = 0.;
  else
    biasedFT[1] = wrench_data.wrench.force.y - bias_[1] - ee_weight_bias_[1];
  if (fabs(wrench_data.wrench.force.z - bias_[2] - ee_weight_bias_[2]) < fabs(deadband_[2]))
    biasedFT[2] = 0.;
  else
    biasedFT[2] = wrench_data.wrench.force.z - bias_[2] - ee_weight_bias_[2];

  if (fabs(wrench_data.wrench.torque.x - bias_[3] - ee_weight_bias_[3]) < fabs(deadband_[3]))
    biasedFT[3] = 0.;
  else
    biasedFT[3] = wrench_data.wrench.torque.x - bias_[3] - ee_weight_bias_[3];
  if (fabs(wrench_data.wrench.torque.y - bias_[4] - ee_weight_bias_[4]) < fabs(deadband_[4]))
    biasedFT[4] = 0.;
  else
    biasedFT[4] = wrench_data.wrench.torque.y - bias_[4] - ee_weight_bias_[4];
  if (fabs(wrench_data.wrench.torque.z - bias_[5] - ee_weight_bias_[5]) < fabs(deadband_[5]))
    biasedFT[5] = 0.;
  else
    biasedFT[5] = wrench_data.wrench.torque.z - bias_[5] - ee_weight_bias_[5];

  for (int i = 0; i < 6; ++i)
  {
    wrench_[i] = wrench_filters_[i].filter(biasedFT[i]);
  }
}

wrench_to_joint_vel_pub::ExitCondition CompliantControl::getVelocity(std::vector<double> v_in,
                                                                     geometry_msgs::WrenchStamped wrench_data,
                                                                     std::vector<double>& v_out, ros::Time time)
{
  wrench_to_joint_vel_pub::ExitCondition exit_condition = wrench_to_joint_vel_pub::NOT_CONTROLLED;

  prev_wrench_ = wrench_;
  updateWrench(wrench_data);

  // Differentiate the wrench
  delta_t_ = time - prev_time_;
  prev_time_ = time;
  // Avoid divide-by-zero
  if (delta_t_ > ros::Duration(0))
  {
    for (int i = 0; i < 6; ++i)
    {
      wrench_dot_[i] = wrench_derivative_filters_[i].filter((wrench_[i] - prev_wrench_[i]) / delta_t_.toSec());
    }
  }
  else
  {
    for (int i = 0; i < 6; ++i)
    {
      wrench_dot_[i] = wrench_derivative_filters_[i].filter(0);
    }
  }

  if (pow(wrench_[0] * wrench_[0] + wrench_[1] * wrench_[1] + wrench_[2] * wrench_[2], 0.5) > safe_force_limit_ ||
      pow(wrench_[3] * wrench_[3] + wrench_[4] * wrench_[4] + wrench_[5] * wrench_[5], 0.5) > safe_torque_limit_)
  {
    ROS_ERROR_NAMED(LOGNAME, "Total force or torque exceeds safety limits. Stopping motion.");
    v_out = std::vector<double>(6, 0.0);
    return wrench_to_joint_vel_pub::FT_VIOLATION;
  }

  for (int i = 0; i < wrench_to_joint_vel_pub::NUM_DIMS; ++i)
  {
    // Target wrench was exceeded
    if (((end_condition_wrench_[i] > 0) && (wrench_[i] > end_condition_wrench_[i])) ||
        ((end_condition_wrench_[i] < 0) && (wrench_[i] < end_condition_wrench_[i])))
    {
      ROS_INFO_STREAM_NAMED(LOGNAME, "Exit condition met in direction: " << i);
      v_out[i] = 0.0;
      exit_condition = wrench_to_joint_vel_pub::CONDITION_MET;
    }
    // Normal compliance calculation
    else
    {
      v_out[i] = v_in[i] + wrench_[i] / stiffness_[i] - wrench_dot_[i] / damping_[i];
      if (exit_condition != wrench_to_joint_vel_pub::CONDITION_MET)
      {
        exit_condition = wrench_to_joint_vel_pub::CONDITION_NOT_MET;
      }
    }
  }
  return exit_condition;
}
}  // end namespace wrench_to_joint_vel_pub
