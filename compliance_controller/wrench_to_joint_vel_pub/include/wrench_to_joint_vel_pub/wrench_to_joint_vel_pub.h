///////////////////////////////////////////////////////////////////////////////
//      Title     : wrench_to_joint_vel_pub.h
//      Project   : wrench_to_joint_vel_pub
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

#pragma once

#include <compliance_control_msgs/AdjustComplianceParams.h>
#include <compliance_control_msgs/CompliantVelocities.h>
#include <compliance_control_msgs/DisableComplianceDimensions.h>
#include <Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <math.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <wrench_to_joint_vel_pub/compliant_control.h>
#include <sstream>

namespace wrench_to_joint_vel_pub
{
static const char* const NODE_NAME = "wrench_to_joint_vel_pub";

struct ROSParameters
{
  double spin_rate, max_allowable_cmd_magnitude, low_pass_filter_param, highest_allowable_force,
      highest_allowable_torque, joint_limit_margin, condition_number_limit, ee_weight;
  std::string jacobian_frame_name, force_torque_frame_name, force_torque_topic, move_group_name,
      outgoing_joint_vel_topic, gravity_frame_name;
  std::vector<double> stiffness, damping, deadband, end_condition_wrench, default_dimensions, ee_com;
  bool compensate_for_ee_weight;
};

class PublishCompliantJointVelocities
{
public:
  /**
   * Create an object which takes wrench data, converts to compliant twist corrections, and publishes.
   * @param filter_param Larger->more smoothing but more lag.
   */
  PublishCompliantJointVelocities();

  /**
   * Spin and publish compliance data, unless disabled by service call.
   */
  void spin();

private:
  /**
   * A service callback. Toggles compliance publication on/off
   */
  bool toggleCompliance(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    compliance_enabled_ = req.data;
    res.success = true;
    return true;
  }

  /**
   * A service callback. Do not control compliance in the given dimensions.
   * TODO: error checking that these indices are betweeen 0-5 and in increasing order
   */
  bool disableComplianceDimensions(compliance_control_msgs::DisableComplianceDimensions::Request& req,
                                   compliance_control_msgs::DisableComplianceDimensions::Response& res);

  /**
   * A service callback. Adjust stiffness constants.
   */
  bool adjustStiffness(compliance_control_msgs::AdjustComplianceParams::Request& req,
                                   compliance_control_msgs::AdjustComplianceParams::Response& res);

  /**
   * A service callback. Adjust damping constants.
   */
  bool adjustDamping(compliance_control_msgs::AdjustComplianceParams::Request& req,
                                   compliance_control_msgs::AdjustComplianceParams::Response& res);

  /**
   * A service callback. Biases (aka tares, aka zeroes) the compliance calculations
   * based on the most-recently-received wrench
   */
  bool biasCompliantCalcs(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  /**
   * Callback for wrench data
   */
  void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    last_wrench_data_ = *msg;
    last_wrench_data_.header.frame_id = compliance_params_.force_torque_frame_name;
  }

  /**
   * Callback for robot state updates
   */
  void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    kinematic_state_->setVariableValues(*msg);
  }

  /**
    * Read parameters from a YAML file.
    */
  void readROSParameters();

  /**
    * Halt if we're past a joint margin and joint velocity is moving even farther past.
    */
  bool checkJointLimits();

  ros::NodeHandle n_;

  // An object to do compliance calculations.
  // Key equation: compliance_velocity[i] = wrench[i]/stiffness[i]
  std::unique_ptr<wrench_to_joint_vel_pub::CompliantControl> compliant_control_ptr_;

  bool compliance_enabled_ = true;

  static ROSParameters compliance_params_;

  // Publish compliance commands unless interrupted by a service call
  ros::ServiceServer enable_compliance_service_, bias_compliance_service_, disable_compliance_dimensions_service_,
      adjust_stiffness_service_, adjust_damping_service_;

  // Subscribe to wrench data from a force/torque sensor
  ros::Subscriber wrench_subscriber_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // MoveIt! setup, required to retrieve the Jacobian
  const robot_state::JointModelGroup* joint_model_group_;
  robot_state::RobotStatePtr kinematic_state_;

  geometry_msgs::WrenchStamped last_wrench_data_;

  ros::Publisher compliant_velocity_pub_;
  ros::Publisher debug_pub_; // DEBUG

  ros::Subscriber joints_sub_;

  // For Jacobian pseudoinverse calculations
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
  Eigen::MatrixXd matrix_s_;
  Eigen::MatrixXd pseudo_inverse_;
  Eigen::VectorXd delta_theta_;

  // Track degrees of freedom to drop
  // By default, ignore compliance in dimensions 3,4,5
  // i.e. roll/pitch/yaw
  std::vector<bool> compliant_dofs_;
  size_t num_output_dofs_; // Tracks the output DOF's (in case it != 6)
};

}  // namespace wrench_to_joint_vel_pub
