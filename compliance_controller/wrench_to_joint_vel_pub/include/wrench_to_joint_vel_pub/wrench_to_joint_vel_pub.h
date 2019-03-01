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

#ifndef WRENCH_TO_JOINT_VEL_PUB_H
#define WRENCH_TO_JOINT_VEL_PUB_H

#include <compliance_control_msgs/ComplianceSettings.h>
#include <compliance_control_msgs/CompliantVelocities.h>
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

namespace wrench_to_joint_vel_pub
{
struct ROSParameters
{
  double spin_rate, max_allowable_cmd_magnitude, low_pass_filter_param, highest_allowable_force,
      highest_allowable_torque, joint_limit_margin;
  std::string jacobian_frame_name, force_torque_frame_name, force_torque_topic, move_group_name,
      outgoing_joint_vel_topic;
  std::vector<double> stiffness, damping, deadband, end_condition_wrench;
};

class PublishCompliantJointVelocities
{
public:
  /**
   * Create an object which takes wrench data, converts to compliant twist corrections, and publishes.
   * @param filter_param Larger->more smoothing but more lag.
   */
  PublishCompliantJointVelocities() : tf_listener_(tf_buffer_)
  {
    readROSParameters();

    // Initialize an object of the compliance library.
    // Use a unique_ptr to avoid memory management issues.
    // Assume a bias wrench of all zeros
    geometry_msgs::WrenchStamped bias;
    compliant_control_ptr_.reset(new wrench_to_joint_vel_pub::CompliantControl(
        compliance_params_.stiffness, compliance_params_.damping, compliance_params_.deadband,
        compliance_params_.end_condition_wrench, compliance_params_.low_pass_filter_param, bias,
        compliance_params_.highest_allowable_force, compliance_params_.highest_allowable_torque));

    enable_compliance_service_ =
        n_.advertiseService(n_.getNamespace() + "/" + ros::this_node::getName() + "/toggle_compliance_publication",
                            &PublishCompliantJointVelocities::toggleCompliance, this);

    bias_compliance_service_ =
        n_.advertiseService(n_.getNamespace() + "/" + ros::this_node::getName() + "/bias_compliance_calcs",
                            &PublishCompliantJointVelocities::biasCompliantCalcs, this);

    adjust_settings_service_ = n_.advertiseService(n_.getNamespace() + "/" + ros::this_node::getName() + "/adjust_compliance_settings",
                            &PublishCompliantJointVelocities::adjustSettings, this);

    wrench_subscriber_ =
        n_.subscribe(compliance_params_.force_torque_topic, 1, &PublishCompliantJointVelocities::wrenchCallback, this);

    std::unique_ptr<robot_model_loader::RobotModelLoader> model_loader_ptr_ =
        std::unique_ptr<robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader);
    const robot_model::RobotModelPtr& kinematic_model = model_loader_ptr_->getModel();
    joint_model_group_ = kinematic_model->getJointModelGroup(compliance_params_.move_group_name);
    kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model);

    compliant_velocity_pub_ =
        n_.advertise<compliance_control_msgs::CompliantVelocities>(compliance_params_.outgoing_joint_vel_topic, 1);

    joints_sub_ = n_.subscribe("joint_states", 1, &PublishCompliantJointVelocities::jointsCallback, this);
  }

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
   * A service callback. Adjusts compliance settings
   * TODO: error checking that these indices are betweeen 0-5 and in increasing order
   */
  bool adjustSettings(compliance_control_msgs::ComplianceSettings::Request& req, compliance_control_msgs::ComplianceSettings::Response& res)
  {
    dof_to_drop_.clear();
    dof_to_drop_.push_back(req.dimensions_to_ignore.data[0]);
    ROS_ERROR_STREAM(dof_to_drop_.front() << " " << dof_to_drop_.back());

    return true;
  }

  /**
   * A service callback. Biases (aka tares, aka zeroes) the compliance calculations
   * based on the most-recently-received wrench
   */
  bool biasCompliantCalcs(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    if (req.data)
    {
      compliant_control_ptr_->biasSensor(last_wrench_data_);
      ROS_INFO_STREAM("The bias of compliance calculations was reset.");
      res.success = true;
    }
    else
      res.success = false;

    return true;
  }

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
  ros::ServiceServer enable_compliance_service_, bias_compliance_service_, adjust_settings_service_;

  // Subscribe to wrench data from a force/torque sensor
  ros::Subscriber wrench_subscriber_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // MoveIt! setup, required to retrieve the Jacobian
  const robot_state::JointModelGroup* joint_model_group_;
  robot_state::RobotStatePtr kinematic_state_;

  geometry_msgs::WrenchStamped last_wrench_data_;

  ros::Publisher compliant_velocity_pub_;

  ros::Subscriber joints_sub_;

  // For Jacobian pseudoinverse calculations
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
  Eigen::MatrixXd matrix_s_;
  Eigen::MatrixXd pseudo_inverse_;
  Eigen::VectorXd delta_theta_;

  // Track degrees of freedom to drop
  // By default, ignore compliance in dimensions 3,4,5
  // i.e. roll/pitch/yaw
  std::vector<int> dof_to_drop_{3,4,5};
};

}  // namespace wrench_to_joint_vel_pub

#endif // WRENCH_TO_JOINT_VEL_PUB_H
