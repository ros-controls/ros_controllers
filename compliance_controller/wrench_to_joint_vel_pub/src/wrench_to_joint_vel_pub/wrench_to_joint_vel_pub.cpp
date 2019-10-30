///////////////////////////////////////////////////////////////////////////////
//      Title     : wrench_to_joint_vel_pub.cpp
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

// Subscribe to a wrench and publish a compliant joint velocity correction

#include <wrench_to_joint_vel_pub/wrench_to_joint_vel_pub.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, wrench_to_joint_vel_pub::NODE_NAME);

  // Do compliance calculations in this class
  wrench_to_joint_vel_pub::PublishCompliantJointVelocities publish_compliance_velocities;

  // Spin and publish compliance velocities, unless disabled by a service call
  publish_compliance_velocities.spin();

  return 0;
}

namespace wrench_to_joint_vel_pub
{
// Initialize static member of class PublishCompliantJointVelocities
ROSParameters PublishCompliantJointVelocities::compliance_params_;

PublishCompliantJointVelocities::PublishCompliantJointVelocities() : tf_listener_(tf_buffer_)
{
  readROSParameters();

  // Initialize an object of the compliance library.
  // Use a unique_ptr to avoid memory management issues.
  // Assume a bias wrench of all zeros
  geometry_msgs::WrenchStamped bias;
  compliant_control_ptr_.reset(
      new CompliantControl(compliance_params_.stiffness, compliance_params_.damping, compliance_params_.deadband,
                           compliance_params_.end_condition_wrench, compliance_params_.low_pass_filter_param, bias,
                           compliance_params_.highest_allowable_force, compliance_params_.highest_allowable_torque));

  enable_compliance_service_ =
      n_.advertiseService(n_.getNamespace() + "/" + ros::this_node::getName() + "/toggle_compliance_publication",
                          &PublishCompliantJointVelocities::toggleCompliance, this);

  bias_compliance_service_ =
      n_.advertiseService(n_.getNamespace() + "/" + ros::this_node::getName() + "/bias_compliance_calcs",
                          &PublishCompliantJointVelocities::biasCompliantCalcs, this);

  disable_compliance_dimensions_service_ =
      n_.advertiseService(n_.getNamespace() + "/" + ros::this_node::getName() + "/disable_compliance_dimensions",
                          &PublishCompliantJointVelocities::disableComplianceDimensions, this);

  adjust_stiffness_service_ =
      n_.advertiseService(n_.getNamespace() + "/" + ros::this_node::getName() + "/adjust_stiffness",
                          &PublishCompliantJointVelocities::adjustStiffness, this);

  adjust_damping_service_ =
      n_.advertiseService(n_.getNamespace() + "/" + ros::this_node::getName() + "/adjust_damping",
                          &PublishCompliantJointVelocities::adjustDamping, this);

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

bool PublishCompliantJointVelocities::disableComplianceDimensions(
    compliance_control_msgs::DisableComplianceDimensions::Request& req,
    compliance_control_msgs::DisableComplianceDimensions::Response& res)
{
  compliant_dofs_.assign(6, true);
  num_compliant_dofs_ = 6 - req.dimensions_to_ignore.data.size();
  for (std::vector<int>::const_iterator it = req.dimensions_to_ignore.data.begin();
       it != req.dimensions_to_ignore.data.end(); ++it)
  {
    compliant_dofs_[*it] = false;
  }

  ROS_INFO_STREAM("Compliant Dimensions are now: [" << compliant_dofs_[0] << ", "
      << compliant_dofs_[1] << ", "
      << compliant_dofs_[2] << ", "
      << compliant_dofs_[3] << ", "
      << compliant_dofs_[4] << ", "
      << compliant_dofs_[5] << "].");

  res.success = true;
  return true;
}

bool PublishCompliantJointVelocities::adjustStiffness(
    compliance_control_msgs::AdjustStiffness::Request& req,
    compliance_control_msgs::AdjustStiffness::Response& res)
{
  std::vector<double> stiffness = req.stiffness.data;
  if (compliant_control_ptr_->setStiffness(stiffness))
  {
    res.success = true;
    return true;
  }
  else
  {
    res.success = false;
    return false;
  }
}

bool PublishCompliantJointVelocities::adjustDamping(
    compliance_control_msgs::AdjustDamping::Request& req,
    compliance_control_msgs::AdjustDamping::Response& res)
{
  std::vector<double> damping = req.damping.data;
  if (compliant_control_ptr_->setDamping(damping))
  {
    res.success = true;
    return true;
  }
  else
  {
    res.success = false;
    return false;
  }
}

bool PublishCompliantJointVelocities::biasCompliantCalcs(std_srvs::SetBool::Request& req,
                                                         std_srvs::SetBool::Response& res)
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

bool PublishCompliantJointVelocities::checkJointLimits()
{
  for (auto joint : joint_model_group_->getJointModels())
  {
    if (!kinematic_state_->satisfiesPositionBounds(joint, -compliance_params_.joint_limit_margin))
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(2, NODE_NAME, ros::this_node::getName() << " " << joint->getName()
                                                                             << " close to a "
                                                                                " position limit. Halting.");
      return true;
    }
  }

  return false;
}

void PublishCompliantJointVelocities::spin()
{
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Rate rate(compliance_params_.spin_rate);

    if (compliance_enabled_)
    {
      // The algorithm:
      // Get a wrench in the force/torque sensor frame
      // With the compliance object, calculate a compliant, Cartesian velocity in the force/torque sensor frame
      // Transform this Cartesian velocity to the MoveIt! planning frame
      // Multiply by the Jacobian pseudo-inverse to calculate a joint velocity vector
      // Publish this joint velocity vector
      // Another node can sum it with nominal joint velocities to result in spring-like motion

      // Input to the compliance calculation is an all-zero nominal velocity
      std::vector<double> velocity(6);
      // Calculate the compliant velocity adjustment
      compliant_control_ptr_->getVelocity(velocity, last_wrench_data_, velocity, ros::Time::now());

      // Remove non-compliant dimensions
      if(num_compliant_dofs_ < 6) // Want to not be compliant in some dimension(s)
      {
        for (size_t i = 0; i < compliant_dofs_.size(); ++i)
        {
          if(!compliant_dofs_[i]) // If we are not compliant:
          {
            // It is as easy as setting the compliant velocity to 0, instead of what it was for the impedance law
            velocity[i] = 0;
          }
        }
      }

      geometry_msgs::Vector3Stamped translational_velocity;
      translational_velocity.header.frame_id = compliance_params_.force_torque_frame_name;
      translational_velocity.vector.x = velocity[0];
      translational_velocity.vector.y = velocity[1];
      translational_velocity.vector.z = velocity[2];

      geometry_msgs::Vector3Stamped rotational_velocity;
      rotational_velocity.header.frame_id = compliance_params_.force_torque_frame_name;
      rotational_velocity.vector.x = velocity[3];
      rotational_velocity.vector.y = velocity[4];
      rotational_velocity.vector.z = velocity[5];

      // Transform this Cartesian velocity to the Jacobian frame
      geometry_msgs::TransformStamped force_torque_to_moveit_tf;
      while (force_torque_to_moveit_tf.header.frame_id == "" && ros::ok())
      {
        try
        {
          force_torque_to_moveit_tf = tf_buffer_.lookupTransform(
              compliance_params_.jacobian_frame_name, compliance_params_.force_torque_frame_name, ros::Time(0));
        }
        catch (tf2::TransformException& ex)
        {
          ROS_WARN_NAMED(NODE_NAME, "%s", ex.what());
          ROS_WARN_NAMED(NODE_NAME,
                         "Waiting for the transform from force/torque to the Jacobian frame to be published.");
          ros::Duration(0.01).sleep();
          continue;
        }
      }
      tf2::doTransform(translational_velocity, translational_velocity, force_torque_to_moveit_tf);
      tf2::doTransform(rotational_velocity, rotational_velocity, force_torque_to_moveit_tf);

      Eigen::VectorXd cartesian_velocity(6);

      cartesian_velocity[0] = translational_velocity.vector.x;
      cartesian_velocity[1] = translational_velocity.vector.y;
      cartesian_velocity[2] = translational_velocity.vector.z;
      cartesian_velocity[3] = rotational_velocity.vector.x;
      cartesian_velocity[4] = rotational_velocity.vector.y;
      cartesian_velocity[5] = rotational_velocity.vector.z;

      // Multiply by the Jacobian pseudo-inverse to calculate a joint velocity vector
      // This Jacobian is w.r.t. to the last link
      Eigen::MatrixXd jacobian = kinematic_state_->getJacobian(joint_model_group_);

      svd_ = Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);

      if(fabs(svd_.singularValues()[0] / svd_.singularValues().tail(1)[0] > compliance_params_.condition_number_limit))
      {
        // If condition number is too high set the compliance-adjusted velocities to 0
        for (int i = 0; i < delta_theta_.size(); ++i)
        {
          delta_theta_[i] = 0.0;
        }
        ROS_WARN_STREAM_THROTTLE_NAMED(1, NODE_NAME, "Jacobian is near singular (condition number: " 
              << svd_.singularValues()[0] / svd_.singularValues().tail(1)[0] << ")! Pausing compliant commands.");
      }
      else
      {
        matrix_s_ = svd_.singularValues().asDiagonal();
        pseudo_inverse_ = svd_.matrixV() * matrix_s_.inverse() * svd_.matrixU().transpose();
        delta_theta_ = pseudo_inverse_ * cartesian_velocity;

        // Check if a command magnitude would be too large.
        double largest_allowable_command = compliance_params_.max_allowable_cmd_magnitude;
        for (int i = 0; i < delta_theta_.size(); ++i)
        {
          if ((fabs(delta_theta_[i]) > largest_allowable_command) || std::isnan(delta_theta_[i]))
          {
            ROS_WARN_STREAM_NAMED(NODE_NAME, "Magnitude of compliant command is too large. Pausing compliant commands.");
            for (int j = 0; j < delta_theta_.size(); ++j)
            {
              delta_theta_[j] = 0.;
            }
            break;
          }
        }
      }

      // Publish this joint velocity vector
      // Type is std_msgs/Float64MultiArray.h
      compliance_control_msgs::CompliantVelocities delta_theta_msg;
      for (int i = 0; i < delta_theta_.size(); ++i)
      {
        delta_theta_msg.compliant_velocities.data.push_back(delta_theta_[i]);
      }

      // Check if the command would cause a joint limit to be exceeded
      delta_theta_msg.near_joint_limit = checkJointLimits();

      compliant_velocity_pub_.publish(delta_theta_msg);
    }

    rate.sleep();
  }
}

void PublishCompliantJointVelocities::readROSParameters()
{
  std::size_t error = 0;

  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/spin_rate", compliance_params_.spin_rate);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/max_allowable_cmd_magnitude",
                                    compliance_params_.max_allowable_cmd_magnitude);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/move_group_name",
                                    compliance_params_.move_group_name);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/jacobian_frame_name",
                                    compliance_params_.jacobian_frame_name);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/force_torque_frame_name",
                                    compliance_params_.force_torque_frame_name);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/force_torque_topic",
                                    compliance_params_.force_torque_topic);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/joint_limit_margin",
                                    compliance_params_.joint_limit_margin);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/condition_number_limit",
                                    compliance_params_.condition_number_limit);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/outgoing_joint_vel_topic",
                                    compliance_params_.outgoing_joint_vel_topic);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/compliance_library/low_pass_filter_param",
                                    compliance_params_.low_pass_filter_param);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/compliance_library/highest_allowable_force",
                                    compliance_params_.highest_allowable_force);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/compliance_library/highest_allowable_torque",
                                    compliance_params_.highest_allowable_torque);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/compliance_library/stiffness",
                                    compliance_params_.stiffness);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/compliance_library/damping",
                                    compliance_params_.damping);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/compliance_library/deadband",
                                    compliance_params_.deadband);
  error += !rosparam_shortcuts::get("", n_, ros::this_node::getName() + "/compliance_library/end_condition_wrench",
                                    compliance_params_.end_condition_wrench);

  rosparam_shortcuts::shutdownIfError(ros::this_node::getName(), error);

  // Input checking
  if ((compliance_params_.spin_rate <= 0) || (compliance_params_.low_pass_filter_param <= 0) ||
      (compliance_params_.max_allowable_cmd_magnitude <= 0) || (compliance_params_.highest_allowable_force <= 0) ||
      (compliance_params_.highest_allowable_torque <= 0) || (compliance_params_.joint_limit_margin <= 0))
  {
    ROS_ERROR_STREAM_NAMED(NODE_NAME, "These parameters should be greater than zero:");
    ROS_ERROR_STREAM_NAMED(NODE_NAME, "spin_rate, low_pass_filter_param, max_allowable_cmd_magnitude, "
                                      "highest_allowable_force, highest_allowable_torque, joint_limit_margin");
    exit(1);
  }

  for (std::size_t i = 0; i < 6; ++i)
  {
    if ((compliance_params_.stiffness[i] < 0) || (compliance_params_.deadband[i] < 0) ||
        (compliance_params_.end_condition_wrench[i] < 0))
    {
      ROS_ERROR_STREAM_NAMED(NODE_NAME,
                             "stiffness/deadband/end_condition_wrench parameters should be greater than zero.");
      exit(1);
    }
  }
}
}  // namespace
