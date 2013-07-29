/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 Author: Vijay Pradeep
 Contributors: Jonathan Bohren, Wim Meeussen, Dave Coleman
 Desc: Effort(force)-based position controller using basic PID loop
*/

#include <effort_controllers/joint_position_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace effort_controllers {

JointPositionController::JointPositionController()
  : loop_count_(0)
{}

JointPositionController::~JointPositionController()
{
  sub_command_.shutdown();
}

bool JointPositionController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name)) 
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Load PID Controller using gains set on parameter server
  pid_controller_.reset(new control_toolbox::Pid());
  if (!pid_controller_->init(ros::NodeHandle(n, "pid")))
    return false;

  // Start realtime state publisher
  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

  // Start command subscriber
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointPositionController::setCommandCB, this);

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  return true;
}

void JointPositionController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_->setGains(p,i,d,i_max,i_min);
}

void JointPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_->getGains(p,i,d,i_max,i_min);
}

std::string JointPositionController::getJointName()
{
  return joint_.getName();
}

// Set the joint position command
void JointPositionController::setCommand(double cmd)
{
  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  command_.writeFromNonRT(cmd);
}

void JointPositionController::starting(const ros::Time& time)
{
  double command = joint_.getPosition();

  // Make sure joint is within limits if applicable
  enforceJointLimits(command);

  command_.initRT(command);
  pid_controller_->reset();
}

void JointPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  double command = *(command_.readFromRT());

  double error, vel_error;

  double current_position = joint_.getPosition();

  // Make sure joint is within limits if applicable
  enforceJointLimits(command);

  // Compute position error
  if (joint_urdf_->type == urdf::Joint::REVOLUTE)
  {
   angles::shortest_angular_distance_with_limits(
      current_position,
      command,
      joint_urdf_->limits->lower,
      joint_urdf_->limits->upper,
      error);
  }
  else if (joint_urdf_->type == urdf::Joint::CONTINUOUS)
  {
    error = angles::shortest_angular_distance(current_position, command);
  }
  else //prismatic
  {
    error = command - current_position;
  }

  // Compute velocity error assuming desired velocity is 0
  vel_error = 0.0 - joint_.getVelocity();

  // Set the PID error and compute the PID command with nonuniform
  // time step size. This also allows the user to pass in a precomputed derivative error.
  double commanded_effort = pid_controller_->computeCommand(error, vel_error, period);
  joint_.setCommand(commanded_effort);

  // publish state
  if (loop_count_ % 10 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command;
      controller_state_publisher_->msg_.process_value = current_position;
      controller_state_publisher_->msg_.process_value_dot = joint_.getVelocity();
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = period.toSec();
      controller_state_publisher_->msg_.command = commanded_effort;

      double dummy;
      getGains(controller_state_publisher_->msg_.p,
        controller_state_publisher_->msg_.i,
        controller_state_publisher_->msg_.d,
        controller_state_publisher_->msg_.i_clamp,
        dummy);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;
}

void JointPositionController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  setCommand(msg->data);
}

void JointPositionController::enforceJointLimits(double &command)
{
  // Check that this joint has applicable limits
  if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
  {
    if( command > joint_urdf_->limits->upper ) // above upper limnit
    {
      command = joint_urdf_->limits->upper;
    }
    else if( command < joint_urdf_->limits->lower ) // below lower limit
    {
      command = joint_urdf_->limits->lower;
    }
  }
}

} // namespace

PLUGINLIB_EXPORT_CLASS( effort_controllers::JointPositionController, controller_interface::ControllerBase)
