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

#include <effort_controllers/joint_velocity_controller.h>
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>


namespace effort_controllers {

JointVelocityController::JointVelocityController()
: command_(0), loop_count_(0)
{
  has_effort_limits_ = false;
  has_velocity_limits_ = false;
}

JointVelocityController::~JointVelocityController()
{
  sub_command_.shutdown();
}

bool JointVelocityController::init(hardware_interface::EffortJointInterface *robot, 
  const std::string &joint_name, const control_toolbox::Pid &pid)
{
  pid_controller_ = pid;

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  return true;
}

bool JointVelocityController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Get the joint's effort and velocity limits.
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse the URDF file.");
    return false;
  }
  const boost::shared_ptr<const urdf::Joint> joint_urdf = urdf.getJoint(joint_name);
  if (!joint_urdf)
  {
    ROS_ERROR("Could not find joint '%s' in the URDF file.", joint_name.c_str());
    return false;
  }
  if (joint_urdf->limits != NULL)
  {
    double limit = joint_urdf->limits->effort;
    if (limit >= 0)
    {
      has_effort_limits_ = true;
      min_effort_ = -limit;
      max_effort_ = limit;
    }

    limit = joint_urdf->limits->velocity;
    if (limit >= 0)
    {
      has_velocity_limits_ = true;
      min_velocity_ = -limit;
      max_velocity_ = limit;
    }
  }

  // Load PID Controller using gains set on parameter server
  if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    return false;

  // Start realtime state publisher
  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
    (n, "state", 1));

  // Start command subscriber
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointVelocityController::setCommandCB, this);

  return true;
}


void JointVelocityController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_.setGains(p,i,d,i_max,i_min);

}

void JointVelocityController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_.getGains(p,i,d,i_max,i_min);
}

void JointVelocityController::printDebug()
{
  pid_controller_.printValues();
}

std::string JointVelocityController::getJointName()
{
  return joint_.getName();
}

// Set the joint velocity command
void JointVelocityController::setCommand(double cmd)
{
  command_ = clampVelocity(cmd);
}

// Return the current velocity command
void JointVelocityController::getCommand(double& cmd)
{
  cmd = command_;
}

void JointVelocityController::starting(const ros::Time& time)
{
  command_ = 0.0;
  pid_controller_.reset();
}

void JointVelocityController::update(const ros::Time& time, const ros::Duration& period)
{
  const double vel = joint_.getVelocity();
  const double error = command_ - vel;

  // Set the PID error and compute the PID command with nonuniform time
  // step size. The derivative error is computed from the change in the error
  // and the timestep dt.
  const double commanded_effort = clampEffort(pid_controller_.computeCommand(error, period));

  joint_.setCommand(commanded_effort);

  if(loop_count_ % 10 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command_;
      controller_state_publisher_->msg_.process_value = vel;
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

void JointVelocityController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  setCommand(msg->data);
}

double JointVelocityController::clampEffort(const double effort) const
{
  if (has_effort_limits_)
  {
    if (effort < min_effort_)
      return min_effort_;
    if (effort > max_effort_)
      return max_effort_;
  }
  return effort;
}

double JointVelocityController::clampVelocity(const double velocity) const
{
  if (has_velocity_limits_)
  {
    if (velocity < min_velocity_)
      return min_velocity_;
    if (velocity > max_velocity_)
      return max_velocity_;
  }
  return velocity;
}

} // namespace

PLUGINLIB_EXPORT_CLASS( effort_controllers::JointVelocityController, controller_interface::ControllerBase)
