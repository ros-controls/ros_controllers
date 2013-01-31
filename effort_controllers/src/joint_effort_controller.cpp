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

#include <effort_controllers/joint_effort_controller.h>
#include <pluginlib/class_list_macros.h>


namespace effort_controllers {

JointEffortController::JointEffortController()
: command_(0)
{}

JointEffortController::~JointEffortController()
{
  sub_command_.shutdown();
}

bool JointEffortController::init(hardware_interface::EffortJointInterface *robot, const std::string &joint_name)
{
  joint_ = robot->getJointHandle(joint_name);
  return true;
}


bool JointEffortController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }
  joint_ = robot->getJointHandle(joint_name);
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointEffortController::commandCB, this);
  return true;
}

void JointEffortController::update(const ros::Time& time, const ros::Duration& period)
{
  joint_.setCommand(command_);
}

void JointEffortController::commandCB(const std_msgs::Float64ConstPtr& msg)
{
  command_ = msg->data;
}
}// namespace

PLUGINLIB_DECLARE_CLASS(effort_controllers, JointEffortController, effort_controllers::JointEffortController, controller_interface::ControllerBase)

