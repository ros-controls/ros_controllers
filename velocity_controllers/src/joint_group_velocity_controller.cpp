/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
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

#include <velocity_controllers/joint_group_velocity_controller.h>
#include <pluginlib/class_list_macros.hpp>

template<>
void velocity_controllers::JointGroupVelocityController::starting(const ros::Time& /*time*/)
{
  // Start controller with 0.0 velocities
  commands_buffer_.readFromRT()->assign(n_joints_, 0.0);
}

template<>
void velocity_controllers::JointGroupVelocityController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  std::vector<double> & commands = *commands_buffer_.readFromRT();

  // Check timeout
  if (command_timeout_ > 0.0)
  {
    ros::Time& last_received_command_time = *last_received_command_time_buffer_.readFromRT();
    const double command_age = (time - last_received_command_time).toSec();
    if (std::abs(command_age) > command_timeout_)
    {
      ROS_WARN_STREAM_THROTTLE(10, "Commands timed out, setting to zero.");
      for (std::size_t i = 0; i < commands.size(); ++i)
      {  commands[i] = 0;  }
    }
  }

  for(unsigned int i=0; i<n_joints_; i++)
  {  joints_[i].setCommand(commands[i]);  }
}

PLUGINLIB_EXPORT_CLASS(velocity_controllers::JointGroupVelocityController,controller_interface::ControllerBase)
