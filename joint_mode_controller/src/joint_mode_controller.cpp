/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Controller to allow joint controllers to easily switch modes between position, velocity, and effort-based control
*/

#include <pluginlib/class_list_macros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_mode_interface.h>

namespace joint_mode_controller {

class JointModeController: public controller_interface::Controller<hardware_interface::JointModeInterface>
{

private:


public:
  JointModeController()
  {}

  ~JointModeController()
  {}

  bool init(
    hardware_interface::JointModeInterface *robot, ros::NodeHandle &nh)
  {

    // Get handle name for this mode mechanism from the parameter server
    std::string handle_name;
    if (!nh.getParam("mode_handle", handle_name)) 
    {
      ROS_DEBUG_STREAM_NAMED("init","No mode_handle specified in namespace '" << nh.getNamespace() 
        << "', defaulting to 'joint_mode'");
      handle_name = "joint_mode"; // default name
    }

    // Get the joint mode, which we represent as an int for speed. User chooses what that mode actually represents
    int joint_mode;
    if (!nh.getParam("joint_mode", joint_mode)) 
    {
      ROS_DEBUG_STREAM_NAMED("init","No joint_mode specified in namespace '" << nh.getNamespace() 
        << "', defaulting to position");
      joint_mode = hardware_interface::MODE_POSITION; // default joint mode
    }

    robot->getHandle(handle_name).setMode(joint_mode);

    ROS_INFO_STREAM_NAMED("init","Set joint mode to " << robot->getHandle(handle_name).getModeName(joint_mode)
      << " (" << joint_mode << ")");

    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {}



};

} // namespace

PLUGINLIB_EXPORT_CLASS(
  joint_mode_controller::JointModeController,
  controller_interface::ControllerBase)
