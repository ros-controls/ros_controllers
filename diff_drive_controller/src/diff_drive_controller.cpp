/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>

#include <urdf_parser/urdf_parser.h>

#include <realtime_tools/realtime_buffer.h>

namespace diff_drive_controller{

  class DiffDriveController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    bool init(hardware_interface::VelocityJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh)
    {
      ROS_INFO("Init DiffDriveController.");
      // get joint names from the parameter server

//      if (!controller_nh.getParam("joints", joints))
//      {
//        ROS_ERROR("Could not find joint names");
//        return false;
//      }

//      // parse robot description
//      std::string model_param_name = "/robot_description";
//      bool res = nh.hasParam(model_param_name);
//      std::string robot_model_str="";
//      if(!res || !nh.getParam(model_param_name,robot_model_str))
//      {
//        ROS_ERROR("Robot descripion couldn't be retrieved from param server.");
//        return false;
//      }

//      boost::shared_ptr<urdf::ModelInterface> modelPtr = urdf::parseURDF(robot_model_str);
//      std::string wheel_joint_name = "wheel_left_joint";
//      boost::shared_ptr<const urdf::Joint> wheelJointPtr    = modelPtr->getJoint(wheel_joint_name);
//      if(!wheelJointPtr.get())
//      {
//        PAL_ORO_ERROR_STREAM_TAG(  PAL_DEV_DEBUG_ON, PAL_FILE <<':'<< __LINE__ << ":failed: "<< wheel_joint_name <<
//                                   " couldn't be retrieved from model description" << '\n');
//        return false;
//      }

//      double wheel_radius = fabs(wheelJointPtr->parent_to_joint_origin_transform.position.z);
//      double wheel_base   = 2.0 * fabs(wheelJointPtr->parent_to_joint_origin_transform.position.y);
//      PAL_ORO_INFO_STREAM_TAG(  PAL_DEV_DEBUG_ON, "Odometry params : wheel base " << wheel_base << ", wheel radius " << wheel_radius);

//      _odometryInfo.wheelsRadius[odometry::OdometryInfo::LEFT_WHEEL]  = wheel_radius;
//      _odometryInfo.wheelsRadius[odometry::OdometryInfo::RIGHT_WHEEL] = wheel_radius;
//      _odometryInfo.wheelBase = wheel_base;


      // get the joint object to use in the realtime loop

      left_wheel_joint_ = hw->getHandle("wheel_left_joint");  // throws on failure
      right_wheel_joint_ = hw->getHandle("wheel_right_joint");  // throws on failure
      return true;
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
      const double vel = 4.0;
      left_wheel_joint_.setCommand(vel);
      right_wheel_joint_.setCommand(vel);
      // do stuff coming from cmd_vel interface

      // publish odom
    }

    void starting(const ros::Time& time)
    {
      // set velocity to 0
    }

    void stopping(const ros::Time& time)
    {
      // set velocity to 0
    }

  private:
    struct Commands
      {
        double position_; // Last commanded position
        double velocity_; // Last commanded velocity
      };

    hardware_interface::JointHandle left_wheel_joint_;
    hardware_interface::JointHandle right_wheel_joint_;
    nav_msgs::Odometry odom;
    boost::shared_ptr<const urdf::Joint> joint_urdf_;
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
  };

  PLUGINLIB_DECLARE_CLASS(diff_drive_controller, DiffDriveController, diff_drive_controller::DiffDriveController, controller_interface::ControllerBase);
}//namespace
