///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Masaru Morita

// NOTE: The contents of this file have been taken largely from the ros_control wiki tutorials

#pragma once


// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

// NaN
#include <limits>

// ostringstream
#include <sstream>

enum INDEX_WHEEL {
    INDEX_RIGHT = 0,
    INDEX_LEFT = 1
};

class AckermannSteeringBot : public hardware_interface::RobotHW
{
public:
  AckermannSteeringBot()
  : running_(true)
  , start_srv_(nh_.advertiseService("start", &AckermannSteeringBot::startCallback, this))
  , stop_srv_(nh_.advertiseService("stop", &AckermannSteeringBot::stopCallback, this))
  , ns_("ackermann_steering_bot_hw_sim/")
  {
    // Intialize raw data
    this->cleanUp();
    this->getJointNames(nh_);
    this->registerHardwareInterfaces();

    nh_.getParam(ns_ + "wheel_separation_w", wheel_separation_w_);
    nh_.getParam(ns_ + "wheel_separation_h", wheel_separation_h_);
    ROS_INFO_STREAM("wheel_separation_w in test ackermann_steering_bot= " << wheel_separation_w_);
    ROS_INFO_STREAM("wheel_separation_h in test ackermann_steering_bot= " << wheel_separation_h_);
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  void read()
  {
    // Read the joint state of the robot into the hardware interface
    std::ostringstream os;
    if (running_)
    {
      // wheels
      rear_wheel_jnt_pos_ += rear_wheel_jnt_vel_*getPeriod().toSec();
      rear_wheel_jnt_vel_ = rear_wheel_jnt_vel_cmd_;
      for (unsigned int i = 0; i < virtual_rear_wheel_jnt_vel_cmd_.size(); i++)
      {
        // Note that pos_[i] will be NaN for one more cycle after we start(),
        // but that is consistent with the knowledge we have about the state
        // of the robot.
        virtual_rear_wheel_jnt_pos_[i] += virtual_rear_wheel_jnt_vel_[i]*getPeriod().toSec();
        virtual_rear_wheel_jnt_vel_[i] = virtual_rear_wheel_jnt_vel_cmd_[i];
      }

      // steers
      front_steer_jnt_pos_ = front_steer_jnt_pos_cmd_;
      for (unsigned int i = 0; i < virtual_front_steer_jnt_pos_cmd_.size(); i++)
      {
        virtual_front_steer_jnt_pos_[i] = virtual_front_steer_jnt_pos_cmd_[i];
      }

      // directly get from controller
      os << rear_wheel_jnt_vel_cmd_ << ", ";
      os << front_steer_jnt_pos_cmd_ << ", ";

      // convert to each joint velocity
      //-- differential drive
      for (unsigned int i = 0; i < virtual_rear_wheel_jnt_pos_.size(); i++)
      {
          os << virtual_rear_wheel_jnt_pos_[i] << ", ";
      }

      //-- ackerman link
      for (unsigned int i = 0; i < virtual_front_steer_jnt_pos_.size(); i++)
      {
          os << virtual_front_steer_jnt_pos_[i] << ", ";
      }
    }
    else
    {
      // wheels
      rear_wheel_jnt_pos_= std::numeric_limits<double>::quiet_NaN();
      rear_wheel_jnt_vel_= std::numeric_limits<double>::quiet_NaN();
      std::fill_n(virtual_rear_wheel_jnt_pos_.begin(), virtual_rear_wheel_jnt_pos_.size(), std::numeric_limits<double>::quiet_NaN());
      std::fill_n(virtual_rear_wheel_jnt_vel_.begin(), virtual_rear_wheel_jnt_vel_.size(), std::numeric_limits<double>::quiet_NaN());
      // steers
      front_steer_jnt_pos_= std::numeric_limits<double>::quiet_NaN();
      front_steer_jnt_vel_= std::numeric_limits<double>::quiet_NaN();
      std::fill_n(virtual_front_steer_jnt_pos_.begin(), virtual_front_steer_jnt_pos_.size(), std::numeric_limits<double>::quiet_NaN());
      std::fill_n(virtual_front_steer_jnt_vel_.begin(), virtual_front_steer_jnt_vel_.size(), std::numeric_limits<double>::quiet_NaN());

      // wheels
      os << rear_wheel_jnt_pos_ << ", ";
      os << rear_wheel_jnt_vel_ << ", ";
      for (unsigned int i = 0; i < virtual_rear_wheel_jnt_pos_.size(); i++)
      {
          os << virtual_rear_wheel_jnt_pos_[i] << ", ";
      }

      // steers
      os << front_steer_jnt_pos_ << ", ";
      os << front_steer_jnt_vel_ << ", ";
      for (unsigned int i = 0; i < virtual_front_steer_jnt_pos_.size(); i++)
      {
          os << virtual_front_steer_jnt_pos_[i] << ", ";
      }
    }
    ROS_INFO_STREAM("running_ = " << running_ << ". commands are " << os.str());
  }

  void write()
  {
    // Write the commands to the joints
    std::ostringstream os;
    // directly get from controller
    os << rear_wheel_jnt_vel_cmd_ << ", ";
    os << front_steer_jnt_pos_cmd_ << ", ";

    // convert to each joint velocity
    //-- differential drive
    for (unsigned int i = 0; i < virtual_rear_wheel_jnt_vel_cmd_.size(); i++)
    {
      virtual_rear_wheel_jnt_vel_cmd_[i] = rear_wheel_jnt_vel_cmd_;
      os << virtual_rear_wheel_jnt_vel_cmd_[i] << ", ";
    }

    //-- ackerman link
    const double h = wheel_separation_h_;
    const double w = wheel_separation_w_;
    virtual_front_steer_jnt_pos_cmd_[INDEX_RIGHT] = atan2(2*h*tan(front_steer_jnt_pos_cmd_), 2*h + w/2.0*tan(front_steer_jnt_pos_cmd_));
    virtual_front_steer_jnt_pos_cmd_[INDEX_LEFT] = atan2(2*h*tan(front_steer_jnt_pos_cmd_), 2*h - w/2.0*tan(front_steer_jnt_pos_cmd_));

    for (unsigned int i = 0; i < virtual_front_steer_jnt_pos_cmd_.size(); i++)
    {
      os << virtual_front_steer_jnt_pos_cmd_[i] << ", ";
    }

    if (rear_wheel_jnt_vel_cmd_ != 0.0 || front_steer_jnt_pos_cmd_ != 0.0)
    {
      ROS_INFO_STREAM("Commands for joints: " << os.str());
    }
  }

  bool startCallback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    ROS_INFO_STREAM("running_ = " << running_ << ".");
    running_ = true;
    return true;
  }

  bool stopCallback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    ROS_INFO_STREAM("running_ = " << running_ << ".");
    running_ = false;
    return true;
  }

private:

  void cleanUp()
  {
    // wheel
    //-- wheel joint names
    rear_wheel_jnt_name_.empty();
    virtual_rear_wheel_jnt_names_.clear();
    //-- actual rear wheel joint
    rear_wheel_jnt_pos_ = 0;
    rear_wheel_jnt_vel_ = 0;
    rear_wheel_jnt_eff_ = 0;
    rear_wheel_jnt_vel_cmd_ = 0;
    //-- virtual rear wheel joint
    virtual_rear_wheel_jnt_pos_.clear();
    virtual_rear_wheel_jnt_vel_.clear();
    virtual_rear_wheel_jnt_eff_.clear();
    virtual_rear_wheel_jnt_vel_cmd_.clear();
    //-- virtual front wheel joint
    virtual_front_wheel_jnt_pos_.clear();
    virtual_front_wheel_jnt_vel_.clear();
    virtual_front_wheel_jnt_eff_.clear();
    virtual_front_wheel_jnt_vel_cmd_.clear();

    // steer
    //-- steer joint names
    front_steer_jnt_name_.empty();
    virtual_front_steer_jnt_names_.clear();
    //-- front steer joint
    front_steer_jnt_pos_ = 0;
    front_steer_jnt_vel_ = 0;
    front_steer_jnt_eff_ = 0;
    front_steer_jnt_pos_cmd_ = 0;
    //-- virtual wheel joint
    virtual_front_steer_jnt_pos_.clear();
    virtual_front_steer_jnt_vel_.clear();
    virtual_front_steer_jnt_eff_.clear();
    virtual_front_steer_jnt_pos_cmd_.clear();
  }

  void getJointNames(ros::NodeHandle &_nh)
  {
    this->getWheelJointNames(_nh);
    this->getSteerJointNames(_nh);
  }

  void getWheelJointNames(ros::NodeHandle &_nh)
  {
    // wheel joint to get linear command
    _nh.getParam(ns_ + "rear_wheel", rear_wheel_jnt_name_);

    // virtual wheel joint for gazebo con_rol
    _nh.getParam(ns_ + "rear_wheels", virtual_rear_wheel_jnt_names_);
    int dof = virtual_rear_wheel_jnt_names_.size();
    virtual_rear_wheel_jnt_pos_.resize(dof);
    virtual_rear_wheel_jnt_vel_.resize(dof);
    virtual_rear_wheel_jnt_eff_.resize(dof);
    virtual_rear_wheel_jnt_vel_cmd_.resize(dof);

    _nh.getParam(ns_ + "front_wheels", virtual_front_wheel_jnt_names_);
    dof = virtual_front_wheel_jnt_names_.size();
    virtual_front_wheel_jnt_pos_.resize(dof);
    virtual_front_wheel_jnt_vel_.resize(dof);
    virtual_front_wheel_jnt_eff_.resize(dof);
    virtual_front_wheel_jnt_vel_cmd_.resize(dof);
  }

  void getSteerJointNames(ros::NodeHandle &_nh)
  {
    // steer joint to get angular command
    _nh.getParam(ns_ + "front_steer", front_steer_jnt_name_);

    // virtual steer joint for gazebo control
    _nh.getParam(ns_ + "front_steers", virtual_front_steer_jnt_names_);

    const int dof = virtual_front_steer_jnt_names_.size();
    virtual_front_steer_jnt_pos_.resize(dof);
    virtual_front_steer_jnt_vel_.resize(dof);
    virtual_front_steer_jnt_eff_.resize(dof);
    virtual_front_steer_jnt_pos_cmd_.resize(dof);
  }

  void registerHardwareInterfaces()
  {
    this->registerSteerInterface();
    this->registerWheelInterface();

    // register mapped interface to ros_control
    registerInterface(&jnt_state_interface_);
    registerInterface(&rear_wheel_jnt_vel_cmd_interface_);
    registerInterface(&front_steer_jnt_pos_cmd_interface_);
  }

  void registerWheelInterface()
  {
    // actual wheel joints
    this->registerInterfaceHandles(
          jnt_state_interface_, rear_wheel_jnt_vel_cmd_interface_,
          rear_wheel_jnt_name_, rear_wheel_jnt_pos_, rear_wheel_jnt_vel_, rear_wheel_jnt_eff_, rear_wheel_jnt_vel_cmd_);

    // virtual rear wheel joints
    for (int i = 0; i < virtual_rear_wheel_jnt_names_.size(); i++)
    {
      this->registerInterfaceHandles(
            jnt_state_interface_, rear_wheel_jnt_vel_cmd_interface_,
            virtual_rear_wheel_jnt_names_[i], virtual_rear_wheel_jnt_pos_[i], virtual_rear_wheel_jnt_vel_[i], virtual_rear_wheel_jnt_eff_[i], virtual_rear_wheel_jnt_vel_cmd_[i]);
    }
    // virtual front wheel joints
    for (int i = 0; i < virtual_front_wheel_jnt_names_.size(); i++)
    {
      this->registerInterfaceHandles(
            jnt_state_interface_, rear_wheel_jnt_vel_cmd_interface_,
            virtual_front_wheel_jnt_names_[i], virtual_front_wheel_jnt_pos_[i], virtual_front_wheel_jnt_vel_[i], virtual_front_wheel_jnt_eff_[i], virtual_front_wheel_jnt_vel_cmd_[i]);
    }
  }

  void registerSteerInterface()
  {
    // actual steer joints
    this->registerInterfaceHandles(
          jnt_state_interface_, front_steer_jnt_pos_cmd_interface_,
          front_steer_jnt_name_, front_steer_jnt_pos_, front_steer_jnt_vel_, front_steer_jnt_eff_, front_steer_jnt_pos_cmd_);

    // virtual steer joints
    for (int i = 0; i < virtual_front_steer_jnt_names_.size(); i++)
    {
      this->registerInterfaceHandles(
            jnt_state_interface_, front_steer_jnt_pos_cmd_interface_,
            virtual_front_steer_jnt_names_[i], virtual_front_steer_jnt_pos_[i], virtual_front_steer_jnt_vel_[i], virtual_front_steer_jnt_eff_[i], virtual_front_steer_jnt_pos_cmd_[i]);
    }
  }

  void registerInterfaceHandles(
          hardware_interface::JointStateInterface& _jnt_state_interface,
          hardware_interface::JointCommandInterface& _jnt_cmd_interface,
          const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff,  double& _jnt_cmd)
  {
    // register joint (both JointState and CommandJoint)
    this->registerJointStateInterfaceHandle(_jnt_state_interface, _jnt_name,
                                            _jnt_pos, _jnt_vel, _jnt_eff);
    this->registerCommandJointInterfaceHandle(_jnt_state_interface, _jnt_cmd_interface,
                                              _jnt_name, _jnt_cmd);
  }

  void registerJointStateInterfaceHandle(
      hardware_interface::JointStateInterface& _jnt_state_interface,
      const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff)
  {
    hardware_interface::JointStateHandle state_handle(_jnt_name,
                                                      &_jnt_pos,
                                                      &_jnt_vel,
                                                      &_jnt_eff);
    _jnt_state_interface.registerHandle(state_handle);

    ROS_INFO_STREAM("Registered joint '" << _jnt_name << " ' in the JointStateInterface");
  }

  void registerCommandJointInterfaceHandle(
      hardware_interface::JointStateInterface& _jnt_state_interface,
      hardware_interface::JointCommandInterface& _jnt_cmd_interface,
      const std::string _jnt_name, double& _jnt_cmd)
  {
    // joint command
    hardware_interface::JointHandle _handle(_jnt_state_interface.getHandle(_jnt_name),
                                            &_jnt_cmd);
    _jnt_cmd_interface.registerHandle(_handle);

    ROS_INFO_STREAM("Registered joint '" << _jnt_name << " ' in the CommandJointInterface");
  }

private:
  // common
  hardware_interface::JointStateInterface jnt_state_interface_;// rear wheel
  //-- actual joint(single actuator)
  //---- joint name
  std::string rear_wheel_jnt_name_;
  //---- joint interface parameters
  double rear_wheel_jnt_pos_;
  double rear_wheel_jnt_vel_;
  double rear_wheel_jnt_eff_;
  //---- joint interface command
  double rear_wheel_jnt_vel_cmd_;
  //---- Hardware interface: joint
  hardware_interface::VelocityJointInterface rear_wheel_jnt_vel_cmd_interface_;
  //hardware_interface::JointStateInterface wheel_jnt_state_interface_;
  //
  //-- virtual joints(two rear wheels)
  //---- joint name
  std::vector<std::string> virtual_rear_wheel_jnt_names_;
  //---- joint interface parameters
  std::vector<double> virtual_rear_wheel_jnt_pos_;
  std::vector<double> virtual_rear_wheel_jnt_vel_;
  std::vector<double> virtual_rear_wheel_jnt_eff_;
  //---- joint interface command
  std::vector<double> virtual_rear_wheel_jnt_vel_cmd_;
  //-- virtual joints(two front wheels)
  //---- joint name
  std::vector<std::string> virtual_front_wheel_jnt_names_;
  //---- joint interface parameters
  std::vector<double> virtual_front_wheel_jnt_pos_;
  std::vector<double> virtual_front_wheel_jnt_vel_;
  std::vector<double> virtual_front_wheel_jnt_eff_;
  //---- joint interface command
  std::vector<double> virtual_front_wheel_jnt_vel_cmd_;

  // front steer
  //-- actual joint(single actuator)
  //---- joint name
  std::string front_steer_jnt_name_;
  //---- joint interface parameters
  double front_steer_jnt_pos_;
  double front_steer_jnt_vel_;
  double front_steer_jnt_eff_;
  //---- joint interface command
  double front_steer_jnt_pos_cmd_;
  //---- Hardware interface: joint
  hardware_interface::PositionJointInterface front_steer_jnt_pos_cmd_interface_;
  //hardware_interface::JointStateInterface steer_jnt_state_interface_;
  //
  //-- virtual joints(two steers)
  //---- joint name
  std::vector<std::string> virtual_front_steer_jnt_names_;
  //---- joint interface parameters
  std::vector<double> virtual_front_steer_jnt_pos_;
  std::vector<double> virtual_front_steer_jnt_vel_;
  std::vector<double> virtual_front_steer_jnt_eff_;
  //---- joint interface command
  std::vector<double> virtual_front_steer_jnt_pos_cmd_;

  // Wheel separation, wrt the midpoint of the wheel width:
  double wheel_separation_w_;
  // Wheel separation, wrt the midpoint of the wheel width:
  double wheel_separation_h_;

  std::string ns_;
  bool running_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
};
