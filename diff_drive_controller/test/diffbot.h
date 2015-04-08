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

// NOTE: The contents of this file have been taken largely from the ros_control wiki tutorials

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

template <unsigned int NUM_JOINTS = 2>
class Diffbot : public hardware_interface::RobotHW
{
public:
  Diffbot()
  : running_(true)
  , start_srv_(nh_.advertiseService("start", &Diffbot::start_callback, this))
  , stop_srv_(nh_.advertiseService("stop", &Diffbot::stop_callback, this))
  {
    // Intialize raw data
    std::fill_n(pos_, NUM_JOINTS, 0.0);
    std::fill_n(vel_, NUM_JOINTS, 0.0);
    std::fill_n(eff_, NUM_JOINTS, 0.0);
    std::fill_n(cmd_, NUM_JOINTS, 0.0);

    // Connect and register the joint state and velocity interface
    for (unsigned int i = 0; i < NUM_JOINTS; ++i)
    {
      std::ostringstream os;
      os << "wheel_" << i << "_joint";

      hardware_interface::JointStateHandle state_handle(os.str(), &pos_[i], &vel_[i], &eff_[i]);
      jnt_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle vel_handle(jnt_state_interface_.getHandle(os.str()), &cmd_[i]);
      jnt_vel_interface_.registerHandle(vel_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_vel_interface_);
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  void read()
  {
    std::ostringstream os;
    for (unsigned int i = 0; i < NUM_JOINTS - 1; ++i)
    {
      os << cmd_[i] << ", ";
    }
    os << cmd_[NUM_JOINTS - 1];

    ROS_INFO_STREAM("Commands for joints: " << os.str());
  }

  void write()
  {
    if (running_)
    {
      for (unsigned int i = 0; i < NUM_JOINTS; ++i)
      {
        // Note that pos_[i] will be NaN for one more cycle after we start(),
        // but that is consistent with the knowledge we have about the state
        // of the robot.
        pos_[i] += vel_[i]*getPeriod().toSec(); // update position
        vel_[i] = cmd_[i]; // might add smoothing here later
      }
    }
    else
    {
      std::fill_n(pos_, NUM_JOINTS, std::numeric_limits<double>::quiet_NaN());
      std::fill_n(vel_, NUM_JOINTS, std::numeric_limits<double>::quiet_NaN());
    }
  }

  bool start_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    running_ = false;
    return true;
  }

private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  double cmd_[NUM_JOINTS];
  double pos_[NUM_JOINTS];
  double vel_[NUM_JOINTS];
  double eff_[NUM_JOINTS];
  bool running_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
};
