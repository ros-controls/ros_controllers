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

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

class SimpleBot : public hardware_interface::RobotHW
{
public:
  static constexpr double period = 0.01;

  SimpleBot()
  : pos_(0)
  , vel_(0)
  , eff_(0)
  {
    // joint state interface
    hardware_interface::JointStateHandle state_handle("joint", &pos_, &vel_, &eff_);
    state_interface_.registerHandle(state_handle);
    registerInterface(&state_interface_);
    // effort interface
    hardware_interface::JointHandle effort_handle(state_interface_.getHandle("joint"), &cmd_);
    effort_interface_.registerHandle(effort_handle);
    registerInterface(&effort_interface_);
  }

  void read() {
    // update current state
    pos_ += period * vel_ + 0.5*std::pow(period, 2.) * eff_;
    vel_ += period * eff_;
  }

  void write() {
    // store current command
    eff_ = cmd_;
  }

private:
  hardware_interface::JointStateInterface state_interface_;
  hardware_interface::EffortJointInterface effort_interface_;
  double cmd_, pos_, vel_, eff_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_bot_hw");
  ros::NodeHandle nh;
  SimpleBot bot;
  controller_manager::ControllerManager cm(&bot, nh);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Rate rate(1./bot.period);
  ros::Duration period(bot.period);

  // main "simulation"
  while(ros::ok()) {
    bot.read();
    cm.update(ros::Time::now(), period);
    bot.write();
    rate.sleep();
  }

  spinner.stop();

  return 0;
}
