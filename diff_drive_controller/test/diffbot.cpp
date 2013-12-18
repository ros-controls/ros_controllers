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

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

class Diffbot : public hardware_interface::RobotHW
{
public:
  Diffbot()
  {
    // Intialize raw data
    pos_[0] = 0.0; pos_[1] = 0.0;
    vel_[0] = 0.0; vel_[1] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0;
    cmd_[0] = 0.0; cmd_[1] = 0.0;

    // Connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_1("joint_w1", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("joint_w2", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_2);

    registerInterface(&jnt_state_interface_);

    // Connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("joint_w1"), &cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("joint_w2"), &cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_2);

    registerInterface(&jnt_vel_interface_);
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  void read()
  {
    ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << cmd_[1]);
  }

  void write()
  {
    for (unsigned int i = 0; i < 2; ++i)
    {
      pos_[i] += vel_[i]*getPeriod().toSec(); // update position
      vel_[i] = cmd_[i]; // might add smoothing here later
    }
  }

private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diffbot");
  ros::NodeHandle nh;

  Diffbot robot;
  ROS_WARN_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok())
  {
    robot.read();
    cm.update(robot.getTime(), robot.getPeriod());
    robot.write();
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
