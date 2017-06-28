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
//   * Neither the name of PAL Robotics S.L. nor the names of its
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

// angles
#include <angles/angles.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

class RRbotWrapping : public hardware_interface::RobotHW
{
public:
  RRbotWrapping()
  {
    // Intialize raw data
    pos_[0] = M_PI - 0.1; pos_[1] = -M_PI + 0.1;
    vel_[0] = 0.0; vel_[1] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0;
    cmd_[0] = pos_[0]; cmd_[1] = pos_[1];  // Command must match position

    // Connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_1("joint1", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("joint2", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_2);

    registerInterface(&jnt_state_interface_);

    // Connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_1(jnt_state_interface_.getHandle("joint1"), &cmd_[0]);
    jnt_vel_interface_.registerHandle(pos_handle_1);

    hardware_interface::JointHandle pos_handle_2(jnt_state_interface_.getHandle("joint2"), &cmd_[1]);
    jnt_vel_interface_.registerHandle(pos_handle_2);

    registerInterface(&jnt_vel_interface_);

    // Smoothing subscriber
    smoothing_sub_ = ros::NodeHandle().subscribe("smoothing", 1, &RRbotWrapping::smoothingCB, this);
    smoothing_.initRT(0.0);
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  void read() {}

  void write()
  {
    const double smoothing = *(smoothing_.readFromRT());
    for (unsigned int i = 0; i < 2; ++i)
    {
      vel_[i] = smoothing * cmd_[i];

      // Compute new position using instantaneous velocity
      const double next_pos = pos_[i] + (vel_[i] * getPeriod().toSec());

      // Wrap position to [-pi, pi]
      pos_[i] = angles::normalize_angle(next_pos);
    }
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];

  realtime_tools::RealtimeBuffer<double> smoothing_;
  void smoothingCB(const std_msgs::Float64& smoothing) {smoothing_.writeFromNonRT(smoothing.data);}

  ros::Subscriber smoothing_sub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrbot_wrapping");
  ros::NodeHandle nh;

  RRbotWrapping robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while (ros::ok())
  {
    robot.read();
    cm.update(robot.getTime(), robot.getPeriod());
    robot.write();
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
