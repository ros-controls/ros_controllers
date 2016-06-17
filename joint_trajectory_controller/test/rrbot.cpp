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

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

class RRbot : public hardware_interface::RobotHW
{
public:
  RRbot()
  {
    // Intialize raw data
    pos_[0] = 0.0; pos_[1] = 0.0;
    vel_[0] = 0.0; vel_[1] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0;
    pos_cmd_[0] = 0.0; pos_cmd_[1] = 0.0;
    vel_cmd_[0] = 0.0; vel_cmd_[1] = 0.0;

    // Connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_1("joint1", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("joint2", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_2);

    registerInterface(&jnt_state_interface_);

    // Connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_1(jnt_state_interface_.getHandle("joint1"), &pos_cmd_[0]);
    jnt_pos_interface_.registerHandle(pos_handle_1);

    hardware_interface::JointHandle pos_handle_2(jnt_state_interface_.getHandle("joint2"), &pos_cmd_[1]);
    jnt_pos_interface_.registerHandle(pos_handle_2);

    registerInterface(&jnt_pos_interface_);

    // Connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("joint1"), &vel_cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("joint2"), &vel_cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_2);

    registerInterface(&jnt_vel_interface_);

    // Smoothing subscriber
    smoothing_sub_ = ros::NodeHandle().subscribe("smoothing", 1, &RRbot::smoothingCB, this);
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
      if(active_interface_[i] == "hardware_interface::PositionJointInterface")
      {
        vel_[i] = (pos_cmd_[i] - pos_[i]) / getPeriod().toSec();

        const double next_pos = smoothing * pos_[i] +  (1.0 - smoothing) * pos_cmd_[i];
        pos_[i] = next_pos;        
      }
      else if(active_interface_[i] == "hardware_interface::VelocityJointInterface")
      {
        vel_[i] = (1.0 - smoothing) * vel_cmd_[i];
        pos_[i] = pos_[i] + vel_[i] * getPeriod().toSec();
      }
    }
  }

  typedef std::list<hardware_interface::ControllerInfo> ControllerList;

  bool prepareSwitch(const ControllerList& start_list,
                     const ControllerList& stop_list)
  {
    for(ControllerList::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
    {
      for(std::vector<hardware_interface::InterfaceResources>::const_iterator iface_it = it->claimed_resources.begin(); iface_it != it->claimed_resources.end(); ++iface_it)
      {
        for(std::set<std::string>::const_iterator res_it = iface_it->resources.begin(); res_it != iface_it->resources.end(); ++res_it)
        {
          if(*res_it == "joint1")
          {
            next_active_interface_[0] = iface_it->hardware_interface;
          }
          else if(*res_it == "joint2")
          {
            next_active_interface_[1] = iface_it->hardware_interface;
          }
        }
      }
    }

    return true;
  }

  void doSwitch(const ControllerList& start_list,
                const ControllerList& stop_list)
  {
    active_interface_[0] = next_active_interface_[0];
    active_interface_[1] = next_active_interface_[1];
  }

private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  double pos_cmd_[2];
  double vel_cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];

  std::string active_interface_[2];
  std::string next_active_interface_[2];

  realtime_tools::RealtimeBuffer<double> smoothing_;
  void smoothingCB(const std_msgs::Float64& smoothing) {smoothing_.writeFromNonRT(smoothing.data);}

  ros::Subscriber smoothing_sub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrbot");
  ros::NodeHandle nh;

  RRbot robot;
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
