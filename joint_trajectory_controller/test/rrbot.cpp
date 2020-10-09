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

// STD
#include <array>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

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
    pos_lastcmd_[0] = 0.0; pos_lastcmd_[1] = 0.0;
    vel_cmd_[0] = 0.0; vel_cmd_[1] = 0.0;
    vel_lastcmd_[0] = 0.0; vel_lastcmd_[1] = 0.0;

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

    ros::NodeHandle nh{};

    // Smoothing subscriber
    smoothing_sub_ = nh.subscribe("smoothing", 1, &RRbot::smoothingCB, this);
    smoothings_sub_ = nh.subscribe("smoothings", 1, &RRbot::smoothingsCB, this);
    smoothings_.initRT({0.0, 0.0});

    // Delay subscriber: delay==0 yields direct control, one cycle delay otherwise
    delay_sub_ = nh.subscribe("delay", 1, &RRbot::delayCB, this);
    delay_.initRT(false);

    // Upper bound subscriber: set positions greater than this value are clipped
    upper_bound_sub_ = nh.subscribe("upper_bound", 1, &RRbot::upper_boundCB, this);
    upper_bound_.initRT(std::numeric_limits<double>::infinity());

    // Notification publisher: notify that a parameter has been updated successfully.
    notify_ready_pub_ = nh.advertise<std_msgs::Empty>("parameter_updated", 1);
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  void read() {}

  void write()
  {
    const std::array<double, 2>& smoothings = *(smoothings_.readFromRT());
    const bool delay = *(delay_.readFromRT());
    const double upper_bound = *(upper_bound_.readFromRT());

    for (unsigned int i = 0; i < 2; ++i)
    {
      // if delay is true, use position from previous cycle
      if(delay != 0)
      {
        std::swap(pos_cmd_[i], pos_lastcmd_[i]);
        std::swap(vel_cmd_[i], vel_lastcmd_[i]);
      }
      else
      {
        pos_lastcmd_[i] = pos_cmd_[i];
        vel_lastcmd_[i] = vel_cmd_[i];
      }

      if(active_interface_[i] == "hardware_interface::PositionJointInterface")
      {
        const double next_pos = smoothings[i] * pos_[i] +  (1.0 - smoothings[i]) * pos_cmd_[i];
        vel_[i] = (next_pos - pos_[i]) / getPeriod().toSec();
        pos_[i] = next_pos;
      }
      else if(active_interface_[i] == "hardware_interface::VelocityJointInterface")
      {
        vel_[i] = (1.0 - smoothings[i]) * vel_cmd_[i];
        pos_[i] = pos_[i] + vel_[i] * getPeriod().toSec();
      }

      // clip position at upper bound
      if(pos_[i] > upper_bound)
      {
        pos_[i] = upper_bound;
        vel_[i] = 0.0;
      }
    }
  }

  typedef std::list<hardware_interface::ControllerInfo> ControllerList;

  bool prepareSwitch(const ControllerList& start_list,
                     const ControllerList& stop_list)
  {
    for(const auto& it : start_list)
    {
      for(const auto& iface_it : it.claimed_resources)
      {
        for(const auto& res_it : iface_it.resources)
        {
          if(res_it == "joint1")
          {
            next_active_interface_[0] = iface_it.hardware_interface;
          }
          else if(res_it == "joint2")
          {
            next_active_interface_[1] = iface_it.hardware_interface;
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
  double pos_lastcmd_[2];
  double vel_lastcmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];

  std::string active_interface_[2];
  std::string next_active_interface_[2];

  realtime_tools::RealtimeBuffer<std::array<double, 2>> smoothings_;
  void setSmoothings(const std::array<double, 2>& smoothings) {
    smoothings_.writeFromNonRT(smoothings);
    std_msgs::Empty msg;
    notify_ready_pub_.publish(msg);
  }
  void smoothingCB(const std_msgs::Float64& smoothing)
  {
    setSmoothings({smoothing.data, smoothing.data});
  }
  void smoothingsCB(const std_msgs::Float64MultiArray& smoothings)
  {
    assert(smoothings.data.size() == 2);
    setSmoothings({smoothings.data[0], smoothings.data[1]});
  }
  ros::Subscriber smoothing_sub_;
  ros::Subscriber smoothings_sub_;

  realtime_tools::RealtimeBuffer<bool> delay_;
  void delayCB(const std_msgs::Bool& delay)
  {
    delay_.writeFromNonRT(delay.data);
    std_msgs::Empty msg;
    notify_ready_pub_.publish(msg);
  }
  ros::Subscriber delay_sub_;

  realtime_tools::RealtimeBuffer<double> upper_bound_;
  void upper_boundCB(const std_msgs::Float64& upper_bound)
  {
    upper_bound_.writeFromNonRT(upper_bound.data);
    std_msgs::Empty msg;
    notify_ready_pub_.publish(msg);
  }
  ros::Subscriber upper_bound_sub_;

  ros::Publisher notify_ready_pub_;
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
