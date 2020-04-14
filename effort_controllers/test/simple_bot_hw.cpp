/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Franco Fusco.
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


#include <chrono>
#include <cmath>
#include <thread>

// ROS
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
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
  : pos_(0.0)
  , vel_(0.0)
  , eff_(0.0)
  , max_eff_(10.0)
  , cmd_(0.0)
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
    pos_ += period * vel_ + 0.5*std::pow(period, 2.0) * eff_;
    vel_ += period * eff_;
  }

  void write() {
    // store current command
    eff_ = cmd_;
    if(std::fabs(eff_) > max_eff_) {
      ROS_WARN("Large effort command received: %f; cutting down to % f", eff_, max_eff_);
      eff_ = eff_ > 0.0 ? max_eff_ : -max_eff_;
    }
  }

private:
  hardware_interface::JointStateInterface state_interface_;
  hardware_interface::EffortJointInterface effort_interface_;
  double cmd_, pos_, vel_, eff_;
  double max_eff_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_bot_hw");
  ros::NodeHandle nh;
  SimpleBot bot;
  ros::Publisher clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
  nh.setParam("/use_sim_time", true);
  controller_manager::ControllerManager cm(&bot, nh);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Rate rate(1.0/bot.period);
  ros::Duration period(bot.period);
  double sim_time = 0.0;

  // main "simulation"
  while(ros::ok()) {
    auto begin = std::chrono::system_clock::now();
    bot.read();
    cm.update(ros::Time(sim_time), period);
    bot.write();
    auto end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::duration<double> >((end - begin)).count();
    if(bot.period > elapsed)
      std::this_thread::sleep_for(std::chrono::duration<double>(bot.period - elapsed));
    rosgraph_msgs::Clock clock;
    clock.clock = ros::Time(sim_time);
    clock_publisher.publish(clock);
    sim_time += bot.period;
  }

  spinner.stop();

  return 0;
}
