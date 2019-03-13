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

#include "diffbot.h"
#include <chrono>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diffbot");
  ros::NodeHandle nh;

  // This should be set in launch files as well
  nh.setParam("/use_sim_time", true);

  Diffbot<> robot;
  ROS_WARN_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Publisher clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();
  std::chrono::system_clock::time_point end   = std::chrono::system_clock::now();

  ros::Time internal_time(0);
  const ros::Duration dt = robot.getPeriod();
  double elapsed_secs = 0;

  while(ros::ok())
  {
    begin = std::chrono::system_clock::now();

    robot.read();
    cm.update(internal_time, dt);

    robot.write();

    end = std::chrono::system_clock::now();

    elapsed_secs = std::chrono::duration_cast<std::chrono::duration<double> >((end - begin)).count();

    if (dt.toSec() - elapsed_secs < 0.0)
    {
      ROS_WARN_STREAM_THROTTLE(
            0.1, "Control cycle is taking to much time, elapsed: " << elapsed_secs);
    }
    else
    {
      ROS_DEBUG_STREAM_THROTTLE(1.0, "Control cycle is, elapsed: " << elapsed_secs);
      usleep((dt.toSec() - elapsed_secs) * 1e6);
    }

    rosgraph_msgs::Clock clock;
    clock.clock = ros::Time(internal_time);
    clock_publisher.publish(clock);
    internal_time += dt;
  }
  spinner.stop();

  return 0;
}
