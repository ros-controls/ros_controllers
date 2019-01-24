///////////////////////////////////////////////////////////////////////////////
//      Title     : compliance_pubisher_demo.cpp
//      Project   : wrench_to_twist_pub
//      Created   : 12/4/2018
//      Author    : Andy Zelenak
//
// BSD 3-Clause License
//
// Copyright (c) 2018, Los Alamos National Security, LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

// Send a small velocity command, for testing.
// Can be used in place of a real force/torque sensor.

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compliance_pubisher_demo");

  ros::NodeHandle n;

  ros::Publisher compliance_command_pub =
      n.advertise<std_msgs::Float64MultiArray>("/compliance_controller/compliance_velocity_adjustment", 1);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    std_msgs::Float64MultiArray joint_velocities;
    joint_velocities.data.resize(6);
    for (std::size_t i = 0; i < joint_velocities.data.size(); ++i)
      joint_velocities.data[i] = 0.001;

    compliance_command_pub.publish(joint_velocities);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
