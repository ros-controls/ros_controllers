///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
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

/// \author: Adolfo Rodriguez Tsouroukdissian

#pragma once


#include <controller_interface/controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/Imu.h>
#include <realtime_tools/realtime_publisher.h>

namespace imu_sensor_controller
{

// this controller gets access to the ImuSensorInterface
class ImuSensorController: public controller_interface::Controller<hardware_interface::ImuSensorInterface>
{
public:
  ImuSensorController(){}

  virtual bool init(hardware_interface::ImuSensorInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:
  std::vector<hardware_interface::ImuSensorHandle> sensors_;
  typedef std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu> > RtPublisherPtr;
  std::vector<RtPublisherPtr> realtime_pubs_;
  std::vector<ros::Time> last_publish_times_;
  double publish_rate_;
};

}
