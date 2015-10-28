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


#include "force_torque_sensor_controller/force_torque_sensor_controller.h"



namespace force_torque_sensor_controller
{

  bool ForceTorqueSensorController::init(hardware_interface::ForceTorqueSensorInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
  {
    // get all joint states from the hardware interface
    const std::vector<std::string>& sensor_names = hw->getNames();
    for (unsigned i=0; i<sensor_names.size(); i++)
      ROS_DEBUG("Got sensor %s", sensor_names[i].c_str());

    // get publishing period
    if (!controller_nh.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
    }

    for (unsigned i=0; i<sensor_names.size(); i++){
      // sensor handle
      sensors_.push_back(hw->getHandle(sensor_names[i]));

      // realtime publisher
      RtPublisherPtr rt_pub(new realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(root_nh, sensor_names[i], 4));
      realtime_pubs_.push_back(rt_pub);
    }

    // Last published times
    last_publish_times_.resize(sensor_names.size());
    return true;
  }

  void ForceTorqueSensorController::starting(const ros::Time& time)
  {
    // initialize time
    for (unsigned i=0; i<last_publish_times_.size(); i++){
      last_publish_times_[i] = time;
    }
  }

  void ForceTorqueSensorController::update(const ros::Time& time, const ros::Duration& /*period*/)
  {
    // limit rate of publishing
    for (unsigned i=0; i<realtime_pubs_.size(); i++){
      if (publish_rate_ > 0.0 && last_publish_times_[i] + ros::Duration(1.0/publish_rate_) < time){
        // try to publish
        if (realtime_pubs_[i]->trylock()){
          // we're actually publishing, so increment time
          last_publish_times_[i] = last_publish_times_[i] + ros::Duration(1.0/publish_rate_);

          // populate message
          realtime_pubs_[i]->msg_.header.stamp = time;
          realtime_pubs_[i]->msg_.header.frame_id = sensors_[i].getFrameId();

          realtime_pubs_[i]->msg_.wrench.force.x  = sensors_[i].getForce()[0];
          realtime_pubs_[i]->msg_.wrench.force.y  = sensors_[i].getForce()[1];
          realtime_pubs_[i]->msg_.wrench.force.z  = sensors_[i].getForce()[2];
          realtime_pubs_[i]->msg_.wrench.torque.x = sensors_[i].getTorque()[0];
          realtime_pubs_[i]->msg_.wrench.torque.y = sensors_[i].getTorque()[1];
          realtime_pubs_[i]->msg_.wrench.torque.z = sensors_[i].getTorque()[2];

          realtime_pubs_[i]->unlockAndPublish();
        }
      }
    }
  }

  void ForceTorqueSensorController::stopping(const ros::Time& /*time*/)
  {}

}

PLUGINLIB_EXPORT_CLASS(force_torque_sensor_controller::ForceTorqueSensorController, controller_interface::ControllerBase)
