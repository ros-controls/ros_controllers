///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
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

/*
 * Author: Wim Meeussen
 */


#include "joint_state_controller/joint_state_controller.h"



namespace joint_state_controller
{

  bool JointStateController::init(hardware_interface::JointStateInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
  {
    // get all joint states from the hardware interface
    const std::vector<std::string>& joint_names = hw->getNames();
    for (unsigned i=0; i<joint_names.size(); i++)
      ROS_DEBUG("Got joint %s", joint_names[i].c_str());

    // get publishing period
    if (!controller_nh.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
    }

    // realtime publisher
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "joint_states", 4));

    // get joints and allocate message
    for (unsigned i=0; i<joint_names.size(); i++){
      joint_state_.push_back(hw->getHandle(joint_names[i]));
      realtime_pub_->msg_.name.push_back(joint_names[i]);
      realtime_pub_->msg_.position.push_back(0.0);
      realtime_pub_->msg_.velocity.push_back(0.0);
      realtime_pub_->msg_.effort.push_back(0.0);
    }

    return true;
  }

  void JointStateController::starting(const ros::Time& time)
  {
    // initialize time
    last_publish_time_ = time;
  }

  void JointStateController::update(const ros::Time& time, const ros::Duration& period)
  {
    // limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){

      // try to publish
      if (realtime_pub_->trylock()){
  // we're actually publishing, so increment time
  last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

  // populate joint state message
  realtime_pub_->msg_.header.stamp = time;
  for (unsigned i=0; i<joint_state_.size(); i++){
    realtime_pub_->msg_.position[i] = joint_state_[i].getPosition();
    realtime_pub_->msg_.velocity[i] = joint_state_[i].getVelocity();
    realtime_pub_->msg_.effort[i] = joint_state_[i].getEffort();
  }
  realtime_pub_->unlockAndPublish();
      }
    }
  }

  void JointStateController::stopping(const ros::Time& time)
  {}

}


PLUGINLIB_EXPORT_CLASS( joint_state_controller::JointStateController, controller_interface::ControllerBase)
