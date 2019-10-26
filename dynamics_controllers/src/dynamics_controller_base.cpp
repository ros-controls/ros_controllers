/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Franco Fusco
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
 *   * Neither the name of the PAL Robotics nor the names of its
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

/*
 * Author: Franco Fusco - franco.fusco@ls2n.fr
 */

#include <dynamics_controllers/dynamics_controller_base.h>

namespace dynamics_controllers {


DynamicsControllerBase::FakeHW::FakeHW(const std::vector<std::string>& joint_names)
: pos_(joint_names.size()),
  vel_(joint_names.size()),
  eff_(joint_names.size()),
  cmd_(joint_names.size())
{
  // create a fake HW to be passed to a "sub-controller"
  for(unsigned int j=0; j<joint_names.size(); j++) {
    ROS_DEBUG("Creating fake handles for joint %s", joint_names[j].c_str());
    // Register joint with state interface
    state_interface_.registerHandle(
      hardware_interface::JointStateHandle(joint_names[j], &pos_[j], &vel_[j], &eff_[j])
    );

    // Register joint with effort interface
    effort_interface_.registerHandle(
      hardware_interface::JointHandle( state_interface_.getHandle(joint_names[j]), &cmd_[j] )
    );
  }

  // Register the interfaces
  registerInterface(&state_interface_);
  registerInterface(&effort_interface_);
}


void DynamicsControllerBase::FakeHW::copyState(const std::vector<hardware_interface::JointHandle>& joint_handles) {
  for(unsigned int j=0; j<joint_handles.size(); j++) {
    ROS_DEBUG("FakeHW: copying state of joint %s;\n - pos: %f\n - vel: %f\n - eff: %f",
      joint_handles[j].getName().c_str(), joint_handles[j].getPosition(),
      joint_handles[j].getVelocity(), joint_handles[j].getEffort()
    );
    pos_[j] = joint_handles[j].getPosition();
    vel_[j] = joint_handles[j].getVelocity();
    eff_[j] = joint_handles[j].getEffort();
  }
}

void DynamicsControllerBase::FakeHW::getEfforts(std::vector<double>& joint_effort_command) {
  for(unsigned int j=0; j<joint_effort_command.size(); j++) {
    joint_effort_command[j] = cmd_[j];
  }
}






DynamicsControllerBase::DynamicsControllerBase()
  : controller_loader_("controller_interface", "controller_interface::ControllerBase")
{
}


bool DynamicsControllerBase::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle& nh) {
  ROS_DEBUG("Initializing DynamicsControllerBase.");

  // sub-class specific initialization
  if(!initDynamics(nh)) {
    ROS_ERROR("Failed to initialize dynamics (initDynamics() returned false)");
    return false;
  }

  // check if the sub-class properly set the joint names
  if(joint_names_.size() == 0) {
    ROS_ERROR("After dynamics initialization, no joint names are available to "
      "build the FakeHW instance"
    );
    return false;
  }

  // retrieve joint handles; will throw an exception on failure (gracefully handled by ControllerManager)
  sub_command_.resize(joint_names_.size());
  joint_handles_.clear();
  joint_handles_.reserve(joint_names_.size());
  for(const auto& j : joint_names_)
    joint_handles_.push_back(hw->getHandle(j));

  ROS_DEBUG("Generating FakeHW");
  fake_hw_.reset( new FakeHW(joint_names_) );

  std::string controller_ns("sub_controller");
  if(!nh.hasParam(controller_ns)) {
    ROS_ERROR("Cannot find sub-namespace '%s' (required to instanciate the "
      "acceleration sub-controller)", controller_ns.c_str());
    return false;
  }

  ros::NodeHandle cnh(nh, controller_ns);
  std::string ctype;
  if(!cnh.getParam("type", ctype)) {
    ROS_ERROR("Could not deduce type of sub-controller from %s/type", controller_ns.c_str());
    return false;
  }

  try {
    ROS_DEBUG("Generating sub-controller plugin instance of type %s", ctype.c_str());
    controller_.reset( controller_loader_.createUnmanagedInstance(ctype) );
    if(controller_ == nullptr)
      throw std::runtime_error("pluginlib::ClassLoader::createUnmanagedInstance returned 'nullptr'");
  }
  catch(std::exception& ex) {
    ROS_ERROR("Exception while loading controller of type %s. What: %s", ctype.c_str(), ex.what());
    return false;
  }

  try {
    // initializes the sub-controller
    ROS_DEBUG("Initializing sub-controller");
    ClaimedResources resources;
    bool init_ok = controller_->initRequest(fake_hw_.get(), nh, cnh, resources);

    if(!init_ok) {
      ROS_ERROR("Failed to initialize sub-controller of type %s", ctype.c_str());
      return false;
    }

    // NOTE: I am not sure if the follwing is the correct way of checking that
    // the sub-controller acquired all joint control resources...
    // I expect the sub-controller to claim resources only from the
    // EffortJointInterface of the FakeHW. Thus, give a warning when multiple
    // interfaces are used. (but it should be impossible, should it?)
    if(resources.size()!=1) {
      std::stringstream ss;
      for(const auto& r : resources)
        ss << " " << r.hardware_interface;
      ROS_WARN("Sub-controller claimed resources from multiple hardware "
        "interfaces, which might lead to problems. Resources were claimed from "
        "the following interfaces: %s", ss.str().c_str()
      );
    }

    // The hardware interface should be the EffortJointInterface
    if(resources[0].hardware_interface != "hardware_interface::EffortJointInterface") {
      ROS_WARN("First hardware interface claimed by the sub-controller is %s, "
        "but hardware_interface::EffortJointInterface was expected. This might "
        "lead to undefined behavior...", resources[0].hardware_interface.c_str()
      );
    }

    // Check that all joints of the controller have been claimed by the
    // subcontroller as well.
    for(const auto& joint : joint_names_) {
      const auto& res = resources[0].resources;
      auto it = std::find(res.begin(), res.end(), joint);
      if(it == res.end()) {
        ROS_ERROR("Sub-controller did not claim resource %s. Note that this "
          "controller expects the sub-controller to claim the same resources",
          joint.c_str()
        );
        return false;
      }
    }

    ROS_DEBUG("Sub-controller initialized");
  }
  catch(std::exception& ex) {
    ROS_ERROR("Exception while initializing sub-controller of type %s. What: %s", ctype.c_str(), ex.what());
    return false;
  }

  return true;
}


void DynamicsControllerBase::starting(const ros::Time& time) {
  // forward the start-request to the low-level controller
  if(!controller_->startRequest(time)) {
    ROS_ERROR("Start request to low-level controller failed");
    throw std::runtime_error("Start request to low-level controller failed");
  }
}


void DynamicsControllerBase::update(const ros::Time& time, const ros::Duration& period) {
  fake_hw_->copyState(joint_handles_);
  controller_->update(time, period);
  fake_hw_->getEfforts(sub_command_);

  try {
    computeEfforts();
  }
  catch(std::exception& e) {
    ROS_ERROR("Caught exception while computing efforts: %s", e.what());
    throw e;
  }
  catch(...) {
    ROS_ERROR("Caught exception while computing efforts");
    throw;
  }
}


void DynamicsControllerBase::stopping(const ros::Time& time) {
  // forward the stop-request to the low-level controller
  if(!controller_->stopRequest(time)) {
    ROS_ERROR("Stop request to low-level controller failed");
    throw std::runtime_error("Stop request to low-level controller failed");
  }
}

}
