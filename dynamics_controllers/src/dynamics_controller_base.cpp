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
#include <xmlrpcpp/XmlRpcException.h>

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



/** The function looks for all parameters available in the provided NodeHandle's
  * namespace and then stores those nested namespaces (assumed to be
  * sub-controllers names) which contain a (string) 'type' parameter.
  */
bool DynamicsControllerBase::getSubControllersMap(
  ros::NodeHandle& nh,
  std::map<std::string,std::string>& controllers_map
)
{
  // this is used to get the list of available controllers
  XmlRpc::XmlRpcValue controllers;
  if(!nh.getParam(nh.getNamespace(), controllers)) {
    ROS_ERROR("Failed to get list of sub-controllers from '%s'",
      nh.getNamespace().c_str()
    );
    return false;
  }

  if(controllers.size() == 0) {
    ROS_ERROR("Sub-controllers namespace '%s' does not contain any parameter",
      nh.getNamespace().c_str()
    );
    return false;
  }

  try {
    std::string type;
    for(const auto& c : controllers) {
      if(!nh.getParam(c.first + "/type", type))
        ROS_WARN_STREAM("Failed to get type of sub-controller " << c.first);
      else
        controllers_map[c.first] = type;
    }
  }
  catch(const XmlRpc::XmlRpcException& ex) {
    ROS_ERROR("Caught XmlRpc exception while loading sub-controllers map: %s",
      ex.getMessage().c_str()
    );
    return false;
  }

  return true;
}





DynamicsControllerBase::DynamicsControllerBase()
  : controller_loader_(new controller_manager::ControllerLoader<controller_interface::ControllerBase>("controller_interface", "controller_interface::ControllerBase"))
{
}


bool DynamicsControllerBase::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle& nh) {
  ROS_DEBUG("Initializing DynamicsControllerBase.");

  // reset sub-controllers
  sub_controllers_.clear();

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
  std::map<std::string,std::string> controllers_map; // name --> type
  if(cnh.getParam("type", ctype)) {
    // single (unnamed) sub-controller
    controllers_map[""] = ctype;
  }
  else {
    // try load multiple controllers from the given namespace
    if(!getSubControllersMap(cnh, controllers_map))
      return false;
    else if(controllers_map.empty()){
      // if no controller has been found, just exit
      ROS_ERROR("No controller has been succesfully located in '%s'",
        cnh.getNamespace().c_str()
      );
    }
  }

  // load all controllers
  try {
    for(const auto& c : controllers_map) {
      ROS_DEBUG("Creating sub-controller instance of type %s", c.second.c_str());
      auto cptr = controller_loader_->createInstance(c.second);
      if(cptr == nullptr) {
        std::stringstream ss;
        ss << "Failed to create sub-controller";
        if(c.first != "")
          ss << " '" << c.first << "'";
        ss << " of type '" << c.second << "'; available controller types are:";
        auto classes = controller_loader_->getDeclaredClasses();
        std::for_each(classes.begin(), classes.end(), [&](const std::string& n) {ss << " " << n << ";";});
        throw std::runtime_error(ss.str());
      }
      sub_controllers_[c.first] = cptr;
    }
  }
  catch(std::exception& ex) {
    ROS_ERROR("Exception while creating sub-controllers. What: %s", ex.what());
    return false;
  }

  // map whose keys are joints (to be claimed) and values are controllers
  // (claiming the joints) - this is used to check resource acquisition
  std::map<std::string,std::string> claimed_resources;

  // initialize all sub-controllers
  try {
    for(auto& c : sub_controllers_) {
      ROS_DEBUG("Initializing sub-controller %s", c.first.c_str());
      ClaimedResources resources;
      bool init_ok = false;
      // init the controller - the namespace definition changes if this is the
      // "unnamed" controller (which directly lives in 'sub_controller')
      if(c.first == "")
        init_ok = c.second->initRequest(fake_hw_.get(), nh, cnh, resources);
      else {
        ros::NodeHandle _cnh(cnh, c.first);
        init_ok = c.second->initRequest(fake_hw_.get(), nh, _cnh, resources);
      }

      // if controller init request fails, give an error and exit
      if(!init_ok) {
        std::stringstream ss;
        ss << "Init request failed for sub-controller";
        if(c.first != "")
          ss << " " << c.first;
        throw std::runtime_error(ss.str());
      }

      // check that resources were claimed from a single interface (this should
      // always be true since the controller has only access to the fake hw)
      if(resources.size() != 1) {
        std::stringstream ss;
        ss << "Sub-controller ";
        if(c.first != "")
          ss << c.first << " ";
        ss << "claimed resources from multiple hardware interfaces, which"
          " might lead to problems. Resources were claimed from the"
          " following interfaces:";
        for(const auto& r : resources)
          ss << " " << r.hardware_interface;
        ROS_WARN_STREAM(ss.str());
      }

      // The hardware interface should be the EffortJointInterface (again, this
      // should be always true as this is the only interface exposed by FakeHW)
      if(resources[0].hardware_interface != "hardware_interface::EffortJointInterface") {
        std::stringstream ss;
        ss << "First hardware interface claimed by the sub-controller ";
        if(c.first != "")
          ss << c.first << " ";
        ss << "is " << resources[0].hardware_interface << ", but "
          "hardware_interface::EffortJointInterface was expected. This might "
          "lead to undefined behavior...";
        ROS_WARN_STREAM(ss.str());
      }

      // check that no resource claimed by this controller is used by any other controller
      for(const auto& joint : resources[0].resources) {
        auto it = claimed_resources.find(joint);
        if(it != claimed_resources.end()) {
          throw std::runtime_error("Joint " + joint + " cannot be handled both"
            " by " + it->second + " and " + c.first);
        }
        claimed_resources[joint] = c.first;
      }
    }
  }
  catch(std::exception& ex) {
    ROS_ERROR("Exception while initializing sub-controllers. What: %s", ex.what());
    return false;
  }

  // check that all joints used in the dynamic model have been claimed by
  // subcontrollers as well
  for(const auto& joint : joint_names_) {
    auto it = claimed_resources.find(joint);
    if(it == claimed_resources.end()) {
      ROS_ERROR("No sub-controller claimed joint %s. Note that this controller"
        " expects the sub-controller(s) to claim the all the joints used by"
        " the dyanmic model", joint.c_str()
      );
      return false;
    }
  }

  return true;
}


void DynamicsControllerBase::starting(const ros::Time& time) {
  // forward the start-request to the low-level controllers
  bool ok = true;
  std::stringstream ss;
  for(auto& c : sub_controllers_) {
    if(!c.second->startRequest(time)) {
      ROS_ERROR("Failed start low-level sub-controller %s", c.first.c_str());
      ok = false;
      ss << " " << c.first;
    }
  }

  // if the request failed for any controller, stop all of them
  if(!ok) {
    for(auto& c : sub_controllers_) {
      c.second->stopRequest(time);
    }
    throw std::runtime_error("Could not start sub-controllers" + ss.str());
  }
}


void DynamicsControllerBase::update(const ros::Time& time, const ros::Duration& period) {
  // update the sub-controllers
  fake_hw_->copyState(joint_handles_);
  for(auto& c : sub_controllers_)
    c.second->update(time, period);
  fake_hw_->getEfforts(sub_command_);

  // perform sub-class specific dynamics
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
  // forward the stop-request to the low-level controllers
  bool ok = true;
  std::stringstream ss;
  for(auto& c : sub_controllers_) {
    if(!c.second->stopRequest(time)) {
      ROS_ERROR("Failed stop sub-controller %s", c.first.c_str());
      ok = false;
      ss << " " << c.first;
    }
  }

  // if the request failed for any controller, stop all of them
  if(!ok) {
    throw std::runtime_error("Could not stop sub-controllers" + ss.str());
  }
}

}
