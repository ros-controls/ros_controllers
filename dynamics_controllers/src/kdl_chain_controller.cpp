/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
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

#include <dynamics_controllers/kdl_chain_controller.h>
#include <pluginlib/class_list_macros.h>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>


namespace dynamics_controllers {

KdlChainController::KdlChainController()
: DynamicsControllerBase()
{
}


bool KdlChainController::initDynamics(ros::NodeHandle& nh) {
  // Look for the parameter robot_description
  std::string robot_description_param;
  if(!nh.searchParam("robot_description", robot_description_param)) {
    ROS_ERROR("Could not locate 'robot_description' in the parameter server ("
      "search started from namespace '%s')", nh.getNamespace().c_str()
    );
    return false;
  }
  ROS_DEBUG("Parameter 'robot_description' found in %s", robot_description_param.c_str());

  // Get the URDF
  std::string robot_description_xml;
  if(!nh.getParam(robot_description_param, robot_description_xml)) {
    ROS_ERROR("Failed to retrieve 'robot_description' from the parameter '%s'",
      robot_description_param.c_str()
    );
    return false;
  }
  ROS_DEBUG("URDF retrieved succesfully");

  KDL::Tree tree;
  if(!kdl_parser::treeFromString(robot_description_xml, tree))
  {
    ROS_ERROR("Failed to build KDL::Tree object from URDF");
    return false;
  }
  ROS_DEBUG("KDL parser: succesfully converted URDF into Tree");

  // Look for the base and tip of the chain. By using "searchParam", it is
  // possible to write these two parameters just once in the "common"
  // controllers namespace.
  std::string base, tip, base_param, tip_param;

  if(!nh.searchParam("chain_base", base_param)) {
    base = tree.getRootSegment()->first;
    ROS_INFO("Could not locate 'chain_base' in the parameter server (search "
      "started from namespace '%s'). Assuming you want to start from the root "
      "'%s' of the kinematic tree.", nh.getNamespace().c_str(), base.c_str()
    );
  }
  else if(!nh.getParam(base_param, base)) {
    ROS_ERROR("Failed to retrieve 'chain_base' from parameter '%s'",
      base_param.c_str()
    );
    return false;
  }
  ROS_DEBUG("'chain_base' is: '%s'", base.c_str());

  bool look_for_tip = false;
  if(!nh.searchParam("chain_tip", tip_param)) {
    ROS_INFO("Could not locate 'chain_tip' in the parameter server (search "
      "started from namespace '%s'). Assuming that there is a single branch "
      "rooted at '%s' and that you want to reach the unique leaf.",
      nh.getNamespace().c_str(), base.c_str()
    );
    look_for_tip = true;
  }
  else if(!nh.getParam(tip_param, tip)) {
    ROS_ERROR("Failed to retrieve 'chain_tip' from parameter '%s'",
      tip_param.c_str()
    );
    return false;
  }
  ROS_DEBUG("'chain_tip' is: '%s'", tip.c_str());

  if(look_for_tip) {
    // check if there is a single branch roted at the given base and get the
    // corresponding tip
    ROS_INFO("Looking for the leaf segment of the chain rooted at '%s'", base.c_str());

    // check if the base is a valid starting point
    auto segment = tree.getSegment(base);
    if(segment == tree.getSegments().end()) {
      ROS_ERROR("Base segment '%s' not found in the tree.", base.c_str());
      return false;
    }

    // traverse the branch looking for unique children until the leaf is reached
    while(GetTreeElementChildren(segment->second).size() > 0) {
      const auto& children = GetTreeElementChildren(segment->second);
      if(children.size() > 1) {
        // multiple children found: this is not a chain!
        const auto& segname = segment->first;
        ROS_ERROR("The branch rooted at '%s' is not a chain: found child "
          "segment '%s' with %lu children", base.c_str(), segname.c_str(),
          children.size()
        );
        return false;
      }
      // there is a unique child: keep searching
      segment = children[0];
    }

    // we got to the end of the chain
    tip = segment->first;
    ROS_INFO("Reached leaf segment '%s'", tip.c_str());
  }

  std::string grav_param;
  std::vector<double> grav(3, 0.);

  if(!nh.searchParam("gravity", grav_param)) {
    ROS_WARN("Could not locate 'gravity' in the parameter server (search "
      "started from namespace '%s')\n** GRAVITY WILL BE SET TO ZERO **",
      nh.getNamespace().c_str()
    );
  }
  else if(!nh.getParam(grav_param, grav))
  {
    ROS_WARN("Failed to retrieve 'gravity' from parameter '%s' (maybe it is "
      "not a list?)\n** GRAVITY WILL BE SET TO ZERO **", grav_param.c_str()
    );
  }

  if(grav.size() != 3)
  {
    ROS_ERROR("Parameter 'gravity' found, but it has %lu values (3 expected)", grav.size());
    return false;
  }

  KDL::Vector gravity;
  gravity(0) = grav[0];
  gravity(1) = grav[1];
  gravity(2) = grav[2];
  if(gravity.Norm() < 1e-6) {
    ROS_INFO("Gravity compensation is OFF (due to zero gravity vector)");
  }
  else if(std::fabs(gravity.Norm()-9.806) > 0.1) {
    ROS_WARN("Gravity vector on Earth is usually around 9.806 m/s^2 in norm. "
      "Declared gravity has norm %f; if this value is correct, just ignore "
      "this warning. Otherwise, check your setup to prevent damages.",
      gravity.Norm()
    );
  }
  ROS_DEBUG_STREAM("Gravity is set to " << gravity << " with respect to " << base);

  // Should we do only gravity compensation?
  nh.param("gravity_compensation_only", gravity_compensation_only_, false);
  if(gravity_compensation_only_) {
    if(gravity.Norm() < 1e-6) {
      ROS_WARN("Requested to perform gravity compensation only, but the "
        "gravity vector has zero-norm. Are you sure you properly configured "
        "the controller?"
      );
    }
    else {
      ROS_INFO("Performing gravity compensation only.");
    }
  }

  if(!tree.getChain(base, tip, chain_)) {
    ROS_ERROR("Failed to extract chain with '%s' as base and '%s' as tip",
      base.c_str(), tip.c_str()
    );
    return false;
  }
  ROS_DEBUG("Chain extracted succesfully");

  if(chain_.getNrOfJoints() == 0) {
    ROS_ERROR("Chain from '%s' to '%s' does not contain any moving joint, "
      "the dynamic solver cannot be initialized", base.c_str(), tip.c_str()
    );
    return false;
  }

  // Initialize joint names
  joint_names_.clear();
  joint_names_.reserve(chain_.getNrOfJoints());
  for(const KDL::Segment& segment : chain_.segments)
    if(segment.getJoint().getType() != KDL::Joint::None)
      joint_names_.push_back(segment.getJoint().getName());
  std::stringstream ss;
  std::ostream_iterator<std::string> out_it(ss, " ");
  std::copy ( joint_names_.begin(), joint_names_.end(), out_it );
  ROS_DEBUG("Found %d joints in chain: %s", (int)joint_names_.size(), ss.str().c_str());

  // Resize internal data structures
  rne_.reset(new KDL::ChainIdSolver_RNE(chain_, gravity));
  wrenches_ = KDL::Wrenches(chain_.getNrOfSegments());
  cfg_ = KDL::JntArrayAcc(chain_.getNrOfJoints());
  efforts_ = KDL::JntArray(chain_.getNrOfJoints());

  return true;
}


void KdlChainController::computeEfforts() {
  // copy the current configuration of the chain
  for(unsigned int i=0; i<joint_names_.size(); i++) {
    cfg_.q(i) = joint_handles_[i].getPosition(); // copy current velocity
    if(!gravity_compensation_only_) {
      cfg_.qdot(i) = joint_handles_[i].getVelocity(); // copy current velocity
      cfg_.qdotdot(i) = sub_command_[i]; // interpret the sub-command as an acceleration
    }
  }

  // evaluate the efforts using the Recursive Newton-Euler algorithm
  if(rne_->CartToJnt(cfg_.q, cfg_.qdot, cfg_.qdotdot, wrenches_, efforts_) < 0) {
    // Failed for some reason: throw an exception
    std::stringstream ss;
    ss << "Failed to compute efforts using the Dynamic Model. Solver error: ";
    ss << rne_->strError(rne_->getError());
    throw std::runtime_error(ss.str());
  }

  // send computed commands to the hardware
  for(unsigned int i=0; i<joint_names_.size(); i++) {
    if(gravity_compensation_only_) {
      joint_handles_[i].setCommand(efforts_(i) + sub_command_[i]);
    }
    else {
      ROS_DEBUG("Setting command for joint %s: %f", joint_names_[i].c_str(), efforts_(i));
      joint_handles_[i].setCommand(efforts_(i));
    }
  }
}

} // end of namespace

PLUGINLIB_EXPORT_CLASS(dynamics_controllers::KdlChainController, controller_interface::ControllerBase)
