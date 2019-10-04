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

#ifndef DYNAMICS_CONTROLLERS_KDL_TREE_CONTROLLER_H
#define DYNAMICS_CONTROLLERS_KDL_TREE_CONTROLLER_H

#include <dynamics_controllers/dynamics_controller_base.h>
#include <kdl/chain.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>


namespace dynamics_controllers {

namespace internal {
/* The "sub-tree extraction" functionality was proposed in:
 * https://github.com/orocos/orocos_kinematics_dynamics/pull/174
 * However, the PR is not available yet. As a workaround, a similar function
 * is given here. If the PR is merged, the following will be removed.
 */
bool extractSubTree(const KDL::Tree& tree, const std::string& root_name, KDL::Tree& subtree);
}

class KdlTreeController : public DynamicsControllerBase {
public:
  KdlTreeController();

protected:
  /// Loads the tree and the dynamics solver.
  bool initDynamics(ros::NodeHandle& nh) override;
  /// IDM of a tree.
  void computeEfforts() override;

  /// If true, just add gravity efforts; if false, do a "full CTC" control.
  bool gravity_compensation_only_;
  /// Tree structure, storing the dynamics parameters.
  KDL::Tree tree_;
  /// Dynamics solver to compute efforts from the position, velocity and acceleration.
  std::unique_ptr<KDL::TreeIdSolver_RNE> rne_;
  /// Configuration of the manipulator (position, velocity and acceleration).
  KDL::JntArrayAcc cfg_;
  /// Vector of external efforts, passed to the solver.
  KDL::WrenchMap wrenches_;
  /// Computed efforts.
  KDL::JntArray efforts_;
};

}

#endif
