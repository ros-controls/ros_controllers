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

#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_loader.h>
#include <stdexcept>


namespace dynamics_controllers {

/// Base class for dynamics controllers.
/** This class defines the common interface for dynamics controllers. Such
  * controllers are supposed to compute the efforts from the acceleration
  * provided by another "sub-controller".
  *
  * To allow wrapping around the sub-controller, this class creates a "fake
  * hardware".
  * It is used to provide the current robot state to the sub-controller and to
  * retrieve its command.
  *
  * To create a specific dynamics controller, simply inherit from this class
  * and implement the pure virtual methods `initDynamics()` and `computeEfforts()`.
  */
class DynamicsControllerBase : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  DynamicsControllerBase();

  /// Initializes the controller.
  /** The method performs the follwing steps:
    * - call `initDynamics()`
    * - initialize the fake hardware
    * - initialize the sub-controller
    */
  bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle& nh) override;

  /// Start the sub-controller.
  void starting(const ros::Time& time) override;

  /// Compute the efforts.
  /** The method performs the following steps:
    * - copy the current state of the robot in the fake hardware
    * - update the sub-controller
    * - copy the command of the sub-controller in `sub_command_`
    * - call `computeEfforts()`
    */
  void update(const ros::Time& time, const ros::Duration& period) override;

  /// Stop the sub-controller.
  void stopping(const ros::Time& time) override;

protected:

  /// Called when initializing the controller.
  /** The method allows to perform sub-class specific initialization, eg,
    * loading dynamic solvers.
    * \note The method should also initialize the member `joint_names_`, which
    * will be used to generate the `FakeHW` instance for the low-level
    * controller.
    */
  virtual bool initDynamics(ros::NodeHandle& nh) = 0;

  /// Sub-classes should override this method so that they do specific dynamic control.
  /** The subclass is required to either set the efforts directly (using
    * `joint_handles_`) or to call the method `setEfforts()`.
    */
  virtual void computeEfforts() = 0;

  /// Utility to set the commanded efforts.
  /** It assumes that the input vector is ordered as in `joint_names_`.
    */
  inline void setEfforts(const std::vector<double>& effort_command) {
    for(unsigned int i=0; i<joint_names_.size(); i++)
      joint_handles_[i].setCommand(effort_command[i]);
  }

  /// Joint names controlled by this object.
  std::vector<std::string> joint_names_;

  /// Stores the command evaluated by the sub-controller.
  std::vector<double> sub_command_;

  /// Handles associated to the controlled joints.
  std::vector<hardware_interface::JointHandle> joint_handles_;

private:
  class FakeHW; // forward declaration
  /// Used to create runtime instances of the controller.
  controller_manager::ControllerLoaderInterfaceSharedPtr controller_loader_;
  /// Sub-controllers instances.
  std::map<std::string,controller_interface::ControllerBaseSharedPtr> sub_controllers_;
  /// Simplified replica of the available hardware, to be given to the sub-controllers.
  std::unique_ptr<FakeHW> fake_hw_;

  // Handling control limits
  std::map<std::string,bool> has_effort_limits_; ///< Tells whether a given joint has effort limits or not.
  std::map<std::string,double> effort_limits_; ///< Effort limit of each joint.

  /// Allows to get a map of controllers names (key) with corresponding types (values).
  static bool getSubControllersMap(ros::NodeHandle&, std::map<std::string,std::string>&);

  /// Enforce joint limits (currently, simply saturate the efforts if needed).
  void enforceLimits();

  /// A fake hardware used to exchange information with a "sub-controller".
  class FakeHW : public hardware_interface::RobotHW {
    public:
      /// Generates a hardware that contains state and effort handles for a set of joints.
      FakeHW(const std::vector<std::string>& joint_names);
      /// Update the internal state of the joints from the information given in `joint_handles`
      void copyState(const std::vector<hardware_interface::JointHandle> &joint_handles);
      /// Retrieve the command stored in `cmd_`.
      void getEfforts(std::vector<double> &joint_effort_command);
    private:
      std::vector<double> pos_; ///< Joint positions.
      std::vector<double> vel_; ///< Joint velocities.
      std::vector<double> eff_; ///< Joint efforts.
      std::vector<double> cmd_; ///< Commanded efforts.
      hardware_interface::JointStateInterface state_interface_; ///< Provides joint state handles.
      hardware_interface::EffortJointInterface effort_interface_; ///< Provides command handles.
  };

};

}
