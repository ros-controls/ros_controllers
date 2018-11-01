#ifndef EXPORT_TRAJ_OR_JOG_CONTROLLERS_H
#define EXPORT_TRAJ_OR_JOG_CONTROLLERS_H

// Set up namespacing of controllers and create their plugins.
namespace traj_or_jog_controllers
{
  /**
   * \brief A combination of a JointTrajectoryController with a ForwardJointGroupCommand controller.
   */
  typedef traj_or_jog_controller::TrajOrJogController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::VelocityJointInterface>
          TrajOrJogController;
}

PLUGINLIB_EXPORT_CLASS(traj_or_jog_controllers::TrajOrJogController, controller_interface::ControllerBase)

#endif  // header guard