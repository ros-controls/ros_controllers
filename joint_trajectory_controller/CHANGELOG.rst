^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2013-09-04)
------------------
* joint_trajectory_controller: New package implementing a controller for executing joint-space trajectories on a
  set of joints.

  * ROS API

    * Commands: FollowJointTrajectory action and trajectory_msgs::JointTrajectory topic.
    * Current controller state is available in a control_msgs::JointTrajectoryControllerState topic.
    * Controller state at any future time can be queried through a control_msgs::JointTrajectoryControllerState
      service.

  * Trajectory segment type

    * Controller is templated on the segment type.
    * Multi-dof quintic spline segment implementation provided by default.

  * Hardware interface type

    * Controller is templated on the hardware interface type.
    * Position and effort control joint interfaces provided by default.

  * Other

    * Realtime-safe.
    * Proper handling of wrapping (continuous) joints.
    * Discontinuous system clock changes do not cause discontinuities in the execution of already queued
      trajectory segments.