^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package effort_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2013-09-04)
------------------
* Removed manifest.xml from all packages to prevent rosdep heirarchy issues in Groovy and Hydro
* Added ignored manifest.xml files, added rule to .gitignore

0.5.2 (2013-08-06)
------------------
* Minor comment fix
* Critical bug: velocity controller init() does not get hardware_interface handle for joint
* Fixes for joint_position_controller
* Consolidated position and velocity command into one realtime buffer
* Tweaked header guard
* Added ability to set target velocity, CMake cleanup
* Removed debug output from realtime context
* Removed blocking msgs from realtime loop
* Added joint limit enforcement for controller set point command

0.5.1 (2013-07-19)
------------------

0.5.0 (2013-07-16)
------------------
* Merged
* Removed controller_msgs
* Fixed PID destructor bug, cleaned up code
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Restore "Fixed PLUGINLIB_DECLARE_CLASS depreacated errors""
  This reverts commit 0862ad93696b0d736b565cd65ea36690dde0eaa7.
* Fixing reversed error computation...
* Adding install targets for plugin xml files
* Revert "Fixed PLUGINLIB_DECLARE_CLASS depreacated errors"
  This reverts commit 2314b8b434e35dc9c1c298140118a004e00febd8.

0.4.0 (2013-06-26)
------------------
* Version 0.4.0
* Fixing position effort controller pid command args
* Fixed control_toolbox deprecated errors with updatePid()
* Fixed PLUGINLIB_DECLARE_CLASS depreacated errors
* Propagate API changes in hardware_interface.
* adding install targets
* adding switches for hybrid buildsystem
* adding back more manifests and makefiles
* Trivial log message fix.
* Fixing library export
* adding these packages which weren't seen by catkinize_stack
* bumping version
* adding package.xml files
* Catkinizing. Building, but could still be cleaned up
* Extend joint_effort_controller to other interfaces
  - Factor-out implementation of simple command-forwarding controller.
  - Provide specializations (typedefs really) for effort, velocity and position
  interfaces.
* Fix documentation typo.
* Add .gitignore files on a per-package basis.
* effort_controllers::joint_velocity_controller was not being built
* Fixing typos in JointVelocityController
* port to new api with time and duration
* fix xml filename
* register controllers
* fixes
* add position controller
* port another controller
* clean up dependencies
* first simple controller for testing
