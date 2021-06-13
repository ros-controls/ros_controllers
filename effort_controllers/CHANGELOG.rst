^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package effort_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.19.0 (2021-06-13)
-------------------

0.18.1 (2020-12-03)
-------------------

0.18.0 (2020-10-11)
-------------------
* Update effort_controllers' package.xml file to format 3
* Apply consistent format to effort_controllers' CMakeLists.txt file
* Clean effort_controllers' dependencies
* [effort_controllers] JointGroupPositionController: Change to initRT
* [effort_controllers] JointGroupPositionController: Set to current position in starting
  Currently, the target position upon starting is all zeros. This is not
  great if a command is not issued immediately. To be safer, set the goal
  position to the current sensed position and also reset the PID internal
  state upon starting.
* Remove unused transmission in effort_controllers test
* Add missing test dependency on joint_state_controller in effort_controllers
* [effort_controllers] JointGroupPositionController: Load URDF from namespace
  Mimics `#245 <https://github.com/ros-controls/ros_controllers/issues/245>`_
* Contributors: Mateus Amarante, Wolfgang Merkt

0.17.0 (2020-05-12)
-------------------

0.16.1 (2020-04-27)
-------------------

0.16.0 (2020-04-16)
-------------------
* Added test for position controller
* Solving issues with large rotational limits
* Bump CMake version to prevent CMP0048
* Replace header guard with #pragma once
* fix minor typo in documentation of setGains method
  Changes Get to Set of setGains method documentation
* Contributors: Bence Magyar, F Pucher, Franco Fusco, Matt Reynolds

0.15.1 (2020-03-09)
-------------------
* add missing pluginlib deps.
* effort_controllers: fix minor typo in setGains doc
* Contributors: G.A. vd. Hoorn, Sean Yen

0.15.0 (2019-03-26)
-------------------
* Default all controller builds to C++14
* Use range-based for loops wherever possible
* boost::scoped_ptr -> std::unique_ptr
* fix install destination for libraries (`#403 <https://github.com/ros-controls/ros_controllers/issues/403>`_)
* Contributors: Bence Magyar, Gennaro Raiola, James Xu

0.14.3 (2019-02-09)
-------------------

0.14.2 (2018-10-23)
-------------------
* Update maintainers
* Contributors: Bence Magyar

0.14.1 (2018-06-26)
-------------------

0.14.0 (2018-04-27)
-------------------
* migrate to new pluginlib headers
* Contributors: Mathias Lüdtke

0.13.2 (2017-12-23)
-------------------

0.13.1 (2017-11-06)
-------------------
* Code cleanup
* Add JointGroupPositionController
* Contributors: Bence Magyar, Maik Mugnai

0.13.0 (2017-08-10)
-------------------

0.12.3 (2017-04-23)
-------------------
* Supply NodeHandle to urdf::Model. Closes `#244 <https://github.com/ros-controls/ros_controllers/issues/244>`_
* Contributors: Piyush Khandelwal

0.12.2 (2017-04-21)
-------------------

0.12.1 (2017-03-08)
-------------------

0.12.0 (2017-02-15)
-------------------
* Fix most catkin lint issues
* Remove unused dependency
* Change for format2
* Add Enrique and Bence to maintainers
* Replace boost::shared_ptr<urdf::XY> with urdf::XYConstSharedPtr when exists
* Contributors: Bence Magyar

0.11.2 (2016-08-16)
-------------------
* Included angles in dependencies
* Contributors: Mr-Yellow

0.11.1 (2016-05-23)
-------------------

0.11.0 (2016-05-03)
-------------------
* Add antinwindup to get and setGains logic for underlying PID controller
* Contributors: Paul Bovbel

0.10.0 (2015-11-20)
-------------------

0.9.2 (2015-05-04)
------------------
* Thread-safe and realtime-safe forward controllers.
* Contributors: Mathias Lüdtke

0.9.1 (2014-11-03)
------------------
* Update package maintainers
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.0 (2014-10-31)
------------------
* Propagate changes made in forward_command_controller
* Contributors: ipa-fxm

0.8.1 (2014-07-11)
------------------
* Add depend on angles
* Contributors: Scott K Logan

0.8.0 (2014-05-12)
------------------
* Remove rosbuild artifacts. Fix `#90 <https://github.com/ros-controls/ros_controllers/issues/90>`_.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.2 (2014-04-01)
------------------

0.7.1 (2014-03-31)
------------------

0.7.0 (2014-03-28)
------------------

0.6.0 (2014-02-05)
------------------
* Added new has_velocity flag that indiciates if a target velocity has been set
* Contributors: Dave Coleman

0.5.4 (2013-09-30)
------------------

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
