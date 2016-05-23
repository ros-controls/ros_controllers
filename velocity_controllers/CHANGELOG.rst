^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package velocity_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Add missing dependencies
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.0 (2014-10-31)
------------------
* New controller: velocity_controllers/JointGroupVelocityController (multi-joint)
* New controller: velocity_controllers/JointPositionController
* Contributors: ipa-fxm, Dave Coleman

0.8.1 (2014-07-11)
------------------

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
* Link shared libraries to catkin libraries
  GCC is quite lenient with missing symbols on shared libraries and
  doesn't event output any warning about it.
  When building with other compilers, missing symbols result in build
  errors.
* Contributors: Paul Mathieu

0.5.4 (2013-09-30)
------------------

0.5.3 (2013-09-04)
------------------
* Removed manifest.xml from all packages to prevent rosdep heirarchy issues in Groovy and Hydro
* Added ignored manifest.xml files, added rule to .gitignore

0.5.2 (2013-08-06)
------------------

0.5.1 (2013-07-19)
------------------

0.5.0 (2013-07-16)
------------------
* Merged
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Restore "Fixed PLUGINLIB_DECLARE_CLASS depreacated errors""
  This reverts commit 0862ad93696b0d736b565cd65ea36690dde0eaa7.
* Adding install targets for plugin xml files
* Revert "Fixed PLUGINLIB_DECLARE_CLASS depreacated errors"
  This reverts commit 2314b8b434e35dc9c1c298140118a004e00febd8.

0.4.0 (2013-06-26)
------------------
* Version 0.4.0
* Fixed PLUGINLIB_DECLARE_CLASS depreacated errors
* adding install targets
* adding switches for hybrid buildsystem
* adding these packages which weren't seen by catkinize_stack
* Extend joint_effort_controller to other interfaces
  - Factor-out implementation of simple command-forwarding controller.
  - Provide specializations (typedefs really) for effort, velocity and position
  interfaces.
