^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_state_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.1 (2016-05-23)
-------------------

0.11.0 (2016-05-03)
-------------------

0.10.0 (2015-11-20)
-------------------
* Address -Wunused-parameter warnings
* Add extra joints support
  Allow to optionally specify a set of extra joints for state publishing that
  are not contained in the JointStateInterface associated to the controller.
  The state of these joints can be specified via ROS parameters, and remains
  constant over time.
* Add test suite
* Migrate to package format2
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.2 (2015-05-04)
------------------

0.9.1 (2014-11-03)
------------------
* Update package maintainers
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.0 (2014-10-31)
------------------
* Buildsystem fixes
* Contributors: Dave Coleman

0.8.1 (2014-07-11)
------------------

0.8.0 (2014-05-12)
------------------
* Add missing controller resources to install target
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
* Install default config files
* Contributors: Paul Mathieu

0.5.4 (2013-09-30)
------------------
* Silence cppcheck warning on unit'ed variables.

0.5.3 (2013-09-04)
------------------
* Removed last manifest.xml
* Added ignored manifest.xml files, added rule to .gitignore

0.5.2 (2013-08-06)
------------------
* Added joint limit enforcement for controller set point command

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
* Fix package URL in package.xml

0.4.0 (2013-06-26)
------------------
* Version 0.4.0
* Removed PR2 references and renamed github repo in docs
* Fix package URL in package.xml
* Fixed PLUGINLIB_DECLARE_CLASS depreacated errors
* Propagate API changes in hardware_interface.
* adding install targets
* adding switches for hybrid buildsystem
* adding back more manifests and makefiles
* Fix package URL.
* bumping version
* adding package.xml files
* Catkinizing. Building, but could still be cleaned up
* use new root nodehandle to publish joint states in the namespace of the controller manager. This fixes a but when pushing the controller manager in a namespace, and keeps the same behavior when the controller manager is not in a namespace
* Add .gitignore files on a per-package basis.
* Add missing include guard.
* Change tab indentation for spaces.
* port to new api with time and duration
* moved package with joint state controller
