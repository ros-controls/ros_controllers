^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package imu_sensor_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.13.4 (2018-06-26)
-------------------

0.13.3 (2018-04-27)
-------------------
* migrate to new pluginlib headers
* Contributors: Mathias Lüdtke

0.13.2 (2017-12-23)
-------------------

0.13.1 (2017-11-06)
-------------------

0.13.0 (2017-08-10)
-------------------

0.12.3 (2017-04-23)
-------------------

0.12.2 (2017-04-21)
-------------------

0.12.1 (2017-03-08)
-------------------

0.12.0 (2017-02-15)
-------------------
* Fix most catkin lint issues
* Change for format2
* Add Enrique and Bence to maintainers
* Contributors: Bence Magyar

0.11.2 (2016-08-16)
-------------------

0.11.1 (2016-05-23)
-------------------

0.11.0 (2016-05-03)
-------------------

0.10.0 (2015-11-20)
-------------------
* Fixed covariances in ImuSensorController::update
* Address -Wunused-parameter warnings
* Contributors: Adolfo Rodriguez Tsouroukdissian, tappan-at-git

0.9.2 (2015-05-04)
------------------

0.9.1 (2014-11-03)
------------------

0.9.0 (2014-10-31)
------------------
* Buildsystem fixes
* Contributors: Adolfo Rodriguez Tsouroukdissian

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
* Install default config files
* Contributors: Paul Mathieu

0.5.4 (2013-09-30)
------------------
* Added install rules for plugin.xml
* Fix license header string for some files.

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
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Make hybrid rosbuild-catkin packages.
  Affects force-torque and imu sensor controllers.
* Add package.xml scripts.
* Fix author name typo.
* Fix PLUGINLIB_DECLARE_CLASS depreacated errors.
* Propagate sensor interfaces API changes.
* Fix package URLs.
* Propagate changes in hardware_interface.
  - force-torque and IMU sensors no longer depend on Eigen.
  - The controllers that publish sensor state don't need the Eigen wrappers
  and now use the raw data directly.
* Controller publishing the state of a IMU sensor.

0.4.0 (2013-06-26)
------------------
