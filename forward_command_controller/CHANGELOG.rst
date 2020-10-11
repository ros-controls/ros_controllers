^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package forward_command_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.18.0 (2020-10-11)
-------------------
* Apply consistent format to forward_command_controller's CMakeLists.txt file
* Update package.xml of forward_ccommand_controller pkg to format 3
* Fix dependencies of forward_command_controller pkg
* Contributors: Mateus Amarante

0.17.0 (2020-05-12)
-------------------

0.16.1 (2020-04-27)
-------------------

0.16.0 (2020-04-16)
-------------------
* Bump CMake version to prevent CMP0048
* Replace header guard with #pragma once
* Contributors: Matt Reynolds

0.15.1 (2020-03-09)
-------------------

0.15.0 (2019-03-26)
-------------------
* Default all controller builds to C++14
* Contributors: Bence Magyar, Gennaro Raiola

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
* Address -Wunused-parameter warnings
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.2 (2015-05-04)
------------------
* Thread-safe and realtime-safe forward controllers.
* Complain if list of joints is empty.
* Contributors: Mathias Lüdtke

0.9.1 (2014-11-03)
------------------

0.9.0 (2014-10-31)
------------------
* ForwardJointGroupCommandController: Class for implementing multi-joint
  command-forwarding controllers
* Contributors: ipa-fxm

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
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.

0.4.0 (2013-06-26)
------------------
* Version 0.4.0
* Propagate API changes in hardware_interface.
* adding install targets
* adding switches for hybrid buildsystem
* Remove unused method (legacy from the past).
* adding these packages which weren't seen by catkinize_stack
* Extend joint_effort_controller to other interfaces
  - Factor-out implementation of simple command-forwarding controller.
  - Provide specializations (typedefs really) for effort, velocity and position
  interfaces.
