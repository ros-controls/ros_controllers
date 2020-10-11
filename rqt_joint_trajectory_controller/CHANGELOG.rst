^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_joint_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.18.0 (2020-10-11)
-------------------
* Use Python3 explicitly
* fix shebang line for python3
* Contributors: Bence Magyar, Mikael Arguedas

0.17.0 (2020-05-12)
-------------------

0.16.1 (2020-04-27)
-------------------

0.16.0 (2020-04-16)
-------------------
* Bump CMake version to prevent CMP0048
* Contributors: Matt Reynolds

0.15.1 (2020-03-09)
-------------------
* Merge pull request `#452 <https://github.com/ros-controls/ros_controllers/issues/452>`_ from etsiogas/add-robot-ns-to-gui
  Added robot namespace to gui of rqt_joint_trajectory_controller
* [rqt joint trajectory controller] Python3 fixes (`#458 <https://github.com/ros-controls/ros_controllers/issues/458>`_)
  - Use explicit relative import (with leading dot)
  - print function with parentheses
* Contributors: Bence Magyar, Bjar Ne, etsiogas

0.15.0 (2019-03-26)
-------------------

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
* fix license string
* Contributors: Patrick Holthaus

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
* Changes in import of Qt modules
* Contributors: Carlos J. Rosales Gallegos

0.11.1 (2016-05-23)
-------------------

0.11.0 (2016-05-03)
-------------------

0.10.0 (2015-11-20)
-------------------
* Adapt to new controller_manager_msgs/ControllerState message definition
* Add vertical scrollbar to joints list
  - Add vertical scrollbar to joints list that appears only when required,
  i.e., when the plugin size cannot accommodate all controller joints.
  - Remove vertical spacer at the bottom of the plugin.
* Clear controllers combo on cm change
  Clear the list of running joint trajectory controllers when the
  controller manager selection changes. This prevents potential conflicts when
  multiple controller managers have controllers with the same names.
* Fail gracefully if URDF is not loaded
  Implement lazy loading of joint limits from URDF.
  This allows to start rqt_joint_trajectory_controller on an otherwise empty ROS
  node graph without crashing.
* Save and restore plugin settings
  - Save current controller_manager and controller selection on plugin close
  - Restore last selection if controller manager and controller are running
* Stricter controller validation
  Only display in the controller combo box those controllers that are running
  _and\_ have position and velocity limits specified in the URDF. In the absence
  of limits information, it's not posible to properly initialize the GUI sliders.
* Fix broken URDF joint limits parsing
* Don't choke on missing URDF vel limits
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.2 (2015-05-04)
------------------
* rqt_joint_traj_controller: Add missing runtime dep
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.1 (2014-11-03)
------------------

0.9.0 (2014-10-31)
------------------
* New rqt plugin: joint_trajectory_controller rqt plugin.
  - Allows to select any running joint trajectory controller from any active
    controller manager
  - Two modes:
    - Monitor: Joint display shows actual positions of controller joints
    - Control: Joint display sends controller commands
  - Max joint speed is read from the URDF, but can be scaled down for safety
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.1 (2014-07-11)
------------------

0.8.0 (2014-05-12)
------------------

0.7.3 (2014-10-28)
------------------

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

0.5.2 (2013-08-06)
------------------

0.5.1 (2013-07-19)
------------------

0.5.0 (2013-07-16)
------------------

0.4.0 (2013-06-26)
------------------
