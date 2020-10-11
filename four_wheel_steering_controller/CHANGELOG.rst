^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package four_wheel_steering_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.18.0 (2020-10-11)
-------------------
* Add missing test dependency on xacro in four_wheel_steering_controller
* Contributors: Mateus Amarante

0.17.0 (2020-05-12)
-------------------

0.16.1 (2020-04-27)
-------------------

0.16.0 (2020-04-16)
-------------------
* Bump CMake version to prevent CMP0048
* Add missing header guards
* Replace header guard with #pragma once
* Modernize xacro
  - Remove '--inorder'
  - Use 'xacro' over 'xacro.py'
* swap implementations of read and write methods
  Follows the intended use of hardware_interface::RobotHW,
  see its documentation for details
* Contributors: Franz, Matt Reynolds

0.15.1 (2020-03-09)
-------------------
* add missing pluginlib deps
* Use C++11 `std::this_thread::sleep_for`.
* add include directories for tests in {ackermann/four_wheel}_steering_controller
* Contributors: Davide Faconti, Mathias Lüdtke, Sean Yen

0.15.0 (2019-03-26)
-------------------
* Default all controller builds to C++14
* boost::chrono -> std::chrono
* Adjust missing bsd note
* boost::assign -> C++ initializer list
* boost::shared_ptr -> std::shared_ptr
* fix install destination for libraries (`#403 <https://github.com/ros-controls/ros_controllers/issues/403>`_)
* Contributors: Bence Magyar, Gennaro Raiola, James Xu

0.14.3 (2019-02-09)
-------------------
* Minor change in one of the ROS_INFO_STREAM
* Contributors: Jan-Felix Klein

0.14.2 (2018-10-23)
-------------------

0.14.1 (2018-06-26)
-------------------

0.14.0 (2018-04-27)
-------------------
* migrate to new pluginlib headers
* fix warning un/signed comparison
* [4ws tests] simulation clock
* [4ws tests] Increase position tolerance
* Contributors: Bence Magyar, Jeremie Deray, Mathias Lüdtke, Vincent Rousseau

0.13.2 (2017-12-23)
-------------------
* Add four_wheel_steering_controller
* Contributors: Vincent Rousseau

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

0.11.2 (2016-08-16)
-------------------

0.11.1 (2016-05-23)
-------------------

0.11.0 (2016-05-03)
-------------------

0.10.0 (2015-11-20)
-------------------

0.9.2 (2015-05-04)
------------------

0.9.1 (2014-11-03)
------------------

0.9.0 (2014-10-31)
------------------

0.8.1 (2014-07-11)
------------------

0.8.0 (2014-05-12)
------------------

0.7.2 (2014-04-01)
------------------

0.7.1 (2014-03-31)
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
