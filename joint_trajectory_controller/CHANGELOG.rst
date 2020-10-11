^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.18.0 (2020-10-11)
-------------------
* Increase joint0 smoothing to force goal tolerance violation
* Add pathToleranceViolationSingleJoint and goalToleranceViolationSingleJoint tests
* Allow setting 'smoothing' to each joint of rrbot
* Only update state_joint_error\_  values in loop where used
* JointTrajectoryController: Fix tolerance checking to use state error from the correct joints
* Fix missing virtual destructor
* Contributors: Bence Magyar, Doug Morrison, Mateus Amarante, Tyler Weaver

0.17.0 (2020-05-12)
-------------------
* Add extension point in update function to allow derived classes to perform e.g. additional checks.
* The stop- and hold-trajectory creation is moved into separate classes,
  to allow derived classes to re-use the stop- and hold-trajectory
  creation code.
* The old desired state and the old time data are now also stored,
  to allow derived classes to perfrom more comprehensive checks, etc.
* Contributors: Pilz GmbH and Co. KG, Bence Magyar

0.16.1 (2020-04-27)
-------------------

0.16.0 (2020-04-16)
-------------------
* Bump CMake version to prevent CMP0048
* Add #pragma once to new joint_trajectory_controller test
* Replace header guard with #pragma once
* Modernize xacro
  - Remove '--inorder'
  - Use 'xacro' over 'xacro.py'
* Add code_coverage target to CMakeLists.txt
* Contributors: Bence Magyar, Immanuel Martini, Matt Reynolds

0.15.1 (2020-03-09)
-------------------
* Fix instabilities in JTC unittests
* Add test_common.h
* Re-enable tolerance tests
* Make initialization of variables consistent
* Let getState() return by value
* Use nullptr (`#447 <https://github.com/ros-controls/ros_controllers/issues/447>`_)
* Comment out tolerance check tests until `#48 <https://github.com/ros-controls/ros_controllers/issues/48>`_ is solved.
* Execution does not stop when goal gets aborted
* Fix how we assert that controller is running
* Introduce EPS for general double comparison
* Make stopramp tests more stable
* add missing pluginlib deps. (`#451 <https://github.com/ros-controls/ros_controllers/issues/451>`_)
* Print error messages for all exceptions
* Contributors: Bence Magyar, Ian Frosst, Immanuel Martini, Matt Reynolds, Sean Yen

0.15.0 (2019-03-26)
-------------------
* Default all controller builds to C++14
* Use range-based for loops wherever possible
* boost::array -> std::array
* mutex to C++11, boost::scoped_lock -> std::lock_guard
* boost::scoped_ptr -> std::unique_ptr
* boost::shared_ptr -> std::shared_ptr
* fix install destination for libraries (`#403 <https://github.com/ros-controls/ros_controllers/issues/403>`_)
* Contributors: Bence Magyar, Gennaro Raiola, James Xu

0.14.3 (2019-02-09)
-------------------
* use operators instead of aliases
* joint_trajectory_controller: fix minor typo in class doc.
* correctly parse joint trajectory options
* Remove deprecated parameter hold_trajectory_duration (`#386 <https://github.com/ros-controls/ros_controllers/issues/386>`_)
* dont print warning about dropped first point, if it is expected behaviour
* Contributors: AndyZe, G.A. vd. Hoorn, Gennaro Raiola, James Xu, Joachim Schleicher, Karsten Knese

0.14.2 (2018-10-23)
-------------------
* Report errors in updateTrajectoryCommand back though action result error_string
* Remove redundant warning messages
* Return error string when failing to initialize trajectory from message
* Changes to allow inheritance from JointTrajectoryController.
* Update maintainers
* Contributors: Alexander Gutenkunst, Miguel Prada, Mathias Lüdtke, Bence Magyar

0.14.1 (2018-06-26)
-------------------
* joint_trajectory_controller tests stability improved
* Use a copy of rt_active_goal in update()
* Contributors: Kei Okada, Ryosuke Tajima

0.14.0 (2018-04-27)
-------------------
* Make the compiler happy in the test.
* migrate to new pluginlib headers
* TrajectoryController: Use desired state to calculate hold trajectory (`#297 <https://github.com/ros-controls/ros_controllers/issues/297>`_)
* Add velocity feedforward term to velocity HardwareInterfaceAdapter (`#227 <https://github.com/ros-controls/ros_controllers/issues/227>`_)
* Contributors: Chris Lalancette, Mathias Lüdtke, Miguel Prada, agutenkunst

0.13.2 (2017-12-23)
-------------------
* Changend the implementation of joint_trajectory_controller to enable the forwarding of the acceleration values from the trajectory
* Contributors: Bence Magyar, Mart Moerdijk

0.13.1 (2017-11-06)
-------------------
* Linted pos_vel joint_trajectory_controllers
* Added posvel joint_trajectory_controller
  Added a simple posvel joint_trajectory_controller that forwards
  the desired state at the current point in time of the trajectory
  to the joint.
* Add support for an joint interfaces are not inherited from JointHandle.
  Add JointTrajectoryController specification for SplineJointInterface.
* Contributors: Gennaro Raiola, Igorec, Zach Anderson

0.13.0 (2017-08-10)
-------------------
* Make rqt_plot optional
* Added tests for issue `#275 <https://github.com/ros-controls/ros_controllers/issues/275>`_
* Address Issue  `#275 <https://github.com/ros-controls/ros_controllers/issues/275>`_ for kinetic
* Address issue `#263 <https://github.com/ros-controls/ros_controllers/issues/263>`_, joint_trajectory_controller - wraparoundOffset
* Added warning to indicate that the verbose flag is enabled
* Set hold trajectory goal handle when empty trajectory received through action.
  Previously, an empty trajectory received through the action interface would
  set hold trajectory and accept the action goal, but the action would never be
  terminated, leaving clients hanging.
* Contributors: Bence Magyar, Miguel Prada, bponsler, gennaro

0.12.3 (2017-04-23)
-------------------

0.12.2 (2017-04-21)
-------------------
* Remove rqt_plot test_depend & make plots optional
* Contributors: Bence Magyar

0.12.1 (2017-03-08)
-------------------

0.12.0 (2017-02-15)
-------------------
* Fix missing controller_manager include
* Ordered dependencies & cleanup
* Change for format2
* Add Enrique and Bence to maintainers
* Add test that sends trajectory entirely in past
* Use xacro instead of xacro.py
* urdf::Model typedefs had to be added to a different repo first
* Updated copyright info
* jtc: Enable sending trajectories with a partial set of joints
* Replace boost::shared_ptr<urdf::XY> with urdf::XYConstSharedPtr when exists
* Infrastructure for testing the velocity_controllers::JointTrajectoryController.
* jtc: Enable sending trajectories with a partial set of joints
* Contributors: Beatriz Leon, Bence Magyar, Miguel Prada

0.11.2 (2016-08-16)
-------------------

0.11.1 (2016-05-23)
-------------------
* Write feedback for the RealtimeServerGoalHandle to publish on the non-realtime thread.
* Contributors: Miguel Prada

0.11.0 (2016-05-03)
-------------------

0.10.0 (2015-11-20)
-------------------
* Add joint limits spec to rrbot test robot
* Address -Wunused-parameter warnings
* Reset to semantic zero in HardwareInterfaceAdapter for PositionJointInterface
* Contributors: Adolfo Rodriguez Tsouroukdissian, ipa-fxm

0.9.2 (2015-05-04)
------------------

0.9.1 (2014-11-03)
------------------

0.9.0 (2014-10-31)
------------------
* Check that waypoint times are strictly increasing before accepting a command
* velocity_controllers::JointTrajectoryController: New plugin variant for
  velocity-controlled joints
* Buildsystem fixes
* Contributors: Adolfo Rodriguez Tsouroukdissian, Lukas Bulwahn, ipa-fxm, Dave Coleman

0.8.1 (2014-07-11)
------------------
* joint_trajectory_controller: Critical targets declared before calling catkin_package
* check for CATKIN_ENABLE_TESTING
* Contributors: Jonathan Bohren, Lukas Bulwahn

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
* Add support for an joint interfaces are not inherited from JointHandle.
* Contributors: Igorec

0.6.0 (2014-02-05)
------------------
* Merge pull request `#72 <https://github.com/ros-controls/ros_controllers/issues/72>`_ from pal-robotics/minor-maintenance
  Minor maintenance
* Default stop_trajectory_duration to zero. Refs `#73 <https://github.com/ros-controls/ros_controllers/issues/73>`_
* Better logs when dropping traj points. Refs `#68 <https://github.com/ros-controls/ros_controllers/issues/68>`_.
* Fix class member reorder warning in constructor.
* Add missing headers to target files.
* Action interface rejects empty goals. Fixes `#70 <https://github.com/ros-controls/ros_controllers/issues/70>`_.
* Reorder how time and traj data are updated.
  In the update method, fetching the currently executed trajectory should be done
  before updating the time data to prevent a potential scenario in which there
  is no trajectory defined for the current control cycle.
* Work tolerance checking methods.
  Until now we used the currently active goal handle for performing tolerance
  checks. Using the goal handle stored in segments is more robust to unexpected
  goal updates by the non-rt thread.
* Refactor how the currrent trajectory is stored.
  - Handle concurrency in the current trajectory between rt and non-rt threads
  using the simpler RealtimeBox instead of the RealtimeBuffer, because our
  usecase does not fit well the non-rt->writes / rt->reads semantics.
  - As a consequence we no longer need to store the msg_trajectory member, but
  only the hold_trajectory, which must still be preallocated.
* Honor unspecified vel/acc in ROS message. Fix `#65 <https://github.com/ros-controls/ros_controllers/issues/65>`_.
* Fixes per Adolfo
* Added verbose flag
* Fixing realtime issues
* Merge branch 'hydro-devel' into joint_trajectory_tweaks
* Tweaked error messages
* Added more debug info
* Fix for microsecond delay that caused header time=0 (now) to start too late
* Reworded debug message
* Image update.
* Update README.md
  Factor out user documentation to the ROS wiki.
* Merge branch 'hydro-devel' of https://github.com/willowgarage/ros_controllers into hydro-devel
* Rename hold_trajectory_duration
  - hold_trajectory_duration -> stop_trajectory_duration for more clarity.
  - During Hydro, hold_trajectory_duration will still work, giving a deprecation
  warning.
* Add basic description in package.xml.
* Add images used in the ROS wiki doc.
* Added better debug info
* Throttled debug output
* Added more debug and error information
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.4 (2013-09-30)
------------------
* Added install rules for plugin.xml
* Remove PID sign flip.
  This is now done in the state error computation.
* Merge pull request `#45 <https://github.com/davetcoleman/ros_controllers/issues/45>`_ from ros-controls/effort_fixes
  Added check for ~/robot_description and fixed hardware interface abstraction bug
* Flip state error sign.
* PID sign was wrong
* Added check for ~/robot_description and fixed hardware interface abstraction bug
* Update README.md
* Create README.md
* Fix license header string for some files.
* Less verbose init logging.
  Statement detailing controller joint count, as well as segment and hardware
  interface types moved from INFO to DEBUG severity.

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
