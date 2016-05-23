^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diff_drive_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.1 (2016-05-23)
-------------------

0.11.0 (2016-05-03)
-------------------

0.10.0 (2015-11-20)
-------------------
* Address -Wunused-parameter warnings
* Limit jerk
* Add param velocity_rolling_window_size
* Minor fixes
  1. Coding style
  2. Tolerance to fall-back to Runge-Kutta 2 integration
  3. Remove unused variables
* Fix the following bugs in the testForward test:
  1. Check traveled distance in XY plane
  2. Use expected speed variable on test check
* Add test for NaN
* Add test for bad URDF
* Contributors: Adolfo Rodriguez Tsouroukdissian, Enrique Fernandez, Paul Mathieu

0.9.2 (2015-05-04)
------------------
* Allow the wheel separation and radius to be set from different sources
  i.e. one can be set from the URDF, the other from the parameter server.
  If wheel separation and wheel diameter is specified in the parameter server, don't look them up from urdf
* Contributors: Bence Magyar, Nils Berg

0.9.1 (2014-11-03)
------------------

0.9.0 (2014-10-31)
------------------
* Add support for multiple wheels per side
* Odometry computation:
  - New option to compute in open loop fashion
  - New option to skip publishing odom frame to tf
* Remove dependency on angles package
* Buildsystem fixes
* Contributors: Bence Magyar, Lukas Bulwahn, efernandez

0.8.1 (2014-07-11)
------------------

0.8.0 (2014-05-12)
------------------
* Add base_frame_id param (defaults to base_link)
  The nav_msgs/Odometry message specifies the child_frame_id field,
  which was previously not set.
  This commit creates a parameter to replace the previously hard-coded
  value of the child_frame_id of the published tf frame, and uses it
  in the odom message as well.
* Contributors: enriquefernandez

0.7.2 (2014-04-01)
------------------

0.7.1 (2014-03-31)
------------------
* Changed test-depend to build-depend for release jobs.
* Contributors: Bence Magyar

0.7.0 (2014-03-28)
------------------
* diff_drive_controller: New controller for differential drive wheel systems.
* Control is in the form of a velocity command, that is split then sent on the two wheels of a differential drive
wheel base.
* Odometry is published to tf and to a dedicated nav__msgs/Odometry topic.
* Realtime-safe implementation.
* Implements task-space velocity and acceleration limits.
* Automatic stop after command time-out.
* Contributors: Bence Magyar, Paul Mathieu, Enrique Fernandez.
