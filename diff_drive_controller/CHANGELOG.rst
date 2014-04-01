^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diff_drive_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
