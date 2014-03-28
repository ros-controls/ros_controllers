^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diff_drive_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* diff_drive_controller: New controller for differential drive wheel systems.
* Control is in the form of a velocity command, that is split then sent on the two wheels of a differential drive
wheel base.
* Odometry is published to tf and to a dedicated nav__msgs/Odometry topic.
* Realtime-safe implementation.
* Implements task-space velocity and acceleration limits.
* Automatic stop after command time-out.
* Contributors: Bence Magyar, Paul Mathieu, Enrique Fernandez.
