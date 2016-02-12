^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_joint_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.3 (2016-02-12)
------------------
* Add vertical scrollbar to joints list
* Clear controllers combo on cm change
  Clear the list of running joint trajectory controllers when the
  controller manager selection changes. This prevents potential conflicts when
  multiple controller managers have controllers with the same names.
* Fail gracefully if URDF is not loaded
* Save and restore plugin settings
* Stricter controller validation
* Fix broken URDF joint limits parsing
* Add controller resources query
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
