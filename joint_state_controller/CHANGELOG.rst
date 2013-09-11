^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_state_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
