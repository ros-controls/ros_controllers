cmake_minimum_required(VERSION 3.0.2)
project(rqt_joint_trajectory_controller)

find_package(catkin REQUIRED COMPONENTS)
catkin_python_setup()
catkin_package()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/rqt_joint_trajectory_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
